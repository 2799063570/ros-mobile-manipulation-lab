#include <aubo_mobile_nav_sorting/navigation_sorting_mission.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

#include <actionlib_msgs/GoalStatus.h>
#include <tf/transform_datatypes.h>

namespace aubo_mobile_nav_sorting
{
NavigationSortingMission::NavigationSortingMission(const ros::NodeHandle &node_handle,
                                                   const ros::NodeHandle &private_node_handle)
    : MissionContext(node_handle, private_node_handle)
{
  base_executor_.reset(new BaseExecutor(*this));
  stop_base = [this] { base_executor_->stopBase(); };
  arm_executor_.reset(new ArmExecutor(*this));
  state_publisher_ = node_handle_.advertise<std_msgs::String>("/nav_sorting/state", 1, true);

  start_service_ = node_handle_.advertiseService(
      "/nav_sorting/start", &NavigationSortingMission::startCallback, this); // 请求启动导航拣选
  stop_service_ = node_handle_.advertiseService(
      "/nav_sorting/stop", &NavigationSortingMission::stopCallback, this); // 请求停止导航拣选

  recover_service_ = node_handle_.advertiseService(
      "/nav_sorting/recover_stop", &NavigationSortingMission::recoverStopCallback, this);

  seedDynamicParameters();
  dynamic_server_.reset(new dynamic_reconfigure::Server<NavSortingConfig>(private_node_handle_));
  dynamic_reconfigure::Server<NavSortingConfig>::CallbackType callback =
      boost::bind(&NavigationSortingMission::reconfigureCallback, this, _1, _2);
  dynamic_server_->setCallback(callback);

  publishState(MissionState::INITIALIZING, "waiting for navigation and sorting");
  if (auto_start_)
  {
    auto_start_timer_ =
        node_handle_.createTimer(ros::Duration(std::max(0.1, startup_delay_)),
                                 &NavigationSortingMission::autoStartCallback, this, true);
  }
  else
  {
    publishState(MissionState::IDLE, "call /nav_sorting/start");
  }
}

NavigationSortingMission::~NavigationSortingMission()
{
  stop_requested_ = true;
  condition_.notify_all();
  base_executor_->cancelNavigation();
  base_executor_->stopBase();
  if (mission_thread_.joinable())
    mission_thread_.join();
}

void NavigationSortingMission::seedDynamicParameters()
{
  private_node_handle_.setParam("goal_x", sorting_goal_.x);
  private_node_handle_.setParam("goal_y", sorting_goal_.y);
  private_node_handle_.setParam("goal_yaw", sorting_goal_.yaw);
  private_node_handle_.setParam("pre_dock_x", pre_dock_goal_.x);
  private_node_handle_.setParam("pre_dock_y", pre_dock_goal_.y);
  private_node_handle_.setParam("pre_dock_yaw", pre_dock_goal_.yaw);
  private_node_handle_.setParam("near_field_enabled", near_field_enabled_);
  private_node_handle_.setParam("near_field_base_clearance", base_clearance_);
  private_node_handle_.setParam("near_field_max_candidates", near_field_max_candidates_);
  private_node_handle_.setParam("navigation_timeout", navigation_timeout_);
  private_node_handle_.setParam("navigation_retries", navigation_retries_);
  private_node_handle_.setParam("server_timeout", server_timeout_);
  private_node_handle_.setParam("sorting_initialization_timeout", initialization_timeout_);
  private_node_handle_.setParam("sorting_operation_timeout", operation_timeout_);
  private_node_handle_.setParam("home_before_navigation", home_before_navigation_);
}

void NavigationSortingMission::reconfigureCallback(NavSortingConfig &config, uint32_t)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (busy_)
  {
    config.goal_x = sorting_goal_.x;
    config.goal_y = sorting_goal_.y;
    config.goal_yaw = sorting_goal_.yaw;
    config.near_field_enabled = near_field_enabled_;
    config.pre_dock_x = pre_dock_goal_.x;
    config.pre_dock_y = pre_dock_goal_.y;
    config.pre_dock_yaw = pre_dock_goal_.yaw;
    config.near_field_base_clearance = base_clearance_;
    config.near_field_max_candidates = near_field_max_candidates_;
    config.navigation_timeout = navigation_timeout_;
    config.navigation_retries = navigation_retries_;
    config.server_timeout = server_timeout_;
    config.sorting_initialization_timeout = initialization_timeout_;
    config.sorting_operation_timeout = operation_timeout_;
    config.home_before_navigation = home_before_navigation_;
    ROS_WARN_THROTTLE(2.0, "nav_sorting parameters cannot change during a mission");
    return;
  }
  sorting_goal_ = {config.goal_x, config.goal_y, config.goal_yaw};
  near_field_enabled_ = config.near_field_enabled;
  pre_dock_goal_ = {config.pre_dock_x, config.pre_dock_y, config.pre_dock_yaw};
  base_clearance_ = config.near_field_base_clearance;
  near_field_max_candidates_ = config.near_field_max_candidates;
  navigation_timeout_ = config.navigation_timeout;
  navigation_retries_ = config.navigation_retries;
  server_timeout_ = config.server_timeout;
  initialization_timeout_ = config.sorting_initialization_timeout;
  operation_timeout_ = config.sorting_operation_timeout;
  home_before_navigation_ = config.home_before_navigation;
}

bool NavigationSortingMission::startCallback(std_srvs::Trigger::Request &,
                                             std_srvs::Trigger::Response &response)
{
  response.success = submitMission(response.message);
  return true;
}

bool NavigationSortingMission::stopCallback(std_srvs::Trigger::Request &,
                                            std_srvs::Trigger::Response &response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!busy_)
  {
    response.success = !stop_unconfirmed_;
    response.message = stop_unconfirmed_
                           ? "stop remains unconfirmed; call /nav_sorting/recover_stop"
                           : "no mission is running";
    return true;
  }
  stop_requested_ = true;
  base_executor_->cancelNavigation();
  base_executor_->stopBase();
  condition_.notify_all();
  response.success = true;
  response.message = "stop requested";
  return true;
}

void NavigationSortingMission::autoStartCallback(const ros::TimerEvent &)
{
  std::string unused;
  submitMission(unused);
}

bool NavigationSortingMission::submitMission(std::string &message)
{
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_unconfirmed_)
    {
      message = "stop unconfirmed; repair the service fault then call /nav_sorting/recover_stop";
      return false;
    }
    if (busy_)
    {
      message = "a mission is already running";
      return false;
    }
    busy_ = true;
    stop_requested_ = false;
    base_pose_failed_ = false;
  }
  if (mission_thread_.joinable())
    mission_thread_.join();
  try
  {
    mission_thread_ = std::thread(&NavigationSortingMission::runMission, this);
  }
  catch (const std::exception &error)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;
    message = error.what();
    return false;
  }
  message = "mission accepted";
  return true;
}

bool NavigationSortingMission::prepareAndObserveOnce()
{
  publishState(MissionState::PREPARING_ARM, "moving arm to work-ready pose");
  if (!arm_executor_->prepare("arm work preparation"))
    return false;
  publishState(MissionState::VALIDATING_DOCK, "planning observation pose");
  return arm_executor_->observe("camera observation");
}

bool NavigationSortingMission::stowForBaseRecovery()
{
  publishState(MissionState::STOWING_ARM, "making cmd_vel recovery motion safe");
  return arm_executor_->home("arm stow before base recovery");
}

bool NavigationSortingMission::prepareAndObserveWithRecovery()
{
  if (prepareAndObserveOnce())
    return true;
  if (!base_recovery_enabled_ || !arm_executor_->planningFailed())
    return false;
  for (std::size_t index = 0; index < recovery_steps_.size(); ++index)
  {
    if (!stowForBaseRecovery() ||
        !base_executor_->moveBaseDirect(recovery_steps_[index][0], recovery_steps_[index][1],
                                        index + 1, recovery_steps_.size()))
      return false;
    if (prepareAndObserveOnce())
      return true;
    if (!arm_executor_->planningFailed())
      return false;
  }
  return false;
}

bool NavigationSortingMission::sortWithRecovery()
{
  if (arm_executor_->sort("sorting"))
    return true;
  if (!base_recovery_enabled_ || !arm_executor_->planningFailed())
    return false;
  for (std::size_t index = 0; index < recovery_steps_.size(); ++index)
  {
    if (!stowForBaseRecovery() ||
        !base_executor_->moveBaseDirect(recovery_steps_[index][0], recovery_steps_[index][1],
                                        index + 1, recovery_steps_.size()))
      return false;
    if (!prepareAndObserveOnce())
    {
      if (arm_executor_->planningFailed())
        continue;
      return false;
    }
    if (arm_executor_->sort("sorting retry"))
      return true;
    if (!arm_executor_->planningFailed())
      return false;
  }
  return false;
}

bool NavigationSortingMission::retreatAfterSorting(const XmlRpc::XmlRpcValue &workspace)
{
  publishState(MissionState::STOWING_ARM,
               memberString(workspace, "id") + " before post-sort retreat");
  if (!arm_executor_->home("arm stow before post-sort retreat"))
    return false;
  if (!memberBool(workspace, "retreat_enabled", post_sort_retreat_enabled_))
    return true;
  const double distance =
      std::max(0.0, memberDouble(workspace, "retreat_distance", post_sort_retreat_distance_));
  if (distance <= 1e-6)
    return true;
  std::string pose_frame = memberString(workspace, "navigation_goal_frame");
  if (pose_frame.empty())
    pose_frame = navigation_frame_;
  Pose2D current;
  if (!base_executor_->currentBasePose(current, pose_frame))
    return false;
  const Pose2D target = {current.x - distance * std::cos(current.yaw),
                         current.y - distance * std::sin(current.yaw), current.yaw};
  return base_executor_->driveStraightTo(target,
                                         memberString(workspace, "id") + " post-sort retreat",
                                         MissionState::RETREATING_BASE, pose_frame);
}

double NavigationSortingMission::scoreCandidate(const Pose2D &pose, bool &valid,
                                                double &clearance) const
{
  const double dx = std::max(std::abs(pose.x - table_geometry_[0]) - 0.5 * table_geometry_[2], 0.0);
  const double dy = std::max(std::abs(pose.y - table_geometry_[1]) - 0.5 * table_geometry_[3], 0.0);
  clearance = std::hypot(dx, dy);
  valid = clearance >= base_clearance_;
  double camera_error = 0.0;
  for (const auto &point : workpiece_points_)
  {
    const double world_x = point[0] - pose.x;
    const double world_y = point[1] - pose.y;
    const double local_x = std::cos(pose.yaw) * world_x + std::sin(pose.yaw) * world_y;
    const double local_y = -std::sin(pose.yaw) * world_x + std::cos(pose.yaw) * world_y;
    if (local_x < detector_workspace_[0] || local_x > detector_workspace_[1] ||
        local_y < detector_workspace_[2] || local_y > detector_workspace_[3])
      valid = false;
    camera_error +=
        std::pow(local_x - camera_target_[0], 2) + std::pow(local_y - camera_target_[1], 2);
  }
  return camera_error + 0.15 * std::hypot(pose.x - sorting_goal_.x, pose.y - sorting_goal_.y) +
         0.05 * std::abs(angleError(pose.yaw, sorting_goal_.yaw)) - 0.02 * clearance;
}

std::vector<NavigationSortingMission::Candidate>
NavigationSortingMission::nearFieldCandidates() const
{
  std::vector<Candidate> candidates;
  for (const double x : candidate_x_)
    for (const double y : candidate_y_)
      for (const double yaw : candidate_yaw_)
      {
        Candidate candidate;
        candidate.pose = {x, y, yaw};
        bool valid = false;
        candidate.score = scoreCandidate(candidate.pose, valid, candidate.clearance);
        if (valid && (!direct_dock_enabled_ ||
                      base_executor_->canDirectDock(pre_dock_goal_, candidate.pose)))
          candidates.push_back(candidate);
      }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate &left, const Candidate &right) { return left.score < right.score; });
  if (candidates.size() > static_cast<std::size_t>(near_field_max_candidates_))
    candidates.resize(near_field_max_candidates_);
  return candidates;
}

bool NavigationSortingMission::coordinateNearField()
{
  if (!base_executor_->navigate(pre_dock_goal_, "pre-dock"))
    return false;
  bool arm_prepared = false;
  const auto candidates = nearFieldCandidates();
  for (std::size_t index = 0; index < candidates.size(); ++index)
  {
    const auto &candidate = candidates[index].pose;
    bool docked = direct_dock_enabled_
                      ? base_executor_->driveStraightTo(candidate, "fine-dock candidate")
                      : base_executor_->navigate(candidate, "fine-dock");
    if (!docked && (base_pose_failed_ || stop_requested_))
      return false;
    if (!docked)
      continue;
    if (!arm_prepared)
    {
      if (!arm_executor_->prepare("arm work preparation"))
        return false;
      arm_prepared = true;
    }
    if (arm_executor_->observe("camera observation"))
      return true;
    if (!arm_executor_->home("arm re-stowing"))
      return false;
    arm_prepared = false;
    if (direct_dock_enabled_ &&
        !base_executor_->driveStraightTo(pre_dock_goal_, "pre-dock retreat"))
      return false;
  }
  return false;
}

bool NavigationSortingMission::runWorkstationSequence()
{
  std::vector<int> enabled;
  for (int index = 0; index < workstations_.size(); ++index)
    if (memberBool(workstations_[index], "enabled", true))
      enabled.push_back(index);
  if (enabled.empty())
    return false;
  for (std::size_t order = 0; order < enabled.size(); ++order)
  {
    const auto &workspace = workstations_[enabled[order]];
    const std::string identifier = memberString(workspace, "id");
    publishState(MissionState::CONFIGURING_WORKSTATION, identifier);
    if ((home_before_navigation_ || order > 0) && !arm_executor_->home("arm homing"))
      return false;
    if (!arm_executor_->configureWorkspace(workspace))
      return false;
    std::string goal_frame = memberString(workspace, "navigation_goal_frame");
    if (goal_frame.empty())
      goal_frame = navigation_frame_;
    const Pose2D goal = memberPose(workspace, "navigation_goal");
    if (workspace.hasMember("pre_dock_goal"))
    {
      const Pose2D pre_dock = memberPose(workspace, "pre_dock_goal");
      if (!base_executor_->navigate(pre_dock, "workstation '" + identifier + "' pre-dock",
                                    goal_frame) ||
          !base_executor_->driveStraightTo(goal, "workstation '" + identifier + "' fine-dock",
                                           MissionState::DIRECT_DOCKING, goal_frame))
        return false;
    }
    else if (!base_executor_->navigate(goal, "workstation '" + identifier + "'", goal_frame))
      return false;
    publishState(MissionState::AT_WORKSTATION, identifier + "; validating arm reach");
    if (!prepareAndObserveWithRecovery())
      return false;
    publishState(MissionState::SORTING, "workstation '" + identifier + "'");
    if (!sortWithRecovery() || !retreatAfterSorting(workspace))
      return false;
    publishState(MissionState::WORKSTATION_COMPLETE, identifier);
  }
  return true;
}

bool NavigationSortingMission::recoverStopCallback(std_srvs::Trigger::Request &,
                                                   std_srvs::Trigger::Response &response)
{
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (busy_ || pending_rpcs_->load() != 0)
    {
      response.success = false;
      response.message = "wait for mission cleanup and outstanding service calls";
      return true;
    }
    if (!stop_unconfirmed_)
    {
      response.success = true;
      response.message = "no unconfirmed stop";
      return true;
    }
    busy_ = true;
    stop_requested_ = true;
  }
  if (mission_thread_.joinable())
    mission_thread_.join();
  try
  {
    mission_thread_ = std::thread(&NavigationSortingMission::recoverStop, this);
  }
  catch (const std::exception &error)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;
    response.success = false;
    response.message = error.what();
    return true;
  }
  response.success = true;
  response.message = "stop verification accepted; wait for STOPPED or STOP_UNCONFIRMED";
  return true;
}

void NavigationSortingMission::recoverStop()
{
  bool recovered = false;
  try
  {
    base_executor_->cancelNavigation();
    base_executor_->stopBase();
    recovered = arm_executor_->recoverStop();
  }
  catch (const std::exception &error)
  {
    ROS_ERROR_STREAM(error.what());
  }
  publishState(recovered ? MissionState::STOPPED : MissionState::STOP_UNCONFIRMED,
               recovered ? "stop confirmed; a new mission may be started"
                         : "stop verification failed; repair fault and retry recover_stop");
  std::lock_guard<std::mutex> lock(mutex_);
  busy_ = false;
}

void NavigationSortingMission::runMission()
{
  bool success = false;
  try
  {
    if (!arm_executor_->waitForSortingReady())
      throw std::runtime_error("sorting node is not ready");
    if (workstations_.size() > 0)
    {
      success = runWorkstationSequence();
    }
    else
    {
      if (home_before_navigation_ && !arm_executor_->home("arm homing"))
        throw std::runtime_error("arm homing failed");
      if (near_field_enabled_)
      {
        if (!coordinateNearField())
          throw std::runtime_error("near-field coordination failed");
      }
      else
      {
        if (!base_executor_->navigate(sorting_goal_, "workstation") ||
            !prepareAndObserveWithRecovery())
          throw std::runtime_error("workstation approach failed");
      }
      publishState(MissionState::AT_WORKSTATION, "dock and camera view validated");
      publishState(MissionState::SORTING, "sorting detected objects");
      success = sortWithRecovery();
    }
  }
  catch (const std::exception &error)
  {
    ROS_ERROR_STREAM("Navigation-sorting mission failed: " << error.what());
  }
  base_executor_->stopBase();
  if (operation_active_)
    arm_executor_->cancelSortingAndWait();
  if (stop_unconfirmed_)
    publishState(MissionState::STOP_UNCONFIRMED,
                 "sorting stop not confirmed; new missions blocked");
  else if (stop_requested_)
    publishState(MissionState::STOPPED, "mission cancelled");
  else if (success)
    publishState(MissionState::SUCCEEDED, "navigation and sorting complete");
  else
    publishState(MissionState::FAILED, base_pose_failed_ ? "base pose unavailable or stale"
                                                         : "inspect move_base and /sorting/state");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;
  }
}

} // namespace aubo_mobile_nav_sorting
