#include <aubo_mobile_nav_sorting/navigation_sorting_mission.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

#include <actionlib_msgs/GoalStatus.h>
#include <tf/transform_datatypes.h>

namespace aubo_mobile_nav_sorting
{
namespace
{
std::vector<double> vectorParam(const ros::NodeHandle& node,
                                const std::string& name,
                                const std::vector<double>& fallback)
{
  std::vector<double> result;
  if (!node.getParam(name, result))
    result = fallback;
  if (result.empty())
    throw std::runtime_error(name + " must be a non-empty list");
  return result;
}

std::vector<std::vector<double>> matrixParam(
    const ros::NodeHandle& node, const std::string& name,
    const std::vector<std::vector<double>>& fallback)
{
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value))
    return fallback;
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::runtime_error(name + " must be a list");
  std::vector<std::vector<double>> result;
  for (int row = 0; row < value.size(); ++row)
  {
    if (value[row].getType() != XmlRpc::XmlRpcValue::TypeArray)
      throw std::runtime_error(name + " rows must be lists");
    std::vector<double> output_row;
    for (int column = 0; column < value[row].size(); ++column)
    {
      const auto type = value[row][column].getType();
      if (type == XmlRpc::XmlRpcValue::TypeInt)
        output_row.push_back(static_cast<int>(value[row][column]));
      else if (type == XmlRpc::XmlRpcValue::TypeDouble)
        output_row.push_back(static_cast<double>(value[row][column]));
      else
        throw std::runtime_error(name + " values must be numeric");
    }
    result.push_back(output_row);
  }
  return result;
}

bool contains(const std::vector<std::string>& values, const std::string& value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

std::string jsonEscape(const std::string& input)
{
  std::ostringstream output;
  for (const char character : input)
  {
    switch (character)
    {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}
}  // namespace

NavigationSortingMission::NavigationSortingMission(
    const ros::NodeHandle& node_handle,
    const ros::NodeHandle& private_node_handle)
  : node_handle_(node_handle), private_node_handle_(private_node_handle)
{
  loadParameters();
  validateWorkstations();
  navigation_client_.reset(new MoveBaseClient(navigation_action_, true));

  state_publisher_ = node_handle_.advertise<std_msgs::String>(
      "/nav_sorting/state", 1, true);
  workspace_publisher_ = node_handle_.advertise<std_msgs::String>(
      "/nav_sorting/current_workstation", 1, true);
  velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>(
      velocity_topic_, 2);
  sorting_state_subscriber_ = node_handle_.subscribe(
      sorting_state_topic_, 5,
      &NavigationSortingMission::sortingStateCallback, this);
  sorting_failure_subscriber_ = node_handle_.subscribe(
      sorting_failure_topic_, 5,
      &NavigationSortingMission::sortingFailureCallback, this);

  clear_costmaps_client_ = node_handle_.serviceClient<std_srvs::Empty>(
      "/move_base/clear_costmaps");
  home_client_ = node_handle_.serviceClient<std_srvs::Trigger>(home_service_name_);
  prepare_client_ = node_handle_.serviceClient<std_srvs::Trigger>(prepare_service_name_);
  observe_client_ = node_handle_.serviceClient<std_srvs::Trigger>(observe_service_name_);
  sort_client_ = node_handle_.serviceClient<std_srvs::Trigger>(sort_service_name_);
  sorting_stop_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
      sorting_stop_service_name_);
  configure_workspace_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
      configure_workspace_service_name_);

  start_service_ = node_handle_.advertiseService(
      "/nav_sorting/start", &NavigationSortingMission::startCallback, this);
  stop_service_ = node_handle_.advertiseService(
      "/nav_sorting/stop", &NavigationSortingMission::stopCallback, this);

  seedDynamicParameters();
  dynamic_server_.reset(new dynamic_reconfigure::Server<NavSortingConfig>(
      private_node_handle_));
  dynamic_reconfigure::Server<NavSortingConfig>::CallbackType callback =
      boost::bind(&NavigationSortingMission::reconfigureCallback, this, _1, _2);
  dynamic_server_->setCallback(callback);

  publishState("INITIALIZING", "waiting for navigation and sorting");
  if (auto_start_)
  {
    auto_start_timer_ = node_handle_.createTimer(
        ros::Duration(std::max(0.1, startup_delay_)),
        &NavigationSortingMission::autoStartCallback, this, true);
  }
  else
  {
    publishState("IDLE", "call /nav_sorting/start");
  }
}

NavigationSortingMission::~NavigationSortingMission()
{
  stop_requested_ = true;
  condition_.notify_all();
  if (navigation_client_)
    navigation_client_->cancelAllGoals();
  stopBase();
  if (mission_thread_.joinable())
    mission_thread_.join();
}

void NavigationSortingMission::loadParameters()
{
  private_node_handle_.param("navigation_action", navigation_action_,
                             std::string("/move_base"));
  private_node_handle_.param("navigation_frame", navigation_frame_,
                             std::string("map"));
  private_node_handle_.param("base_frame", base_frame_,
                             std::string("base_footprint"));

  const auto goal = vectorParam(private_node_handle_, "sorting_goal",
                                {2.15, 0.0, 0.0});
  if (goal.size() != 3)
    throw std::runtime_error("sorting_goal must be [x, y, yaw]");
  sorting_goal_ = {goal[0], goal[1], goal[2]};
  const auto pre_dock = vectorParam(private_node_handle_, "pre_dock_goal",
                                    {1.85, 0.0, 0.0});
  if (pre_dock.size() != 3)
    throw std::runtime_error("pre_dock_goal must be [x, y, yaw]");
  pre_dock_goal_ = {pre_dock[0], pre_dock[1], pre_dock[2]};

  private_node_handle_.param("near_field_enabled", near_field_enabled_, false);
  candidate_x_ = vectorParam(private_node_handle_, "near_field_candidate_x",
                             {2.16, 2.10, 2.06});
  candidate_y_ = vectorParam(private_node_handle_, "near_field_candidate_y",
                             {0.0, -0.08, 0.08});
  candidate_yaw_ = vectorParam(private_node_handle_, "near_field_candidate_yaw",
                               {0.0, -0.08, 0.08});
  private_node_handle_.param("near_field_max_candidates",
                             near_field_max_candidates_, 6);
  table_geometry_ = vectorParam(private_node_handle_, "near_field_table",
                                {3.0, 0.0, 0.80, 1.20});
  detector_workspace_ = vectorParam(
      private_node_handle_, "near_field_detector_workspace",
      {0.40, 0.82, -0.22, 0.22});
  camera_target_ = vectorParam(private_node_handle_, "near_field_camera_target",
                               {0.62, 0.0});
  workpiece_points_ = matrixParam(
      private_node_handle_, "near_field_workpieces",
      {{2.78, -0.12}, {2.86, 0.0}, {2.78, 0.12}});
  if (table_geometry_.size() != 4 || detector_workspace_.size() != 4 ||
      camera_target_.size() != 2)
    throw std::runtime_error("invalid near-field geometry dimensions");
  for (const auto& point : workpiece_points_)
    if (point.size() != 2)
      throw std::runtime_error("near_field_workpieces entries must be [x, y]");

  private_node_handle_.param("near_field_base_clearance", base_clearance_, 0.40);
  private_node_handle_.param("near_field_direct_dock_enabled",
                             direct_dock_enabled_, true);
  private_node_handle_.param("near_field_direct_dock_max_distance",
                             direct_dock_max_distance_, 0.50);
  private_node_handle_.param("near_field_direct_dock_lateral_tolerance",
                             direct_dock_lateral_tolerance_, 0.04);
  private_node_handle_.param("near_field_direct_dock_yaw_tolerance",
                             direct_dock_yaw_tolerance_, 0.04);
  private_node_handle_.param("near_field_direct_dock_goal_tolerance",
                             direct_dock_goal_tolerance_, 0.06);
  private_node_handle_.param("near_field_direct_dock_timeout",
                             direct_dock_timeout_, 15.0);
  private_node_handle_.param("near_field_direct_dock_stall_timeout",
                             direct_dock_stall_timeout_, 2.5);
  private_node_handle_.param("near_field_direct_dock_progress_epsilon",
                             direct_dock_progress_epsilon_, 0.005);
  private_node_handle_.param("near_field_heading_alignment_enabled",
                             heading_alignment_enabled_, true);
  private_node_handle_.param("near_field_heading_max_correction",
                             heading_max_correction_, 0.12);
  private_node_handle_.param("near_field_heading_speed", heading_speed_, 0.12);
  private_node_handle_.param("near_field_heading_goal_tolerance",
                             heading_goal_tolerance_, 0.015);
  private_node_handle_.param("near_field_heading_final_tolerance",
                             heading_final_tolerance_, 0.025);
  private_node_handle_.param("near_field_heading_timeout", heading_timeout_, 4.0);
  private_node_handle_.param("near_field_heading_stall_timeout",
                             heading_stall_timeout_, 1.5);

  private_node_handle_.param("server_timeout", server_timeout_, 45.0);
  private_node_handle_.param("navigation_timeout", navigation_timeout_, 180.0);
  private_node_handle_.param("navigation_retries", navigation_retries_, 1);
  private_node_handle_.param("sorting_initialization_timeout",
                             initialization_timeout_, 60.0);
  private_node_handle_.param("sorting_operation_timeout", operation_timeout_, 300.0);
  private_node_handle_.param("startup_delay", startup_delay_, 3.0);
  private_node_handle_.param("home_before_navigation",
                             home_before_navigation_, true);
  private_node_handle_.param("auto_start", auto_start_, false);
  if (!private_node_handle_.getParam("workstations", workstations_))
    workstations_.setSize(0);

  private_node_handle_.param("base_recovery_enabled", base_recovery_enabled_, true);
  private_node_handle_.param("base_recovery_cmd_vel_topic", velocity_topic_,
                             std::string("/cmd_vel_raw"));
  private_node_handle_.param("base_recovery_speed", base_recovery_speed_, 0.04);
  private_node_handle_.param("base_recovery_rate", base_recovery_rate_, 20.0);
  private_node_handle_.param("base_recovery_settle_time",
                             base_recovery_settle_time_, 0.8);
  recovery_steps_ = matrixParam(
      private_node_handle_, "base_recovery_steps",
      {{0.0, 0.06}, {0.0, -0.12}, {0.0, 0.06}, {0.05, 0.0}, {-0.10, 0.0}});
  for (const auto& step : recovery_steps_)
    if (step.size() != 2 ||
        (std::abs(step[0]) > 1e-6 && std::abs(step[1]) > 1e-6))
      throw std::runtime_error("base_recovery_steps must contain single-axis [dx, dy]");
  private_node_handle_.param("post_sort_retreat_enabled",
                             post_sort_retreat_enabled_, false);
  private_node_handle_.param("post_sort_retreat_distance",
                             post_sort_retreat_distance_, 0.30);

  private_node_handle_.param("sorting_state_topic", sorting_state_topic_,
                             std::string("/sorting/state"));
  private_node_handle_.param("sorting_failure_topic", sorting_failure_topic_,
                             std::string("/sorting/failure"));
  private_node_handle_.param("sorting_home_service", home_service_name_,
                             std::string("/sorting/home"));
  private_node_handle_.param("sorting_prepare_service", prepare_service_name_,
                             std::string("/sorting/prepare_work"));
  private_node_handle_.param("sorting_observe_service", observe_service_name_,
                             std::string("/sorting/move_to_observation"));
  private_node_handle_.param("sorting_start_service", sort_service_name_,
                             std::string("/sorting/start"));
  private_node_handle_.param("sorting_stop_service", sorting_stop_service_name_,
                             std::string("/sorting/stop"));
  private_node_handle_.param("sorting_configure_service",
                             configure_workspace_service_name_,
                             std::string("/sorting/configure_workspace"));
  private_node_handle_.param("sorting_workspace_param", workspace_parameter_,
                             std::string("/sorting/workspace_config"));
}

void NavigationSortingMission::validateWorkstations() const
{
  if (workstations_.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::runtime_error("workstations must be a list");
  std::set<std::string> identifiers;
  for (int index = 0; index < workstations_.size(); ++index)
  {
    const auto& workspace = workstations_[index];
    if (workspace.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      throw std::runtime_error("each workstation must be a mapping");
    const std::string identifier = memberString(workspace, "id");
    if (identifier.empty() || !identifiers.insert(identifier).second)
      throw std::runtime_error("each workstation needs a unique non-empty id");
    if (workspace.hasMember("navigation_goal_frame") &&
        memberString(workspace, "navigation_goal_frame").empty())
      throw std::runtime_error(identifier +
                               " has invalid navigation_goal_frame");
    memberPose(workspace, "navigation_goal");
    if (workspace.hasMember("pre_dock_goal"))
      memberPose(workspace, "pre_dock_goal");
    for (const std::string key : {"table_center", "table_size"})
    {
      if (!workspace.hasMember(key) ||
          workspace[key].getType() != XmlRpc::XmlRpcValue::TypeArray ||
          workspace[key].size() != 3)
        throw std::runtime_error(identifier + " has invalid " + key);
    }
  }
}

void NavigationSortingMission::seedDynamicParameters()
{
  private_node_handle_.setParam("goal_x", sorting_goal_.x);
  private_node_handle_.setParam("goal_y", sorting_goal_.y);
  private_node_handle_.setParam("goal_yaw", sorting_goal_.yaw);
  private_node_handle_.setParam("pre_dock_x", pre_dock_goal_.x);
  private_node_handle_.setParam("pre_dock_y", pre_dock_goal_.y);
  private_node_handle_.setParam("pre_dock_yaw", pre_dock_goal_.yaw);
}

void NavigationSortingMission::reconfigureCallback(NavSortingConfig& config,
                                                    uint32_t)
{
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

bool NavigationSortingMission::startCallback(std_srvs::Trigger::Request&,
                                             std_srvs::Trigger::Response& response)
{
  response.success = submitMission(response.message);
  return true;
}

bool NavigationSortingMission::stopCallback(std_srvs::Trigger::Request&,
                                            std_srvs::Trigger::Response& response)
{
  if (!busy_)
  {
    response.success = true;
    response.message = "no mission is running";
    return true;
  }
  stop_requested_ = true;
  navigation_client_->cancelAllGoals();
  stopBase();
  std_srvs::Trigger service;
  if (!sorting_stop_client_.call(service))
    ROS_WARN("Could not call sorting stop service");
  condition_.notify_all();
  publishState("STOPPING", "cancelling active operation");
  response.success = true;
  response.message = "stop requested";
  return true;
}

void NavigationSortingMission::sortingStateCallback(
    const std_msgs::String::ConstPtr& message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sorting_state_ = message->data;
  ++sorting_sequence_;
  condition_.notify_all();
}

void NavigationSortingMission::sortingFailureCallback(
    const std_msgs::String::ConstPtr& message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sorting_failure_ = message->data;
  condition_.notify_all();
}

void NavigationSortingMission::autoStartCallback(const ros::TimerEvent&)
{
  std::string unused;
  submitMission(unused);
}

bool NavigationSortingMission::submitMission(std::string& message)
{
  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true))
  {
    message = "a mission is already running";
    return false;
  }
  if (mission_thread_.joinable())
    mission_thread_.join();
  stop_requested_ = false;
  mission_thread_ = std::thread(&NavigationSortingMission::runMission, this);
  message = "mission accepted";
  return true;
}

bool NavigationSortingMission::waitForSortingReady()
{
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(initialization_timeout_);
  std::unique_lock<std::mutex> lock(mutex_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    const std::string state = stateName(sorting_state_);
    if (state == "IDLE" || state == "READY" || state == "STOPPED")
      return true;
    if (state == "ERROR")
      return false;
    condition_.wait_for(lock, std::chrono::milliseconds(200));
  }
  ROS_ERROR("Timed out waiting for sorting node readiness");
  return false;
}

bool NavigationSortingMission::callSortingOperation(
    ros::ServiceClient& client, const std::vector<std::string>& running_states,
    const std::string& label)
{
  if (stop_requested_)
    return false;
  unsigned long start_sequence;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    start_sequence = sorting_sequence_;
  }
  std_srvs::Trigger service;
  if (!client.call(service) || !service.response.success)
  {
    ROS_ERROR_STREAM(label << " command failed: " << service.response.message);
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(operation_timeout_);
  bool saw_operation = false;
  std::unique_lock<std::mutex> lock(mutex_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    const std::string state = stateName(sorting_state_);
    if (sorting_sequence_ > start_sequence)
    {
      if (contains(running_states, state))
        saw_operation = true;
      else if (state == "READY" &&
               (saw_operation || sorting_sequence_ > start_sequence + 1))
        return true;
      else if (state == "ERROR" || state == "STOPPED")
      {
        ROS_ERROR_STREAM(label << " failed: " << sorting_state_);
        return false;
      }
    }
    if (stop_requested_)
      return false;
    condition_.wait_for(lock, std::chrono::milliseconds(200));
  }
  ROS_ERROR_STREAM(label << " timed out after " << operation_timeout_ << " s");
  return false;
}

bool NavigationSortingMission::planningFailed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sorting_failure_.find("PLANNING_FAILED") == 0;
}

bool NavigationSortingMission::configureWorkspace(
    const XmlRpc::XmlRpcValue& workspace)
{
  XmlRpc::XmlRpcValue payload;
  for (auto iterator = workspace.begin(); iterator != workspace.end(); ++iterator)
  {
    if (iterator->first != "navigation_goal" &&
        iterator->first != "pre_dock_goal" &&
        iterator->first != "navigation_goal_frame" &&
        iterator->first != "enabled")
      payload[iterator->first] = iterator->second;
  }
  ros::param::set(workspace_parameter_, payload);
  std_msgs::String message;
  message.data = toJson(workspace);
  workspace_publisher_.publish(message);
  if (!ros::service::waitForService(configure_workspace_service_name_,
                                    ros::Duration(server_timeout_)))
  {
    ROS_ERROR("Sorting workspace service is unavailable");
    return false;
  }
  std_srvs::Trigger service;
  if (!configure_workspace_client_.call(service) || !service.response.success)
  {
    ROS_ERROR_STREAM("Workstation configuration rejected: "
                     << service.response.message);
    return false;
  }
  return true;
}

bool NavigationSortingMission::navigateOnce(const Pose2D& target,
                                             const std::string& goal_frame)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = goal_frame;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = target.x;
  goal.target_pose.pose.position.y = target.y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target.yaw);
  navigation_client_->sendGoal(goal);
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(navigation_timeout_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (stop_requested_)
    {
      navigation_client_->cancelGoal();
      return false;
    }
    if (navigation_client_->waitForResult(ros::Duration(0.2)))
      return navigation_client_->getState() ==
          actionlib::SimpleClientGoalState::SUCCEEDED;
  }
  navigation_client_->cancelGoal();
  return false;
}

bool NavigationSortingMission::navigate(const Pose2D& target,
                                        const std::string& stage,
                                        const std::string& requested_frame)
{
  const std::string goal_frame = requested_frame.empty() ?
      navigation_frame_ : requested_frame;
  if (!navigation_client_->waitForServer(ros::Duration(server_timeout_)))
  {
    ROS_ERROR_STREAM("Navigation action unavailable: " << navigation_action_);
    return false;
  }
  for (int attempt = 0; attempt <= navigation_retries_; ++attempt)
  {
    std::ostringstream detail;
    detail << stage << " attempt " << attempt + 1 << "/"
           << navigation_retries_ + 1 << " to [" << std::fixed
           << std::setprecision(2) << target.x << ", " << target.y << ", "
           << target.yaw << "] in " << goal_frame;
    publishState("NAVIGATING", detail.str());
    if (navigateOnce(target, goal_frame))
      return true;
    if (stop_requested_)
      return false;
    std_srvs::Empty clear;
    if (!clear_costmaps_client_.call(clear))
      ROS_WARN("Could not clear costmaps before navigation retry");
  }
  return false;
}

bool NavigationSortingMission::currentBasePose(
    Pose2D& pose, const std::string& requested_frame) const
{
  const std::string pose_frame = requested_frame.empty() ?
      navigation_frame_ : requested_frame;
  try
  {
    tf_listener_.waitForTransform(pose_frame, base_frame_, ros::Time(0),
                                  ros::Duration(1.0));
    tf::StampedTransform transform;
    tf_listener_.lookupTransform(pose_frame, base_frame_, ros::Time(0),
                                 transform);
    pose = {transform.getOrigin().x(), transform.getOrigin().y(),
            tf::getYaw(transform.getRotation())};
    return true;
  }
  catch (const tf::TransformException& error)
  {
    ROS_WARN_STREAM("Cannot read base pose: " << error.what());
    return false;
  }
}

bool NavigationSortingMission::alignHeading(double target_yaw,
                                            const std::string& label,
                                            const std::string& pose_frame)
{
  Pose2D actual;
  if (!currentBasePose(actual, pose_frame))
    return false;
  const double initial_error = angleError(target_yaw, actual.yaw);
  if (std::abs(initial_error) <= heading_goal_tolerance_)
    return true;
  if (!heading_alignment_enabled_ ||
      std::abs(initial_error) > heading_max_correction_)
    return false;
  navigation_client_->cancelAllGoals();
  stopBase();
  ros::WallDuration(0.2).sleep();
  publishState("ALIGNING_BASE", label + " bounded heading correction");
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(heading_timeout_);
  auto last_progress = std::chrono::steady_clock::now();
  double best_error = std::abs(initial_error);
  ros::WallRate rate(base_recovery_rate_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (stop_requested_ || !currentBasePose(actual, pose_frame))
      return false;
    const double error = angleError(target_yaw, actual.yaw);
    const double absolute_error = std::abs(error);
    if (absolute_error <= heading_goal_tolerance_)
    {
      stopBase();
      return true;
    }
    if (absolute_error < best_error - 0.002)
    {
      best_error = absolute_error;
      last_progress = std::chrono::steady_clock::now();
    }
    else if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                           last_progress).count() >
             heading_stall_timeout_)
    {
      stopBase();
      return false;
    }
    geometry_msgs::Twist command;
    command.angular.z = std::copysign(
        std::min(heading_speed_, std::max(0.04, absolute_error)), error);
    velocity_publisher_.publish(command);
    rate.sleep();
  }
  stopBase();
  return false;
}

bool NavigationSortingMission::canDirectDock(const Pose2D& start,
                                             const Pose2D& target) const
{
  const double dx = target.x - start.x;
  const double dy = target.y - start.y;
  const double longitudinal = std::cos(start.yaw) * dx + std::sin(start.yaw) * dy;
  const double lateral = -std::sin(start.yaw) * dx + std::cos(start.yaw) * dy;
  return std::abs(longitudinal) <= direct_dock_max_distance_ &&
      std::abs(lateral) <= direct_dock_lateral_tolerance_ &&
      std::abs(angleError(target.yaw, start.yaw)) <= heading_max_correction_;
}

bool NavigationSortingMission::driveStraightTo(const Pose2D& target,
                                               const std::string& label,
                                               const std::string& state,
                                               const std::string& pose_frame)
{
  Pose2D start;
  if (!currentBasePose(start, pose_frame) || !canDirectDock(start, target) ||
      !alignHeading(target.yaw, label, pose_frame) ||
      !currentBasePose(start, pose_frame))
    return false;
  const auto longitudinalError = [&](const Pose2D& pose) {
    return std::cos(pose.yaw) * (target.x - pose.x) +
           std::sin(pose.yaw) * (target.y - pose.y);
  };
  double longitudinal = longitudinalError(start);
  publishState(state, label + " closed-loop cmd_vel motion");
  navigation_client_->cancelAllGoals();
  stopBase();
  ros::WallDuration(0.2).sleep();
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(direct_dock_timeout_);
  auto last_progress = std::chrono::steady_clock::now();
  double best_error = std::abs(longitudinal);
  ros::WallRate rate(base_recovery_rate_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    Pose2D actual;
    if (stop_requested_ || !currentBasePose(actual, pose_frame))
    {
      stopBase();
      return false;
    }
    longitudinal = longitudinalError(actual);
    const double dx = target.x - actual.x;
    const double dy = target.y - actual.y;
    const double lateral = -std::sin(actual.yaw) * dx + std::cos(actual.yaw) * dy;
    const double yaw_error = angleError(target.yaw, actual.yaw);
    if (std::hypot(dx, dy) <= direct_dock_goal_tolerance_ &&
        std::abs(yaw_error) <= heading_final_tolerance_)
    {
      stopBase();
      return true;
    }
    if (std::abs(lateral) > direct_dock_lateral_tolerance_ ||
        std::abs(yaw_error) > direct_dock_yaw_tolerance_)
    {
      stopBase();
      return false;
    }
    const double current_error = std::abs(longitudinal);
    if (current_error <= best_error - direct_dock_progress_epsilon_)
    {
      best_error = current_error;
      last_progress = std::chrono::steady_clock::now();
    }
    else if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                           last_progress).count() >
             direct_dock_stall_timeout_)
    {
      stopBase();
      return false;
    }
    geometry_msgs::Twist command;
    command.linear.x = std::copysign(
        std::min(base_recovery_speed_, std::max(0.02, current_error)),
        longitudinal);
    velocity_publisher_.publish(command);
    rate.sleep();
  }
  stopBase();
  return false;
}

bool NavigationSortingMission::moveBaseDirect(double dx, double dy,
                                              int attempt, int total)
{
  const double distance = std::max(std::abs(dx), std::abs(dy));
  if (distance <= 1e-6)
    return true;
  navigation_client_->cancelAllGoals();
  std::ostringstream detail;
  detail << "cmd_vel step " << attempt << "/" << total << " dx=" << dx
         << " dy=" << dy;
  publishState("ADJUSTING_BASE", detail.str());
  geometry_msgs::Twist command;
  command.linear.x = std::abs(dx) > 1e-6 ? std::copysign(base_recovery_speed_, dx) : 0.0;
  command.linear.y = std::abs(dy) > 1e-6 ? std::copysign(base_recovery_speed_, dy) : 0.0;
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(distance / base_recovery_speed_);
  ros::WallRate rate(base_recovery_rate_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (stop_requested_)
    {
      stopBase();
      return false;
    }
    velocity_publisher_.publish(command);
    rate.sleep();
  }
  stopBase();
  ros::WallDuration(base_recovery_settle_time_).sleep();
  return !stop_requested_;
}

void NavigationSortingMission::stopBase()
{
  if (velocity_publisher_)
    velocity_publisher_.publish(geometry_msgs::Twist());
}

bool NavigationSortingMission::prepareAndObserveOnce()
{
  publishState("PREPARING_ARM", "moving arm to work-ready pose");
  if (!callSortingOperation(prepare_client_, {"PREPARING"},
                            "arm work preparation"))
    return false;
  publishState("VALIDATING_DOCK", "planning observation pose");
  return callSortingOperation(observe_client_, {"OBSERVING"},
                              "camera observation");
}

bool NavigationSortingMission::stowForBaseRecovery()
{
  publishState("STOWING_ARM", "making cmd_vel recovery motion safe");
  return callSortingOperation(home_client_, {"HOMING"},
                              "arm stow before base recovery");
}

bool NavigationSortingMission::prepareAndObserveWithRecovery()
{
  if (prepareAndObserveOnce())
    return true;
  if (!base_recovery_enabled_ || !planningFailed())
    return false;
  for (std::size_t index = 0; index < recovery_steps_.size(); ++index)
  {
    if (!stowForBaseRecovery() ||
        !moveBaseDirect(recovery_steps_[index][0], recovery_steps_[index][1],
                        index + 1, recovery_steps_.size()))
      return false;
    if (prepareAndObserveOnce())
      return true;
    if (!planningFailed())
      return false;
  }
  return false;
}

bool NavigationSortingMission::sortWithRecovery()
{
  const std::vector<std::string> states = {
      "SORTING", "DETECTING", "PICKING", "OBSERVING", "HOMING"};
  if (callSortingOperation(sort_client_, states, "sorting"))
    return true;
  if (!base_recovery_enabled_ || !planningFailed())
    return false;
  for (std::size_t index = 0; index < recovery_steps_.size(); ++index)
  {
    if (!stowForBaseRecovery() ||
        !moveBaseDirect(recovery_steps_[index][0], recovery_steps_[index][1],
                        index + 1, recovery_steps_.size()))
      return false;
    if (!prepareAndObserveOnce())
    {
      if (planningFailed())
        continue;
      return false;
    }
    if (callSortingOperation(sort_client_, states, "sorting retry"))
      return true;
    if (!planningFailed())
      return false;
  }
  return false;
}

bool NavigationSortingMission::retreatAfterSorting(
    const XmlRpc::XmlRpcValue& workspace)
{
  publishState("STOWING_ARM", memberString(workspace, "id") +
               " before post-sort retreat");
  if (!callSortingOperation(home_client_, {"HOMING"},
                            "arm stow before post-sort retreat"))
    return false;
  if (!memberBool(workspace, "retreat_enabled", post_sort_retreat_enabled_))
    return true;
  const double distance = std::max(
      0.0, memberDouble(workspace, "retreat_distance",
                        post_sort_retreat_distance_));
  if (distance <= 1e-6)
    return true;
  std::string pose_frame = memberString(workspace, "navigation_goal_frame");
  if (pose_frame.empty())
    pose_frame = navigation_frame_;
  Pose2D current;
  if (!currentBasePose(current, pose_frame))
    return false;
  const Pose2D target = {current.x - distance * std::cos(current.yaw),
                         current.y - distance * std::sin(current.yaw),
                         current.yaw};
  return driveStraightTo(target, memberString(workspace, "id") +
                         " post-sort retreat", "RETREATING_BASE", pose_frame);
}

double NavigationSortingMission::scoreCandidate(const Pose2D& pose, bool& valid,
                                                double& clearance) const
{
  const double dx = std::max(std::abs(pose.x - table_geometry_[0]) -
                             0.5 * table_geometry_[2], 0.0);
  const double dy = std::max(std::abs(pose.y - table_geometry_[1]) -
                             0.5 * table_geometry_[3], 0.0);
  clearance = std::hypot(dx, dy);
  valid = clearance >= base_clearance_;
  double camera_error = 0.0;
  for (const auto& point : workpiece_points_)
  {
    const double world_x = point[0] - pose.x;
    const double world_y = point[1] - pose.y;
    const double local_x = std::cos(pose.yaw) * world_x +
                           std::sin(pose.yaw) * world_y;
    const double local_y = -std::sin(pose.yaw) * world_x +
                           std::cos(pose.yaw) * world_y;
    if (local_x < detector_workspace_[0] || local_x > detector_workspace_[1] ||
        local_y < detector_workspace_[2] || local_y > detector_workspace_[3])
      valid = false;
    camera_error += std::pow(local_x - camera_target_[0], 2) +
                    std::pow(local_y - camera_target_[1], 2);
  }
  return camera_error + 0.15 * std::hypot(pose.x - sorting_goal_.x,
                                          pose.y - sorting_goal_.y) +
         0.05 * std::abs(angleError(pose.yaw, sorting_goal_.yaw)) -
         0.02 * clearance;
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
                      canDirectDock(pre_dock_goal_, candidate.pose)))
          candidates.push_back(candidate);
      }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& left, const Candidate& right) {
              return left.score < right.score;
            });
  if (candidates.size() > static_cast<std::size_t>(near_field_max_candidates_))
    candidates.resize(near_field_max_candidates_);
  return candidates;
}

bool NavigationSortingMission::coordinateNearField()
{
  if (!navigate(pre_dock_goal_, "pre-dock"))
    return false;
  bool arm_prepared = false;
  const auto candidates = nearFieldCandidates();
  for (std::size_t index = 0; index < candidates.size(); ++index)
  {
    const auto& candidate = candidates[index].pose;
    bool docked = direct_dock_enabled_ ?
        driveStraightTo(candidate, "fine-dock candidate") :
        navigate(candidate, "fine-dock");
    if (!docked)
      continue;
    if (!arm_prepared)
    {
      if (!callSortingOperation(prepare_client_, {"PREPARING"},
                                "arm work preparation"))
        return false;
      arm_prepared = true;
    }
    if (callSortingOperation(observe_client_, {"OBSERVING"},
                             "camera observation"))
      return true;
    if (!callSortingOperation(home_client_, {"HOMING"}, "arm re-stowing"))
      return false;
    arm_prepared = false;
    if (direct_dock_enabled_ &&
        !driveStraightTo(pre_dock_goal_, "pre-dock retreat"))
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
    const auto& workspace = workstations_[enabled[order]];
    const std::string identifier = memberString(workspace, "id");
    publishState("CONFIGURING_WORKSTATION", identifier);
    if ((home_before_navigation_ || order > 0) &&
        !callSortingOperation(home_client_, {"HOMING"}, "arm homing"))
      return false;
    if (!configureWorkspace(workspace))
      return false;
    std::string goal_frame = memberString(workspace, "navigation_goal_frame");
    if (goal_frame.empty())
      goal_frame = navigation_frame_;
    const Pose2D goal = memberPose(workspace, "navigation_goal");
    if (workspace.hasMember("pre_dock_goal"))
    {
      const Pose2D pre_dock = memberPose(workspace, "pre_dock_goal");
      if (!navigate(pre_dock, "workstation '" + identifier + "' pre-dock",
                    goal_frame) ||
          !driveStraightTo(goal, "workstation '" + identifier + "' fine-dock",
                           "DIRECT_DOCKING", goal_frame))
        return false;
    }
    else if (!navigate(goal, "workstation '" + identifier + "'", goal_frame))
      return false;
    publishState("AT_WORKSTATION", identifier + "; validating arm reach");
    if (!prepareAndObserveWithRecovery())
      return false;
    publishState("SORTING", "workstation '" + identifier + "'");
    if (!sortWithRecovery() || !retreatAfterSorting(workspace))
      return false;
    publishState("WORKSTATION_COMPLETE", identifier);
  }
  return true;
}

void NavigationSortingMission::runMission()
{
  bool success = false;
  try
  {
    if (!waitForSortingReady())
      throw std::runtime_error("sorting node is not ready");
    if (workstations_.size() > 0)
    {
      success = runWorkstationSequence();
    }
    else
    {
      if (home_before_navigation_ &&
          !callSortingOperation(home_client_, {"HOMING"}, "arm homing"))
        throw std::runtime_error("arm homing failed");
      if (near_field_enabled_)
      {
        if (!coordinateNearField())
          throw std::runtime_error("near-field coordination failed");
      }
      else
      {
        if (!navigate(sorting_goal_, "workstation") ||
            !prepareAndObserveWithRecovery())
          throw std::runtime_error("workstation approach failed");
      }
      publishState("AT_WORKSTATION", "dock and camera view validated");
      publishState("SORTING", "sorting detected objects");
      success = sortWithRecovery();
    }
  }
  catch (const std::exception& error)
  {
    ROS_ERROR_STREAM("Navigation-sorting mission failed: " << error.what());
  }
  stopBase();
  busy_ = false;
  if (stop_requested_)
    publishState("STOPPED", "mission cancelled");
  else if (success)
    publishState("SUCCEEDED", "navigation and sorting complete");
  else
    publishState("FAILED", "inspect move_base and /sorting/state");
}

void NavigationSortingMission::publishState(const std::string& state,
                                            const std::string& detail)
{
  std_msgs::String message;
  message.data = detail.empty() ? state : state + " | " + detail;
  state_publisher_.publish(message);
  ROS_INFO_STREAM("Navigation-sorting mission: " << message.data);
}

std::string NavigationSortingMission::stateName(const std::string& state)
{
  const auto delimiter = state.find('|');
  std::string result = state.substr(0, delimiter);
  const auto first = result.find_first_not_of(" \t");
  const auto last = result.find_last_not_of(" \t");
  return first == std::string::npos ? std::string() :
      result.substr(first, last - first + 1);
}

double NavigationSortingMission::angleError(double target, double actual)
{
  return std::atan2(std::sin(target - actual), std::cos(target - actual));
}

double NavigationSortingMission::number(const XmlRpc::XmlRpcValue& value)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    return static_cast<int>(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    return static_cast<double>(value);
  throw std::runtime_error("expected a numeric XML-RPC value");
}

bool NavigationSortingMission::memberBool(const XmlRpc::XmlRpcValue& value,
                                         const std::string& key, bool fallback)
{
  if (!value.hasMember(key))
    return fallback;
  if (value[key].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    throw std::runtime_error(key + " must be boolean");
  return static_cast<bool>(value[key]);
}

double NavigationSortingMission::memberDouble(const XmlRpc::XmlRpcValue& value,
                                              const std::string& key,
                                              double fallback)
{
  return value.hasMember(key) ? number(value[key]) : fallback;
}

std::string NavigationSortingMission::memberString(
    const XmlRpc::XmlRpcValue& value, const std::string& key)
{
  if (!value.hasMember(key) ||
      value[key].getType() != XmlRpc::XmlRpcValue::TypeString)
    return std::string();
  return static_cast<std::string>(value[key]);
}

NavigationSortingMission::Pose2D NavigationSortingMission::memberPose(
    const XmlRpc::XmlRpcValue& value, const std::string& key)
{
  if (!value.hasMember(key) ||
      value[key].getType() != XmlRpc::XmlRpcValue::TypeArray ||
      value[key].size() != 3)
    throw std::runtime_error(key + " must be [x, y, yaw]");
  return {number(value[key][0]), number(value[key][1]), number(value[key][2])};
}

std::string NavigationSortingMission::toJson(const XmlRpc::XmlRpcValue& value)
{
  std::ostringstream output;
  switch (value.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      output << (static_cast<bool>(value) ? "true" : "false");
      break;
    case XmlRpc::XmlRpcValue::TypeInt:
      output << static_cast<int>(value);
      break;
    case XmlRpc::XmlRpcValue::TypeDouble:
      output << std::setprecision(16) << static_cast<double>(value);
      break;
    case XmlRpc::XmlRpcValue::TypeString:
      output << '"' << jsonEscape(static_cast<std::string>(value)) << '"';
      break;
    case XmlRpc::XmlRpcValue::TypeArray:
      output << '[';
      for (int index = 0; index < value.size(); ++index)
      {
        if (index)
          output << ',';
        output << toJson(value[index]);
      }
      output << ']';
      break;
    case XmlRpc::XmlRpcValue::TypeStruct:
    {
      output << '{';
      bool first = true;
      for (auto iterator = value.begin(); iterator != value.end(); ++iterator)
      {
        if (!first)
          output << ',';
        first = false;
        output << '"' << jsonEscape(iterator->first) << "\":"
               << toJson(iterator->second);
      }
      output << '}';
      break;
    }
    default:
      output << "null";
      break;
  }
  return output.str();
}

}  // namespace aubo_mobile_nav_sorting

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_sorting_mission");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");
  try
  {
    aubo_mobile_nav_sorting::NavigationSortingMission mission(
        node_handle, private_node_handle);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM("Could not start C++ navigation-sorting mission: "
                     << error.what());
    return 1;
  }
  return 0;
}
