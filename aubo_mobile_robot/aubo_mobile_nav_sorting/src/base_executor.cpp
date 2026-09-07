#include <aubo_mobile_nav_sorting/base_executor.h>
#include <aubo_mobile_nav_sorting/bounded_rpc.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
namespace aubo_mobile_nav_sorting
{
BaseExecutor::BaseExecutor(MissionContext &context) : context_(context)
{
  navigation_client_.reset(new MoveBaseClient(context_.navigation_action_, true));
  velocity_publisher_ = context_.node_handle_.advertise<geometry_msgs::Twist>(
      context_.velocity_topic_, 2); // 发布导航拣选速度 用于近场精靠
  clear_costmaps_client_ =
      context_.node_handle_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
}

bool BaseExecutor::navigateOnce(const Pose2D &target, const std::string &goal_frame)
{
  if (context_.stop_requested_ || context_.base_pose_failed_)
    return false;
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = goal_frame;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = target.x;
  goal.target_pose.pose.position.y = target.y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target.yaw);
  if (!sendGoal(goal))
    return false;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(context_.navigation_timeout_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    std::unique_lock<std::mutex> state_lock(context_.mutex_);
    if (context_.stop_requested_ || context_.base_locked_)
    {
      cancelNavigation();
      return false;
    }
    state_lock.unlock();
    if (navigationState().isDone())
      return navigationState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    ros::WallDuration(0.05).sleep();
  }
  cancelNavigation();
  return false;
}

bool BaseExecutor::navigate(const Pose2D &target, const std::string &stage,
                            const std::string &requested_frame)
{
  const std::string goal_frame =
      requested_frame.empty() ? context_.navigation_frame_ : requested_frame;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(context_.server_timeout_);
  while (ros::ok() && !context_.stop_requested_ && !serverConnected() &&
         std::chrono::steady_clock::now() < deadline)
    ros::WallDuration(0.05).sleep();
  if (!ros::ok() || context_.stop_requested_ || !serverConnected())
  {
    ROS_ERROR_STREAM("Navigation action unavailable: " << context_.navigation_action_);
    return false;
  }
  for (int attempt = 0; attempt <= context_.navigation_retries_; ++attempt)
  {
    std::ostringstream detail;
    detail << stage << " attempt " << attempt + 1 << "/" << context_.navigation_retries_ + 1
           << " to [" << std::fixed << std::setprecision(2) << target.x << ", " << target.y << ", "
           << target.yaw << "] in " << goal_frame;
    context_.publishState(MissionState::NAVIGATING, detail.str());
    if (navigateOnce(target, goal_frame))
      return true;
    if (context_.stop_requested_)
      return false;
    if (attempt < context_.navigation_retries_)
    {
      std_srvs::Empty clear;
      const auto result = boundedRpc(
          clear_costmaps_client_, clear, context_.server_timeout_,
          [this]() { return context_.stop_requested_.load(); }, {}, context_.pending_rpcs_);
      if (result != RpcStatus::Completed)
      {
        if (result == RpcStatus::Abandoned)
          context_.stop_unconfirmed_ = true;
        ROS_WARN("Could not clear costmaps before navigation retry");
        return false;
      }
    }
  }
  return false;
}

bool BaseExecutor::currentBasePose(Pose2D &pose, const std::string &requested_frame)
{
  const std::string pose_frame =
      requested_frame.empty() ? context_.navigation_frame_ : requested_frame;
  try
  {
    tf::StampedTransform transform;
    tf_listener_.lookupTransform(pose_frame, context_.base_frame_, ros::Time(0), transform);
    const double age = (ros::Time::now() - transform.stamp_).toSec();
    if (transform.stamp_.isZero() || age < -0.05 || age > context_.tf_max_age_)
    {
      context_.base_pose_failed_ = true;
      ROS_ERROR_STREAM("Base pose is stale or future-dated (age " << age << " s)");
      stopBase();
      return false;
    }
    pose = {transform.getOrigin().x(), transform.getOrigin().y(),
            tf::getYaw(transform.getRotation())};
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw))
    {
      context_.base_pose_failed_ = true;
      stopBase();
      ROS_ERROR("Base pose contains non-finite coordinates");
      return false;
    }
    return true;
  }
  catch (const tf::TransformException &error)
  {
    context_.base_pose_failed_ = true;
    stopBase();
    ROS_WARN_STREAM("Cannot read base pose: " << error.what());
    return false;
  }
}

bool BaseExecutor::alignHeading(double target_yaw, const std::string &label,
                                const std::string &pose_frame)
{
  if (context_.stop_requested_ || context_.base_pose_failed_)
  {
    stopBase();
    return false;
  }
  Pose2D actual;
  if (!currentBasePose(actual, pose_frame))
    return false;
  const double initial_error = context_.angleError(target_yaw, actual.yaw);
  if (std::abs(initial_error) <= context_.heading_goal_tolerance_)
    return true;
  if (!context_.heading_alignment_enabled_ ||
      std::abs(initial_error) > context_.heading_max_correction_)
    return false;
  cancelNavigation();
  stopBase();
  ros::WallDuration(0.2).sleep();
  context_.publishState(MissionState::ALIGNING_BASE, label + " bounded heading correction");
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(context_.heading_timeout_);
  auto last_progress = std::chrono::steady_clock::now();
  double best_error = std::abs(initial_error);
  ros::WallRate rate(context_.base_recovery_rate_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (context_.stop_requested_ || !currentBasePose(actual, pose_frame))
    {
      stopBase();
      return false;
    }
    const double error = context_.angleError(target_yaw, actual.yaw);
    const double absolute_error = std::abs(error);
    if (absolute_error <= context_.heading_goal_tolerance_)
    {
      stopBase();
      return true;
    }
    if (absolute_error < best_error - 0.002)
    {
      best_error = absolute_error;
      last_progress = std::chrono::steady_clock::now();
    }
    else if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_progress)
                 .count() > context_.heading_stall_timeout_)
    {
      stopBase();
      return false;
    }
    geometry_msgs::Twist command;
    command.angular.z =
        std::copysign(std::min(context_.heading_speed_, std::max(0.04, absolute_error)), error);
    if (!publishVelocity(command))
      return false;
    rate.sleep();
  }
  stopBase();
  return false;
}

bool BaseExecutor::canDirectDock(const Pose2D &start, const Pose2D &target) const
{
  const double dx = target.x - start.x;
  const double dy = target.y - start.y;
  const double longitudinal = std::cos(start.yaw) * dx + std::sin(start.yaw) * dy;
  const double lateral = -std::sin(start.yaw) * dx + std::cos(start.yaw) * dy;
  return std::abs(longitudinal) <= context_.direct_dock_max_distance_ &&
         std::abs(lateral) <= context_.direct_dock_lateral_tolerance_ &&
         std::abs(context_.angleError(target.yaw, start.yaw)) <= context_.heading_max_correction_;
}

bool BaseExecutor::driveStraightTo(const Pose2D &target, const std::string &label,
                                   MissionState state, const std::string &pose_frame)
{
  if (context_.stop_requested_ || context_.base_pose_failed_)
  {
    stopBase();
    return false;
  }
  Pose2D start;
  if (!currentBasePose(start, pose_frame) || !canDirectDock(start, target) ||
      !alignHeading(target.yaw, label, pose_frame) || !currentBasePose(start, pose_frame))
    return false;
  const auto longitudinalError = [&](const Pose2D &pose) {
    return std::cos(pose.yaw) * (target.x - pose.x) + std::sin(pose.yaw) * (target.y - pose.y);
  };
  double longitudinal = longitudinalError(start);
  context_.publishState(state, label + " closed-loop cmd_vel motion");
  cancelNavigation();
  stopBase();
  ros::WallDuration(0.2).sleep();
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(context_.direct_dock_timeout_);
  auto last_progress = std::chrono::steady_clock::now();
  double best_error = std::abs(longitudinal);
  ros::WallRate rate(context_.base_recovery_rate_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    Pose2D actual;
    if (context_.stop_requested_ || !currentBasePose(actual, pose_frame))
    {
      stopBase();
      return false;
    }
    longitudinal = longitudinalError(actual);
    const double dx = target.x - actual.x;
    const double dy = target.y - actual.y;
    const double lateral = -std::sin(actual.yaw) * dx + std::cos(actual.yaw) * dy;
    const double yaw_error = context_.angleError(target.yaw, actual.yaw);
    if (std::hypot(dx, dy) <= context_.direct_dock_goal_tolerance_ &&
        std::abs(yaw_error) <= context_.heading_final_tolerance_)
    {
      stopBase();
      return true;
    }
    if (std::abs(lateral) > context_.direct_dock_lateral_tolerance_ ||
        std::abs(yaw_error) > context_.direct_dock_yaw_tolerance_)
    {
      stopBase();
      return false;
    }
    const double current_error = std::abs(longitudinal);
    if (current_error <= best_error - context_.direct_dock_progress_epsilon_)
    {
      best_error = current_error;
      last_progress = std::chrono::steady_clock::now();
    }
    else if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_progress)
                 .count() > context_.direct_dock_stall_timeout_)
    {
      stopBase();
      return false;
    }
    geometry_msgs::Twist command;
    command.linear.x = std::copysign(
        std::min(context_.base_recovery_speed_, std::max(0.02, current_error)), longitudinal);
    if (!publishVelocity(command))
      return false;
    rate.sleep();
  }
  stopBase();
  return false;
}

bool BaseExecutor::moveBaseDirect(double dx, double dy, int attempt, int total)
{
  if (context_.stop_requested_ || context_.base_pose_failed_)
    return false;
  const double distance = std::max(std::abs(dx), std::abs(dy));
  if (distance <= 1e-6)
    return true;
  cancelNavigation();
  std::ostringstream detail;
  detail << "cmd_vel step " << attempt << "/" << total << " dx=" << dx << " dy=" << dy;
  context_.publishState(MissionState::ADJUSTING_BASE, detail.str());
  geometry_msgs::Twist command;
  command.linear.x = std::abs(dx) > 1e-6 ? std::copysign(context_.base_recovery_speed_, dx) : 0.0;
  command.linear.y = std::abs(dy) > 1e-6 ? std::copysign(context_.base_recovery_speed_, dy) : 0.0;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(distance / context_.base_recovery_speed_);
  ros::WallRate rate(context_.base_recovery_rate_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (context_.stop_requested_)
    {
      stopBase();
      return false;
    }
    if (!publishVelocity(command))
      return false;
    rate.sleep();
  }
  stopBase();
  ros::WallDuration(context_.base_recovery_settle_time_).sleep();
  return !context_.stop_requested_;
}

bool BaseExecutor::sendGoal(const move_base_msgs::MoveBaseGoal &goal)
{
  std::lock_guard<std::mutex> state_lock(context_.mutex_);
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (context_.stop_requested_ || context_.stop_unconfirmed_ || context_.base_locked_)
    return false;
  navigation_client_->sendGoal(goal);
  return true;
}

actionlib::SimpleClientGoalState BaseExecutor::navigationState()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return navigation_client_->getState();
}

bool BaseExecutor::serverConnected()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return navigation_client_->isServerConnected();
}

void BaseExecutor::cancelNavigation()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  navigation_client_->cancelAllGoals();
}

bool BaseExecutor::publishVelocity(const geometry_msgs::Twist &command)
{
  std::lock_guard<std::mutex> state_lock(context_.mutex_);
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (context_.stop_requested_ || context_.stop_unconfirmed_ || context_.base_locked_)
  {
    velocity_publisher_.publish(geometry_msgs::Twist());
    return false;
  }
  velocity_publisher_.publish(command);
  return true;
}

void BaseExecutor::stopBase()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (velocity_publisher_)
    velocity_publisher_.publish(geometry_msgs::Twist());
}

} // namespace aubo_mobile_nav_sorting
