#include <aubo_sorting_core/color_sorting_task.hpp>
#include "task_utils.hpp"
#include <aubo_sorting_core/height_recovery.hpp>
#include <actionlib_msgs/GoalStatus.h>
#include <inspire_gripper/move_max.h>
#include <inspire_gripper/move_min.h>
#include <inspire_gripper/set_es.h>
#include <inspire_gripper/get_state.h>
#include <control_msgs/JointTolerance.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <algorithm>
#include <cmath>
#include <iterator>

namespace aubo_sorting_core
{
using detail::wallSleep;

geometry_msgs::PoseStamped ColorSortingTask::makePose(double x, double y, double z) const
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = target_frame_;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  tf::Quaternion quaternion;
  quaternion.setRPY(grasp_rpy_[0], grasp_rpy_[1], grasp_rpy_[2] + active_grasp_angle_);
  tf::quaternionTFToMsg(quaternion, pose.pose.orientation);
  return pose;
}

bool ColorSortingTask::xyInTargetFrame(const std::string& source_frame,
                                       const std::vector<double>& xy, double& x, double& y)
{
  if (source_frame == target_frame_)
  {
    x = xy[0];
    y = xy[1];
    return true;
  }
  geometry_msgs::PoseStamped source;
  source.header.frame_id = source_frame;
  source.header.stamp = ros::Time(0);
  source.pose.position.x = xy[0];
  source.pose.position.y = xy[1];
  source.pose.orientation.w = 1.0;
  try
  {
    tf_listener_.waitForTransform(target_frame_, source_frame, ros::Time(0),
                                  ros::Duration(scene_update_timeout_));
    geometry_msgs::PoseStamped target;
    tf_listener_.transformPose(target_frame_, source, target);
    x = target.pose.position.x;
    y = target.pose.position.y;
    return true;
  }
  catch (const tf::TransformException& error)
  {
    ROS_ERROR("Cannot transform place target from %s to %s: %s", source_frame.c_str(),
              target_frame_.c_str(), error.what());
    return false;
  }
}

bool ColorSortingTask::moveToPose(const geometry_msgs::PoseStamped& pose,
                                  const std::string& description,
                                  bool constrain_pregrasp_wrist1)
{
  if (stop_requested_.load())
    return false;
  const bool constrain_wrist1 = constrain_pregrasp_wrist1 && pregrasp_wrist1_limit_ > 0.0;
  if (constrain_wrist1)
  {
    // 同一 TCP 位姿存在 wrist1≈0 和 wrist1≈±π 两组逆解；只在预抓取阶段排除翻腕分支。
    // 观察位也必须位于这个范围内，否则 MoveIt 的路径约束会使规划失败。
    moveit_msgs::Constraints constraints;
    moveit_msgs::JointConstraint wrist;
    wrist.joint_name = "wrist1_joint";
    wrist.position = 0.0;
    wrist.tolerance_above = pregrasp_wrist1_limit_;
    wrist.tolerance_below = pregrasp_wrist1_limit_;
    wrist.weight = 1.0;
    constraints.joint_constraints.push_back(wrist);
    arm_->setPathConstraints(constraints);
  }
  const auto clear_constraints = [this](void*) { arm_->clearPathConstraints(); };
  std::unique_ptr<void, decltype(clear_constraints)> constraint_guard(
      constrain_wrist1 ? this : nullptr, clear_constraints);
  ROS_INFO_STREAM("Planning arm to " << description << " in " << pose.header.frame_id
                  << " at [" << pose.pose.position.x << ", " << pose.pose.position.y
                  << ", " << pose.pose.position.z << "]");
  arm_->setPoseTarget(pose, end_effector_link_);// 设置目标姿态(末端执行器的笛卡尔空间位姿)
  return planAndExecute(description, constrain_wrist1 ? pregrasp_wrist1_limit_ : 0.0) &&
      !stop_requested_.load();
}

bool ColorSortingTask::planAndExecute(const std::string& description, double wrist1_limit)
{
  arm_->setStartStateToCurrentState();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto planned = arm_->plan(plan);
  if (planned != moveit::planning_interface::MoveItErrorCode::SUCCESS ||
      plan.trajectory_.joint_trajectory.points.empty())
  {
    arm_->stop();
    arm_->clearPoseTargets();
    setFailure("PLANNING_FAILED", description);
    return false;
  }
  if (wrist1_limit > 0.0)
  {
    // 路径约束交给 MoveIt 求解，执行前仍逐点检查，避免错误轨迹驱动翻腕。
    const auto& trajectory = plan.trajectory_.joint_trajectory;
    const auto found = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), "wrist1_joint");
    const std::size_t index = static_cast<std::size_t>(std::distance(trajectory.joint_names.begin(), found));
    bool within_limit = found != trajectory.joint_names.end();
    for (const auto& point : trajectory.points)
      within_limit = within_limit && index < point.positions.size() &&
          std::isfinite(point.positions[index]) && std::abs(point.positions[index]) <= wrist1_limit + 1e-3;
    if (!within_limit)
    {
      arm_->stop();
      arm_->clearPoseTargets();
      setFailure("PLANNING_FAILED", description + " violates pre-grasp wrist1 limit");
      return false;
    }
  }
  const auto executed = arm_->execute(plan);
  arm_->stop();
  arm_->clearPoseTargets();
  const bool success = executed == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  if (!success)
    setFailure("EXECUTION_FAILED", description);
  return success;
}

bool ColorSortingTask::moveNamed(const std::string& target)
{
  // 先判断这个目标是否已经在命名目标中
  // 判断当前关节值是否已经满足阈值
  // 设置目标为当前命名位置 
  if (target.empty())
    return true;
  if (stop_requested_.load())
    return false;
  const std::vector<std::string> names = arm_->getNamedTargets();// 获取所有命名目标
  if (std::find(names.begin(), names.end(), target) == names.end()) // 验证target是否在命名目标中
  {
    setFailure("CONFIGURATION_FAILED", "unknown named target '" + target + "'");
    return false;
  }
  const std::map<std::string, double> target_values = arm_->getNamedTargetValues(target);
  const std::vector<std::string> active_joints = arm_->getActiveJoints();
  const std::vector<double> current = arm_->getCurrentJointValues();
  std::map<std::string, double> current_values;// 当前关节值映射
  for (std::size_t index = 0; index < active_joints.size() && index < current.size(); ++index)
    current_values[active_joints[index]] = current[index];
  bool already_there = !target_values.empty();
  for (const auto& value : target_values)
  {
    const auto found = current_values.find(value.first);// 在当前关节值中查找目标关节
    if (found == current_values.end() || std::abs(found->second - value.second) > 1.0e-3) // 判断误差
    {
      already_there = false;
      break;
    }
  }
  if (already_there)
  {
    ROS_INFO_STREAM("Arm is already at named target " << target);
    return true;
  }
  ROS_INFO_STREAM("Moving arm to named target " << target);
  if (!arm_->setNamedTarget(target))
  {
    setFailure("CONFIGURATION_FAILED", "cannot select SRDF target '" + target + "'");
    return false;
  }
  return planAndExecute("named target '" + target + "'") && !stop_requested_.load();
}

bool ColorSortingTask::cartesianTo(const geometry_msgs::PoseStamped& target_pose,
                                   const std::string& description, bool require_complete)
{
  if (stop_requested_.load())
    return false;
  const double required_fraction = require_complete ? 1.0 : minimum_cartesian_fraction_;
  std::vector<geometry_msgs::Pose> waypoints(1, target_pose.pose);
  moveit_msgs::RobotTrajectory trajectory;
  auto compute_path = [this, &waypoints, &trajectory]() {
    arm_->setStartStateToCurrentState();
    return arm_->computeCartesianPath(waypoints, cartesian_step_, 0.0, trajectory, true);// 计算笛卡尔路径
  };
  double fraction = compute_path();// 返回计算的路径的比例
  ROS_INFO("Cartesian path to %s: %.1f%%", description.c_str(), 100.0 * fraction);
  if (fraction < required_fraction && require_octomap_)// 如果路径比例小于最小值 且使用了八叉树地图
  {
    ROS_WARN("Cartesian fraction too low; refreshing OctoMap and retrying once");
    if (refreshOctomap() && !stop_requested_.load())// 刷新八叉树 并重新规划
    {
      fraction = compute_path();
      ROS_INFO("Cartesian path retry to %s: %.1f%%", description.c_str(), 100.0 * fraction);
    }
  }
  if (!std::isfinite(fraction) || (require_complete && fraction < 1.0))
  {
    setFailure("PLANNING_FAILED", "incomplete Cartesian lift to " + description);
    return false;  // Never execute a partial lift and mistake it for reaching the lower bound.
  }
  if (fraction < required_fraction)
  {
    ROS_WARN("Cartesian fraction too low; falling back to pose planning");
    return moveToPose(target_pose, description);// 回退到关节空间路径规划
  }
  // 求解出了笛卡尔路径后，使用IterativeParabolicTimeParameterization对路径进行时间参数化
  robot_trajectory::RobotTrajectory robot_trajectory(arm_->getRobotModel(), group_name_);
  robot_trajectory.setRobotTrajectoryMsg(*arm_->getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization parameterization;
  if (!parameterization.computeTimeStamps(robot_trajectory, velocity_scaling_, acceleration_scaling_))
  {
    setFailure("PLANNING_FAILED", "cannot retime Cartesian path to " + description);
    return false;
  }
  robot_trajectory.getRobotTrajectoryMsg(trajectory);
  if (trajectory.joint_trajectory.points.empty())
  {
    setFailure("PLANNING_FAILED", "empty Cartesian path to " + description);
    return false;
  }
  ROS_INFO("Retimed Cartesian path to %s: %.2f s at velocity scale %.2f",
           description.c_str(), trajectory.joint_trajectory.points.back().time_from_start.toSec(),
           velocity_scaling_);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  if (stop_requested_.load() || !ros::ok())
    return false;
  const bool success = arm_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  arm_->stop();
  if (!success)
    setFailure("EXECUTION_FAILED", "Cartesian path to " + description);
  return success && !stop_requested_.load();
}

bool ColorSortingTask::liftWithRecovery(double x, double y, const std::string& description)
{
  const auto heights = liftHeightCandidates(lift_height_, lift_min_height_,
                                            lift_height_step_, lift_max_attempts_);
  std::size_t attempt = 0;
  const auto result = runHeightRecovery(heights, [&](double height) {
    ++attempt;
    ROS_INFO("Lift attempt %zu/%zu: height=%.3f m above table, target_z=%.3f m, minimum=%.3f m",
             attempt, heights.size(), height, table_z_ + height, lift_min_height_);
    // Clear the previous attempt's failure before classifying this result.
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_failure_.clear();
    }
    if (cartesianTo(makePose(x, y, table_z_ + height), description, true))
    {
      std_msgs::String clear;
      failure_publisher_.publish(clear);
      return HeightAttemptResult::Succeeded;
    }
    if (stop_requested_.load())
      return HeightAttemptResult::Cancelled;
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_failure_.find("PLANNING_FAILED |") == 0 ?
        HeightAttemptResult::PlanningFailed : HeightAttemptResult::ExecutionFailed;
  }, [this] { return stop_requested_.load() || !ros::ok(); });
  if (result == HeightAttemptResult::PlanningFailed)
    setFailure("PLANNING_FAILED", description + " exhausted bounded height attempts");
  return result == HeightAttemptResult::Succeeded;
}

bool ColorSortingTask::commandGripper(double position)
{
  // 通过action通信请求夹爪的控制器执行目标位置
  if (stop_requested_.load())
    return false;
  if (gripper_backend_ == "inspire")
  {
    bool accepted = false;
    if (std::abs(position - gripper_open_) < 1e-6)
    {
      inspire_gripper::move_max request;
      request.request.speed = inspire_speed_;
      accepted = inspire_open_client_.call(request) && request.response.movemax_accepted;
    }
    else
    {
      inspire_gripper::move_min request;
      request.request.speed = inspire_speed_;
      request.request.power = inspire_force_;
      accepted = inspire_close_client_.call(request) && request.response.movemin_accepted;
    }
    if (!accepted)
    {
      setFailure("GRIPPER_FAILED", "Inspire command rejected");
      return false;
    }
    const bool opening = std::abs(position - gripper_open_) < 1e-6;
    const auto deadline = ros::WallTime::now() + ros::WallDuration(inspire_motion_timeout_);
    while (wallSleep(0.1, stop_requested_) && ros::WallTime::now() < deadline)
    {
      inspire_gripper::get_state status;
      if (!inspire_state_client_.call(status) || status.response.error_code != 0)
        break;
      const int motion = status.response.motion_state;
      if (opening ? motion == 1 : (motion == 2 || motion == 6))
        return !stop_requested_.load();
    }
    inspire_gripper::set_es stop;
    inspire_stop_client_.call(stop);
    setFailure("GRIPPER_FAILED", "Inspire motion failed or timed out");
    return false;
  }
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = {"joint1", "joint2"};
  const auto current = arm_->getCurrentState(2.0);
  if (!current)
    return false;
  trajectory_msgs::JointTrajectoryPoint initial;
  for (const auto& joint : goal.trajectory.joint_names)
    initial.positions.push_back(current->getVariablePosition(joint));
  initial.velocities = {0.0, 0.0};
  initial.time_from_start = ros::Duration(0.0);
  goal.trajectory.points.push_back(initial);
  trajectory_msgs::JointTrajectoryPoint point;// 夹爪关节轨迹点 最后一个点
  point.positions = {position, position};
  point.velocities = {0.0, 0.0};
  point.time_from_start = ros::Duration(gripper_motion_time_);// 夹爪执行时间
  goal.trajectory.points.push_back(point);
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
  for (const std::string& joint_name : goal.trajectory.joint_names)
  {
    control_msgs::JointTolerance path;
    path.name = joint_name;
    path.position = 0.10;
    goal.path_tolerance.push_back(path);// 夹爪关节轨迹容差
    control_msgs::JointTolerance target;
    target.name = joint_name;
    target.position = 0.03;
    goal.goal_tolerance.push_back(target);// 夹爪关节目标容差
  }
  goal.goal_time_tolerance = ros::Duration(3.0);
  gripper_client_->sendGoal(goal);// 通过actionlib客户端发送夹爪目标轨迹
  if (!gripper_client_->waitForResult(ros::Duration(gripper_motion_time_ + 3.0)))
  {
    gripper_client_->cancelGoal();
    ROS_ERROR("Gripper command timed out");
    return false;
  }
  const actionlib::SimpleClientGoalState state = gripper_client_->getState();
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    const auto result = gripper_client_->getResult();
    ROS_ERROR("Gripper action failed: state=%s, error_code=%d, error_string='%s'",
              state.toString().c_str(), result ? result->error_code : std::numeric_limits<int>::min(),
              result ? result->error_string.c_str() : "");
    return false;
  }
  return !stop_requested_.load();
}

}  // namespace aubo_sorting_core
