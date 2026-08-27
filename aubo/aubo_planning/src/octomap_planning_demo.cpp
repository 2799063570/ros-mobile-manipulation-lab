#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <sensor_msgs/PointCloud2.h>

#include <atomic>
#include <cstddef>
#include <string>

namespace
{
std::atomic<bool> cloud_received(false);
std::atomic<bool> octomap_received(false);
std::atomic<std::size_t> cloud_points(0);

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message)
{
  cloud_points.store(static_cast<std::size_t>(message->width) * message->height);
  cloud_received.store(cloud_points.load() > 0);
}

void planningSceneCallback(const moveit_msgs::PlanningSceneConstPtr& message)
{
  if (!message->world.octomap.octomap.data.empty())
    octomap_received.store(true);
}

bool waitFor(const std::atomic<bool>& condition, double timeout, const std::string& description)
{
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout);
  ros::WallRate rate(20.0);
  while (ros::ok() && !condition.load() && ros::WallTime::now() < deadline)
    rate.sleep();

  if (!condition.load())
  {
    ROS_ERROR_STREAM("Timed out waiting for " << description << " after " << timeout << " seconds.");
    return false;
  }
  return true;
}

bool moveToNamedTarget(moveit::planning_interface::MoveGroupInterface& arm,
                       const std::string& target,
                       int attempts)
{
  for (int attempt = 1; ros::ok() && attempt <= attempts; ++attempt)
  {
    arm.setStartStateToCurrentState();
    if (!arm.setNamedTarget(target))
    {
      ROS_ERROR_STREAM("Unknown SRDF named pose: " << target);
      return false;
    }

    ROS_INFO_STREAM("Moving to camera observation pose '" << target << "' (attempt "
                                                            << attempt << '/' << attempts << ").");
    const moveit::planning_interface::MoveItErrorCode result = arm.move();
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      return true;

    ROS_WARN_STREAM("Observation move stopped with MoveIt code " << result.val
                    << ". The depth camera may have revealed new geometry; "
                       "waiting for the OctoMap update before replanning.");
    ros::WallDuration(1.0).sleep();
  }
  return false;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aubo_octomap_planning_demo");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string group_name;
  std::string sensor_pose;
  std::string target_pose;
  std::string point_cloud_topic;
  std::string planning_scene_topic;
  double wait_timeout;
  double planning_time;
  bool move_to_sensor_pose;
  bool execute;
  int sensor_pose_attempts;
  private_nh.param<std::string>("group_name", group_name, "aubo_i5");
  private_nh.param<std::string>("sensor_pose", sensor_pose, "observe");
  private_nh.param<std::string>("target_pose", target_pose, "home");
  private_nh.param<std::string>("point_cloud_topic", point_cloud_topic,
                                "/camera/depth/color/points");
  private_nh.param<std::string>("planning_scene_topic", planning_scene_topic,
                                "/move_group/monitored_planning_scene");
  private_nh.param("wait_timeout", wait_timeout, 30.0);
  private_nh.param("planning_time", planning_time, 15.0);
  private_nh.param("move_to_sensor_pose", move_to_sensor_pose, true);
  private_nh.param("execute", execute, false);
  private_nh.param("sensor_pose_attempts", sensor_pose_attempts, 3);

  const ros::Subscriber cloud_subscriber =
      nh.subscribe(point_cloud_topic, 1, cloudCallback);
  const ros::Subscriber scene_subscriber =
      nh.subscribe(planning_scene_topic, 10, planningSceneCallback);
  ros::Publisher display_publisher =
      nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  (void)cloud_subscriber;
  (void)scene_subscriber;

  moveit::planning_interface::MoveGroupInterface arm(group_name);
  arm.setPlanningTime(planning_time);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.15);
  arm.setMaxAccelerationScalingFactor(0.15);

  if (!arm.getCurrentState(wait_timeout))
  {
    ROS_ERROR("No current robot state was received. Check /joint_states.");
    return 1;
  }

  if (move_to_sensor_pose)
  {
    if (!moveToNamedTarget(arm, sensor_pose, sensor_pose_attempts))
    {
      ROS_ERROR_STREAM("Could not reach camera observation pose after "
                       << sensor_pose_attempts << " attempts.");
      return 3;
    }
  }

  ROS_INFO_STREAM("Waiting for depth cloud on " << point_cloud_topic << '.');
  if (!waitFor(cloud_received, wait_timeout, "a non-empty depth point cloud"))
    return 4;
  ROS_INFO_STREAM("Depth camera is publishing " << cloud_points.load() << " points per cloud.");

  ROS_INFO("Waiting for MoveIt to insert the cloud into its OctoMap.");
  if (!waitFor(octomap_received, wait_timeout, "a non-empty MoveIt OctoMap"))
  {
    ROS_ERROR("Check use_sensor_manager:=true and the TF from the cloud frame to base_link.");
    return 5;
  }

  arm.setStartStateToCurrentState();
  if (!arm.setNamedTarget(target_pose))
  {
    ROS_ERROR_STREAM("Unknown SRDF named pose: " << target_pose);
    return 6;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ROS_INFO_STREAM("Planning to '" << target_pose << "' with the live OctoMap enabled.");
  const moveit::planning_interface::MoveItErrorCode plan_result = arm.plan(plan);
  if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR_STREAM("OctoMap-aware planning failed; MoveIt code " << plan_result.val << '.');
    return 7;
  }

  moveit_msgs::DisplayTrajectory display;
  display.model_id = arm.getRobotModel()->getName();
  display.trajectory_start = plan.start_state_;
  display.trajectory.push_back(plan.trajectory_);
  display_publisher.publish(display);
  ROS_INFO_STREAM("Plan found with " << plan.trajectory_.joint_trajectory.points.size()
                                     << " trajectory points.");

  if (execute)
  {
    ROS_WARN("Executing the OctoMap-aware trajectory.");
    const moveit::planning_interface::MoveItErrorCode execute_result = arm.execute(plan);
    if (execute_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Trajectory execution failed; MoveIt code " << execute_result.val << '.');
      return 8;
    }
  }
  else
  {
    ROS_INFO("Planning-only mode: inspect the trajectory in RViz; no motion was commanded.");
  }

  ros::shutdown();
  return 0;
}
