#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <cmath>
#include <string>
#include <vector>

namespace
{
double& coordinate(geometry_msgs::Pose& pose, char axis)
{
  if (axis == 'x')
    return pose.position.x;
  if (axis == 'y')
    return pose.position.y;
  return pose.position.z;
}

bool validParameters(const std::string& shape, const std::string& plane, double radius,
                     int samples, double eef_step, double velocity, double acceleration,
                     double max_joint_step, double dx, double dy, double dz)
{
  if (shape != "circle" && shape != "line")
    return false;
  if (plane != "xy" && plane != "xz" && plane != "yz")
    return false;
  return std::isfinite(radius) && radius > 0.0 && samples >= 8 &&
         std::isfinite(eef_step) && eef_step > 0.0 &&
         std::isfinite(velocity) && velocity > 0.0 && velocity <= 1.0 &&
         std::isfinite(acceleration) && acceleration > 0.0 && acceleration <= 1.0 &&
         std::isfinite(max_joint_step) && max_joint_step > 0.0 &&
         std::isfinite(dx) && std::isfinite(dy) && std::isfinite(dz);
}

bool hasJointJump(const moveit_msgs::RobotTrajectory& trajectory, double max_joint_step)
{
  const auto& points = trajectory.joint_trajectory.points;
  for (std::size_t i = 1; i < points.size(); ++i)
  {
    if (points[i].positions.size() != points[i - 1].positions.size())
      return true;
    for (std::size_t joint = 0; joint < points[i].positions.size(); ++joint)
    {
      if (std::abs(points[i].positions[joint] - points[i - 1].positions[joint]) > max_joint_step)
        return true;
    }
  }
  return false;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aubo_cartesian_path_demo");
  ros::NodeHandle private_nh("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string group_name, end_effector_link, shape, plane;
  private_nh.param<std::string>("group_name", group_name, "aubo_i5");
  private_nh.param<std::string>("end_effector_link", end_effector_link, "tcp_link");
  private_nh.param<std::string>("shape", shape, "circle");
  private_nh.param<std::string>("plane", plane, "xy");

  double radius, eef_step, velocity, acceleration, max_joint_step, dx, dy, dz;
  int samples;
  bool execute;
  private_nh.param("radius", radius, 0.03);
  private_nh.param("samples", samples, 36);
  private_nh.param("eef_step", eef_step, 0.005);
  private_nh.param("velocity_scaling", velocity, 0.1);
  private_nh.param("acceleration_scaling", acceleration, 0.1);
  private_nh.param("max_joint_step", max_joint_step, 0.35);
  private_nh.param("dx", dx, 0.05);
  private_nh.param("dy", dy, 0.0);
  private_nh.param("dz", dz, 0.0);
  private_nh.param("execute", execute, false);

  if (!validParameters(shape, plane, radius, samples, eef_step, velocity,
                       acceleration, max_joint_step, dx, dy, dz))
  {
    ROS_ERROR("Invalid Cartesian path parameters. Check shape, plane, radius, step and scaling.");
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface arm(group_name);
  arm.setEndEffectorLink(end_effector_link);
  arm.setMaxVelocityScalingFactor(velocity);
  arm.setMaxAccelerationScalingFactor(acceleration);

  const moveit::core::RobotStatePtr current_state = arm.getCurrentState(10.0);
  if (!current_state)
  {
    ROS_ERROR("No current robot state. Check /joint_states and move_group.");
    return 2;
  }

  // Generate waypoints in the MoveIt planning frame, keeping the current TCP orientation.
  arm.setPoseReferenceFrame(arm.getPlanningFrame());
  const geometry_msgs::Pose start = arm.getCurrentPose(end_effector_link).pose;
  std::vector<geometry_msgs::Pose> waypoints;
  if (shape == "line")
  {
    geometry_msgs::Pose end = start;
    end.position.x += dx;
    end.position.y += dy;
    end.position.z += dz;
    waypoints.push_back(end);
  }
  else
  {
    const char first_axis = plane[0];
    const char second_axis = plane[1];
    for (int i = 1; i <= samples; ++i)
    {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / samples;
      geometry_msgs::Pose waypoint = start;
      coordinate(waypoint, first_axis) += radius * (std::cos(angle) - 1.0);
      coordinate(waypoint, second_axis) += radius * std::sin(angle);
      waypoints.push_back(waypoint);
    }
  }

  arm.setStartStateToCurrentState();
  moveit_msgs::RobotTrajectory trajectory_message;
  const double fraction = arm.computeCartesianPath(waypoints, eef_step, trajectory_message, true);
  ROS_INFO_STREAM("Cartesian " << shape << " path fraction: " << fraction);
  if (fraction < 0.999 || trajectory_message.joint_trajectory.points.size() < 2)
  {
    ROS_ERROR("Cartesian path is incomplete; no trajectory will be executed.");
    return 3;
  }
  if (hasJointJump(trajectory_message, max_joint_step))
  {
    ROS_ERROR("Cartesian path contains a joint step above max_joint_step; no execution.");
    return 4;
  }

  robot_trajectory::RobotTrajectory robot_trajectory(arm.getRobotModel(), group_name);
  robot_trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(robot_trajectory, velocity, acceleration))
  {
    ROS_ERROR("Could not time-parameterize Cartesian trajectory.");
    return 5;
  }
  robot_trajectory.getRobotTrajectoryMsg(trajectory_message);

  moveit_msgs::DisplayTrajectory display;
  display.model_id = arm.getRobotModel()->getName();
  display.trajectory_start = moveit_msgs::RobotState();
  moveit::core::robotStateToRobotStateMsg(*current_state, display.trajectory_start);
  display.trajectory.push_back(trajectory_message);
  ros::Publisher display_publisher =
      private_nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  ros::WallDuration(0.5).sleep();
  display_publisher.publish(display);

  ROS_INFO_STREAM("Planned " << trajectory_message.joint_trajectory.points.size()
                  << " trajectory points; execution " << (execute ? "enabled" : "disabled") << '.');
  if (execute)
  {
    const auto result = arm.execute(trajectory_message);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Cartesian path execution failed with MoveIt code " << result.val);
      return 6;
    }
  }

  ros::shutdown();
  return 0;
}
