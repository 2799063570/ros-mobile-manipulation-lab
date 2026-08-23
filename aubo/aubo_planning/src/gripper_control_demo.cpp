#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <string>

namespace
{
bool moveToNamedTarget(moveit::planning_interface::MoveGroupInterface& gripper,
                       const std::string& target)
{
  if (!gripper.setNamedTarget(target))
  {
    ROS_ERROR_STREAM("Unknown gripper target: " << target);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto planned = gripper.plan(plan);
  if (planned != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to plan gripper target: " << target);
    return false;
  }

  const auto executed = gripper.execute(plan);
  if (executed != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to execute gripper target: " << target);
    return false;
  }

  ROS_INFO_STREAM("Gripper reached target: " << target);
  return true;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aubo_gripper_control_demo");
  ros::NodeHandle private_nh("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string command;
  private_nh.param<std::string>("command", command, "cycle");

  moveit::planning_interface::MoveGroupInterface gripper("gripper");
  gripper.setPlanningTime(5.0);
  gripper.setMaxVelocityScalingFactor(0.3);
  gripper.setMaxAccelerationScalingFactor(0.3);

  if (!gripper.getCurrentState(10.0))
  {
    ROS_ERROR("No current robot state received within 10 seconds.");
    return 1;
  }

  bool success = false;
  if (command == "open" || command == "closed")
  {
    success = moveToNamedTarget(gripper, command);
  }
  else if (command == "close")
  {
    success = moveToNamedTarget(gripper, "closed");
  }
  else if (command == "cycle")
  {
    success = moveToNamedTarget(gripper, "open");
    if (success)
    {
      ros::Duration(1.0).sleep();
      success = moveToNamedTarget(gripper, "closed");
    }
    if (success)
    {
      ros::Duration(1.0).sleep();
      success = moveToNamedTarget(gripper, "open");
    }
  }
  else
  {
    ROS_ERROR_STREAM("Invalid ~command '" << command
                     << "'. Use open, close, closed, or cycle.");
    return 2;
  }

  ros::shutdown();
  return success ? 0 : 1;
}
