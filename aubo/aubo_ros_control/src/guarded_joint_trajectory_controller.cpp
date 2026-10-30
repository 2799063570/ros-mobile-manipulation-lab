#include <aubo_ros_control/guarded_joint_trajectory_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(aubo_ros_control::GuardedJointTrajectoryController,
                      controller_interface::ControllerBase)
