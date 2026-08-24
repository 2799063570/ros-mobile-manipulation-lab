#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <string>
#include <vector>

namespace
{
trajectory_msgs::JointTrajectory gripperPosture(double position, double duration)
{
  trajectory_msgs::JointTrajectory posture;
  posture.joint_names = { "joint1", "joint2" };

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = { position, position };
  point.time_from_start = ros::Duration(duration);
  posture.points.push_back(point);
  return posture;
}

geometry_msgs::Quaternion orientationFromRpy(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  quaternion.normalize();

  geometry_msgs::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

moveit_msgs::CollisionObject makeBox(const std::string& id,
                                     const std::string& frame,
                                     double x,
                                     double y,
                                     double z,
                                     double size_x,
                                     double size_y,
                                     double size_z)
{
  moveit_msgs::CollisionObject object;
  object.header.frame_id = frame;
  object.id = id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = { size_x, size_y, size_z };

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;
  return object;
}

void configureTranslation(moveit_msgs::GripperTranslation& translation,
                          const std::string& frame,
                          double z_direction)
{
  translation.direction.header.frame_id = frame;
  translation.direction.vector.z = z_direction;
  translation.min_distance = 0.05;
  translation.desired_distance = 0.10;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aubo_pick_place_demo");
  ros::NodeHandle private_nh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string planning_frame;
  std::string end_effector_link;
  std::string object_id;
  std::string table_id;
  private_nh.param<std::string>("planning_frame", planning_frame, "base_link");
  private_nh.param<std::string>("end_effector_link", end_effector_link, "tcp_link");
  private_nh.param<std::string>("object_id", object_id, "pick_object");
  private_nh.param<std::string>("table_id", table_id, "work_table");

  double object_x;
  double object_y;
  double object_z;
  double place_x;
  double place_y;
  double place_z;
  double object_size_x;
  double object_size_y;
  double object_size_z;
  double gripper_closed;
  double grasp_offset_z;
  double grasp_roll;
  double grasp_pitch;
  double grasp_yaw;
  private_nh.param("object_x", object_x, 0.45);
  private_nh.param("object_y", object_y, -0.15);
  private_nh.param("object_z", object_z, 0.15);
  private_nh.param("place_x", place_x, 0.45);
  private_nh.param("place_y", place_y, 0.15);
  private_nh.param("place_z", place_z, 0.15);
  private_nh.param("object_size_x", object_size_x, 0.04);
  private_nh.param("object_size_y", object_size_y, 0.04);
  private_nh.param("object_size_z", object_size_z, 0.10);
  private_nh.param("gripper_closed", gripper_closed, 0.45);
  private_nh.param("grasp_offset_z", grasp_offset_z, 0.12);
  private_nh.param("grasp_roll", grasp_roll, 0.0);
  private_nh.param("grasp_pitch", grasp_pitch, 3.141592653589793);
  private_nh.param("grasp_yaw", grasp_yaw, 0.0);

  moveit::planning_interface::MoveGroupInterface arm("aubo_i5");
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  arm.setPoseReferenceFrame(planning_frame);
  arm.setEndEffectorLink(end_effector_link);
  arm.setPlanningTime(15.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);
  arm.setSupportSurfaceName(table_id);

  if (!arm.getCurrentState(10.0))
  {
    ROS_ERROR("No current robot state received within 10 seconds.");
    return 1;
  }

  const double table_height = 0.05;
  const double table_top_z = object_z - object_size_z * 0.5;
  const auto table = makeBox(table_id, planning_frame, 0.45, 0.0,
                             table_top_z - table_height * 0.5,
                             0.70, 0.70, table_height);
  const auto object = makeBox(object_id, planning_frame, object_x, object_y, object_z,
                              object_size_x, object_size_y, object_size_z);
  planning_scene.applyCollisionObjects({ table, object });
  ros::Duration(1.0).sleep();

  const auto grasp_orientation =
      orientationFromRpy(grasp_roll, grasp_pitch, grasp_yaw);

  moveit_msgs::Grasp grasp;
  grasp.id = "top_grasp";
  grasp.grasp_pose.header.frame_id = planning_frame;
  grasp.grasp_pose.pose.position.x = object_x;
  grasp.grasp_pose.pose.position.y = object_y;
  // Grasp/Place poses describe the end-effector parent link.  With a top-down
  // grasp, the gripper base therefore stays above the object's centre.
  grasp.grasp_pose.pose.position.z = object_z + grasp_offset_z;
  grasp.grasp_pose.pose.orientation = grasp_orientation;
  grasp.pre_grasp_posture = gripperPosture(0.0, 0.8);
  grasp.grasp_posture = gripperPosture(gripper_closed, 0.8);
  configureTranslation(grasp.pre_grasp_approach, planning_frame, -1.0);
  configureTranslation(grasp.post_grasp_retreat, planning_frame, 1.0);
  grasp.allowed_touch_objects.push_back(object_id);
  grasp.max_contact_force = 0.0;

  ROS_INFO_STREAM("Planning pick for object '" << object_id << "'.");
  const auto pick_result = arm.pick(object_id, grasp);
  if (pick_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR_STREAM("Pick failed with MoveIt error code " << pick_result.val << ".");
    return 2;
  }

  moveit_msgs::PlaceLocation place;
  place.id = "place_target";
  place.place_pose.header.frame_id = planning_frame;
  place.place_pose.pose.position.x = place_x;
  place.place_pose.pose.position.y = place_y;
  place.place_pose.pose.position.z = place_z + grasp_offset_z;
  place.place_pose.pose.orientation = grasp_orientation;
  place.post_place_posture = gripperPosture(0.0, 0.8);
  configureTranslation(place.pre_place_approach, planning_frame, -1.0);
  configureTranslation(place.post_place_retreat, planning_frame, 1.0);

  ROS_INFO_STREAM("Planning place for object '" << object_id << "'.");
  const auto place_result = arm.place(object_id, std::vector<moveit_msgs::PlaceLocation>{ place });
  if (place_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR_STREAM("Place failed with MoveIt error code " << place_result.val << ".");
    return 3;
  }

  ROS_INFO("Pick and place completed successfully.");
  ros::shutdown();
  return 0;
}
