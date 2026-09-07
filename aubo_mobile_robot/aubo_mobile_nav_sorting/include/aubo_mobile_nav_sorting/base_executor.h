#pragma once
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <aubo_mobile_nav_sorting/mission_context.h>
namespace aubo_mobile_nav_sorting
{
class BaseExecutor
{
public:
  using Pose2D = MissionContext::Pose2D;
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  explicit BaseExecutor(MissionContext &context);
  BaseExecutor(const BaseExecutor &) = delete;
  BaseExecutor &operator=(const BaseExecutor &) = delete;
  bool navigateOnce(const Pose2D &target, const std::string &goal_frame);
  bool navigate(const Pose2D &target, const std::string &stage,
                const std::string &goal_frame = std::string(),
                MissionState state = MissionState::NAVIGATING);
  bool currentBasePose(Pose2D &pose, const std::string &pose_frame = std::string());
  bool alignHeading(double target_yaw, const std::string &label,
                    const std::string &pose_frame = std::string());
  bool canDirectDock(const Pose2D &start, const Pose2D &target) const;
  bool driveStraightTo(const Pose2D &target, const std::string &label,
                       MissionState state = MissionState::DIRECT_DOCKING,
                       const std::string &pose_frame = std::string());
  bool moveBaseDirect(double dx, double dy, int attempt, int total);
  void stopBase();
  void cancelNavigation();

private:
  bool publishVelocity(const geometry_msgs::Twist &command);
  bool sendGoal(const move_base_msgs::MoveBaseGoal &goal);
  actionlib::SimpleClientGoalState navigationState();
  bool serverConnected();
  std::mutex command_mutex_;
  MissionContext &context_;
  std::unique_ptr<MoveBaseClient> navigation_client_;// 导航客户端，用于发送导航目标
  mutable tf::TransformListener tf_listener_;
  ros::Publisher velocity_publisher_;
  ros::ServiceClient clear_costmaps_client_;
};
} // namespace aubo_mobile_nav_sorting
