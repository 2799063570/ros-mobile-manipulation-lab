#ifndef AUBO_ROS_CONTROL_DIRECT_SDK_BACKEND_H
#define AUBO_ROS_CONTROL_DIRECT_SDK_BACKEND_H

#include <aubo_ros_control/visual_servo_common.h>
#include <aubo_ros_control/motion_lease.h>

#include <ros/ros.h>

#include <aubo_driver/AuboRobotMetaType.h>
#include <aubo_driver/serviceinterface.h>

#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace aubo_ros_control
{
namespace visual_servo_internal
{

// 真实机械臂后端：管理 SDK 连接，并将关节指令连续、安全地下发到控制柜。
class DirectSdkBackend
{
public:
  DirectSdkBackend(CommandQueue& queue,
                   const JointPoint& velocity_limits,
                   const JointPoint& acceleration_limits,
                   double output_rate);
  ~DirectSdkBackend();

  bool connect(ros::NodeHandle& nh);
  // 点动专用：在 connect 前设置。控制柜需要补点时才调用，避免软件队列积压。
  // 回调只由输出线程串行执行；false 表示当前尚未准备好，不生成新点。
  void setPointGenerator(std::function<bool(JointPoint&)> generator);
  bool readState(JointPoint& position);
  bool healthy() const;
  void shutdown();

private:
  bool login(ServiceInterface& service);
  JointPoint optimize(const JointPoint& requested);
  void outputLoop();

  CommandQueue& queue_;
  JointPoint velocity_limits_;
  JointPoint acceleration_limits_;
  double period_;
  std::string host_, username_, password_;
  int port_{8899}, collision_class_{6}, mac_buffer_target_{60};
  bool require_real_robot_{true};
  ServiceInterface control_service_, state_service_, mac_service_;
  std::atomic<bool> connected_{false}, running_{false}, shutdown_started_{false};
  bool in_stream_mode_{false}, have_last_position_{false};
  std::thread output_thread_;
  JointPoint last_position_, last_velocity_;
  std::function<bool(JointPoint&)> point_generator_;
  MotionLease motion_lease_;
};

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control

#endif  // AUBO_ROS_CONTROL_DIRECT_SDK_BACKEND_H
