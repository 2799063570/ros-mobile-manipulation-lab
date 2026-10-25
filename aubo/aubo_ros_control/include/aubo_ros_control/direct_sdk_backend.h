#ifndef AUBO_ROS_CONTROL_DIRECT_SDK_BACKEND_H
#define AUBO_ROS_CONTROL_DIRECT_SDK_BACKEND_H

#include <aubo_ros_control/visual_servo_common.h>
#include <aubo_ros_control/motion_lease.h>
#include <aubo_ros_control/sdk_tcp_transport.h>
#include <aubo_ros_control/sdk_flow_control.h>
#include <aubo_ros_control/sdk_io_diagnostics.h>
#include <aubo_ros_control/sdk_jog_timing.h>
#include <aubo_ros_control/motion_timing_trace.h>

#include <ros/ros.h>

#include <aubo_driver/AuboRobotMetaType.h>
#include <aubo_driver/serviceinterface.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
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
  // 在线点动/PBVS：在 connect 前设置。控制柜需要补点时才调用，避免软件队列积压。
  // 回调只由输出线程串行执行；false 表示当前尚未准备好，不生成新点。
  void setPointGenerator(std::function<bool(JointPoint&, PointTiming*)> generator);
  // Configure before connect; the owner dumps only after SDK shutdown and all
  // frontend callbacks/control threads have stopped.
  void setTimingTrace(const std::shared_ptr<MotionTimingTrace>& trace) { timing_trace_ = trace; }
  bool readState(JointPoint& position);
  bool healthy();
  bool motionReady() const
  {
    return connected_.load() && running_.load() &&
           (!flow_ || flow_->motionReady(std::chrono::steady_clock::now()));
  }
  bool armMotion()
  {
    return connected_.load() && running_.load() &&
           (!flow_ || flow_->armMotion(std::chrono::steady_clock::now()));
  }
  // 首个传输故障原因；不把停流后的断开误报成实时反馈超时。
  std::string failureReason() const;
  void shutdown();

private:
  friend struct DirectSdkBackendTestAccess;
  bool login(ServiceInterface& service);
  bool healthyAt(std::chrono::steady_clock::time_point now);
  JointPoint optimize(const JointPoint& requested);
  void outputLoop();
  void jogOutputLoop();
  bool queryDiagnosis(std::size_t& mac_scalars);
  void stopOutput(const char* reason, SdkFlowControl::Fault fault = SdkFlowControl::Fault::Stopped);
  void logIoSnapshot();
  void logSendReturn(std::uint64_t id, int result, double duration_ms, bool after_stop);
  static void receiveWaypoint(const aubo_robot_namespace::wayPoint_S* waypoint, void* context);
  static void receiveRobotEvent(const aubo_robot_namespace::RobotEventInfo* event, void* context);

  CommandQueue& queue_;
  JointPoint velocity_limits_;
  JointPoint acceleration_limits_;
  double period_;
  std::string host_, username_, password_;
  int port_{8899}, collision_class_{6}, mac_buffer_target_{300};
  int mac_buffer_limit_{360};
  double diagnosis_timeout_{0.15};
  bool require_real_robot_{true};
  ServiceInterface control_service_, state_service_, mac_service_;
  std::atomic<bool> connected_{false}, running_{false}, shutdown_started_{false};
  mutable std::mutex failure_mutex_;
  std::string failure_reason_;
  bool in_stream_mode_{false}, have_last_position_{false};
  std::thread output_thread_;
  std::unique_ptr<SdkFlowControl> flow_;
  std::atomic<std::int64_t> send_started_ns_{0}, first_send_ns_{0};
  std::atomic<bool> mac_confirmed_{false};
  JointPoint last_position_, last_velocity_;
  std::function<bool(JointPoint&, PointTiming*)> point_generator_;
  std::shared_ptr<MotionTimingTrace> timing_trace_;
  std::uint64_t trace_query_id_{0}, trace_batch_id_{0}, trace_point_id_{0};
  MotionLease motion_lease_;
  SdkTcpTransport tcp_transport_;
  SdkIoDiagnostics io_diagnostics_;
  std::mutex feedback_mutex_;
  std::condition_variable feedback_ready_;
  JointPoint pushed_position_{};
  std::chrono::steady_clock::time_point feedback_received_{};
  double feedback_timeout_{0.15};
  bool state_push_registered_{false}, have_pushed_position_{false};
  bool event_registered_{false}, feedback_stale_reported_{false};
  std::size_t feedback_samples_{0};
  double max_feedback_gap_ms_{0.0};
};

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control

#endif  // AUBO_ROS_CONTROL_DIRECT_SDK_BACKEND_H
