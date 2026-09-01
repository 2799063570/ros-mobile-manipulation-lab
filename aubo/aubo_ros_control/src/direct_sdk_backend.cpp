#include <aubo_ros_control/direct_sdk_backend.h>

#include <ros/package.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>
#include <unistd.h>

namespace aubo_ros_control
{
namespace visual_servo_internal
{

DirectSdkBackend::DirectSdkBackend(CommandQueue& queue,
                                   const JointPoint& velocity_limits,
                                   const JointPoint& acceleration_limits,
                                   double output_rate)
  : queue_(queue), velocity_limits_(velocity_limits),
    acceleration_limits_(acceleration_limits), period_(1.0 / output_rate)
{
  last_position_.fill(0.0);
  last_velocity_.fill(0.0);
}

DirectSdkBackend::~DirectSdkBackend()
{
  shutdown();
}

bool DirectSdkBackend::connect(ros::NodeHandle& nh)
{
  nh.param<std::string>("robot_ip", host_, "192.168.1.2");
  nh.param<int>("server_port", port_, 8899);
  nh.param<std::string>("username", username_, "aubo");
  nh.param<std::string>("password", password_, "123456");
  nh.param<int>("collision_class", collision_class_, 6);
  nh.param<bool>("require_real_robot", require_real_robot_, true);
  nh.param<int>("sdk_mac_buffer_target", mac_buffer_target_, 60);

  const std::string sdk_path = ros::package::getPath("aubo_sdk");
  if (!sdk_path.empty() && ::chdir(sdk_path.c_str()) != 0)
    ROS_WARN("[visual_servo/sdk] 无法切换到 aubo_sdk 目录，继续使用当前目录");

  if (!login(control_service_) || !login(state_service_) || !login(mac_service_))
  {
    ROS_ERROR("[visual_servo/sdk] SDK 三连接登录失败");
    shutdown();
    return false;
  }

  bool real_robot = false;
  state_service_.robotServiceGetIsRealRobotExist(real_robot);
  if (require_real_robot_ && !real_robot)
  {
    ROS_ERROR("[visual_servo/sdk] 控制柜未报告真实机械臂，拒绝下发运动");
    shutdown();
    return false;
  }

  aubo_robot_namespace::ToolDynamicsParam tool{};
  aubo_robot_namespace::ROBOT_SERVICE_STATE startup_state;
  int result = control_service_.rootServiceRobotStartup(
      tool, static_cast<uint8>(collision_class_), true, true, 1000, startup_state);
  if (result != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_ERROR("[visual_servo/sdk] 机器人启动失败，错误码 %d", result);
    shutdown();
    return false;
  }
  result = control_service_.robotServiceRobotHandShake(true);
  if (result != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_ERROR("[visual_servo/sdk] 握手失败，错误码 %d", result);
    shutdown();
    return false;
  }

  bool ready = false;
  for (int attempt = 0; attempt < 100; ++attempt)
  {
    aubo_robot_namespace::RobotDiagnosis diagnosis{};
    if (state_service_.robotServiceGetRobotDiagnosisInfo(diagnosis) ==
            aubo_robot_namespace::InterfaceCallSuccCode &&
        diagnosis.armPowerStatus && !diagnosis.softEmergency &&
        !diagnosis.remoteEmergency && !diagnosis.robotCollision)
    {
      ready = true;
      break;
    }
    usleep(100000);
  }
  if (!ready)
  {
    ROS_ERROR("[visual_servo/sdk] 机器人在 10 秒内未进入安全就绪状态");
    shutdown();
    return false;
  }

  aubo_robot_namespace::wayPoint_S waypoint{};
  if (state_service_.robotServiceGetCurrentWaypointInfo(waypoint) !=
      aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_ERROR("[visual_servo/sdk] 无法读取初始关节位置");
    shutdown();
    return false;
  }
  for (std::size_t i = 0; i < kDof; ++i)
    last_position_[i] = waypoint.jointpos[i];
  have_last_position_ = true;

  // 预填一段静止轨迹，为 ROS 的常规调度抖动预留缓冲，再接收实时伺服点。
  const int lead_in_points = std::max(4, mac_buffer_target_ / 6);
  for (int i = 0; i < lead_in_points; ++i)
    queue_.push(last_position_);

  result = control_service_.robotServiceEnterTcp2CanbusMode();
  if (result == aubo_robot_namespace::ErrCode_ResponseReturnError)
  {
    control_service_.robotServiceLeaveTcp2CanbusMode();
    usleep(200000);
    result = control_service_.robotServiceEnterTcp2CanbusMode();
  }
  if (result != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_ERROR("[visual_servo/sdk] 无法进入 TCP2CANBUS 模式，错误码 %d", result);
    shutdown();
    return false;
  }

  in_stream_mode_ = true;
  running_.store(true);
  output_thread_ = std::thread(&DirectSdkBackend::outputLoop, this);
  ROS_INFO("[visual_servo/sdk] 已连接 %s:%d，启动直接 SDK 队列下发",
           host_.c_str(), port_);
  return true;
}

bool DirectSdkBackend::readState(JointPoint& position)
{
  if (!connected_.load())
    return false;
  aubo_robot_namespace::wayPoint_S waypoint{};
  if (state_service_.robotServiceGetCurrentWaypointInfo(waypoint) !=
      aubo_robot_namespace::InterfaceCallSuccCode)
    return false;
  for (std::size_t i = 0; i < kDof; ++i)
    position[i] = waypoint.jointpos[i];
  return finitePoint(position);
}

bool DirectSdkBackend::healthy() const
{
  return connected_.load() && running_.load();
}

void DirectSdkBackend::shutdown()
{
  if (shutdown_started_.exchange(true))
    return;
  running_.store(false);
  if (output_thread_.joinable())
    output_thread_.join();
  if (in_stream_mode_)
    control_service_.robotServiceLeaveTcp2CanbusMode();
  control_service_.robotServiceLogout();
  state_service_.robotServiceLogout();
  mac_service_.robotServiceLogout();
  connected_.store(false);
  in_stream_mode_ = false;
}

bool DirectSdkBackend::login(ServiceInterface& service)
{
  for (int attempt = 1; attempt <= 5; ++attempt)
  {
    if (service.robotServiceLogin(host_.c_str(), port_, username_.c_str(), password_.c_str()) ==
        aubo_robot_namespace::InterfaceCallSuccCode)
    {
      connected_.store(true);
      return true;
    }
    ROS_WARN("[visual_servo/sdk] 第 %d 次登录失败", attempt);
    usleep(500000);
  }
  return false;
}

JointPoint DirectSdkBackend::optimize(const JointPoint& requested)
{
  if (!have_last_position_)
    return requested;
  JointPoint optimized = last_position_;
  for (std::size_t i = 0; i < kDof; ++i)
  {
    const double requested_velocity =
        clampValue((requested[i] - last_position_[i]) / period_,
                   -velocity_limits_[i], velocity_limits_[i]);
    const double velocity =
        clampValue(requested_velocity,
                   last_velocity_[i] - acceleration_limits_[i] * period_,
                   last_velocity_[i] + acceleration_limits_[i] * period_);
    optimized[i] = last_position_[i] + velocity * period_;
    last_velocity_[i] = velocity;
  }
  last_position_ = optimized;
  return optimized;
}

void DirectSdkBackend::outputLoop()
{
  while (running_.load() && ros::ok())
  {
    aubo_robot_namespace::RobotDiagnosis diagnosis{};
    const int result = mac_service_.robotServiceGetRobotDiagnosisInfo(diagnosis);
    if (result != aubo_robot_namespace::InterfaceCallSuccCode ||
        diagnosis.softEmergency || diagnosis.remoteEmergency || diagnosis.robotCollision)
    {
      ROS_ERROR("[visual_servo/sdk] 诊断失败或安全状态触发，停止 SDK 下发");
      running_.store(false);
      connected_.store(false);
      break;
    }

    if (diagnosis.macTargetPosDataSize < mac_buffer_target_)
    {
      const int wanted = std::max(1, static_cast<int>(std::ceil(
          (mac_buffer_target_ - diagnosis.macTargetPosDataSize) / 6.0)));
      std::vector<aubo_robot_namespace::wayPoint_S> batch;
      batch.reserve(static_cast<std::size_t>(wanted));
      for (int index = 0; index < wanted; ++index)
      {
        JointPoint requested{};
        if (!queue_.pop(requested))
          break;
        if (!finitePoint(requested))
          continue;
        const JointPoint safe = optimize(requested);
        aubo_robot_namespace::wayPoint_S waypoint{};
        std::copy(safe.begin(), safe.end(), waypoint.jointpos);
        batch.push_back(waypoint);
      }
      if (!batch.empty() &&
          mac_service_.robotServiceSetRobotPosData2Canbus(batch) !=
              aubo_robot_namespace::InterfaceCallSuccCode)
      {
        ROS_ERROR("[visual_servo/sdk] 控制柜轨迹点下发失败");
        running_.store(false);
        connected_.store(false);
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
}

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control
