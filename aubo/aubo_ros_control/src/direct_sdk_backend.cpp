#include <aubo_ros_control/direct_sdk_backend.h>

#include <ros/package.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <vector>
#include <utility>
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

void DirectSdkBackend::setPointGenerator(std::function<bool(JointPoint&)> generator)
{
  if (!running_.load())
    point_generator_ = std::move(generator);
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

  if (!std::isfinite(period_) || period_ <= 0.0 ||
      mac_buffer_target_ < 6 || mac_buffer_target_ > 400 || mac_buffer_target_ % 6 != 0 ||
      (point_generator_ && (std::abs(period_ - 0.005) > 1e-9 || mac_buffer_target_ > 60)))
  {
    ROS_ERROR("[sdk] 无效缓冲参数；点动周期须为 5 ms，MAC 缓冲须为 6..60 的六倍数");
    return false;
  }
  if (!motion_lease_.acquire())
  {
    ROS_ERROR("[sdk] 无法取得运动写入锁，请先停止已有 SDK/ros_control 控制节点");
    return false;
  }

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
  state_service_.robotServiceGetIsRealRobotExist(real_robot); // 获取控制柜是否报告真实机械臂
  if (require_real_robot_ && !real_robot)
  {
    ROS_ERROR("[visual_servo/sdk] 控制柜未报告真实机械臂，拒绝下发运动");
    shutdown();
    return false;
  }

  if (point_generator_)
  {
    aubo_robot_namespace::RobotState state;
    if (state_service_.robotServiceGetRobotCurrentState(state) !=
            aubo_robot_namespace::InterfaceCallSuccCode || state != aubo_robot_namespace::RobotStopped)
    {
      ROS_ERROR("[sdk/jog] 机械臂未报告停止状态，拒绝接管正在运行的运动");
      shutdown();
      return false;
    }
  }

  aubo_robot_namespace::ToolDynamicsParam tool{};
  aubo_robot_namespace::ROBOT_SERVICE_STATE startup_state;
  int result = control_service_.rootServiceRobotStartup(
      tool, static_cast<uint8>(collision_class_), true, true, 1000, startup_state);// 启动机械臂（上电 + 松刹车）
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
        !diagnosis.remoteEmergency && !diagnosis.robotCollision)// 获取诊断信息 判断电压、软件紧急、远程紧急、碰撞是否正常
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

  aubo_robot_namespace::wayPoint_S waypoint{};// 状态点序列 （位置(xyz, orientation), 关节位置
  if (state_service_.robotServiceGetCurrentWaypointInfo(waypoint) !=
      aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_ERROR("[visual_servo/sdk] 无法读取初始关节位置");
    shutdown();
    return false;
  }
  for (std::size_t i = 0; i < kDof; ++i)
    last_position_[i] = waypoint.jointpos[i];   // 获取机械臂的当前状态 更新到 last_position_ 中
  have_last_position_ = true;

  // 预填一段静止轨迹，为 ROS 的常规调度抖动预留缓冲，再接收实时伺服点。
  const int lead_in_points = std::max(4, mac_buffer_target_ / 6);
  for (int i = 0; !point_generator_ && i < lead_in_points; ++i)
    queue_.push(last_position_);

  result = control_service_.robotServiceEnterTcp2CanbusMode();// 进入 TCP2CANBUS 模式，直接下发轨迹点到控制柜
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
  motion_lease_.release();
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
  bool stream_started = false;
  while (running_.load() && ros::ok())
  {
    aubo_robot_namespace::RobotDiagnosis diagnosis{};// 机器人诊断信息 包含mac缓冲区的大小
    const int result = mac_service_.robotServiceGetRobotDiagnosisInfo(diagnosis);
    if (result != aubo_robot_namespace::InterfaceCallSuccCode ||
        diagnosis.softEmergency || diagnosis.remoteEmergency || diagnosis.robotCollision)// 急停 或者 碰撞
    {
      ROS_ERROR("[visual_servo/sdk] 诊断失败或安全状态触发，停止 SDK 下发");
      running_.store(false);
      connected_.store(false);
      break;
    }

    if (point_generator_ && stream_started && diagnosis.macTargetPosDataSize == 0)
    {
      // 缓冲已经断流时，继续补发会把一次停顿伪装成正常运动，故要求人工重启。
      ROS_ERROR("[sdk/jog] 控制柜缓冲欠载，停止透传；检查计算耗时和网络抖动");
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
        JointPoint requested = last_position_;
        bool have_point = false;
        try
        {
          have_point = point_generator_ ? point_generator_(requested) : queue_.pop(requested);
        }
        catch (const std::exception& error)
        {
          ROS_ERROR("[sdk] 生成轨迹点失败：%s", error.what());
          running_.store(false);
          connected_.store(false);
          return;
        }
        if (!have_point)
          break;
        if (!finitePoint(requested) && point_generator_)
        {
          ROS_ERROR("[sdk/jog] 非有限点，停止供点");
          running_.store(false);
          return;
        }
        if (!finitePoint(requested))
          continue;
        // 点动已在生成端限制速度/加速度/jerk，不能再用二次滤波破坏其离散导数。
        // 在消费边界验证，一旦越界就停止供点，禁止悄悄截断某个关节。
        if (point_generator_)
        {
          bool valid = true;
          for (std::size_t i = 0; i < kDof; ++i)
          {
            const double velocity = (requested[i] - last_position_[i]) / period_;
            valid = valid && std::abs(velocity) <= velocity_limits_[i] + 1e-7 &&
                    std::abs(velocity - last_velocity_[i]) <= acceleration_limits_[i] * period_ + 1e-7;
            last_velocity_[i] = velocity;
          }
          if (!valid)
          {
            ROS_ERROR("[sdk/jog] 生成点不连续或越限，停止供点");
            running_.store(false);
            return;
          }
          last_position_ = requested;
        }
        const JointPoint safe = point_generator_ ? requested : optimize(requested);
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
      if (!batch.empty())
        stream_started = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(point_generator_ ? 1 : 4));
  }
}

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control
