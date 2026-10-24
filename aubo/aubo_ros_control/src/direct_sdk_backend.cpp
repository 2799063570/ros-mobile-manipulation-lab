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
namespace
{
using Clock = std::chrono::steady_clock;
std::int64_t steadyNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
double milliseconds(Clock::duration duration)
{
  return std::chrono::duration<double, std::milli>(duration).count();
}
}

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

void DirectSdkBackend::setPointGenerator(std::function<bool(JointPoint&, PointTiming*)> generator)
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
  // Keep the queue-based visual-servo default; the larger bounded pipeline is
  // specific to the on-demand jog generator and its synchronous SDK sends.
  nh.param<int>("sdk_mac_buffer_target", mac_buffer_target_, point_generator_ ? 300 : 60);
  nh.param<double>("feedback_timeout", feedback_timeout_, 0.15);
  nh.param<int>("sdk_mac_buffer_limit", mac_buffer_limit_, 360);
  nh.param<double>("sdk_diagnosis_timeout", diagnosis_timeout_, 0.15);

  if (!std::isfinite(period_) || period_ <= 0.0 ||
      !std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0.0 ||
      mac_buffer_target_ < 6 || mac_buffer_target_ > 400 || mac_buffer_target_ % 6 != 0 ||
      (point_generator_ && (std::abs(period_ - 0.005) > 1e-9 ||
       mac_buffer_target_ < 24 || mac_buffer_target_ > 300 ||
       mac_buffer_limit_ < mac_buffer_target_ || mac_buffer_limit_ > 396 || mac_buffer_limit_ % 6 != 0 ||
       !std::isfinite(diagnosis_timeout_) || diagnosis_timeout_ <= 0.0 || diagnosis_timeout_ > 0.5)))
  {
    ROS_ERROR("[sdk] 无效参数；点动周期 5 ms，名义 MAC 目标 24..300，硬上限须为目标..396 的六倍数，诊断超时 (0,0.5] s");
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

  // SDK 默认未禁用 Nagle；同步请求/响应容易受小包与延迟 ACK 影响。
  // 在 startup 和进入透传前设置三个连接；点动流量上限另行显式配置。
  if (!tcp_transport_.configure(host_, port_) || tcp_transport_.size() < 3)
  {
    ROS_ERROR("[sdk] 无法为三个控制柜连接启用 TCP_NODELAY / TCP_QUICKACK，拒绝启动透传");
    shutdown();
    return false;
  }
  ROS_INFO("[sdk] 已为 %zu 个控制柜 TCP 连接启用 TCP_NODELAY / TCP_QUICKACK",
           tcp_transport_.size());
  event_registered_ = control_service_.robotServiceRegisterRobotEventInfoCallback(
      &DirectSdkBackend::receiveRobotEvent, this) == aubo_robot_namespace::InterfaceCallSuccCode;
  if (!event_registered_)
    ROS_WARN("[sdk] 控制柜事件日志订阅失败；原有诊断/欠载保护仍有效");

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

  state_push_registered_ = true;
  if (state_service_.robotServiceRegisterRealTimeRoadPointCallback(
          &DirectSdkBackend::receiveWaypoint, this) !=
      aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ROS_ERROR("[sdk] 无法订阅实时关节位置推送");
    shutdown();
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(feedback_mutex_);
    const bool ready = feedback_ready_.wait_for(lock, std::chrono::seconds(2),
        [this] { return have_pushed_position_; });
    if (!ready)
    {
      lock.unlock();
      ROS_ERROR("[sdk] 2 秒内未收到实时关节反馈，拒绝进入透传");
      shutdown();
      return false;
    }
    last_position_ = pushed_position_;
  }
  ROS_INFO("[sdk] 已收到实时关节反馈；反馈独立推送，MAC查询与下发使用同一SDK连接串行执行");
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
  if (point_generator_)
    flow_.reset(new SdkFlowControl(static_cast<std::size_t>(mac_buffer_limit_ / 6),
        diagnosis_timeout_, Clock::now(), SdkFlowControl::Policy::SequentialMac));
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
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (!have_pushed_position_)
    return false;
  const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - feedback_received_).count();
  if (age > feedback_timeout_)
  {
    if (!feedback_stale_reported_)
    {
      ROS_ERROR("[sdk/feedback] 实时推送超时：样本年龄=%.2f ms，阈值=%.2f ms，"
                "累计=%zu 帧，历史最大帧间隔=%.2f ms",
                age * 1000.0, feedback_timeout_ * 1000.0,
                feedback_samples_, max_feedback_gap_ms_);
      feedback_stale_reported_ = true;
    }
    return false;
  }
  position = pushed_position_;
  return true;
}

void DirectSdkBackend::receiveWaypoint(const aubo_robot_namespace::wayPoint_S* waypoint, void* context)
{
  if (!waypoint || !context)
    return;
  auto* backend = static_cast<DirectSdkBackend*>(context);
  JointPoint point;
  std::copy(waypoint->jointpos, waypoint->jointpos + kDof, point.begin());
  if (!finitePoint(point))
    return;
  {
    std::lock_guard<std::mutex> lock(backend->feedback_mutex_);
    const auto received = std::chrono::steady_clock::now();
    if (backend->have_pushed_position_)
      backend->max_feedback_gap_ms_ = std::max(backend->max_feedback_gap_ms_,
          std::chrono::duration<double, std::milli>(received - backend->feedback_received_).count());
    backend->pushed_position_ = point;
    backend->feedback_received_ = received;
    backend->have_pushed_position_ = true;
    backend->feedback_stale_reported_ = false;
    ++backend->feedback_samples_;
  }
  backend->feedback_ready_.notify_one();
}

void DirectSdkBackend::receiveRobotEvent(const aubo_robot_namespace::RobotEventInfo* event, void* context)
{
  if (event)
  {
    ROS_INFO("[sdk/event] type=%d code=%d content=%s",
             static_cast<int>(event->eventType), event->eventCode, event->eventContent.c_str());
    auto* backend = static_cast<DirectSdkBackend*>(context);
    if (backend && backend->point_generator_ && backend->running_.load() &&
        event->eventType == aubo_robot_namespace::RobotEvent_MacDataInterruptWarning)
      backend->stopOutput("控制柜事件报告 MAC 数据中断");
  }
}

void DirectSdkBackend::stopOutput(const char* reason, SdkFlowControl::Fault fault)
{
  bool first = false;
  {
    std::lock_guard<std::mutex> lock(failure_mutex_);
    first = running_.exchange(false);
    if (first)
      failure_reason_ = reason;
    connected_.store(false);
  }
  if (flow_)
    flow_->stop(fault);
  if (first)
  {
    ROS_ERROR("[sdk] %s；停止供点，需人工重启", reason);
    if (point_generator_)
      logIoSnapshot();
  }
}

std::string DirectSdkBackend::failureReason() const
{
  std::lock_guard<std::mutex> lock(failure_mutex_);
  return failure_reason_;
}

void DirectSdkBackend::logIoSnapshot()
{
  // 每个锁只用于复制本组状态。日志不可持有反馈/流控锁，亦不可延后停流锁存。
  const auto io = io_diagnostics_.snapshot(Clock::now());
  double feedback_age_ms = -1.0, feedback_gap_ms = 0.0;
  std::size_t feedback_samples = 0;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_samples = feedback_samples_;
    feedback_gap_ms = max_feedback_gap_ms_;
    if (have_pushed_position_)
      feedback_age_ms = std::max(0.0, milliseconds(Clock::now() - feedback_received_));
  }
  ROS_ERROR("[sdk/transport] 已开始=%llu 批，已返回=%llu 批；在途批次=%llu，点数=%zu，"
            "已阻塞=%.2f ms，生成=%.2f ms，最大关节增量=%.9f rad；MAC账面=%zu 点",
            static_cast<unsigned long long>(io.started_sends),
            static_cast<unsigned long long>(io.returned_sends),
            static_cast<unsigned long long>(io.send_active ? io.active_send_id : 0),
            io.send_active ? io.active_send_points : 0,
            io.send_active ? io.active_send_elapsed_ms : -1.0,
            io.send_active ? io.active_send_generation_ms : -1.0,
            io.send_active ? io.active_send_max_delta : -1.0,
            flow_ ? flow_->upperBound() : 0);
  ROS_ERROR("[sdk/transport] 最近返回批次=%llu ret=%d 耗时=%.2f ms；诊断在途=%.2f ms，"
            "最近MAC=%lld 标量（响应年龄=%.2f ms，请求年龄=%.2f ms，RPC=%.2f ms）；"
            "反馈年龄=%.2f ms，最大帧间隔=%.2f ms，累计=%zu 帧；-1 表示无样本/无在途",
            static_cast<unsigned long long>(io.last_send_valid ? io.last_send_id : 0),
            io.last_send_valid ? io.last_send_result : -1,
            io.last_send_valid ? io.last_send_duration_ms : -1.0,
            io.diagnosis_active ? io.diagnosis_elapsed_ms : -1.0,
            io.mac_valid ? static_cast<long long>(io.mac_scalars) : -1LL,
            io.mac_valid ? io.mac_response_age_ms : -1.0,
            io.mac_valid ? io.mac_request_age_ms : -1.0,
            io.mac_valid ? io.mac_diagnosis_duration_ms : -1.0,
            feedback_age_ms, feedback_gap_ms, feedback_samples);
  const auto records = io_diagnostics_.history();
  const auto now = Clock::now();
  for (const auto& event : records)
    ROS_ERROR("[sdk/history] 距现在 %.2f ms：%s 批次=%llu %s=%zu ret=%d RPC=%.2f ms",
              std::max(0.0, milliseconds(now - event.finished)),
              event.diagnosis ? "MAC查询" : "发送",
              static_cast<unsigned long long>(event.send_id),
              event.diagnosis ? "标量" : "点数", event.count, event.result, event.duration_ms);
}

void DirectSdkBackend::logSendReturn(std::uint64_t id, int result, double duration_ms, bool after_stop)
{
  // 仅停流路径调用；正常供点不能因终端/rosout 阻塞而错过下一批期限。
  ROS_WARN("[sdk/send] 批次=%llu SDK已返回 ret=%d 实际耗时=%.2f ms 停流后返回=%s；"
           "返回不代表轨迹已执行完毕",
           static_cast<unsigned long long>(id), result, duration_ms, after_stop ? "是" : "否");
}

bool DirectSdkBackend::healthy()
{
  return healthyAt(Clock::now());
}

bool DirectSdkBackend::healthyAt(Clock::time_point checked_at)
{
  if (!connected_.load() || !running_.load())
    return false;
  if (flow_)
  {
    if (!flow_->healthy(checked_at))
    {
      stopOutput("诊断超时、计数失效或轨迹积压超限");
      ROS_ERROR("[sdk/jog] 异步流量保护故障=%d，未消费点数保守上界=%zu",
                static_cast<int>(flow_->fault()), flow_->upperBound());
      return false;
    }
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        checked_at.time_since_epoch()).count();
    const auto sending = send_started_ns_.load();
    if (sending != 0 && (now - sending) * 1e-9 >= SdkJogTiming(mac_buffer_target_).sendBudget())
    {
      stopOutput("SDK 下发阻塞超过单次发送预算");
      return false;
    }
    const auto first = first_send_ns_.load();
    if (first != 0 && !mac_confirmed_.load() &&
        (now - first) * 1e-9 >= mac_buffer_limit_ / 6 * period_)
    {
      stopOutput("在硬上限覆盖时间内仍未观察到首个非零 MAC 缓冲");
      return false;
    }
  }
  return true;
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
  if (state_push_registered_)
  {
    state_service_.robotServiceRegisterRealTimeRoadPointCallback(nullptr, nullptr);
    state_push_registered_ = false;
  }
  if (event_registered_)
  {
    control_service_.robotServiceRegisterRobotEventInfoCallback(nullptr, nullptr);
    event_registered_ = false;
  }
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

bool DirectSdkBackend::queryDiagnosis(std::size_t& mac_scalars)
{
  // No SDK send is in flight while querying. Diagnosis exposes no hardware
  // sample timestamp or point sequence; accept the observed count instead of
  // forcing it to agree with the previous local prediction.
  const auto started = Clock::now();
  const auto before_query = flow_->upperBound();
  const auto token = flow_->beginQuery(started);
  if (!token || !tcp_transport_.prepare())
  {
    stopOutput("无法开始 MAC 诊断或 TCP 连接失效");
    return false;
  }
  aubo_robot_namespace::RobotDiagnosis diagnosis{};
  MotionTimingTrace::Event query_event;
  if (timing_trace_)
  {
    query_event.kind = MotionTimingTrace::QueryBegin;
    query_event.query_id = ++trace_query_id_;
    query_event.begin_ns = MotionTimingTrace::ns();
    timing_trace_->append(query_event);
  }
  io_diagnostics_.beginDiagnosis(Clock::now());
  const int result = mac_service_.robotServiceGetRobotDiagnosisInfo(diagnosis);
  const auto received = Clock::now();
  io_diagnostics_.finishDiagnosis(result, diagnosis.macTargetPosDataSize, received);
  if (timing_trace_)
  {
    query_event.kind = MotionTimingTrace::QueryEnd;
    query_event.end_ns = MotionTimingTrace::ns(received);
    query_event.mac_scalars = diagnosis.macTargetPosDataSize;
    query_event.result = result;
    timing_trace_->append(query_event);  // also retain replies arriving after a stop
  }
  if (!running_.load())
    return false;
  if (result != aubo_robot_namespace::InterfaceCallSuccCode || diagnosis.softEmergency ||
      diagnosis.remoteEmergency || diagnosis.robotCollision || diagnosis.macDataInterruptWarning)
  {
    stopOutput("诊断失败或控制柜保护触发", SdkFlowControl::Fault::DiagnosisFailed);
    ROS_ERROR("[sdk/diagnosis] ret=%d soft=%d remote=%d collision=%d MAC中断=%u CAN=%u",
              result, diagnosis.softEmergency, diagnosis.remoteEmergency, diagnosis.robotCollision,
              static_cast<unsigned>(diagnosis.macDataInterruptWarning),
              static_cast<unsigned>(diagnosis.armCanbusStatus));
    return false;
  }
  if (first_send_ns_.load() != 0)
  {
    if (diagnosis.macTargetPosDataSize != 0)
    {
      if (!mac_confirmed_.exchange(true))
        ROS_INFO("[sdk/jog] 首个非零 MAC 缓冲已观察到：%u 标量",
                 static_cast<unsigned>(diagnosis.macTargetPosDataSize));
    }
    else if (mac_confirmed_.load() && token.completed_before_request != 0)
    {
      stopOutput("已建立的 MAC 轨迹流出现零缓冲，拒绝断流后恢复");
      return false;
    }
  }
  if (!flow_->acceptReply(token, diagnosis.macTargetPosDataSize, received))
  {
    stopOutput("MAC 诊断超时、缓冲超限或响应计数失效");
    ROS_ERROR("[sdk/diagnosis] 流控故障=%d，本次MAC=%u 标量，查询前账面=%zu 点，限额=%d 点",
              static_cast<int>(flow_->fault()), static_cast<unsigned>(diagnosis.macTargetPosDataSize),
              before_query, mac_buffer_limit_ / 6);
    return false;
  }
  const auto observed_points = flow_->upperBound();
  if (observed_points > static_cast<std::size_t>(mac_buffer_target_ / 6))
    ROS_WARN_THROTTLE(1.0, "[sdk/diagnosis] MAC观测=%zu 点（%u 标量），高于目标 %d 点；"
                      "暂停补点，观测/预留限额=%d 点",
                      observed_points, static_cast<unsigned>(diagnosis.macTargetPosDataSize),
                      mac_buffer_target_ / 6, mac_buffer_limit_ / 6);
  else if (observed_points > before_query)
    // SequentialMac deliberately adopts new observations, including upward
    // corrections. Within-target corrections are telemetry, not a flow fault.
    ROS_INFO_THROTTLE(5.0, "[sdk/diagnosis] MAC计数更新：账面 %zu 点 -> 观测 %zu 点（%u 标量）；"
                      "目标=%d 点，观测/预留限额=%d 点",
                      before_query, observed_points, static_cast<unsigned>(diagnosis.macTargetPosDataSize),
                      mac_buffer_target_ / 6, mac_buffer_limit_ / 6);
  if (first_send_ns_.load() != 0)
  {
    const double query_ms = milliseconds(received - started);
    // Reporting threshold only. Keep every existing timeout/capacity/underflow
    // check above and in healthyAt(). Shorter configured watchdogs also get an
    // earlier warning; no elapsed time is treated as cabinet consumption.
    const double warn_ms = std::min(50.0, diagnosis_timeout_ * 500.0);
    if (query_ms >= warn_ms)
      ROS_WARN_THROTTLE(1.0, "[sdk/diagnosis] 查询偏慢 %.2f ms；MAC观测=%zu 点，名义覆盖 %.1f ms，"
                        "诊断看门狗=%.1f ms；MAC无硬件采样时间戳，覆盖值不代表即时余量",
                        query_ms, observed_points, observed_points * period_ * 1000.0,
                        diagnosis_timeout_ * 1000.0);
    else if (query_ms > 20.0)
      ROS_INFO_THROTTLE(5.0, "[sdk/diagnosis] 查询 %.2f ms；MAC观测=%zu 点，目标=%d 点",
                        query_ms, observed_points, mac_buffer_target_ / 6);
  }
  mac_scalars = diagnosis.macTargetPosDataSize;
  return true;
}

void DirectSdkBackend::jogOutputLoop()
{
  const SdkJogTiming timing(mac_buffer_target_);
  SdkPointPacer pacer(timing.target_points);
  bool started = false;
  ROS_INFO("[sdk/jog] MAC 按需补点：目标 %zu 点 / %.1f ms，点周期 5 ms；补点额度按200点/s积累，至少20ms缺口；"
           "发送预算 %.1f ms，MAC观测/预留限额 %d 点 / %.1f ms，诊断看门狗 %.1f ms",
           timing.target_points, timing.lookahead() * 1000.0, timing.sendBudget() * 1000.0,
           mac_buffer_limit_ / 6, mac_buffer_limit_ / 6 * period_ * 1000.0,
           diagnosis_timeout_ * 1000.0);
  while (ros::ok() && healthy())
  {
    std::size_t mac_scalars = 0;
    if (!queryDiagnosis(mac_scalars))
      return;
    if (started && !mac_confirmed_.load())
    {
      // The SDK call may return before its points appear in a cached MAC
      // snapshot. Wait without re-sending; healthyAt bounds this phase.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const auto deficit = timing.wanted(mac_scalars);
    if (started && !pacer.started())
      pacer.start(Clock::now(), deficit);
    if (deficit == 0)
    {
      if (started) pacer.discard(Clock::now());
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const auto wanted = started ? pacer.allowance(deficit, Clock::now()) : deficit;
    if (wanted == 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const auto generation_started = Clock::now();
    const auto batch_id = timing_trace_ ? ++trace_batch_id_ : 0;
    const auto batch_origin = last_position_;
    std::vector<aubo_robot_namespace::wayPoint_S> batch;
    batch.reserve(wanted);
    for (std::size_t index = 0; index < wanted; ++index)
    {
      JointPoint requested = last_position_;
      bool generated = false;
      PointTiming point_timing;
      try
      {
        generated = point_generator_(requested, timing_trace_ ? &point_timing : nullptr);
      }
      catch (const std::exception& error)
      {
        ROS_ERROR("[sdk/jog] 轨迹点生成异常：%s", error.what());
        stopOutput("轨迹点生成异常");
        return;
      }
      if (!generated)
        break;
      const auto point_id = timing_trace_ ? ++trace_point_id_ : 0;
      if (timing_trace_)
      {
        MotionTimingTrace::Event event;
        event.kind = MotionTimingTrace::Point;
        event.id = point_id;
        event.batch_id = batch_id;
        event.query_id = trace_query_id_;
        event.command_id = point_timing.command_id;
        event.target_id = point_timing.target_id;
        event.begin_ns = point_timing.begin_ns;
        event.locked_ns = point_timing.locked_ns;
        event.end_ns = point_timing.end_ns;
        event.q = requested;
        event.velocity = point_timing.velocity;
        event.desired = point_timing.desired;
        timing_trace_->append(event);
      }
      if (!finitePoint(requested))
      {
        stopOutput("生成了非有限轨迹点");
        return;
      }
      if (!started && batch.empty() && requested == last_position_)
        break;
      JointPoint velocity;
      for (std::size_t i = 0; i < kDof; ++i)
      {
        velocity[i] = (requested[i] - last_position_[i]) / period_;
        if (std::abs(velocity[i]) > velocity_limits_[i] + 1e-7 ||
            std::abs(velocity[i] - last_velocity_[i]) > acceleration_limits_[i] * period_ + 1e-7)
        {
          stopOutput("生成点不连续或速度/加速度越限");
          return;
        }
      }
      last_position_ = requested;
      last_velocity_ = velocity;
      aubo_robot_namespace::wayPoint_S waypoint{};
      std::copy(requested.begin(), requested.end(), waypoint.jointpos);
      batch.push_back(waypoint);
      if (timing_trace_)
      {
        MotionTimingTrace::Event event;
        event.kind = MotionTimingTrace::BatchAppend;
        event.id = point_id;
        event.batch_id = batch_id;
        event.begin_ns = MotionTimingTrace::ns();
        event.count = batch.size();
        timing_trace_->append(event);
      }
    }
    if (batch.empty())
    {
      if (started)
      {
        stopOutput("运动开始后生成器停止供点");
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    if (batch.size() != wanted || !healthy())
    {
      stopOutput("本批轨迹未完整及时生成，停止供点");
      return;
    }
    if (!tcp_transport_.prepare() || !flow_->reserve(batch.size(), Clock::now()) ||
        (started && !pacer.reserve(batch.size(), Clock::now())))
    {
      stopOutput("没有可验证的缓冲余量或 TCP 连接失效");
      return;
    }
    const auto send_started = Clock::now();
    const auto send_ns = steadyNs();
    send_started_ns_.store(send_ns);
    if (!started)
      first_send_ns_.store(send_ns);
    if (!healthy())
    {
      send_started_ns_.store(0);
      return;
    }
    double max_delta = 0.0;
    for (std::size_t i = 0; i < kDof; ++i)
      max_delta = std::max(max_delta, std::abs(last_position_[i] - batch_origin[i]));
    const auto send_id = io_diagnostics_.beginSend(batch.size(), max_delta,
        milliseconds(send_started - generation_started), send_started);
    MotionTimingTrace::Event send_event;
    if (timing_trace_)
    {
      send_event.kind = MotionTimingTrace::SendBegin;
      send_event.id = send_id;
      send_event.batch_id = batch_id;
      send_event.query_id = trace_query_id_;
      send_event.count = batch.size();
      send_event.mac_scalars = mac_scalars;
      send_event.begin_ns = MotionTimingTrace::ns();  // SDK boundary, after health/metadata work
      timing_trace_->append(send_event);
    }
    const int result = mac_service_.robotServiceSetRobotPosData2Canbus(batch);
    const auto send_returned = Clock::now();
    if (timing_trace_)
    {
      send_event.kind = MotionTimingTrace::SendEnd;
      send_event.end_ns = MotionTimingTrace::ns(send_returned);
      send_event.result = result;
      timing_trace_->append(send_event);
    }
    const bool stopped_when_returned = !running_.load();
    const double send_ms = milliseconds(send_returned - send_started);
    send_started_ns_.store(0);
    io_diagnostics_.finishSend(send_id, result, send_returned);
    if (result != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      stopOutput("SDK 下发失败；保留在途占用，不恢复供点", SdkFlowControl::Fault::SendFailed);
      logSendReturn(send_id, result, send_ms, stopped_when_returned);
      return;
    }
    if (!healthy())
    {
      logSendReturn(send_id, result, send_ms, stopped_when_returned);
      return;
    }
    if (!flow_->completeSend(batch.size()))
    {
      stopOutput("下发完成计数失效");
      logSendReturn(send_id, result, send_ms, stopped_when_returned);
      return;
    }
    if (send_ms >= timing.sendBudget() * 1000.0)
    {
      stopOutput("SDK 下发超过发送预算，拒绝继续供点");
      logSendReturn(send_id, result, send_ms, stopped_when_returned);
      return;
    }
    started = true;
    // Re-query the actual cabinet after acceptance; never manufacture catch-up
    // batches from elapsed wall time or keep an unbounded software trajectory.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void DirectSdkBackend::outputLoop()
{
  if (point_generator_)
  {
    jogOutputLoop();
    return;
  }
  // 视觉伺服的软件队列后端沿用原有按 MAC 余量补点方式。
  while (running_.load() && ros::ok())
  {
    aubo_robot_namespace::RobotDiagnosis diagnosis{};
    if (!tcp_transport_.prepare() ||
        mac_service_.robotServiceGetRobotDiagnosisInfo(diagnosis) !=
            aubo_robot_namespace::InterfaceCallSuccCode ||
        diagnosis.softEmergency || diagnosis.remoteEmergency || diagnosis.robotCollision)
    {
      stopOutput("诊断失败或安全状态触发");
      return;
    }
    if (diagnosis.macTargetPosDataSize < mac_buffer_target_)
    {
      const int wanted = std::max(1, static_cast<int>(std::ceil(
          (mac_buffer_target_ - diagnosis.macTargetPosDataSize) / 6.0)));
      std::vector<aubo_robot_namespace::wayPoint_S> batch;
      for (int index = 0; index < wanted; ++index)
      {
        JointPoint requested;
        if (!queue_.pop(requested))
          break;
        if (!finitePoint(requested))
          continue;
        const JointPoint safe = optimize(requested);
        aubo_robot_namespace::wayPoint_S waypoint{};
        std::copy(safe.begin(), safe.end(), waypoint.jointpos);
        batch.push_back(waypoint);
      }
      if (!batch.empty() && (!tcp_transport_.prepare() ||
          mac_service_.robotServiceSetRobotPosData2Canbus(batch) !=
              aubo_robot_namespace::InterfaceCallSuccCode))
      {
        stopOutput("控制柜轨迹点下发失败");
        return;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
}

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control
