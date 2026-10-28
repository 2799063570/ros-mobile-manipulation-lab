/**
 * @file aubo_hardware_interface.cpp
 * @brief AUBO 机器人 ros_control 硬件接口实现（参照 aubo_driver.cpp 优化版）
 *
 * 关键设计（来自 aubo_driver.cpp）：
 *  1. connectToRobotController(): 最多重试5次 + 双连接（send/receive分离）
 *  2. timerCallback (50 Hz): 使用 robot_receive_service_ 轮询关节角（GetCurrentWaypointInfo）
 *  3. write() → ros_motion_queue_.enqueue()  （无锁生产）
 *  4. publishWaypointToRobot(): 监控 MAC 缓冲区，按需批量补充点位
 *  5. tryPopWaypoint(): 有符号速度/加速度越限时锁定停止
 *  6. 连续供给 5 ms 采样，保留静止点与低速小位移
 */

#include "aubo_ros_control/aubo_hardware_interface.h"
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <cstring>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>
#include <unistd.h>   // usleep

using namespace aubo_robot_namespace;

namespace aubo_ros_control
{
namespace
{
void updateAtomicMax(std::atomic<double>& destination, double value)
{
    double current = destination.load(std::memory_order_relaxed);
    while (value > current &&
           !destination.compare_exchange_weak(current, value,
               std::memory_order_relaxed, std::memory_order_relaxed))
    {
    }
}

void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status,
                        const std::string& key, const std::string& value)
{
    diagnostic_msgs::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
}
}

// =============================================================================
// 构造 / 析构
// =============================================================================

AuboHardwareInterface::AuboHardwareInterface(ros::NodeHandle& nh)
    : nh_(nh)
    , server_host_("192.168.1.2")
    , server_port_(SERVER_PORT)
    , username_("aubo")
    , password_("123456")
    , ros_motion_queue_(4096)         // 预分配 4096 元素，来自 aubo_driver buf_queue_ 大小参考
    , feed_thread_(nullptr)
    , feed_thread_running_(false)
    , rib_buffer_size_(0)
    , controller_connected_flag_(false)
    , real_robot_exist_(false)
    , emergency_stopped_(false)
    , protective_stopped_(false)
    , in_tcp2canbus_mode_(false)
    , enable_motion_control_(true)
    , require_real_robot_(true)
    , shutdown_started_(false)
    , collision_class_(6)
    , enable_command_smoothing_(false)
    , command_filter_alpha_(1.0)
    , command_deadband_(0.000001)
    , max_command_step_scale_(1.0)
    , control_period_s_(1.0 / DEFAULT_CONTROL_FREQUENCY_HZ)
    , mac_buffer_target_(DEFAULT_MAC_BUF_SIZE)
    , trajectory_preload_points_(DEFAULT_TRAJECTORY_PRELOAD_POINTS)
{
    joint_position_.fill(0.0);
    joint_velocity_.fill(0.0);
    joint_effort_.fill(0.0);
    joint_position_cmd_.fill(0.0);
    last_cmd_sent_.fill(0.0);
    filtered_cmd_.fill(0.0);
    joint_filter_.fill(0.0);
    last_state_position_.fill(0.0);

    for (int i = 0; i < NUM_JOINTS; ++i) {
        joint_position_rt_[i].store(0.0);
        joint_velocity_rt_[i].store(0.0);
        joint_effort_rt_[i].store(0.0);
    }

    memset(&last_joint_velc_,   0, sizeof(last_joint_velc_));

    // 关节名称（与 aubo_driver.cpp joint_name_ 完全一致）
    joint_names_ = {
        "shoulder_joint",
        "upperArm_joint",
        "foreArm_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint"
    };
}

AuboHardwareInterface::~AuboHardwareInterface()
{
    shutdown();
}

// =============================================================================
// init()
// =============================================================================

bool AuboHardwareInterface::init()
{
    // ------------------------------------------------------------------
    // 1. 读取参数
    // ------------------------------------------------------------------
    nh_.param<std::string>("server_host", server_host_, "192.168.1.2");
    nh_.param<std::string>("username",    username_,    "aubo");
    nh_.param<std::string>("password",    password_,    "123456");
    nh_.param<int>        ("server_port", server_port_, SERVER_PORT);
    nh_.param<int>        ("collision_class", collision_class_, 6);
    nh_.param<bool>       ("enable_motion_control", enable_motion_control_, true);
    nh_.param<bool>       ("require_real_robot", require_real_robot_, true);
    nh_.param<bool>       ("use_realtime_feedback", use_realtime_feedback_, false);
    // JointTrajectoryController 已按轨迹时间做三次样条插值。再次低通和
    // 大死区会造成相位滞后与台阶命令，从而导致抖动和跟踪超差。
    nh_.param<bool>       ("enable_command_smoothing", enable_command_smoothing_, false);
    nh_.param<double>     ("command_filter_alpha", command_filter_alpha_, 1.0);
    nh_.param<double>     ("command_deadband", command_deadband_, 0.000001);
    nh_.param<double>     ("max_command_step_scale", max_command_step_scale_, 1.0);
    nh_.param<int>        ("mac_buffer_target", mac_buffer_target_, DEFAULT_MAC_BUF_SIZE);
    nh_.param<int>        ("trajectory_preload_points", trajectory_preload_points_,
                           mac_buffer_target_ / NUM_JOINTS);
    double control_frequency_hz = DEFAULT_CONTROL_FREQUENCY_HZ;
    nh_.param<double>     ("control_frequency", control_frequency_hz,
                           DEFAULT_CONTROL_FREQUENCY_HZ);

    if (server_port_ <= 0 || server_port_ > 65535 || collision_class_ < 1 || collision_class_ > 6 ||
        command_filter_alpha_ <= 0.0 || command_filter_alpha_ > 1.0 ||
        command_deadband_ < 0.0 || max_command_step_scale_ <= 0.0 || max_command_step_scale_ > 1.0 ||
        mac_buffer_target_ < NUM_JOINTS || mac_buffer_target_ > 400 ||
        mac_buffer_target_ % NUM_JOINTS != 0 ||
        trajectory_preload_points_ < 2 ||
        trajectory_preload_points_ * NUM_JOINTS > mac_buffer_target_ ||
        !std::isfinite(control_frequency_hz) ||
        std::fabs(control_frequency_hz - DEFAULT_CONTROL_FREQUENCY_HZ) > 1e-6)
    {
        ROS_ERROR("[AuboHW] 参数无效：请检查端口、碰撞等级和命令平滑参数");
        return false;
    }
    control_period_s_ = 1.0 / control_frequency_hz;
    ROS_INFO("[AuboHW] 透传节拍 %.1f Hz，轨迹预装 %d 点，MAC 目标 %d",
             control_frequency_hz, trajectory_preload_points_, mac_buffer_target_);

    if (nh_.hasParam("joint_names"))
    {
        nh_.getParam("joint_names", joint_names_);
        if (static_cast<int>(joint_names_.size()) != NUM_JOINTS)
        {
            ROS_ERROR("[AuboHW] joint_names 长度必须为 %d，当前为 %zu",
                      NUM_JOINTS, joint_names_.size());
            return false;
        }
    }

    raw_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("raw_joint_states", 1);
    flow_diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        "flow_diagnostics", 10);
    nh_.param<std::string>("motion_trace_prefix", motion_trace_prefix_, "");
    if (!motion_trace_prefix_.empty())
    {
        int capacity = 120000;
        nh_.param("motion_trace_capacity", capacity, capacity);
        try
        {
            if (capacity <= 0) throw std::invalid_argument("motion_trace_capacity must be positive");
            motion_trace_.reset(new HardwareMotionTrace(motion_trace_prefix_, capacity));
            ROS_INFO("[AuboHW] 逐点/RPC 记录已准备：%s.{commands,rpc}.csv；正常退出后导出", motion_trace_prefix_.c_str());
        }
        catch (const std::exception& error)
        {
            ROS_ERROR("[AuboHW] 无法准备记录：%s", error.what());
            return false;
        }
    }

    // ------------------------------------------------------------------
    // 2. 连接机械臂（带重试，来自 aubo_driver.cpp connectToRobotController）
    // ------------------------------------------------------------------
    if (!connectToRobotController())
    {
        ROS_FATAL("[AuboHW] 无法连接机械臂，节点退出！");
        return false;
    }

    // ------------------------------------------------------------------
    // 3. 若启用上层运动控制，则等待机械臂就绪并切入 TCP2CANBUS
    //    示例教器模式下不接管控制，只保留状态读取
    // ------------------------------------------------------------------
    if (enable_motion_control_)
    {
        if (!waitForRobotReady())
        {
            ROS_ERROR("[AuboHW] 等待机械臂就绪超时！");
            shutdown();
            return false;
        }
        ROS_INFO("[AuboHW] 机械臂已就绪。");

        if (!enterTcp2CanbusMode())
        {
            ROS_ERROR("[AuboHW] 进入 TCP2CANBUS 失败；为避免误发指令，终止启动");
            shutdown();
            return false;
        }
    }
    else
    {
        ROS_INFO("[AuboHW] 已关闭上层运动控制，示教器模式仅保留状态读取。");
    }

    // ------------------------------------------------------------------
    // 5. 读取当前关节角，同步初始化所有缓冲区
    //    （来自 aubo_driver.cpp run() setCurrentPosition / setTagrtPosition）
    // ------------------------------------------------------------------
    int ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(current_waypoint_);
    if (ret == InterfaceCallSuccCode)
    {
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            joint_position_[i]     = current_waypoint_.jointpos[i];
            joint_position_cmd_[i] = current_waypoint_.jointpos[i];
            last_cmd_sent_[i]      = current_waypoint_.jointpos[i];
            filtered_cmd_[i]       = current_waypoint_.jointpos[i];
            joint_filter_[i]       = current_waypoint_.jointpos[i];
            last_state_position_[i] = current_waypoint_.jointpos[i];

            joint_position_rt_[i].store(current_waypoint_.jointpos[i], std::memory_order_relaxed);
            joint_velocity_rt_[i].store(0.0, std::memory_order_relaxed);
            joint_effort_rt_[i].store(0.0, std::memory_order_relaxed);
        }
        ROS_INFO("[AuboHW] 初始关节角已读取，Joint0 = %.4f rad",
                 joint_position_[0]);
        last_state_wall_time_ = ros::WallTime::now();
    }
    else
    {
        ROS_ERROR("[AuboHW] 读取初始关节角失败，错误码: %d；拒绝以零位命令启动", ret);
        shutdown();
        return false;
    }

    // ------------------------------------------------------------------
    // 6. 注册 ros_control 接口
    // ------------------------------------------------------------------
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        hardware_interface::JointStateHandle state_handle(
            joint_names_[i],
            &joint_position_[i],
            &joint_velocity_[i],
            &joint_effort_[i]);
        joint_state_interface_.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(
            joint_state_interface_.getHandle(joint_names_[i]),
            &joint_position_cmd_[i]);
        position_joint_interface_.registerHandle(pos_handle);

    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    // ------------------------------------------------------------------
    // 7. 点动入口用接收线程推送关节反馈；同步诊断只由下方定时器执行。
    //    初始位置已在上面读取；收到真实的新推送之前不启动控制器/喂点。
    // ------------------------------------------------------------------
    if (use_realtime_feedback_)
    {
        accept_feedback_callbacks_.store(true);
        // This connection is idle after startup/EnterTcp2Canbus. Keep pushed
        // feedback off both the diagnosis connection and the MAC send socket.
        const int callback_ret = robot_send_service_.robotServiceRegisterRealTimeRoadPointCallback(
            &AuboHardwareInterface::receiveWaypoint, this);
        feedback_callback_registered_ = callback_ret == InterfaceCallSuccCode;
        if (!feedback_callback_registered_)
        {
            accept_feedback_callbacks_.store(false);
            ROS_ERROR("[AuboHW] 注册实时路点推送失败，错误码: %d", callback_ret);
            shutdown();
            return false;
        }
        std::unique_lock<std::mutex> lock(feedback_mutex_);
        if (!feedback_ready_.wait_for(lock, std::chrono::seconds(2),
                                     [this] { return have_feedback_sample_; }))
        {
            lock.unlock();
            ROS_ERROR("[AuboHW] 2 秒内未收到实时关节推送，拒绝启动控制器");
            shutdown();
            return false;
        }
        // Start all command buffers from the confirmed pushed position too.
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            const double position = joint_position_rt_[i].load(std::memory_order_relaxed);
            joint_position_[i] = joint_position_cmd_[i] = position;
            last_cmd_sent_[i] = filtered_cmd_[i] = joint_filter_[i] = position;
        }
        lock.unlock();
        ROS_INFO("[AuboHW] 实时关节推送已就绪；诊断查询不阻塞反馈发布");
        feedback_poll_running_.store(true);
        feedback_poll_thread_ = std::thread(&AuboHardwareInterface::pollFeedback, this);
        ROS_INFO("[AuboHW] 独立反馈查询已启动：推送间隔达 60 ms 时查询真实路点，保留请求时间戳");
    }

    // 状态定时器：默认读取路点和诊断；推送模式下只读取诊断。
    const int state_frequency_hz = enable_motion_control_ ? TIMER_SPAN_HZ : 1;
    state_timer_ = nh_.createTimer(
        ros::Duration(1.0 / state_frequency_hz),
        &AuboHardwareInterface::timerCallback,
        this);
    state_timer_.start();
    ROS_INFO("[AuboHW] %s定时器已启动（%d Hz）",
             use_realtime_feedback_ ? "安全诊断（关节反馈使用推送）" : "状态读取", state_frequency_hz);

    // ------------------------------------------------------------------
    // 8. 启动喂点线程（仅在启用上层运动控制时才需要）
    // ------------------------------------------------------------------
    if (enable_motion_control_)
    {
        feed_thread_running_.store(true);
        feed_thread_ = new std::thread([this]() { publishWaypointToRobot(); });
        ROS_INFO("[AuboHW] 喂点线程已启动。");
    }

    ROS_INFO("[AuboHW] 硬件接口初始化完成。模式: %s",
             in_tcp2canbus_mode_ ? "TCP2CANBUS 透传" : "仅状态读取");
    return true;
}

// =============================================================================
// read() — 从 timerCallback 填充的状态缓冲区读取
// =============================================================================

void AuboHardwareInterface::read(const ros::Time& /*time*/,
                                 const ros::Duration& /*period*/)
{
    // Copy a coherent six-axis sample. This lock never covers SDK/ROS I/O.
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        joint_position_[i] = joint_position_rt_[i].load(std::memory_order_relaxed);
        joint_velocity_[i] = joint_velocity_rt_[i].load(std::memory_order_relaxed);
        joint_effort_[i]   = joint_effort_rt_[i].load(std::memory_order_relaxed);
    }
}

// =============================================================================
// write() — 将位置命令推入无锁队列（生产端）
//           保留固定周期采样，包括静止保持点
// =============================================================================

void AuboHardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                    const std::list<hardware_interface::ControllerInfo>& /*stop_list*/)
{
    // Latch initial readiness only. JointStateController claims no command
    // interface and must not start a stream of stale pre-controller holds.
    for (const auto& controller : start_list)
        for (const auto& claim : controller.claimed_resources)
            if (claim.hardware_interface == "hardware_interface::PositionJointInterface" &&
                std::all_of(joint_names_.begin(), joint_names_.end(), [&](const std::string& name) {
                    return claim.resources.count(name) != 0;
                }))
                position_controller_started_.store(true, std::memory_order_release);
}

void AuboHardwareInterface::write(const ros::Time& time,
                                  const ros::Duration& period)
{
    if (std::isfinite(period.toSec()) && period.toSec() > 0.0)
        updateAtomicMax(controller_loop_period_max_ms_, period.toSec() * 1000.0);

    if (!controller_connected_flag_.load() || !in_tcp2canbus_mode_)
        return;

    if (!enable_motion_control_)
        return;
    if (!position_controller_started_.load(std::memory_order_acquire))
        return;

    // 若处于急停状态，清空队列不发送（来自 aubo_driver.cpp setRobotJointsByMoveIt）
    if (emergency_stopped_.load() || protective_stopped_.load())
        return;

    std::array<double, NUM_JOINTS> smoothed_cmd = joint_position_cmd_;
    for (double value : smoothed_cmd)
    {
        if (!std::isfinite(value))
        {
            ROS_ERROR("[AuboHW] 收到非有限关节命令，已锁定停止");
            controller_connected_flag_.store(false);
            feed_thread_running_.store(false);
            return;
        }
    }

    if (enable_command_smoothing_)
    {
        // --- 低通：抑制上层控制器的小幅抖动 ---
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            smoothed_cmd[i] = command_filter_alpha_ * joint_position_cmd_[i]
                            + (1.0 - command_filter_alpha_) * filtered_cmd_[i];

            double delta = smoothed_cmd[i] - filtered_cmd_[i];
            double max_step = MAX_VELC[i] * control_period_s_ * max_command_step_scale_;
            if (std::fabs(delta) > max_step)
            {
                smoothed_cmd[i] = filtered_cmd_[i] + (delta > 0.0 ? max_step : -max_step);
            }
        }
    }

    // Every controller tick represents 5 ms, including identical positions,
    // dwells and sub-microradian moves. Deduplication compresses trajectory time
    // and drains the MAC queue near slow starts, reversals and stops.

    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        if (enable_command_smoothing_ &&
            std::fabs(smoothed_cmd[i] - last_cmd_sent_[i]) < command_deadband_)
        {
            smoothed_cmd[i] = last_cmd_sent_[i];
        }
    }

    // --- 推入无锁队列（来自 aubo_driver.cpp moveItPosCallback buf_queue_.push） ---
    const auto command_record_ns = motion_trace_ ? HardwareMotionTrace::ns() : 0;
    const auto command_ros_ns = motion_trace_ ? ros::Time::now().toNSec() : 0;
    std::array<double, NUM_JOINTS> cmd;
    for (int i = 0; i < NUM_JOINTS; ++i)
        cmd[i] = smoothed_cmd[i];

    if (!ros_motion_queue_.try_enqueue(cmd))
    {
        motion_queue_full_count_.fetch_add(1, std::memory_order_relaxed);
        ROS_ERROR("[AuboHW] write(): 运动队列已满，已锁定停止；禁止丢点后继续执行");
        controller_connected_flag_.store(false);
        feed_thread_running_.store(false);
        return;
    }

    last_cmd_sent_ = smoothed_cmd;
    ++command_sequence_;
    if (motion_trace_)
    {
        HardwareMotionTrace::Command event;
        event.id = command_sequence_;
        event.seed = command_sequence_ == 1;
        event.steady_ns = command_record_ns;
        event.ros_ns = command_ros_ns;
        event.sample_ns = time.toNSec();
        event.q = cmd;
        motion_trace_->append(event);
    }
    if (enable_command_smoothing_)
    {
        filtered_cmd_ = smoothed_cmd;
    }
}

// =============================================================================
// shutdown()
// =============================================================================

void AuboHardwareInterface::shutdown()
{
    if (shutdown_started_.exchange(true))
        return;
    accept_feedback_callbacks_.store(false);
    feedback_poll_running_.store(false);
    feed_thread_running_.store(false);
    feedback_ready_.notify_all();
    state_timer_.stop();
    if (feedback_poll_thread_.joinable())
        feedback_poll_thread_.join();

    // 停止喂点线程
    feed_thread_running_.store(false);
    if (feed_thread_)
    {
        if (feed_thread_->joinable())
            feed_thread_->join();
        delete feed_thread_;
        feed_thread_ = nullptr;
    }

    if (feedback_callback_registered_)
    {
        robot_send_service_.robotServiceRegisterRealTimeRoadPointCallback(nullptr, nullptr);
        feedback_callback_registered_ = false;
    }

    // 离开 TCP2CANBUS 模式（来自 aubo_driver.cpp 析构）
    if (enable_motion_control_ && in_tcp2canbus_mode_)
    {
        robot_send_service_.robotServiceLeaveTcp2CanbusMode();
        in_tcp2canbus_mode_ = false;
    }

    // 注销 SDK 连接（独立反馈查询连接仅在实时反馈模式下登录）。
    robot_send_service_.robotServiceLogout();
    robot_receive_service_.robotServiceLogout();
    robot_mac_service_.robotServiceLogout();
    if (feedback_poll_connected_)
    {
        feedback_poll_service_.robotServiceLogout();
        feedback_poll_connected_ = false;
    }
    controller_connected_flag_.store(false);
    ROS_INFO("[AuboHW] SDK 已安全退出。");
    motion_lease_.release();
    // Export only after stream mode/logout; file IO must not delay the cabinet stop.
    if (motion_trace_)
    {
        if (!motion_trace_->dumpAfterJoin())
            ROS_ERROR("[AuboHW] 导出逐点/RPC 记录失败：%s", motion_trace_prefix_.c_str());
        else
            ROS_INFO("[AuboHW] 已导出逐点/RPC 记录：%s.{commands,rpc}.csv", motion_trace_prefix_.c_str());
    }
}

// =============================================================================
// connectToRobotController()
// 来自 aubo_driver.cpp connectToRobotController()
// 核心改进：最多重试 MAX_CONNECT_RETRIES 次 + 双连接登录
// =============================================================================

bool AuboHardwareInterface::connectToRobotController()
{
    if (enable_motion_control_ && !motion_lease_.acquire())
    {
        ROS_ERROR("[AuboHW] 无法取得运动写入锁，请先停止点动/视觉伺服控制节点");
        return false;
    }
    // 从 ROS 参数服务器读取 IP（与 aubo_driver.cpp ros::param::get 一致）
    std::string s;
    if (ros::param::get("/aubo_driver/server_host", s) && !s.empty())
        server_host_ = s;

    ROS_INFO("[AuboHW] 正在连接机械臂 %s:%d ...", server_host_.c_str(), server_port_);

    // --- 带重试的 send 连接（来自 aubo_driver.cpp max_link_times = 5） ---
    int ret1 = -1;
    for (int count = 0; count < MAX_CONNECT_RETRIES; ++count)
    {
        ret1 = robot_send_service_.robotServiceLogin(
            server_host_.c_str(), server_port_,
            username_.c_str(), password_.c_str());
        if (ret1 == InterfaceCallSuccCode)
            break;
        ROS_WARN("[AuboHW] 第 %d 次连接失败（send），错误码: %d", count + 1, ret1);
        usleep(500000);  // 500ms 后重试
    }

    if (ret1 != InterfaceCallSuccCode)
    {
        ROS_ERROR("[AuboHW] 连接机械臂失败（超过最大重试次数）");
        controller_connected_flag_.store(false);
        return false;
    }

    // --- receive 连接（来自 aubo_driver.cpp robot_receive_service_.robotServiceLogin） ---
    int ret2 = robot_receive_service_.robotServiceLogin(
        server_host_.c_str(), server_port_,
        username_.c_str(), password_.c_str());
    if (ret2 != InterfaceCallSuccCode)
    {
        ROS_ERROR("[AuboHW] receive 连接失败，错误码: %d", ret2);
        robot_send_service_.robotServiceLogout();
        return false;
    }

    // Observe the one new socket opened by the MAC login. This is a trace
    // attribution hint, not access to the SDK's private socket layout.
    tcp_transport_.configure(server_host_, server_port_);
    const auto before_mac_login = tcp_transport_.snapshot();
    // --- MAC 缓冲区监控连接（来自 aubo_driver.cpp robot_mac_size_service_） ---
    const int ret3 = robot_mac_service_.robotServiceLogin(
        server_host_.c_str(), server_port_,
        username_.c_str(), password_.c_str());

    if (ret3 != InterfaceCallSuccCode)
    {
        ROS_ERROR("[AuboHW] MAC 连接失败，错误码: %d", ret3);
        robot_send_service_.robotServiceLogout();
        robot_receive_service_.robotServiceLogout();
        return false;
    }
    tcp_transport_.configure(server_host_, server_port_);
    const auto after_mac_login = tcp_transport_.snapshot();
    if (before_mac_login.valid && after_mac_login.valid)
    {
        std::size_t new_ports = 0;
        for (std::size_t i = 0; i < after_mac_login.detail_count; ++i)
        {
            const auto port = after_mac_login.sockets[i].local_port;
            bool existed = false;
            for (std::size_t j = 0; j < before_mac_login.detail_count; ++j)
                existed = existed || before_mac_login.sockets[j].local_port == port;
            if (!existed) { ++new_ports; mac_socket_port_hint_ = port; }
        }
        if (new_ports != 1) mac_socket_port_hint_ = 0;
    }

    if (use_realtime_feedback_)
    {
        const int feedback_ret = feedback_poll_service_.robotServiceLogin(
            server_host_.c_str(), server_port_, username_.c_str(), password_.c_str());
        if (feedback_ret != InterfaceCallSuccCode)
        {
            ROS_ERROR("[AuboHW] 独立反馈查询连接失败，错误码: %d", feedback_ret);
            shutdown();
            return false;
        }
        feedback_poll_connected_ = true;
    }

    if (!tcp_transport_.configure(server_host_, server_port_))
    {
        ROS_ERROR("[AuboHW] SDK TCP 低延迟配置失败，拒绝开始运动流");
        shutdown();
        return false;
    }
    ROS_INFO("[AuboHW] 已为 %zu 个 SDK TCP 连接启用 NODELAY/QUICKACK",
             tcp_transport_.size());
    controller_connected_flag_.store(true);
    ROS_INFO("[AuboHW] 连接成功！");

    // --- 查询是否存在真实机械臂（来自 aubo_driver.cpp） ---
    robot_receive_service_.robotServiceGetIsRealRobotExist(real_robot_exist_);
    ROS_INFO("[AuboHW] 真实机械臂: %s", real_robot_exist_ ? "存在" : "不存在（仿真模式）");
    if (enable_motion_control_ && require_real_robot_ && !real_robot_exist_)
    {
        ROS_ERROR("[AuboHW] 控制器未报告真实机械臂，拒绝进入实机运动模式");
        shutdown();
        return false;
    }

    // --- 机械臂启动（上电 + 松刹车）---
    // 示教器模式下只保留状态读取，不主动唤醒或接管控制权
    if (enable_motion_control_)
    {
        ToolDynamicsParam tool_param;
        memset(&tool_param, 0, sizeof(tool_param));
        ROBOT_SERVICE_STATE startup_result;

        int ret = robot_send_service_.rootServiceRobotStartup(
            tool_param,
            static_cast<uint8>(collision_class_),  // 碰撞等级（来自 aubo_driver.cpp robotControlCallback）
            true,    // readPose
            true,    // staticCollisionDetect
            1000,    // boardBaxAcc
            startup_result);

        if (ret != InterfaceCallSuccCode)
        {
            ROS_ERROR("[AuboHW] 机械臂启动失败，错误码: %d", ret);
            shutdown();
            return false;
        }
        ROS_INFO("[AuboHW] 机械臂启动成功，state = %d",
                 static_cast<int>(startup_result));

        ret = robot_send_service_.robotServiceRobotHandShake(true);
        if (ret != InterfaceCallSuccCode)
        {
            ROS_ERROR("[AuboHW] 机械臂握手失败，错误码: %d", ret);
            shutdown();
            return false;
        }
    }
    else
    {
        ROS_INFO("[AuboHW] 示教器模式：跳过机器人 startup，仅保留状态读取。");
    }

    return true;
}

// =============================================================================
// waitForRobotReady()
// =============================================================================

bool AuboHardwareInterface::waitForRobotReady(int max_attempts)
{
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        RobotDiagnosis diag;
        int ret = robot_receive_service_.robotServiceGetRobotDiagnosisInfo(diag);
        if (ret != InterfaceCallSuccCode)
        {
            ROS_WARN("[AuboHW] 读取诊断信息失败，错误码: %d", ret);
            return false;
        }

        if (diag.armPowerStatus &&
            !diag.softEmergency &&
            !diag.remoteEmergency &&
            !diag.robotCollision)
        {
            return true;
        }

        if (attempt % 20 == 0)
        {
            ROS_INFO("[AuboHW] 等待就绪... 电源=%s 刹车=%s 软急停=%s 远程急停=%s 碰撞=%s",
                     diag.armPowerStatus  ? "ON" : "OFF",
                     diag.brakeStuats     ? "ON" : "OFF",
                     diag.softEmergency   ? "ON" : "OFF",
                     diag.remoteEmergency ? "ON" : "OFF",
                     diag.robotCollision  ? "ON" : "OFF");
        }
        usleep(100000);  // 100 ms
    }
    return false;
}

// =============================================================================
// enterTcp2CanbusMode() / leaveTcp2CanbusMode()
// 来自 aubo_driver.cpp run() / controllerSwitchCallback()
// =============================================================================

bool AuboHardwareInterface::enterTcp2CanbusMode()
{
    int ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();

    if (ret == InterfaceCallSuccCode)
    {
        in_tcp2canbus_mode_ = true;
        ROS_INFO("[AuboHW] 已进入 TCP2CANBUS 透传模式。");
        return true;
    }
    else if (ret == ErrCode_ResponseReturnError)
    {
        // 已在模式中，先离开再重新进入（来自 aubo_driver.cpp run()）
        robot_send_service_.robotServiceLeaveTcp2CanbusMode();
        usleep(200000);
        ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
        if (ret == InterfaceCallSuccCode)
        {
            in_tcp2canbus_mode_ = true;
            ROS_INFO("[AuboHW] 重新进入 TCP2CANBUS 透传模式。");
            return true;
        }
    }

    ROS_WARN("[AuboHW] 进入 TCP2CANBUS 失败，错误码: %d", ret);
    return false;
}

bool AuboHardwareInterface::leaveTcp2CanbusMode()
{
    if (!in_tcp2canbus_mode_)
        return true;

    int ret = robot_send_service_.robotServiceLeaveTcp2CanbusMode();
    in_tcp2canbus_mode_ = false;

    if (ret != InterfaceCallSuccCode)
    {
        ROS_WARN("[AuboHW] 离开 TCP2CANBUS 失败，错误码: %d", ret);
        return false;
    }
    return true;
}

// =============================================================================
// 反馈接收：推送模式只从 SDK 接收线程进入，不等待同步诊断请求。
// =============================================================================

void AuboHardwareInterface::receiveWaypoint(const wayPoint_S* waypoint, void* context)
{
    if (!waypoint || !context)
        return;
    auto* hardware = static_cast<AuboHardwareInterface*>(context);
    if (hardware->accept_feedback_callbacks_.load())
        hardware->updateFeedback(*waypoint);
}

void AuboHardwareInterface::updateFeedback(const wayPoint_S& waypoint)
{
    updateFeedbackAt(waypoint, ros::Time::now(), std::chrono::steady_clock::now(),
                     use_realtime_feedback_);
}

void AuboHardwareInterface::updateFeedbackAt(const wayPoint_S& waypoint,
                                            const ros::Time& stamp,
                                            std::chrono::steady_clock::time_point sampled,
                                            bool pushed)
{
    for (int i = 0; i < NUM_JOINTS; ++i)
        if (!std::isfinite(waypoint.jointpos[i]))
            return;
    sensor_msgs::JointState sample;
    sample.header.stamp = stamp;
    sample.name = joint_names_;
    sample.position.assign(waypoint.jointpos, waypoint.jointpos + NUM_JOINTS);
    // Two independent sources may finish out of order. Serialize acceptance
    // and publication without holding the short six-axis read() mutex for IO.
    std::lock_guard<std::mutex> publish_lock(feedback_publish_mutex_);
    double push_gap = 0.0;
    std::uint64_t count = 0;
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        if (shutdown_started_.load() || !controller_connected_flag_.load())
            return;
        if (have_feedback_sample_ && sampled <= last_feedback_sample_)
            return;
        const double dt = have_feedback_sample_
            ? std::chrono::duration<double>(sampled - last_feedback_sample_).count() : 0.0;
        if (pushed)
        {
            if (last_push_sample_ != std::chrono::steady_clock::time_point{})
                push_gap = std::chrono::duration<double>(sampled - last_push_sample_).count();
            last_push_sample_ = sampled;
        }
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            const double velocity = dt > 1e-6 ? (waypoint.jointpos[i] - last_state_position_[i]) / dt : 0.0;
            joint_position_rt_[i].store(waypoint.jointpos[i], std::memory_order_relaxed);
            joint_velocity_rt_[i].store(velocity, std::memory_order_relaxed);
            last_state_position_[i] = waypoint.jointpos[i];
            sample.velocity.push_back(velocity);
        }
        last_state_wall_time_ = ros::WallTime::now();
        last_feedback_sample_ = sampled;
        count = ++feedback_sample_count_;
        have_feedback_sample_ = true;
    }
    raw_joint_state_pub_.publish(sample);
    feedback_ready_.notify_all();
    if (push_gap > 0.08)
        ROS_WARN_THROTTLE(1.0, "[AuboHW/feedback] SDK 推送间隔 %.2f ms，独立查询仍可补充真实采样；累计 %llu 帧",
                          push_gap * 1000.0, static_cast<unsigned long long>(count));
}

void AuboHardwareInterface::pollFeedback()
{
    using Clock = std::chrono::steady_clock;
    while (feedback_poll_running_.load())
    {
        {
            std::unique_lock<std::mutex> lock(feedback_mutex_);
            feedback_ready_.wait_for(lock, std::chrono::milliseconds(5),
                                     [this] { return !feedback_poll_running_.load(); });
            if (!feedback_poll_running_.load())
                break;
            if (have_feedback_sample_ && Clock::now() - last_feedback_sample_ < std::chrono::milliseconds(60))
                continue;
        }
        const auto requested = Clock::now();
        const ros::Time stamp = ros::Time::now();
        wayPoint_S point{};
        const int ret = readFeedbackSnapshot(point);
        if (ret == InterfaceCallSuccCode)
            // The controller may have sampled at any point during this RPC.
            // Use the request time conservatively; never freshen a late reply.
            updateFeedbackAt(point, stamp, requested, false);
        else
            ROS_WARN_THROTTLE(1.0, "[AuboHW/feedback] 独立路点查询失败 %d；不刷新反馈时间", ret);
        const double duration_ms = std::chrono::duration<double, std::milli>(Clock::now() - requested).count();
        if (duration_ms > 60.0)
            ROS_WARN_THROTTLE(1.0, "[AuboHW/feedback] 独立路点查询耗时 %.2f ms，仍使用请求时间戳", duration_ms);
    }
}

int AuboHardwareInterface::readFeedbackSnapshot(wayPoint_S& point)
{
    return feedback_poll_service_.robotServiceGetCurrentWaypointInfo(point);
}

// =============================================================================
// timerCallback (50 Hz)
// 来自 aubo_driver.cpp timerCallback()
// 负责：① 读取关节角  ② 读取诊断信息  ③ 断线重连
// =============================================================================

void AuboHardwareInterface::timerCallback(const ros::TimerEvent& /*e*/)
{
    if (!controller_connected_flag_.load())
        return;

    int ret = InterfaceCallSuccCode;
    if (!use_realtime_feedback_)
    {
        ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(current_waypoint_);
        if (ret == InterfaceCallSuccCode)
            updateFeedback(current_waypoint_);
        else
        {
            ROS_ERROR("[AuboHW] 读取路点失败，错误码: %d；已锁定停止", ret);
            controller_connected_flag_.store(false);
            feed_thread_running_.store(false);
            return;
        }
    }

    // 示教器模式下只做关节位姿轮询，避免高频诊断查询放大底层 SDK 噪声。
    if (!enable_motion_control_)
        return;

    // --- 读取诊断信息，更新急停/保护停止标志（来自 aubo_driver.cpp timerCallback） ---
    ret = readSafetyDiagnosis(robot_diagnosis_info_);
    if (ret == InterfaceCallSuccCode && real_robot_exist_)
    {
        // 安全 IO 急停检测（来自 aubo_driver.cpp publishIOMsg digitalIn[0/8]）
        // 简化版：直接使用 softEmergency / remoteEmergency
        emergency_stopped_.store(robot_diagnosis_info_.softEmergency ||
                                 robot_diagnosis_info_.remoteEmergency);
        protective_stopped_.store(robot_diagnosis_info_.robotCollision);
        if (emergency_stopped_.load() || protective_stopped_.load())
        {
            ROS_ERROR("[AuboHW] 检测到急停或碰撞保护，已锁定停止；检查后重启节点");
            controller_connected_flag_.store(false);
            feed_thread_running_.store(false);
        }
    }
    else if (ret != InterfaceCallSuccCode)
    {
        ROS_ERROR("[AuboHW] 读取安全诊断失败，错误码: %d；已锁定停止", ret);
        controller_connected_flag_.store(false);
        feed_thread_running_.store(false);
    }
}

int AuboHardwareInterface::readSafetyDiagnosis(RobotDiagnosis& diagnosis)
{
    return robot_receive_service_.robotServiceGetRobotDiagnosisInfo(diagnosis);
}

int AuboHardwareInterface::readMacDiagnosis(RobotDiagnosis& diagnosis)
{
    if (!tcp_transport_.prepare())
        return -1;
    return robot_mac_service_.robotServiceGetRobotDiagnosisInfo(diagnosis);
}

int AuboHardwareInterface::sendWaypoints(const std::vector<wayPoint_S>& points)
{
    if (!tcp_transport_.prepare())
        return -1;
    return robot_mac_service_.robotServiceSetRobotPosData2Canbus(points);
}

// =============================================================================
// publishWaypointToRobot()
// 来自 aubo_driver.cpp publishWaypointToRobot()
// 核心：持续监控 MAC 缓冲区大小，按需批量补充轨迹点
// =============================================================================

void AuboHardwareInterface::publishWaypointToRobot()
{
    using SteadyClock = std::chrono::steady_clock;
    std::vector<wayPoint_S> waypoint_vector;
    int current_macsz = 0;
    int cnt = 0;
    bool stream_started = false;
    bool mac_confirmed = false;
    bool first_send_accepted = false;
    SteadyClock::time_point first_send_completed{};
    std::size_t interrupt_warning_samples = 0;
    std::size_t active_empty_samples = 0;
    bool stream_fault = false;
    bool fault_reported = false;
    std::uint64_t query_sequence = 0, send_sequence = 0, submitted_points = 0;
    auto previous_loop_start = SteadyClock::now();
    auto next_mac_query = previous_loop_start;
    auto next_report = previous_loop_start + std::chrono::milliseconds(100);
    double max_feed_loop_ms = 0.0;
    double max_diagnosis_ms = 0.0;
    double max_send_ms = 0.0;
    int min_mac_window = std::numeric_limits<int>::max();
    int min_active_mac_window = std::numeric_limits<int>::max();
    int max_mac_window = 0;
    std::size_t max_queue_window = 0;
    std::size_t query_count = 0, diagnosis_failures = 0;
    std::size_t send_count = 0, send_points = 0, send_failures = 0;
    std::size_t low_water_samples = 0, empty_with_pending_samples = 0;
    std::size_t above_target_samples = 0;
    int last_diagnosis_result = InterfaceCallSuccCode;
    int last_send_result = InterfaceCallSuccCode;
    std::uint64_t last_queue_full_count =
        motion_queue_full_count_.load(std::memory_order_relaxed);
    std::uint64_t last_tcp_retransmissions = tcp_transport_.snapshot().total_retransmissions;
    std::size_t small_batch_wait_samples = 0;
    const int minimum_batch_points = std::min(4, std::max(1, mac_buffer_target_ / NUM_JOINTS / 2));

    const auto publish_flow_window = [&](bool force)
    {
        const auto now = SteadyClock::now();
        if (!force && now < next_report)
            return;

        const std::uint64_t queue_full_total =
            motion_queue_full_count_.load(std::memory_order_relaxed);
        const std::uint64_t queue_full_window = queue_full_total - last_queue_full_count;
        last_queue_full_count = queue_full_total;

        diagnostic_msgs::DiagnosticArray array;
        array.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "Aubo hardware motion flow";
        status.hardware_id = server_host_;
        const bool rpc_error = diagnosis_failures > 0 || send_failures > 0 || stream_fault;
        fault_reported = rpc_error;
        const double mac_target_coverage_ms =
            mac_buffer_target_ / static_cast<double>(NUM_JOINTS) * control_period_s_ * 1000.0;
        const std::size_t mac_target_points =
            static_cast<std::size_t>(mac_buffer_target_ / NUM_JOINTS);
        const bool feed_loop_exceeded_coverage = stream_started &&
            max_feed_loop_ms > mac_target_coverage_ms;
        const bool queue_backlog = stream_started && max_queue_window > mac_target_points;
        const bool flow_warning = low_water_samples > 0 ||
            empty_with_pending_samples > 0 || above_target_samples > 0 || queue_full_window > 0 ||
            feed_loop_exceeded_coverage || queue_backlog;
        status.level = rpc_error ? diagnostic_msgs::DiagnosticStatus::ERROR :
            (flow_warning ? diagnostic_msgs::DiagnosticStatus::WARN :
                            diagnostic_msgs::DiagnosticStatus::OK);
        status.message = rpc_error ? "SDK or continuous-stream fault (restart required)" :
            (empty_with_pending_samples > 0 ? "MAC empty while commands were pending" :
             (low_water_samples > 0 ? "MAC low-water samples observed" :
              (feed_loop_exceeded_coverage ? "feed loop slower than MAC target coverage" :
               (queue_backlog ? "ROS motion queue backlog exceeds MAC target" :
                (above_target_samples > 0 ? "MAC occupancy exceeded refill target" :
                 (queue_full_window > 0 ? "ROS motion queue overflow" : "flow nominal"))))));

        const std::size_t queue_current = ros_motion_queue_.size_approx();
        const double control_loop_max = controller_loop_period_max_ms_.exchange(
            0.0, std::memory_order_relaxed);
        const auto add = [&](const std::string& key, const std::string& value)
        {
            addDiagnosticValue(status, key, value);
        };
        add("stream_active", stream_started ? "true" : "false");
        add("controller_connected", controller_connected_flag_.load() ? "true" : "false");
        add("tcp2canbus_active", in_tcp2canbus_mode_ ? "true" : "false");
        add("mac_current_scalars", std::to_string(current_macsz));
        add("mac_current_points", std::to_string(current_macsz / NUM_JOINTS));
        add("mac_target_scalars", std::to_string(mac_buffer_target_));
        add("mac_target_points", std::to_string(mac_target_points));
        add("mac_target_coverage_ms", std::to_string(mac_target_coverage_ms));
        add("mac_window_min_scalars", min_mac_window == std::numeric_limits<int>::max() ?
            "-1" : std::to_string(min_mac_window));
        add("mac_active_window_min_scalars",
            min_active_mac_window == std::numeric_limits<int>::max() ?
            "-1" : std::to_string(min_active_mac_window));
        add("mac_window_max_scalars", std::to_string(max_mac_window));
        add("mac_low_water_samples_below_5_points", std::to_string(low_water_samples));
        add("mac_empty_with_pending_queue_samples", std::to_string(empty_with_pending_samples));
        add("mac_active_empty_samples", std::to_string(active_empty_samples));
        add("mac_data_interrupt_warning_samples", std::to_string(interrupt_warning_samples));
        add("stream_fault_latched", stream_fault ? "true" : "false");
        add("mac_above_target_samples", std::to_string(above_target_samples));
        add("queue_current_points", std::to_string(queue_current));
        add("queue_window_max_points", std::to_string(max_queue_window));
        add("queue_backlog_warning_points", std::to_string(mac_target_points));
        add("queue_window_max_nominal_horizon_ms", std::to_string(
            max_queue_window * control_period_s_ * 1000.0));
        add("queue_full_total", std::to_string(queue_full_total));
        add("queue_full_window", std::to_string(queue_full_window));
        add("query_count", std::to_string(query_count));
        add("diagnosis_failures", std::to_string(diagnosis_failures));
        add("last_diagnosis_result", std::to_string(last_diagnosis_result));
        add("diagnosis_max_ms", std::to_string(max_diagnosis_ms));
        add("send_count", std::to_string(send_count));
        add("send_points", std::to_string(send_points));
        add("send_failures", std::to_string(send_failures));
        add("last_send_result", std::to_string(last_send_result));
        add("send_max_ms", std::to_string(max_send_ms));
        add("feed_loop_max_ms", std::to_string(max_feed_loop_ms));
        add("controller_loop_period_max_ms", std::to_string(control_loop_max));
        add("command_velocity_max_rad_s", std::to_string(command_velocity_max_.exchange(0.0)));
        add("command_acceleration_max_rad_s2", std::to_string(command_acceleration_max_.exchange(0.0)));
        add("command_jerk_max_rad_s3", std::to_string(command_jerk_max_.exchange(0.0)));
        add("trajectory_preload_points", std::to_string(trajectory_preload_points_));
        add("position_controller_started", position_controller_started_.load() ? "true" : "false");
        add("startup_reference_seeded", startup_reference_seeded_.load() ? "true" : "false");
        add("motion_trace_prefix", motion_trace_prefix_);
        add("mac_socket_local_port_hint", std::to_string(mac_socket_port_hint_));
        add("minimum_send_batch_points", std::to_string(minimum_batch_points));
        add("small_batch_wait_samples", std::to_string(small_batch_wait_samples));
        add("minimum_mac_query_interval_ms", "5");
        const auto tcp = tcp_transport_.snapshot();
        add("tcp_info_valid", tcp.valid ? "true" : "false");
        if (tcp.valid)
        {
            add("tcp_connections", std::to_string(tcp.connections));
            add("tcp_total_retransmissions", std::to_string(tcp.total_retransmissions));
            add("tcp_retransmissions_window", std::to_string(tcp.total_retransmissions >= last_tcp_retransmissions ?
                tcp.total_retransmissions - last_tcp_retransmissions : 0));
            add("tcp_max_rtt_ms", std::to_string(tcp.max_rtt_us * 0.001));
            add("tcp_max_rto_ms", std::to_string(tcp.max_rto_us * 0.001));
            add("tcp_unacknowledged_segments", std::to_string(tcp.unacknowledged_segments));
            last_tcp_retransmissions = tcp.total_retransmissions;
        }
        status.message += "; interval: queries=" + std::to_string(query_count) +
            ", sends=" + std::to_string(send_count) +
            ", max_query_ms=" + std::to_string(max_diagnosis_ms) +
            ", max_send_ms=" + std::to_string(max_send_ms);
        array.status.push_back(status);
        flow_diagnostics_pub_.publish(array);

        max_feed_loop_ms = max_diagnosis_ms = max_send_ms = 0.0;
        min_mac_window = std::numeric_limits<int>::max();
        min_active_mac_window = std::numeric_limits<int>::max();
        max_mac_window = 0;
        max_queue_window = 0;
        query_count = diagnosis_failures = send_count = send_points = send_failures = 0;
        low_water_samples = empty_with_pending_samples = above_target_samples = 0;
        active_empty_samples = interrupt_warning_samples = 0;
        small_batch_wait_samples = 0;
        last_diagnosis_result = last_send_result = InterfaceCallSuccCode;
        next_report = now + std::chrono::milliseconds(100);
    };

    while (feed_thread_running_.load() && controller_connected_flag_.load())
    {
        // Limit polling pressure, never infer buffer space from elapsed time.
        // Slow RPCs already consume this interval: no extra 5ms sleep afterward.
        std::this_thread::sleep_until(next_mac_query);
        if (!feed_thread_running_.load() || !controller_connected_flag_.load()) break;
        const bool stream_was_active = stream_started;
        const auto loop_start = SteadyClock::now();
        next_mac_query = loop_start + std::chrono::milliseconds(5);
        max_feed_loop_ms = std::max(max_feed_loop_ms,
            std::chrono::duration<double, std::milli>(loop_start - previous_loop_start).count());
        previous_loop_start = loop_start;

        RobotDiagnosis mac_diagnosis{};
        // --- 读取 MAC 缓冲区大小（来自 aubo_driver.cpp robot_mac_size_service_） ---
        HardwareMotionTrace::Rpc query_event;
        if (motion_trace_)
        {
            query_event.id = ++query_sequence;
            query_event.port_hint = mac_socket_port_hint_;
            query_event.before = tcp_transport_.snapshot();
            query_event.ros_ns = ros::Time::now().toNSec();
            query_event.begin_ns = HardwareMotionTrace::ns();
        }
        const auto diagnosis_start = SteadyClock::now();
        const int diagnosis_ret = readMacDiagnosis(mac_diagnosis);
        const auto diagnosis_end = SteadyClock::now();
        if (motion_trace_)
        {
            query_event.end_ns = HardwareMotionTrace::ns();
            query_event.after = tcp_transport_.snapshot();
            query_event.result = diagnosis_ret;
            query_event.mac_scalars = mac_diagnosis.macTargetPosDataSize;
            motion_trace_->append(query_event);
        }
        const double diagnosis_ms = std::chrono::duration<double, std::milli>(
            diagnosis_end - diagnosis_start).count();
        ++query_count;
        max_diagnosis_ms = std::max(max_diagnosis_ms, diagnosis_ms);
        last_diagnosis_result = diagnosis_ret;
        const bool diagnosis_ok = diagnosis_ret == InterfaceCallSuccCode;
        if (diagnosis_ok)
        {
            rib_buffer_size_ = mac_diagnosis.macTargetPosDataSize;
            current_macsz    = rib_buffer_size_;

        }
        else
        {
            ++diagnosis_failures;
            ROS_ERROR("[AuboHW] 读取 MAC 缓冲状态失败，错误码: %d；已锁定停止", diagnosis_ret);
            controller_connected_flag_.store(false);
            feed_thread_running_.store(false);
            publish_flow_window(true);
        }

        if (!diagnosis_ok)
            continue;

        const std::size_t queued_points = ros_motion_queue_.size_approx();
        min_mac_window = std::min(min_mac_window, current_macsz);
        max_mac_window = std::max(max_mac_window, current_macsz);
        if (stream_was_active)
            min_active_mac_window = std::min(min_active_mac_window, current_macsz);
        max_queue_window = std::max(max_queue_window, queued_points);
        if (stream_was_active && queued_points > 0 &&
            current_macsz < 5 * NUM_JOINTS)
            ++low_water_samples;
        if (stream_was_active && current_macsz == 0 && queued_points > 0)
            ++empty_with_pending_samples;
        if (current_macsz > mac_buffer_target_ + NUM_JOINTS - 1)
            ++above_target_samples;

        if (stream_was_active && current_macsz > 0)
            mac_confirmed = true;
        if (stream_was_active && current_macsz == 0 && mac_confirmed)
            ++active_empty_samples;
        if (mac_diagnosis.macDataInterruptWarning)
            ++interrupt_warning_samples;
        // This stream includes stationary holds and has no planned empty tail.
        // Never resume a confirmed empty stream by sending old queued motion.
        if (mac_diagnosis.macDataInterruptWarning ||
            (stream_was_active && current_macsz == 0 && mac_confirmed) ||
            mac_diagnosis.softEmergency || mac_diagnosis.remoteEmergency ||
            mac_diagnosis.robotCollision)
        {
            stream_fault = true;
            ROS_ERROR("[AuboHW] 连续流/控制柜保护故障：MAC=%d，数据中断=%u；已锁定停止，禁止自动续发",
                      current_macsz, static_cast<unsigned>(mac_diagnosis.macDataInterruptWarning));
            controller_connected_flag_.store(false);
            feed_thread_running_.store(false);
            publish_flow_window(true);
            break;
        }

        // A cached startup zero must not authorize another full preload after
        // the first successful send. Wait for a nonzero cabinet observation;
        // elapsed time is a timeout, never evidence of free buffer space.
        if (first_send_accepted && !mac_confirmed && current_macsz == 0)
        {
            const double waiting = std::chrono::duration<double>(
                SteadyClock::now() - first_send_completed).count();
            if (waiting >= mac_buffer_target_ / static_cast<double>(NUM_JOINTS) * control_period_s_)
            {
                stream_fault = true;
                ROS_ERROR("[AuboHW] 首批发送后未确认非零 MAC 水位，已锁定停止；禁止按缓存零值重复预装");
                controller_connected_flag_.store(false);
                feed_thread_running_.store(false);
                publish_flow_window(true);
                break;
            }
            publish_flow_window(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(FEED_THREAD_SLEEP_MS));
            continue;
        }

        // 旧驱动先生成一段固定 5 ms 路点再启动控制柜。标准
        // JointTrajectoryController 是逐周期产生点，若立即发送，MAC 会反复
        // 归零；原有的零缓冲 10 ms sleep 会进一步形成周期性停顿。
        if (!stream_started)
        {
            if (queued_points < static_cast<std::size_t>(trajectory_preload_points_))
            {
                publish_flow_window(false);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(FEED_THREAD_SLEEP_MS));
                continue;
            }
            stream_started = true;
            ROS_INFO("[AuboHW] 轨迹已预装 %zu 点，开始连续透传", queued_points);
        }

        // --- 当缓冲区未满且队列有数据时批量补充 ---
        if (diagnosis_ok &&
            feed_thread_running_.load() &&
            stream_started &&
            current_macsz < mac_buffer_target_ &&
            queued_points > 0 &&
            controller_connected_flag_.load() &&
            in_tcp2canbus_mode_ &&
            !emergency_stopped_.load() &&
            !protective_stopped_.load())
        {
            // Partial points consume a complete six-axis slot; never round
            // available capacity up and exceed the configured refill target.
            cnt = (mac_buffer_target_ - current_macsz) / NUM_JOINTS;
            // Amortize synchronous sends over >=20ms of samples at the default
            // target. Keep all points; wait for both observed capacity and data.
            if (cnt < minimum_batch_points || queued_points < static_cast<std::size_t>(minimum_batch_points))
                ++small_batch_wait_samples;
            else
                waypoint_vector = tryPopWaypoint(cnt);
            if (!controller_connected_flag_.load())
                stream_fault = true;

            if (!waypoint_vector.empty() && feed_thread_running_.load() &&
                controller_connected_flag_.load())
            {
                HardwareMotionTrace::Rpc send_event;
                if (motion_trace_)
                {
                    send_event.send = true;
                    send_event.id = ++send_sequence;
                    send_event.port_hint = mac_socket_port_hint_;
                    send_event.first_point_id = submitted_points + 1;
                    send_event.count = waypoint_vector.size();
                    send_event.mac_scalars = current_macsz;
                    send_event.before = tcp_transport_.snapshot();
                    send_event.ros_ns = ros::Time::now().toNSec();
                    send_event.begin_ns = HardwareMotionTrace::ns();
                }
                const auto send_start = SteadyClock::now();
                const int send_ret = sendWaypoints(waypoint_vector);
                const auto send_end = SteadyClock::now();
                if (motion_trace_)
                {
                    send_event.end_ns = HardwareMotionTrace::ns();
                    send_event.after = tcp_transport_.snapshot();
                    send_event.result = send_ret;
                    motion_trace_->append(send_event);
                }
                submitted_points += waypoint_vector.size();
                const double send_ms = std::chrono::duration<double, std::milli>(
                    send_end - send_start).count();
                ++send_count;
                send_points += waypoint_vector.size();
                max_send_ms = std::max(max_send_ms, send_ms);
                last_send_result = send_ret;
                if (send_ret == InterfaceCallSuccCode && !first_send_accepted)
                {
                    first_send_accepted = true;
                    first_send_completed = SteadyClock::now();
                }
                if (send_ret != InterfaceCallSuccCode)
                {
                    ++send_failures;
                    ROS_ERROR("[AuboHW] 轨迹点发送失败，错误码: %d；已停止发送", send_ret);
                    controller_connected_flag_.store(false);
                    feed_thread_running_.store(false);
                }
            }
            waypoint_vector.clear();
        }

        publish_flow_window(!feed_thread_running_.load());

        // 1 ms 检查周期，给 5 ms 硬件消费周期留出调度余量。
        std::this_thread::sleep_for(std::chrono::milliseconds(FEED_THREAD_SLEEP_MS));
    }
    // A producer-side queue overflow or safety callback can stop this thread
    // between iterations. Preserve its final counters even without another RPC.
    if (!controller_connected_flag_.load() && !fault_reported)
    {
        stream_fault = true;
        publish_flow_window(true);
    }
}

// =============================================================================
// tryPopWaypoint()
// 来自 aubo_driver.cpp tryPopWaypoint()
// 核心：保留固定采样时间；速度/加速度越限时停止，不在线改写轨迹
// =============================================================================

std::vector<wayPoint_S> AuboHardwareInterface::tryPopWaypoint(int count)
{
    std::vector<wayPoint_S> points;
    points.reserve(std::max(0, count));
    std::array<double, NUM_JOINTS> joint;
    for (int k = 0; k < count; ++k)
    {
        if (!ros_motion_queue_.try_dequeue(joint))
            break;
        if (!stream_reference_initialized_)
        {
            // This is the controller's first hold after its starting() hook,
            // not the older SDK-login snapshot. Never hide a large mismatch.
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            for (int i = 0; i < NUM_JOINTS; ++i)
            {
                const double actual = joint_position_rt_[i].load();
                if (!std::isfinite(joint[i]) || !std::isfinite(actual) || std::fabs(joint[i] - actual) > 0.001)
                {
                    ROS_ERROR("[AuboHW] 首个控制器命令与反馈不一致（Joint %d），已锁定停止", i);
                    controller_connected_flag_.store(false);
                    feed_thread_running_.store(false);
                    return {};
                }
            }
            joint_filter_ = joint;
            std::fill(std::begin(last_joint_velc_.jointPara), std::end(last_joint_velc_.jointPara), 0.0);
            last_joint_acceleration_.fill(0.0);
            stream_reference_initialized_ = true;
            startup_reference_seeded_.store(true);
        }
        std::array<double, NUM_JOINTS> velocity{}, acceleration{};
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            // Keep the sign: abs(v_new)-abs(v_old) misses direction reversals.
            velocity[i] = (joint[i] - joint_filter_[i]) / control_period_s_;
            acceleration[i] = (velocity[i] - last_joint_velc_.jointPara[i]) / control_period_s_;
            const double jerk = (acceleration[i] - last_joint_acceleration_[i]) / control_period_s_;
            updateAtomicMax(command_velocity_max_, std::fabs(velocity[i]));
            updateAtomicMax(command_acceleration_max_, std::fabs(acceleration[i]));
            updateAtomicMax(command_jerk_max_, std::fabs(jerk));
            if (!std::isfinite(joint[i]) || std::fabs(velocity[i]) > MAX_VELC[i] ||
                std::fabs(acceleration[i]) > MAX_ACC[i])
            {
                ROS_ERROR("[AuboHW] Joint %d 轨迹点不连续/越限：v=%.4f rad/s，a=%.4f rad/s²；"
                          "已锁定停止，不在线插点或压缩时间", i, velocity[i], acceleration[i]);
                controller_connected_flag_.store(false);
                feed_thread_running_.store(false);
                return {};  // Reject the complete batch; no partial trajectory rewrite.
            }
        }
        wayPoint_S point{};
        std::copy(joint.begin(), joint.end(), point.jointpos);
        points.push_back(point);
        for (int i = 0; i < NUM_JOINTS; ++i)
            last_joint_velc_.jointPara[i] = velocity[i];
        last_joint_acceleration_ = acceleration;
        joint_filter_ = joint;
    }
    return points;
}

}  // namespace aubo_ros_control
