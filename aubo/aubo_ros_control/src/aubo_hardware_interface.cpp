/**
 * @file aubo_hardware_interface.cpp
 * @brief AUBO 机器人 ros_control 硬件接口实现（参照 aubo_driver.cpp 优化版）
 *
 * 关键设计（来自 aubo_driver.cpp）：
 *  1. connectToRobotController(): 最多重试5次 + 双连接（send/receive分离）
 *  2. timerCallback (50 Hz): 使用 robot_receive_service_ 轮询关节角（GetCurrentWaypointInfo）
 *  3. write() → ros_motion_queue_.enqueue()  （无锁生产）
 *  4. publishWaypointToRobot(): 监控 MAC 缓冲区，按需批量补充点位
 *  5. tryPopWaypoint(): 速度/加速度限幅 + 超速等分插值
 *  6. roadPointCompare(): THRESHHOLD = 0.000001 rad 去重
 */

#include "aubo_ros_control/aubo_hardware_interface.h"

#include <cstring>
#include <cmath>
#include <algorithm>
#include <unistd.h>   // usleep

using namespace aubo_robot_namespace;

namespace aubo_ros_control
{

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
    , over_speed_flag_(false)
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
    , enable_command_smoothing_(true)
    , command_filter_alpha_(0.25)
    , command_deadband_(0.0005)
    , max_command_step_scale_(1.0)
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

    memset(&target_joint_velc_, 0, sizeof(target_joint_velc_));
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
    nh_.param<bool>       ("enable_command_smoothing", enable_command_smoothing_, true);
    nh_.param<double>     ("command_filter_alpha", command_filter_alpha_, 0.25);
    nh_.param<double>     ("command_deadband", command_deadband_, 0.0005);
    nh_.param<double>     ("max_command_step_scale", max_command_step_scale_, 1.0);

    if (server_port_ <= 0 || server_port_ > 65535 || collision_class_ < 1 || collision_class_ > 6 ||
        command_filter_alpha_ <= 0.0 || command_filter_alpha_ > 1.0 ||
        command_deadband_ < 0.0 || max_command_step_scale_ <= 0.0 || max_command_step_scale_ > 1.0)
    {
        ROS_ERROR("[AuboHW] 参数无效：请检查端口、碰撞等级和命令平滑参数");
        return false;
    }

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
    // 7. 启动状态读取定时器（来自 aubo_driver.cpp TIMER_SPAN_ = 50 Hz）
    // ------------------------------------------------------------------
    const int state_frequency_hz = enable_motion_control_ ? TIMER_SPAN_HZ : 1;
    state_timer_ = nh_.createTimer(
        ros::Duration(1.0 / state_frequency_hz),
        &AuboHardwareInterface::timerCallback,
        this);
    state_timer_.start();
    ROS_INFO("[AuboHW] 状态读取定时器已启动（%d Hz）", state_frequency_hz);

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
    // 从 lock-free 原子前台缓冲区安全读取硬件状态
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        joint_position_[i] = joint_position_rt_[i].load(std::memory_order_relaxed);
        joint_velocity_[i] = joint_velocity_rt_[i].load(std::memory_order_relaxed);
        joint_effort_[i]   = joint_effort_rt_[i].load(std::memory_order_relaxed);
    }
}

// =============================================================================
// write() — 将位置命令推入无锁队列（生产端）
//           对应 aubo_driver.cpp moveItPosCallback + roadPointCompare
// =============================================================================

void AuboHardwareInterface::write(const ros::Time& /*time*/,
                                  const ros::Duration& /*period*/)
{
    if (!controller_connected_flag_.load() || !in_tcp2canbus_mode_)
        return;

    if (!enable_motion_control_)
        return;

    // 若处于急停状态，清空队列不发送（来自 aubo_driver.cpp setRobotJointsByMoveIt）
    if (emergency_stopped_.load() || protective_stopped_.load())
        return;

    std::array<double, NUM_JOINTS> smoothed_cmd = joint_position_cmd_;
    for (double value : smoothed_cmd)
    {
        if (!std::isfinite(value))
        {
            ROS_ERROR_THROTTLE(1.0, "[AuboHW] 收到非有限关节命令，已拒绝");
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
            double max_step = MAX_VELC[i] * CTRL_PERIOD_S * max_command_step_scale_;
            if (std::fabs(delta) > max_step)
            {
                smoothed_cmd[i] = filtered_cmd_[i] + (delta > 0.0 ? max_step : -max_step);
            }
        }
    }

    // --- 点位去重（来自 aubo_driver.cpp roadPointCompare + THRESHHOLD） ---
    if (!roadPointCompare(smoothed_cmd.data(), last_cmd_sent_.data()))
        return;

    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        if (enable_command_smoothing_ &&
            std::fabs(smoothed_cmd[i] - last_cmd_sent_[i]) < command_deadband_)
        {
            smoothed_cmd[i] = last_cmd_sent_[i];
        }
    }

    // --- 推入无锁队列（来自 aubo_driver.cpp moveItPosCallback buf_queue_.push） ---
    std::array<double, NUM_JOINTS> cmd;
    for (int i = 0; i < NUM_JOINTS; ++i)
        cmd[i] = smoothed_cmd[i];

    if (!ros_motion_queue_.try_enqueue(cmd))
    {
        ROS_WARN_THROTTLE(1.0, "[AuboHW] write(): 运动队列已满，丢弃该点");
        return;
    }

    last_cmd_sent_ = smoothed_cmd;
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

    // 停止喂点线程
    feed_thread_running_.store(false);
    if (feed_thread_)
    {
        if (feed_thread_->joinable())
            feed_thread_->join();
        delete feed_thread_;
        feed_thread_ = nullptr;
    }

    // 停止定时器
    state_timer_.stop();

    // 离开 TCP2CANBUS 模式（来自 aubo_driver.cpp 析构）
    if (enable_motion_control_ && in_tcp2canbus_mode_)
    {
        robot_send_service_.robotServiceLeaveTcp2CanbusMode();
        in_tcp2canbus_mode_ = false;
    }

    // 注销三个 SDK 连接（来自 aubo_driver.cpp 析构）
    robot_send_service_.robotServiceLogout();
    robot_receive_service_.robotServiceLogout();
    robot_mac_service_.robotServiceLogout();
    controller_connected_flag_.store(false);
    ROS_INFO("[AuboHW] SDK 已安全退出。");
}

// =============================================================================
// connectToRobotController()
// 来自 aubo_driver.cpp connectToRobotController()
// 核心改进：最多重试 MAX_CONNECT_RETRIES 次 + 双连接登录
// =============================================================================

bool AuboHardwareInterface::connectToRobotController()
{
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
// timerCallback (50 Hz)
// 来自 aubo_driver.cpp timerCallback()
// 负责：① 读取关节角  ② 读取诊断信息  ③ 断线重连
// =============================================================================

void AuboHardwareInterface::timerCallback(const ros::TimerEvent& /*e*/)
{
    if (!controller_connected_flag_.load())
        return;

    // --- 读取当前路点（来自 aubo_driver.cpp timerCallback GetCurrentWaypointInfo） ---
    int ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(current_waypoint_);

    if (ret == InterfaceCallSuccCode)
    {
        const ros::WallTime now = ros::WallTime::now();
        const double dt = last_state_wall_time_.isZero()
            ? 0.0 : (now - last_state_wall_time_).toSec();
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            joint_position_rt_[i].store(current_waypoint_.jointpos[i], std::memory_order_relaxed);
            if (dt > 1e-6)
            {
                const double velocity =
                    (current_waypoint_.jointpos[i] - last_state_position_[i]) / dt;
                joint_velocity_rt_[i].store(velocity, std::memory_order_relaxed);
            }
            last_state_position_[i] = current_waypoint_.jointpos[i];
        }
        last_state_wall_time_ = now;
    }
    else if (ret == ErrCode_SocketDisconnect)
    {
        // 运动过程中自动重连可能恢复到未知控制模式；故障后要求人工检查并重启。
        ROS_ERROR("[AuboHW] SDK 连接断开，已停止发送。检查机械臂后重启节点。");
        controller_connected_flag_.store(false);
        feed_thread_running_.store(false);
        return;
    }
    else
    {
        ROS_ERROR("[AuboHW] 读取路点失败，错误码: %d；已锁定停止", ret);
        controller_connected_flag_.store(false);
        feed_thread_running_.store(false);
        return;
    }

    // 示教器模式下只做关节位姿轮询，避免高频诊断查询放大底层 SDK 噪声。
    if (!enable_motion_control_)
        return;

    // --- 读取诊断信息，更新急停/保护停止标志（来自 aubo_driver.cpp timerCallback） ---
    ret = robot_receive_service_.robotServiceGetRobotDiagnosisInfo(robot_diagnosis_info_);
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

// =============================================================================
// publishWaypointToRobot()
// 来自 aubo_driver.cpp publishWaypointToRobot()
// 核心：持续监控 MAC 缓冲区大小，按需批量补充轨迹点
// =============================================================================

void AuboHardwareInterface::publishWaypointToRobot()
{
    std::vector<wayPoint_S> waypoint_vector;
    int current_macsz = 0;
    int cnt = 0;

    while (feed_thread_running_.load())
    {
        RobotDiagnosis mac_diagnosis{};
        // --- 读取 MAC 缓冲区大小（来自 aubo_driver.cpp robot_mac_size_service_） ---
        const int diagnosis_ret =
            robot_mac_service_.robotServiceGetRobotDiagnosisInfo(mac_diagnosis);
        const bool diagnosis_ok = diagnosis_ret == InterfaceCallSuccCode;
        if (diagnosis_ok)
        {
            rib_buffer_size_ = mac_diagnosis.macTargetPosDataSize;
            current_macsz    = rib_buffer_size_;

            if (current_macsz == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        else
        {
            ROS_ERROR("[AuboHW] 读取 MAC 缓冲状态失败，错误码: %d；已锁定停止", diagnosis_ret);
            controller_connected_flag_.store(false);
            feed_thread_running_.store(false);
        }

        // --- 当缓冲区未满且队列有数据时批量补充（来自 aubo_driver.cpp） ---
        if (diagnosis_ok &&
            current_macsz < EXPECT_MAC_BUF_SIZE &&
            ros_motion_queue_.size_approx() > 0 &&
            controller_connected_flag_.load() &&
            in_tcp2canbus_mode_ &&
            !emergency_stopped_.load() &&
            !protective_stopped_.load())
        {
            // 计算需要补充的点数（来自 aubo_driver.cpp: ceil((expect-current)/6.0)）
            cnt = static_cast<int>(
                std::ceil(static_cast<double>(EXPECT_MAC_BUF_SIZE - current_macsz) / 6.0));

            waypoint_vector = tryPopWaypoint(cnt);

            if (!waypoint_vector.empty())
            {
                const int send_ret =
                    robot_mac_service_.robotServiceSetRobotPosData2Canbus(waypoint_vector);
                if (send_ret != InterfaceCallSuccCode)
                {
                    ROS_ERROR("[AuboHW] 轨迹点发送失败，错误码: %d；已停止发送", send_ret);
                    controller_connected_flag_.store(false);
                    feed_thread_running_.store(false);
                }
            }
            waypoint_vector.clear();
        }

        // 4ms 间隔（来自 aubo_driver.cpp sleep_for(milliseconds(4))）
        std::this_thread::sleep_for(std::chrono::milliseconds(FEED_THREAD_SLEEP_MS));
    }
}

// =============================================================================
// tryPopWaypoint()
// 来自 aubo_driver.cpp tryPopWaypoint()
// 核心：速度/加速度越限保护 + 等分插值
// =============================================================================

std::vector<wayPoint_S> AuboHardwareInterface::tryPopWaypoint(int count)
{
    std::vector<wayPoint_S> waypoint_vector;
    std::array<double, NUM_JOINTS> joint;   // 从队列中取出的点位
    std::array<double, NUM_JOINTS> interpolation_joint;
    wayPoint_S wp;
    uint8_t same_point = 0;

    for (int k = 0; k < count; ++k)
    {
        if (!ros_motion_queue_.try_dequeue(joint))  // 尝试从队列中弹出一个关节位置
            break;

        // --- 同一点位检测（来自 aubo_driver.cpp: < 0.00015 rad 视为相同） ---
        same_point = 0;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            if (std::fabs(joint[i] - joint_filter_[i]) < SAME_POINT_THRESHOLD)  // 如果当前关节位置与过滤后的关节位置差小于阈值，则认为相同
                same_point |= (1 << i);  // 将对应的位置标记为相同
        }

        if (same_point == 0x3F)  // 全部关节相同，跳过(6位)
            continue;

        // --- 速度限幅检测（来自 aubo_driver.cpp，控制周期 = 0.005s） ---
        over_speed_flag_ = false;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            target_joint_velc_.jointPara[i] =
                std::fabs(joint[i] - joint_filter_[i]) / CTRL_PERIOD_S;

            if (target_joint_velc_.jointPara[i] > MAX_VELC[i])
            {
                ROS_ERROR("[AuboHW] Joint %d 速度超限: %.4f > %.4f rad/s",
                          i, target_joint_velc_.jointPara[i], MAX_VELC[i]);
                over_speed_flag_ = true;
            }
        }

        // --- 超速时等分插值（来自 aubo_driver.cpp over_speed_flag_ 处理） ---
        if (over_speed_flag_)
        {
            // 找到最大速度关节，计算等分数
            double max_velc = *std::max_element(
                target_joint_velc_.jointPara,
                target_joint_velc_.jointPara + NUM_JOINTS);
            int n_equalpart = static_cast<int>(
                std::ceil(max_velc / MAX_VELC[0]));  // 以最小 MaxVelc 为基准

            ROS_WARN("[AuboHW] 超速插值：max_velc=%.4f，等分数=%d", max_velc, n_equalpart);

            interpolation_joint = joint_filter_;
            for (int step = 0; step < n_equalpart - 1; ++step)
            {
                for (int i = 0; i < NUM_JOINTS; ++i)
                {
                    interpolation_joint[i] +=
                        (joint[i] - joint_filter_[i]) / n_equalpart;
                }
                memcpy(wp.jointpos, interpolation_joint.data(),
                       NUM_JOINTS * sizeof(double));
                waypoint_vector.push_back(wp);
            }
            over_speed_flag_ = false;
        }

        // --- 加速度检测（来自 aubo_driver.cpp checkTargetAcc） ---
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            double acc = std::fabs(
                target_joint_velc_.jointPara[i] - last_joint_velc_.jointPara[i])
                / CTRL_PERIOD_S;
            if (acc > MAX_ACC[i])
            {
                ROS_WARN_THROTTLE(1.0,
                    "[AuboHW] Joint %d 加速度超限: %.2f > %.2f rad/s²",
                    i, acc, MAX_ACC[i]);
            }
        }

        // --- 将目标点推入发送向量 ---
        memcpy(wp.jointpos, joint.data(), NUM_JOINTS * sizeof(double));
        waypoint_vector.push_back(wp);

        // 更新滤波状态（来自 aubo_driver.cpp joint_filter_ = joint）
        memcpy(last_joint_velc_.jointPara,
               target_joint_velc_.jointPara,
               sizeof(last_joint_velc_.jointPara));
        joint_filter_ = joint;
    }

    return waypoint_vector;
}

// =============================================================================
// roadPointCompare()
// 来自 aubo_driver.cpp roadPointCompare() + THRESHHOLD = 0.000001
// =============================================================================

bool AuboHardwareInterface::roadPointCompare(const double* p1, const double* p2) const
{
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        if (std::fabs(p1[i] - p2[i]) >= THRESHHOLD)
            return true;
    }
    return false;
}

}  // namespace aubo_ros_control
