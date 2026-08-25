/**
 * @file aubo_hardware_interface.h
 * @brief AUBO 机器人 ros_control 硬件接口（参照 aubo_driver.cpp 优化版）
 *
 * 核心优化点（对照 aubo_driver.cpp）：
 *  1. 双 ServiceInterface：robot_send_service_ 发送 / robot_receive_service_ 接收，
 *     避免单连接在高频轮询时阻塞写指令。
 *  2. ReaderWriterQueue 无锁单生产者-单消费者队列：
 *     write() 生产轨迹点 → publishWaypointToRobot 线程消费，完全无锁。
 *  3. 独立喂点线程 publishWaypointToRobot：
 *     持续监控 MAC 缓冲区 (macTargetPosDataSize)，
 *     当缓冲区 < expect_macsz(400) 时批量补充点位。
 *  4. 速度/加速度越限保护（tryPopWaypoint）：
 *     MaxVelc / MaxAcc 来自 aubo_driver.cpp，5ms 控制周期下验证。
 *  5. 点位去重阈值 THRESHHOLD = 0.000001 rad。
 *  6. 首次连接最多重试 5 次；运动中断线后保持停止并要求人工重启。
 *  7. timerCallback 方式读取当前路点（与 aubo_driver.cpp 一致）。
 */

#pragma once

#include <string>
#include <vector>
#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <queue>
#include <cmath>

#include <ros/ros.h>

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// AUBO SDK
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include "readerwriterqueue.h"   // 无锁单生产者-单消费者队列

namespace aubo_ros_control
{

// =============================================================================
// 常量定义 — 直接来自 aubo_driver.h / aubo_driver.cpp
// =============================================================================

/// 关节数
static constexpr int NUM_JOINTS = aubo_robot_namespace::ARM_DOF;  // 6

/// TCP2CANBUS MAC 缓冲区目标大小（与 aubo_driver.cpp publishWaypointToRobot 一致）
static constexpr int EXPECT_MAC_BUF_SIZE = 400;

/// 最小缓冲区阈值（参照 aubo_driver.h MINIMUM_BUFFER_SIZE）
static constexpr int MINIMUM_BUFFER_SIZE = 300;

/// 点位去重阈值（来自 aubo_driver.h THRESHHOLD）
static constexpr double THRESHHOLD = 0.000001;

/// 速度不一致判断阈值（来自 tryPopWaypoint: < 0.00015 rad 视为相同点）
static constexpr double SAME_POINT_THRESHOLD = 0.00015;

/// 控制周期 5ms（来自 aubo_driver.cpp: fabs(joint-joint_filter)/0.005）
static constexpr double CTRL_PERIOD_S = 0.005;

/// 喂点线程休眠间隔 4ms（来自 aubo_driver.cpp publishWaypointToRobot）
static constexpr int FEED_THREAD_SLEEP_MS = 4;

/// 最大连接重试次数（来自 aubo_driver.cpp connectToRobotController）
static constexpr int MAX_CONNECT_RETRIES = 5;

/// SDK 端口号（来自 aubo_driver.h server_port）
static constexpr int SERVER_PORT = 8899;

/// 各关节最大速度 rad/s（来自 aubo_driver.cpp MaxVelc）
static const double MAX_VELC[NUM_JOINTS] = {
    2.596177, 2.596177, 2.596177,
    3.110177, 3.110177, 3.110177
};

/// 各关节最大加速度 rad/s²（来自 aubo_driver.cpp MaxAcc）
static const double MAX_ACC[NUM_JOINTS] = {
    17.30878, 17.30878, 17.30878,
    20.73676, 20.73676, 20.73676
};


// =============================================================================
// 轨迹点结构（对应 aubo_driver.h PlanningState）
// =============================================================================
struct JointVelcAccParam
{
    double jointPara[NUM_JOINTS];
};

/**
 * @class AuboHardwareInterface
 * @brief 将 AUBO SDK 封装为 ros_control 标准 RobotHW 接口
 *
 * 数据流（TCP2CANBUS 模式）：
 *   controller_manager::write()
 *     └─ joint_position_cmd_ → ros_motion_queue_ (ReaderWriterQueue, 无锁)
 *                                    ↓
 *                           publishWaypointToRobot 线程
 *                             ├─ 监控 macTargetPosDataSize
 *                             ├─ tryPopWaypoint(): 速度/加速度限幅
 *                             └─ robot_mac_service_.SetRobotPosData2Canbus()
 *
 *   timerCallback (ros::Timer, 50 Hz)
 *     └─ robot_receive_service_.GetCurrentWaypointInfo()
 *           └─ joint_position_[i] = wp.jointpos[i]  (互斥锁保护)
 */
class AuboHardwareInterface : public hardware_interface::RobotHW
{
public:
    explicit AuboHardwareInterface(ros::NodeHandle& nh);
    ~AuboHardwareInterface() override;

    /** 初始化：SDK 登录、机械臂启动、注册 ros_control 接口 */
    bool init();

    /** 是否启用 ROS 上层运动控制（默认开启；示教器模式关闭） */
    bool motionControlEnabled() const { return enable_motion_control_; }

    /** read(): 从 joint_state 缓冲区更新 joint_position_/velocity_/effort_ */
    void read(const ros::Time& time, const ros::Duration& period) override;

    /**
     * write(): 将位置命令推入 ros_motion_queue_（无锁）
     *          喂点线程负责消费并发送到机械臂
     */
    void write(const ros::Time& time, const ros::Duration& period) override;

    /** 安全关闭：离开 TCP2CANBUS、注销 SDK */
    void shutdown();

    bool isConnected() const { return controller_connected_flag_.load(); }

private:
    // -------------------------------------------------------------------------
    // 内部辅助
    // -------------------------------------------------------------------------

    /** 带重试的 SDK 连接（来自 aubo_driver.cpp connectToRobotController） */
    bool connectToRobotController();

    /** 等待机械臂电源/急停就绪 */
    bool waitForRobotReady(int max_attempts = 200);

    /** 进入 TCP2CANBUS 模式（带自动恢复逻辑） */
    bool enterTcp2CanbusMode();

    /** 离开 TCP2CANBUS 模式 */
    bool leaveTcp2CanbusMode();

    /**
     * @brief 喂点线程主函数（对应 aubo_driver.cpp publishWaypointToRobot）
     *
     * 持续运行，监控 MAC 缓冲区大小：
     *  - 当 macTargetPosDataSize < expect_macsz 时，批量弹出轨迹点并发送
     *  - 循环间隔 FEED_THREAD_SLEEP_MS ms
     */
    void publishWaypointToRobot();

    /**
     * @brief 从 ros_motion_queue_ 中弹出并验证速度/加速度的轨迹点
     *        （对应 aubo_driver.cpp tryPopWaypoint）
     *
     * @param count  最多弹出的点数
     * @return       可安全发送的 wayPoint_S 向量
     */
    std::vector<aubo_robot_namespace::wayPoint_S> tryPopWaypoint(int count);

    /**
     * @brief ROS 定时器回调，负责读取关节状态并发布
     *        （对应 aubo_driver.cpp timerCallback，50Hz）
     */
    void timerCallback(const ros::TimerEvent& e);

    /** 比较两个关节位置数组是否存在足够差异（> THRESHHOLD） */
    bool roadPointCompare(const double* p1, const double* p2) const;

    // -------------------------------------------------------------------------
    // ROS 参数
    // -------------------------------------------------------------------------
    ros::NodeHandle nh_;

    std::string server_host_;    ///< 机械臂 IP
    int         server_port_;    ///< 机械臂端口（默认 8899）
    std::string username_;
    std::string password_;

    std::vector<std::string> joint_names_;

    // -------------------------------------------------------------------------
    // ros_control 接口使用的原始数据缓冲区（由 read() 更新）
    // -------------------------------------------------------------------------
    std::array<double, NUM_JOINTS> joint_position_;
    std::array<double, NUM_JOINTS> joint_velocity_;
    std::array<double, NUM_JOINTS> joint_effort_;

    std::array<double, NUM_JOINTS> joint_position_cmd_;
    std::array<double, NUM_JOINTS> last_cmd_sent_;    ///< 上次实际发送的命令（去重用）
    std::array<double, NUM_JOINTS> filtered_cmd_;      ///< 命令低通后的内部目标

    // -------------------------------------------------------------------------
    // 线程安全的前台影子缓冲区（由 timerCallback 写，read() 读，消除数据竞争）
    // -------------------------------------------------------------------------
    std::array<std::atomic<double>, NUM_JOINTS> joint_position_rt_;
    std::array<std::atomic<double>, NUM_JOINTS> joint_velocity_rt_;
    std::array<std::atomic<double>, NUM_JOINTS> joint_effort_rt_;

    // -------------------------------------------------------------------------
    // ros_control 接口
    // -------------------------------------------------------------------------
    hardware_interface::JointStateInterface    joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    // -------------------------------------------------------------------------
    // AUBO SDK — 三连接（发送、状态读取和 MAC 缓冲操作分离）
    // -------------------------------------------------------------------------
    ServiceInterface robot_send_service_;      ///< 发送指令
    ServiceInterface robot_receive_service_;   ///< 读取状态
    ServiceInterface robot_mac_service_;       ///< 监控 MAC 缓冲区 + 发点

    // -------------------------------------------------------------------------
    // 无锁运动队列（对应 aubo_driver.cpp ros_motion_queue_）
    // -------------------------------------------------------------------------
    moodycamel::ReaderWriterQueue<std::array<double, NUM_JOINTS>> ros_motion_queue_;

    // -------------------------------------------------------------------------
    // 喂点线程相关（对应 aubo_driver.cpp publishWaypointToRobot）
    // -------------------------------------------------------------------------
    std::thread* feed_thread_;          ///< 喂点线程
    std::atomic<bool> feed_thread_running_;

    // 过速插值状态（对应 aubo_driver.cpp tryPopWaypoint）
    std::array<double, NUM_JOINTS> joint_filter_;         ///< 上次发出的关节角（关节平滑参考）
    JointVelcAccParam target_joint_velc_;
    JointVelcAccParam last_joint_velc_;
    bool over_speed_flag_;
    int  rib_buffer_size_;                                ///< MAC 缓冲区当前大小

    // 命令整形参数（用于降低 MoveIt / 视觉伺服的抖动）
    bool enable_command_smoothing_;
    double command_filter_alpha_;
    double command_deadband_;
    double max_command_step_scale_;

    // -------------------------------------------------------------------------
    // 机器人诊断（对应 aubo_driver.cpp rs.robot_diagnosis_info_）
    // -------------------------------------------------------------------------
    aubo_robot_namespace::RobotDiagnosis robot_diagnosis_info_;
    aubo_robot_namespace::wayPoint_S     current_waypoint_;

    // -------------------------------------------------------------------------
    // 状态标志（对应 aubo_driver.cpp）
    // -------------------------------------------------------------------------
    std::atomic<bool> controller_connected_flag_;
    bool real_robot_exist_;
    std::atomic<bool> emergency_stopped_;
    std::atomic<bool> protective_stopped_;
    bool in_tcp2canbus_mode_;
    bool enable_motion_control_;
    bool require_real_robot_;
    std::atomic<bool> shutdown_started_;
    int  collision_class_;              ///< 碰撞等级（默认 6）

    std::array<double, NUM_JOINTS> last_state_position_;
    ros::WallTime last_state_wall_time_;

    // -------------------------------------------------------------------------
    // ROS 定时器（对应 aubo_driver.cpp timer_，TIMER_SPAN_ = 50 Hz）
    // -------------------------------------------------------------------------
    ros::Timer state_timer_;
    static constexpr int TIMER_SPAN_HZ = 50;  ///< 状态读取频率 Hz
};

}  // namespace aubo_ros_control
