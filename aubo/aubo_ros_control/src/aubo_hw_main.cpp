/**
 * @file aubo_hw_main.cpp
 * @brief AUBO ros_control 硬件接口节点入口（优化版）
 *
 * 启动流程：
 *  1. 初始化 ROS 节点
 *  2. 构造并初始化 AuboHardwareInterface
 *     - init() 内部完成：连接重试、启动、TCP2CANBUS 进入、喂点线程启动
 *  3. 构造 controller_manager
 *  4. AsyncSpinner 处理回调
 *  5. 主循环：read() → cm.update() → write()
 *     注意：状态读取由内部 timerCallback (50 Hz) 负责，
 *           主循环主要驱动 controller_manager 更新和命令写入队列
 *
 * 用法：
 *  roslaunch aubo_ros_control aubo_control.launch \
 *    server_host:=192.168.1.2 \
 *    control_frequency:=200.0
 *
 *  teach_pendant 模式：
 *  roslaunch aubo_ros_control teach_pendant.launch \
 *    server_host:=192.168.1.2
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <controller_manager/controller_manager.h>

#include <unistd.h>
#include <cmath>
#include <clocale>

#include "aubo_ros_control/aubo_hardware_interface.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "aubo_hw_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ROS_INFO("[aubo_hw_node] 节点启动...");

    // ------------------------------------------------------------------
    // 控制频率（主循环驱动 controller_manager）
    // aubo_driver.cpp UPDATE_RATE_ = 500 Hz，但主循环与喂点线程分离，
    // 此处设为 250 Hz（4ms 周期，与 tryPopWaypoint 中 CTRL_PERIOD_S 对应）
    // ------------------------------------------------------------------
    double control_freq = 250.0;
    nh_priv.param<double>("control_frequency", control_freq, 250.0);
    if (!std::isfinite(control_freq) || control_freq <= 0.0 || control_freq > 1000.0)
    {
        ROS_FATAL("[aubo_hw_node] control_frequency 必须在 (0, 1000] Hz 范围内");
        return -1;
    }
    ros::Rate loop_rate(control_freq);

    bool enable_motion_control = true;
    nh_priv.param<bool>("enable_motion_control", enable_motion_control, true);
    ROS_INFO("[aubo_hw_node] 上层运动控制: %s",
             enable_motion_control ? "开启" : "关闭（示教器模式）");

    const std::string sdk_package_path = ros::package::getPath("aubo_sdk");
    if (!sdk_package_path.empty())
    {
        if (::chdir(sdk_package_path.c_str()) != 0)
        {
            ROS_WARN("[aubo_hw_node] 切换到 aubo_sdk 目录失败，将继续使用当前工作目录。");
        }
    }
    else
    {
        ROS_WARN("[aubo_hw_node] 未找到 aubo_sdk 包路径，无法切换工作目录。");
    }

    ROS_INFO("[aubo_hw_node] 控制频率: %.1f Hz (%.2f ms)",
             control_freq, 1000.0 / control_freq);

    // ------------------------------------------------------------------
    // 硬件接口初始化
    // ------------------------------------------------------------------
    aubo_ros_control::AuboHardwareInterface aubo_hw(nh_priv);

    if (!aubo_hw.init())
    {
        ROS_FATAL("[aubo_hw_node] 硬件接口初始化失败！");
        return -1;
    }

    // ------------------------------------------------------------------
    // controller_manager
    // ------------------------------------------------------------------
    controller_manager::ControllerManager cm(&aubo_hw, nh);

    // ------------------------------------------------------------------
    // AsyncSpinner 处理 controller_manager 的服务回调
    // ------------------------------------------------------------------
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // ------------------------------------------------------------------
    // 主控制循环
    // read()   — 实际状态已由 timerCallback 50Hz 填充到缓冲区
    // update() — 运行控制器算法，写命令到 joint_position_cmd_
    // write()  — 将命令差分后推入 ros_motion_queue_（无锁）
    // ------------------------------------------------------------------
    ros::Time last_time = ros::Time::now();

    ROS_INFO("[aubo_hw_node] 进入主控制循环 (%.0f Hz)...", control_freq);

    while (ros::ok() && aubo_hw.isConnected())
    {
        ros::Time   now    = ros::Time::now();
        ros::Duration period = now - last_time;
        last_time = now;

        aubo_hw.read(now, period);
        cm.update(now, period);
        aubo_hw.write(now, period);

        loop_rate.sleep();
    }

    if (ros::ok() && !aubo_hw.isConnected())
        ROS_ERROR("[aubo_hw_node] 与机械臂连接已失效，退出控制循环。");

    ROS_INFO("[aubo_hw_node] 节点停止，执行安全关闭...");
    aubo_hw.shutdown();

    return 0;
}
