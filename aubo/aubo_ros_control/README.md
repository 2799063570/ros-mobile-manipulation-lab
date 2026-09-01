# aubo_ros_control

该功能包为真实 AUBO i5 提供 ROS 1 硬件接口。其实现参考了 [`aubo_ros_control`](https://github.com/2799063570/aubo_perception_planning/tree/main/aubo_ros_control) 的 AUBO SDK 驱动模式，并适配当前工作空间的关节名称和 MoveIt 控制器：`/aubo_i5/aubo_i5_controller/follow_joint_trajectory`。

## 安全机制

- 只有三个 SDK 会话均已连接、控制器确认为真实机器人、启动与握手成功且已进入 TCP2CANBUS 模式后，才接受位置指令。
- 加载控制器前必须成功读取硬件初始关节位置。
- 急停、碰撞或保护性停止、非法数值指令、发送失败或套接字断开都会停止后续指令输出。连接断开后必须检查现场并重启节点，驱动不会自动恢复运动。
- 指令线程会限制每个关节的速度步长，并独立于 ROS 控制循环填充控制器 MAC 缓冲区。

## 编译

```bash
cd ~/catkin_ws
catkin_make --pkg aubo_sdk aubo_ros_control
source devel/setup.bash
```

ROS Melodic 环境若缺少依赖，请安装 `ros-control`、`ros-controllers`、`joint-state-controller` 和 `position-controllers`。

## 运动前验证

先进行 SDK 只读连接测试，该操作不会启动或移动机械臂：

```bash
rosrun aubo_sdk sdk_test 192.168.1.2
```

随后启动仅状态模式。该模式不会调用机器人启动流程，也不会进入 TCP2CANBUS 模式：

```bash
roslaunch aubo_ros_control aubo_state.launch robot_ip:=192.168.1.2
rostopic echo /aubo_i5/joint_states
```

## 真实机械臂控制

执行轨迹前，请确保示教器和急停按钮随时可用、工作区内无人员或障碍物，并核对 RViz 与真实机械臂的关节方向一致。

仅启动硬件控制器：

```bash
roslaunch aubo_ros_control aubo_control.launch robot_ip:=192.168.1.2
```

同时启动硬件、MoveIt、`robot_state_publisher` 和 RViz：

```bash
roslaunch aubo_ros_control aubo_real_bringup.launch robot_ip:=192.168.1.2
```

重要话题和接口：

- 硬件状态：`/aubo_i5/joint_states`
- MoveIt/TF 合并状态：`/joint_states`
- 轨迹动作：`/aubo_i5/aubo_i5_controller/follow_joint_trajectory`
- 控制器管理器：`/aubo_i5/controller_manager`

`server_port`、登录凭据、碰撞等级、指令平滑和控制频率等参数均由 `aubo_control.launch` 暴露，不在代码中写死。

真实机械臂启动文件默认使用 `aubo_i5_with_camera.xacro`。若不加载手眼相机，可执行：

```bash
roslaunch aubo_ros_control aubo_real_bringup.launch \
  robot_ip:=192.168.1.2 \
  robot_xacro:=aubo_i5.xacro robot_srdf:=aubo_i5.srdf
```

眼在手视觉伺服在 Gazebo 和真实硬件上使用相同的 RealSense 风格 RGB-D 话题。`eye_in_hand_visual_servo_real.launch` 可启动真实 RealSense 驱动并对齐深度图；机器人 URDF 提供已标定的安装 TF，公共光学坐标系为 `camera_color_optical_frame`。

`eye_in_hand_visual_servo_gazebo.launch` 会把 `/aubo_i5/joint_states` 转发到 `robot_state_publisher` 使用的全局 `/joint_states`。如果 RViz 报告机械臂和相机链路均无法变换到 `base_link`，应先确认这两个话题都在发布，再排查相机 TF。

`visual_servo_gazebo.launch` 和 `visual_servo_real.launch` 是为已有部署保留的兼容入口，现已弃用。
