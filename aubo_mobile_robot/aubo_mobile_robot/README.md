# AUBO 差速移动机械臂

该 ROS 1 功能包将已有的 AUBO i5 机械臂模型与圆形差速底盘组合，用于 Gazebo
Classic 仿真。

## 机器人模型

- 直径 `0.68 m` 的圆形底盘、两个驱动轮和两个被动支撑轮
- 复用 `aubo_description` 中的 AUBO i5 机械臂和双指夹爪
- 前雷达：`front_laser_link`，话题 `/front/scan`，扫描前方 180°
- 后雷达：`rear_laser_link`，话题 `/rear/scan`，扫描后方 180°
- 手部 RGB 相机：`hand_camera_optical_frame`
- 相机话题：`/hand_camera/image_raw` 和 `/hand_camera/camera_info`
- 两个雷达带有可见且参与碰撞计算的安装支柱
- 手部相机通过 L 形支架连接在夹爪侧面
- 相机光轴与夹爪接近方向一致，TCP 朝下时相机同时观察桌面
- 底盘速度指令：`/cmd_vel`
- 轮式里程计：`/odom`，并发布 TF `odom -> base_footprint`
- 机械臂轨迹接口：`/aubo_i5_controller/follow_joint_trajectory`
- 夹爪轨迹接口：`/gripper_controller/follow_joint_trajectory`

前后雷达保留为两个独立的扫描话题。导航功能可以分别使用，也可以通过激光
雷达合并节点生成统一的 `/scan`。

## 启动 Gazebo

完成 catkin 工作空间编译并加载环境后执行：

```bash
roslaunch aubo_mobile_robot gazebo.launch
```

在另一个终端发送底盘速度：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.15}, angular: {z: 0.0}}'
```

按 `Ctrl-C` 停止持续发布，必要时再发送一次零速度指令。

发送简单的机械臂关节轨迹：

```bash
rostopic pub -1 /aubo_i5_controller/command trajectory_msgs/JointTrajectory \
  "joint_names: [shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint]
points:
- positions: [0.0, -0.8, 1.0, 0.0, 0.8, 0.0]
  time_from_start: {secs: 4}"
```

闭合双指夹爪：

```bash
rostopic pub -1 /gripper_controller/command trajectory_msgs/JointTrajectory \
  "joint_names: [joint1, joint2]
points:
- positions: [0.35, 0.35]
  time_from_start: {secs: 2}"
```

只在 RViz 中检查运动学模型：

```bash
roslaunch aubo_mobile_robot display.launch
```

## 常用检查命令

```bash
rostopic echo /front/scan
rostopic echo /rear/scan
rostopic echo /hand_camera/camera_info
rostopic echo /odom
rosservice call /controller_manager/list_controllers
```

原有的独立 AUBO 机械臂启动文件仍可继续使用。对
`aubo_description/urdf/arm.xacro` 的调整只是让世界关节和 ros_control 命名空间
可以配置，并保留了原来的默认值。

## 配置 MoveIt Setup Assistant

不要覆盖原来的纯机械臂 `aubo_moveit_config`，应单独生成
`aubo_mobile_moveit_config`：

```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```

选择 **Create New MoveIt Configuration Package**，加载
`aubo_mobile_robot/urdf/aubo_mobile_robot.xacro`，然后按以下方式设置：

1. 使用较高采样密度生成自碰撞矩阵，重点检查机械臂与底盘的碰撞对。
2. 添加平面虚拟关节 `world_joint`：父坐标系为 `odom`，子链接为
   `base_footprint`。该关节用于跟踪底盘位姿，不负责驱动差速底盘。
3. 创建机械臂规划组 `aubo_i5`，运动链从 `base_link` 到 `tcp_link`，运动学
   求解器选择 KDL。`tcp_link` 是抓取工具中心，`gripper_link` 仅作为兼容链接。
4. 创建夹爪规划组 `gripper`，包含 `joint1` 和 `joint2`。
5. 添加末端执行器 `aubo_gripper`：末端组为 `gripper`，父链接为
   `gripper_base_link`，父规划组为 `aubo_i5`。
6. 将 `left_wheel_joint` 和 `right_wheel_joint` 标记为被动关节。导航通过
   `/cmd_vel` 管理底盘，机械臂规划组不应包含车轮关节。
7. 添加机械臂 `zero`、`home`、`down` 以及夹爪 `open`、`closed` 等常用姿态。
8. 六个机械臂关节使用 `/aubo_i5_controller`，夹爪关节使用
   `/gripper_controller`。
9. 将新配置包生成到集合目录中，与核心机器人包平行：
   `aubo_mobile_robot/aubo_mobile_moveit_config`。

当前手部相机只有 RGB 图像，因此 MoveIt 的三维感知部分暂时留空。更换为深度
相机或增加点云来源后，再配置 OctoMap 传感器插件。
