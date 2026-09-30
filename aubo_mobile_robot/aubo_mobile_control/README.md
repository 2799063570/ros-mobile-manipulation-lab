# AUBO 移动机器人控制

## 真机规划中的相机支架

测量实际相机支架尺寸与位置后，复制并修改
`aubo_planning/config/camera_stand_reference.yaml`，可在
`navigation_arm.launch` 加入碰撞物体：

```bash
roslaunch aubo_mobile_control navigation_arm.launch \
  publish_camera_stand:=true \
  camera_stand_config:=/absolute/path/to/measured_camera_stand.yaml
```

该开关默认关闭。移动底盘旁的固定支架应使用固定世界坐标系，
并确认它与 MoveIt 的规划坐标系存在有效 TF。

## 手眼标定（固定工位相机）

先启动移动机器人模型、底盘状态与 `robot_state_publisher`，并使底盘在整个采样期间保持静止。模型中的手部相机应关闭（`enable_hand_camera:=false`）。确认没有其他节点发布 `base_link` 到工位相机的旧标定 TF，也不要同时启动 `eye_to_hand_camera_real.launch`。将 ArUco 标记固定在机械臂末端，测量实际边长（单位：米）。

示教器移动、手动采样：

```bash
roslaunch aubo_mobile_control handeye_calibration.launch \
  motion_mode:=teach robot_ip:=192.168.1.2 \
  marker_id:=30 marker_size:=0.1
```

MoveIt 规划并执行采样位姿：

```bash
roslaunch aubo_mobile_control handeye_calibration.launch \
  motion_mode:=moveit robot_ip:=192.168.1.2 \
  marker_id:=30 marker_size:=0.1
```

`teach` 模式只启动 `joint_state_controller`，在示教器移动并停稳后到 easy_handeye 的 rqt 界面点击采样；`moveit` 模式启动 `aubo_i5_controller` 和移动机器人 MoveIt，用标定运动界面规划、检查轨迹并执行，再采样。两种模式均从实际关节状态获取 `base_link → tcp_link`，并从 ArUco 检测获取相机到标记的变换。改变机械臂姿态和标记朝向，采集多组清晰、分布充分的样本后计算并保存结果。

默认使用 `/workspace_camera/color/image_raw`、`/workspace_camera/color/camera_info`、`workspace_camera_color_optical_frame` 和 `aubo_i5` 规划组。相机或标记不同，可传 `camera_name`、`camera_serial_no`、`image_topic`、`camera_info_topic`、`tracking_base_frame`、`tracking_marker_frame`、`marker_id`、`marker_size`；使用已有相机驱动时传 `start_camera:=false`。已有机械臂驱动时传 `start_arm_driver:=false`，并确保其控制模式和 `motion_mode` 一致。示教器模式不要另行启动 MoveIt 执行控制器。

标定结果以 `aubo_mobile_handeye_eye_on_base` 命名空间保存。实际路径由 easy_handeye 的保存操作决定；在日常运行时，将保存的 YAML 路径传给 `aubo_mobile_nav_sorting/navigation_sorting.launch` 或 `aubo_mobile_bringup/mobile_manipulation_visual_servo.launch` 的 `calibration_file` 参数。具体链路检查见 `aubo_mobile_robot/EYE_TO_HAND_RGBD.md`。再次标定前关闭旧外参发布节点。当前入口默认是眼在手外：相机相对底盘固定、标记随末端运动。

该 ROS 1 功能包提供底盘键盘控制，以及一个按顺序执行导航和机械臂规划的简单
协调节点。机械臂动作均先规划，只有 MoveIt 返回非空、无碰撞轨迹时才会执行；
规划失败时保持当前位置，不会盲目下发目标。

## 键盘控制

先启动机器人或建图程序，再在可交互终端中运行：

```bash
rosrun aubo_mobile_control keyboard_teleop.py
```

也可以使用 `keyboard_teleop.launch`，但 roslaunch 所在终端必须支持交互式 TTY。
如果提示没有 TTY，请优先使用上面的 `rosrun` 命令。

按键说明：

- `w/s`：前进/后退
- `a/d`：左转/右转
- `q/e`：向左前方/右前方弧线行驶
- `z/c`：向左后方/右后方弧线行驶
- 空格或 `x`：停止
- `+/-`：提高/降低速度
- `Ctrl-C`：退出，并在退出前发布零速度

如果一段时间没有收到键盘输入，安全看门狗会自动停止底盘。默认速度较低，适合
搭载机械臂的移动平台。

## 导航完成后规划机械臂

`nav_arm_coordinator.py` 按以下顺序执行一次任务：

```text
MoveIt 移动到命名姿态 down
→ 向 /move_base 发送 map 坐标系导航目标
→ 导航成功后，MoveIt 规划并执行命名姿态 up
```

导航、MoveIt 和机器人控制器已经运行时：

```bash
roslaunch aubo_mobile_control nav_arm_coordinator.launch \
  goal_x:=1.0 goal_y:=0.5 goal_yaw:=1.57 \
  pre_navigation_target:=down post_navigation_target:=up
```

默认使用 launch 参数中的目标。此模式下，组合 launch 会拦截 RViz 的 2D Nav Goal，
避免它绕过协调器并抢占当前任务：

```bash
roslaunch aubo_mobile_control navigation_arm.launch \
  goal_source:=launch goal_x:=1.0 goal_y:=0.5 goal_yaw:=1.57
```

需要在 RViz 中交互选择导航目标时，使用 `rviz` 模式。协调器会先让机械臂进入
导航姿态，然后等待一次 **2D Nav Goal**，并在导航成功后执行后续机械臂动作：

```bash
roslaunch aubo_mobile_control navigation_arm.launch \
  goal_source:=rviz goal_wait_timeout:=0.0
```

`goal_wait_timeout:=0.0` 表示一直等待。设为正数时，超过指定秒数仍未收到 RViz
目标，协调器会结束当前任务，但不会关闭导航、MoveIt 或 RViz。

机器人或 Gazebo 已经运行，需要同时启动地图导航和 MoveIt 时：

```bash
roslaunch aubo_mobile_control navigation_arm.launch \
  map_file:=/absolute/path/to/map.yaml \
  goal_x:=1.0 goal_y:=0.5 goal_yaw:=1.57
```

使用训练场景进行完整 Gazebo 测试：

```bash
roslaunch aubo_mobile_control navigation_arm_gazebo.launch \
  map_file:=$(rospack find aubo_mobile_navigation)/maps/map.yaml \
  goal_x:=1.0 goal_y:=0.5 goal_yaw:=1.57
```

地图必须已经存在并与 Gazebo 场景一致。如果初始位姿未知，需要在 RViz 中使用
**2D Pose Estimate** 初始化 AMCL。

导航后的机械臂目标也可以设置为 TCP 笛卡尔位姿，在自定义 launch 中配置：

```text
arm_target_type: pose
arm_pose_frame: base_link
arm_x, arm_y, arm_z
arm_roll, arm_pitch, arm_yaw
```

这里采用顺序式任务协调，并不是底盘与机械臂同时参与的全身运动规划。差速底盘
仍由 `move_base` 控制，MoveIt 只控制 `aubo_i5` 规划组，末端链接为 `tcp_link`。
