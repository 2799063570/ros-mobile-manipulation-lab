# AUBO 移动机器人控制

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

## 雷达保护导航后观察环境

手眼相机需要在导航结束后观察工作区域时，可直接运行：

```bash
roslaunch aubo_mobile_control environment_observation.launch \
  goal_x:=1.0 goal_y:=0.5 goal_yaw:=1.57
```

该入口先将机械臂规划到紧凑的 `transport` 姿态，再通过双激光雷达导航；到达后，
机械臂规划到 SRDF 中的 `observe` 关节姿态，并等待
`/hand_camera/image_raw` 的有效图像。`transport` 和 `observe` 是确定关节角的命名
姿态，本身不依赖在线逆解；MoveIt 仍会根据当前关节状态和规划场景做关节限位、
自碰撞及路径可达性检查。

Gazebo 中也可以使用已有完整入口：

```bash
roslaunch aubo_mobile_control navigation_arm_gazebo.launch \
  pre_navigation_target:=transport post_navigation_target:=observe \
  wait_for_camera:=true
```

若相机话题不同，可设置 `camera_topic`；在超时时间内没有收到非空图像，任务会
明确失败。激光安全层默认开启，可用 `laser_safety:=false` 关闭（仅建议调试时使用）。
