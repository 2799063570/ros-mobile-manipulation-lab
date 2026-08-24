# AUBO 移动机器人控制

该 ROS 1 功能包提供底盘键盘控制，以及一个按顺序执行导航和机械臂规划的简单
协调节点。

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
