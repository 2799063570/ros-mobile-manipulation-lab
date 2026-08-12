---
tags: [控制, 入门]
---

# wheeltec_robot_rc

## 定位

键盘遥控功能包，适合用作本项目 ROS 节点学习的第一个例子。

## 启动

```bash
roslaunch wheeltec_robot_rc keyboard_teleop.launch
```

通常需要先启动底盘：

```bash
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```

## 关键源码

- `scripts/turtlebot_teleop_key.py`
- `src/turtlebot_joy.cpp`

## 学习目标

- 键盘事件如何映射到线速度与角速度。
- `geometry_msgs/Twist` 的结构。
- 速度平滑节点是否对话题进行了 remap。
- 停止操作时是否可靠发布零速度。

关联：[[功能包/turn_on_wheeltec_robot]]、[[分类/01-底盘与控制]]。

