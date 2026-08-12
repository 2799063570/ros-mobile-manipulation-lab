---
tags: [gazebo, 差速底盘, 教学包]
status: 已创建
---

# simple_diff_robot_gazebo

一个为学习而重新实现的简洁 Gazebo 移动机器人包，包含激光、相机、Gmapping 和 Navigation Stack。

## 结构

- 两个主动轮：`left_wheel_link`、`right_wheel_link`
- 一个万向支撑：`caster_link`
- 一个底盘：`base_link`
- 地面投影坐标：`base_footprint`
- 二维雷达：`laser_link`，发布 `/scan`
- RGB 相机：`camera_link`，发布 `/camera/image_raw`

## 接口

`/cmd_vel → gazebo_ros_diff_drive → /odom + 轮子运动`

## 启动

```bash
roslaunch simple_diff_robot_gazebo gazebo.launch
roslaunch simple_diff_robot_gazebo teleop.launch
```

## 建图与导航

```bash
roslaunch simple_diff_robot_gazebo mapping.launch
roslaunch simple_diff_robot_gazebo map_saver.launch
roslaunch simple_diff_robot_gazebo navigation.launch
```

## 下一步实验

- [ ] 改变轮径，观察相同速度指令下的表现。
- [ ] 改变轮距，观察转弯半径。
- [ ] 使用 `rostopic echo /odom` 查看里程计。
- [ ] 使用 `rqt_graph` 查看节点连接。
- [ ] 添加二维激光雷达和 `/scan`。

仓库源码：`simple_diff_robot_gazebo/`。
