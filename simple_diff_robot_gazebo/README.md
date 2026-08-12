# simple_diff_robot_gazebo

一个用于 ROS1 Melodic + Gazebo 的简洁差速移动机器人：左右两个驱动轮、一个低摩擦球形万向支撑轮、二维激光雷达和 RGB 相机，并包含 Gmapping 建图与 Navigation Stack 导航。

## 数据接口

- 订阅：`/cmd_vel` (`geometry_msgs/Twist`)
- 发布：`/odom` (`nav_msgs/Odometry`)
- TF：`odom -> base_footprint -> base_link -> wheels/caster`
- 激光：`/scan` (`sensor_msgs/LaserScan`)
- 相机：`/camera/image_raw`、`/camera/camera_info`

## 编译

把本包放在 Catkin 工作空间的 `src` 目录后执行：

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

如果整个当前仓库就是工作空间的 `src`，则在它的上一级目录执行 `catkin_make`。

## 启动

终端 1：

```bash
roslaunch simple_diff_robot_gazebo gazebo.launch
```

终端 2：

```bash
roslaunch simple_diff_robot_gazebo teleop.launch
```

也可以直接测试：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.2, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.5}"
```

## 修改尺寸

打开 `urdf/simple_diff_robot.xacro`，修改顶部属性：

- `base_length`、`base_width`、`base_height`
- `wheel_radius`、`wheel_width`
- `wheel_separation`
- `caster_radius`、`caster_x`

`wheel_separation` 必须同时代表左右轮接地点之间的横向距离；插件直接复用该参数，因此不会出现模型尺寸和运动学配置不一致。

## 建图、保存与导航

```bash
# 终端 1：Gazebo + Gmapping + RViz
roslaunch simple_diff_robot_gazebo mapping.launch
# 终端 2：键盘控制
roslaunch simple_diff_robot_gazebo teleop.launch
# 地图完整后保存
roslaunch simple_diff_robot_gazebo map_saver.launch
```

关闭建图进程后启动导航：

```bash
roslaunch simple_diff_robot_gazebo navigation.launch
```

在 RViz 中先用 **2D Pose Estimate** 指定初始位姿，再用 **2D Nav Goal** 指定目标。其他地图可通过 `map_file:=/绝对路径/map.yaml` 指定。

查看相机：

```bash
rqt_image_view /camera/image_raw
```

## 学习顺序

1. 用 `display.launch` 只观察 URDF 关节和坐标系。
2. 阅读 `simple_diff_robot.xacro` 中三个 link 和 joint。
3. 阅读末尾 `libgazebo_ros_diff_drive.so` 的参数。
4. 在 Gazebo 中观察 `/cmd_vel`、`/odom` 和 TF。
5. 掌握基础后再增加激光雷达、IMU、建图和导航。
