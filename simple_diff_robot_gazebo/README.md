# simple_diff_robot_gazebo

## 单障碍物自动绕行

下面的命令会启动一个封闭 Gazebo 场地。小车从左侧出发，沿三个相对航点从红色障碍物下方绕行，最后停在右侧绿色圆形标记处：

```bash
roslaunch simple_diff_robot_gazebo single_obstacle_demo.launch
```

节点同时读取 `/scan`；当前方小于 0.42 m 时会停止直行并执行安全转向。若只想加载环境并手动控制，可使用：

```bash
roslaunch simple_diff_robot_gazebo single_obstacle_demo.launch auto_start:=false
roslaunch simple_diff_robot_gazebo teleop.launch
```

一个用于 ROS1 Melodic + Gazebo 的简洁差速移动机器人：左右两个驱动轮、一个低摩擦球形万向支撑轮、二维激光雷达和 RGB 相机，并包含 Gmapping 建图与 Navigation Stack 导航。

## 数据接口

- 订阅：`/cmd_vel` (`geometry_msgs/Twist`)
- 发布：`/odom` (`nav_msgs/Odometry`)
- TF：`odom -> base_footprint -> base_link -> wheels/caster`
- 激光：`/scan` (`sensor_msgs/LaserScan`)
- 相机：`/camera/image_raw`、`/camera/camera_info`

差速插件使用编码器里程计，而不是 Gazebo 世界真值，使 Gmapping 和 AMCL 实验更接近真实机器人。

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

键位为 `W/S` 前进后退、`A/D` 左右转、空格或 `X` 停车、`Q` 退出。需要按住运动键；超过 0.5 秒没有键盘输入会自动停车。键盘节点已经包含在本包中，不需要额外安装 `teleop_twist_keyboard`。

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

默认保存到源码包的 `maps` 目录。如果包位于只读安装目录，请使用：

```bash
mkdir -p ~/maps
roslaunch simple_diff_robot_gazebo map_saver.launch map_name:=$HOME/maps/my_map
```

关闭建图进程后启动导航：

```bash
roslaunch simple_diff_robot_gazebo navigation.launch
```

导航依赖已经存在的 YAML/PGM 地图；如果尚未生成 `maps/my_map.yaml`，启动会失败。指定其他地图：

```bash
roslaunch simple_diff_robot_gazebo navigation.launch map_file:=$HOME/maps/my_map.yaml
```

在 RViz 中先用 **2D Pose Estimate** 指定初始位姿，再用 **2D Nav Goal** 指定目标。其他地图可通过 `map_file:=/绝对路径/map.yaml` 指定。

## RRT 自主探索建图

联合启动 Gazebo、Gmapping、Navigation Stack 和 `rrt_exploration`：

```bash
roslaunch simple_diff_robot_gazebo rrt_exploration.launch
```

等待 RViz 中 `/map` 出现后，使用 **Publish Point** 依次点击探索矩形的四个角，确保第 1、3 点互为对角点；第 5 点点击机器人附近的已知自由空间，作为 RRT 根节点。初始化后机器人会自动选择 frontier 并通过 `/move_base` 导航。

该启动文件不会启动 AMCL、静态 `map_server` 或真机版 `wait_for_fin`。完成实验后可另开终端保存地图：

```bash
roslaunch simple_diff_robot_gazebo map_saver.launch
```

查看相机：

```bash
rqt_image_view /camera/image_raw
```

## 学习顺序

1. 用 `display.launch` 只观察 URDF 关节和坐标系。
2. 阅读 `simple_diff_robot.xacro` 中底盘、车轮、万向轮、雷达和相机的 link/joint。
3. 阅读末尾 `libgazebo_ros_diff_drive.so` 的参数。
4. 在 Gazebo 中观察 `/cmd_vel`、`/odom` 和 TF。
5. 依次实验激光雷达、相机、Gmapping、AMCL、代价地图和 DWA。
6. 掌握现有链路后，可继续增加 IMU、传感器融合或视觉算法。

## 运行前检查

```bash
rospack find simple_diff_robot_gazebo
rosrun xacro xacro --inorder $(rospack find simple_diff_robot_gazebo)/urdf/simple_diff_robot.xacro > /tmp/simple_diff_robot.urdf
check_urdf /tmp/simple_diff_robot.urdf
```

遇到启动、TF、地图或导航问题时，参见 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)。
