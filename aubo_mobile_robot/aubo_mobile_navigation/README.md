# AUBO 移动机器人导航

该 ROS 1 功能包用于圆形 AUBO 移动机械臂的建图、定位和导航，坐标系及话题与
`aubo_mobile_robot` 核心模型保持一致。

## 数据流

```text
/front/scan ─┐
             ├─ dual_laser_merger ─ /scan ─ GMapping 或 AMCL + move_base
/rear/scan  ─┘

odom ───────── 差速底盘 Gazebo 插件
map -> odom ── 建图时由 GMapping 发布，加载地图导航时由 AMCL 发布
```

雷达合并节点先将前后扫描变换到 `base_footprint`，再生成包含 720 个采样点的
360° `/scan`。该过程会考虑两个雷达的安装位置和方向，并非直接拼接距离数组。

## 编译

在 catkin 工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

底盘行驶前应将机械臂收回到折叠或 `home` 姿态。导航参数中的 `0.40 m` 机器人
半径只覆盖收拢后的机器人，不覆盖完全伸展的机械臂。

不要同时运行导航和使用假控制器的 `aubo_mobile_moveit_config/demo.launch`。
该演示会发布静态 `odom -> base_footprint`，与差速底盘里程计冲突。底盘与机械臂
联合仿真时，应使用 Gazebo 真实控制器版本的 MoveIt 启动文件。

## Gazebo 建图

同时启动机器人、双雷达合并、GMapping 和 RViz：

```bash
roslaunch aubo_mobile_navigation mapping_gazebo.launch
```

指定其他 Gazebo 场景：

```bash
roslaunch aubo_mobile_navigation mapping_gazebo.launch \
  world:=/absolute/path/to/site.world
```

通过 `/cmd_vel` 控制底盘探索环境，也可以运行本项目的键盘节点：

```bash
rosrun aubo_mobile_control keyboard_teleop.py
```

建图结束后保存地图：

```bash
roslaunch aubo_mobile_navigation map_saver.launch
```

默认生成 `maps/map.yaml` 和 `maps/map.pgm`。也可以指定绝对路径：

```bash
roslaunch aubo_mobile_navigation map_saver.launch \
  map_name:=/absolute/writable/path/site_map
```

## 使用已有地图导航

同时启动 Gazebo、地图服务器、AMCL、move_base、雷达合并和 RViz：

```bash
roslaunch aubo_mobile_navigation navigation_gazebo.launch \
  map_file:=$(rospack find aubo_mobile_navigation)/maps/map.yaml
```

在 RViz 中：

1. 使用 **2D Pose Estimate** 设置 AMCL 初始位姿。
2. 使用 **2D Nav Goal** 发送导航目标。

真实机器人或 Gazebo 已经运行时，不再启动仿真器：

```bash
roslaunch aubo_mobile_navigation navigation.launch \
  map_file:=/absolute/path/to/map.yaml
```

需要边建图边运行 move_base 时，先启动机器人，再执行：

```bash
roslaunch aubo_mobile_navigation mapping_navigation.launch
```

按照参考工程 `simple_diff_robot_gazebo/launch/mapping_nav.launch` 的结构，同时
启动 Gazebo、GMapping 和 move_base：

```bash
roslaunch aubo_mobile_navigation mapping_nav.launch
```

## 检查命令

```bash
rostopic hz /front/scan
rostopic hz /rear/scan
rostopic hz /scan
rosrun tf tf_echo odom base_footprint
rostopic echo -n 1 /map
rostopic echo -n 1 /move_base/status
```

正常情况下 TF 发布关系为：

- 差速底盘插件发布 `odom -> base_footprint`
- 建图时 GMapping 发布 `map -> odom`
- 加载已有地图导航时 AMCL 发布 `map -> odom`

不要同时运行 GMapping 和 AMCL，否则二者会同时尝试发布 `map -> odom`，产生
TF 冲突。
