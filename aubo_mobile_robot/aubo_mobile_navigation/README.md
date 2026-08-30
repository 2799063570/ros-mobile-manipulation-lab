# AUBO 移动机器人导航

## 双雷达与速度安全层

`navigation.launch` 会把前、后两个 180° 激光雷达合并为 360° 的 `/scan`。导航器
输出先进入 `/cmd_vel_raw`，再由 `laser_safety_filter.py` 检查机器人运动方向上的
扫掠空间后发布 `/cmd_vel`。

安全距离会随当前速度按反应时间和制动距离自动增加。前进、后退、原地旋转分别
检查对应区域；普通障碍需至少三个相邻采样点才触发，极近障碍单点即可停车，以兼顾
抗噪和紧急保护。雷达数据超过 0.5 秒未更新时采用失效安全策略，停止底盘。

该节点是 `move_base` 代价地图之外的最后一道保护，不替代正确的地图、定位、底盘
制动参数或现场急停。单独调试可运行：

```bash
roslaunch aubo_mobile_navigation laser_safety_filter.launch
```

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
roslaunch aubo_mobile_navigation mapping_nav.launch
```

按照参考工程 `simple_diff_robot_gazebo/launch/mapping_nav.launch` 的结构，同时
启动 Gazebo、GMapping 和 move_base：

```bash
roslaunch aubo_mobile_navigation mapping_nav_gazebo.launch
```

## RRT 自主探索建图

一键启动 Gazebo、双雷达、GMapping、move_base 和 RRT 前沿探索：

```bash
roslaunch aubo_mobile_navigation rrt_exploration_gazebo.launch
```

RViz 地图出现后，确认机械臂处于收拢姿态，然后选择 **Publish Point**：

1. 按顺序点击探索区域的四个角点；
2. 第五次点击机器人附近的自由区域，作为全局 RRT 的起点；
3. 第五个点发布后，机器人将自动检测前沿并通过 `move_base` 探索。

探索范围应落在当前地图坐标系内，并留出覆盖机器人半径的安全边界。可通过参数
调整 RRT 步长及信息增益权重：

```bash
roslaunch aubo_mobile_navigation rrt_exploration_gazebo.launch \
  global_eta:=2.0 local_eta:=0.5 info_radius:=1.0 info_multiplier:=3.0
```

机器人或仿真、GMapping 与 move_base 已经启动时，只启动探索节点：

```bash
roslaunch aubo_mobile_navigation rrt_exploration.launch
```

探索完成后仍使用项目已有入口保存地图：

```bash
roslaunch aubo_mobile_navigation map_saver.launch
```

RRT 探索使用在线 GMapping 地图，不要同时启动 `navigation.launch` 中的 AMCL。
需要中止探索时可取消当前 `/move_base` 目标并停止该 launch；重新启动探索后需要
再次发布四个边界点和一个起点。

## 检查命令

```bash
rostopic hz /front/scan
rostopic hz /rear/scan
rostopic hz /scan
rosrun tf tf_echo odom base_footprint
rostopic echo -n 1 /map
rostopic echo -n 1 /move_base/status
rostopic echo -n 1 /filtered_points
```

正常情况下 TF 发布关系为：

- 差速底盘插件发布 `odom -> base_footprint`
- 建图时 GMapping 发布 `map -> odom`
- 加载已有地图导航时 AMCL 发布 `map -> odom`

不要同时运行 GMapping 和 AMCL，否则二者会同时尝试发布 `map -> odom`，产生
TF 冲突。
