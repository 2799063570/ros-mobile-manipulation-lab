# simple_diff_robot_gazebo

## 激光跟随、颜色跟随和循线

本包复用 `aubo_mobile_follower` 的三个控制节点，但直接使用本车的 `/scan`、
`/camera/image_raw` 和 `/cmd_vel`，不启动机械臂或 MoveIt。每次只运行一种模式，
不要同时启动导航、键盘控制或其他向 `/cmd_vel` 发布速度的节点。

在工作空间根目录编译并加载环境后，任选其一：

```bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
catkin_make
source devel/setup.bash

# 前方测试立柱的激光跟随，默认保持约 1 m 距离
roslaunch simple_diff_robot_gazebo laser_follow.launch

# 例如将激光跟随目标距离设为 1.5 m
roslaunch simple_diff_robot_gazebo laser_follow.launch target_distance:=1.5

# 前视相机跟随 Gazebo 红色色块
roslaunch simple_diff_robot_gazebo color_follow.launch

# 将与红色目标正面的间隔设为约 1.5 m
roslaunch simple_diff_robot_gazebo color_follow.launch target_distance:=1.5

# 下视相机沿弯曲黑线行驶，线离开视野后停车
roslaunch simple_diff_robot_gazebo line_follow.launch
```

没有桌面环境时追加 `gui:=false start_rviz:=false`。三个入口默认各自启动 Gazebo、
RViz 和专用场景；
若 Gazebo 已经运行，可用 `start_robot:=false`，并确认现有机器人话题和场景与所选模式匹配。
激光入口可用 `spawn_test_target:=false start_target_control:=false` 关闭测试立柱及其
RViz 交互控制；在 RViz 选 **Interact** 可拖动测试立柱或红色目标。颜色入口的场景自带红色目标，
外部场景没有它时可用 `start_target_control:=false` 关闭目标交互节点。

循线入口用 `camera_pitch:=0.8` 将相机向地面倾斜，其他入口保持原有前视安装。
默认的循线 `line_steering_sign:=-1.0` 与本车相机左右方向匹配。若使用外部相机，
应检查其画面方向，并按需覆盖这两个参数。颜色和循线可追加
`start_rqt_reconfigure:=true`，在 `/simple_diff_follower` 调 HSV、速度和 PID；
初始参数来自 `aubo_mobile_follower/config/follower.yaml`。循线速度由
`follow_base.launch` 覆盖，当前设为 0.40 m/s。

颜色模式传入正数 `target_distance` 时，使用 `/camera/camera_info` 的焦距与红色目标
的已知宽度估计距离，单位为米；当前启动默认值为 `0.5`。
显式传入 `target_distance:=0.0` 可使用原来的画面面积控制。
演示目标宽 0.36 m，换成其他目标时需同时设置 `target_width:=实际宽度`。
估计值发布在 `/simple_diff_robot/color_distance`。目标偏离画面中心时，小车先原地
转向对准，再按距离前进；目标被画面裁切或相机标定信息不可用时仍可朝可见目标
转向，但不会前进。单目估距属于近似值，设定距离应留出车体与目标的安全余量。
米数控制接近目标时，以至少 0.08 m/s 发送有效的前进命令；进入目标距离
±0.05 m 后停车，只有误差再次超过 0.10 m 才重新起步，避免估距小幅波动造成
反复蠕动。可用 `min_approach_speed` 和 `distance_resume_deadband` 调整这两个参数。

颜色目标消失但相机仍在更新时，小车先等待 0.5 秒，再以 0.30 rad/s 左右交替
原地搜索；首次向左转 4 秒，随后向右转 8 秒，继续往复。若搜索超时设得比换向
周期短，换向周期会自动缩短，确保左右方向都得到搜索机会。重新识别到目标就恢复
跟随；搜索超过 20 秒仍未找到则停车。相机图像超时、机械臂未就绪（AUBO 模式）
时都不会盲转。可用
`search_enabled:=false` 关闭搜索，或用 `search_angular_speed`、
`search_switch_period`、`search_start_delay`、`search_timeout` 调整搜索行为。
状态话题会发布 `color_searching`、`color_aligning`、`color_following`、
`color_waiting_for_range` 或 `color_target_lost`。

状态、调试图像或标记、行驶轨迹分别发布到
`/simple_diff_robot/follower_state`、`/simple_diff_robot/follower_debug`
（激光为 `/simple_diff_robot/laser_debug`）和 `/simple_diff_robot/path`。
可用 `rqt_image_view` 查看相机调试图像。速度先经过激光安全过滤器，再送到底盘；
传感器超时会停车；激光目标丢失也会停车。仅在排查过滤器时才用
`use_laser_safety:=false` 将控制速度直接转发给底盘。

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

导航、边建图边导航和 RRT 探索均使用 `global_planner/GlobalPlanner` 全局规划器和 TEB 局部规划器，参数分别见
`config/global_planner.yaml` 与 `config/teb_local_planner.yaml`。
运行前可用 `rospack find global_planner` 检查全局规划器插件是否已安装；在 ROS Noetic 下缺少时安装
`ros-noetic-global-planner`。
运行前可用 `rospack find teb_local_planner` 检查插件是否已安装；在 ROS Noetic 下缺少时安装 `ros-noetic-teb-local-planner`。

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
5. 依次实验激光雷达、相机、Gmapping、AMCL、代价地图和 TEB。
6. 掌握现有链路后，可继续增加 IMU、传感器融合或视觉算法。

## 运行前检查

```bash
rospack find simple_diff_robot_gazebo
rosrun xacro xacro --inorder $(rospack find simple_diff_robot_gazebo)/urdf/simple_diff_robot.xacro > /tmp/simple_diff_robot.urdf
check_urdf /tmp/simple_diff_robot.urdf
```

遇到启动、TF、地图或导航问题时，参见 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)。
