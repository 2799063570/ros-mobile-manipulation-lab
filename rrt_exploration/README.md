# rrt_exploration

该 ROS 功能包基于快速扩展随机树（RRT）实现单机器人和多机器人地图探索，以占据栅格作为地图表示。它包含全局/局部 RRT 边界点检测器、OpenCV 边界点检测器、过滤器和任务分配器五类节点。

[演示视频列表](https://www.youtube.com/playlist?list=PLoGH52eUIHsc1B_xPLL6ogzYxrWy675kr)展示了 Kobuki 真机和 Gazebo 仿真中的单机器人、多机器人运行效果。

本功能包最初由作者在沙迦美国大学攻读硕士期间开发。如果在研究中使用，请引用[对应论文](http://ieeexplore.ieee.org/document/8202319/)。

## 1. 环境要求

该功能包已在 ROS Indigo 和 Kinetic 上测试，其他相近版本也可能适用。运行前需要：

1. ROS Indigo 或更新版本（推荐 Indigo/Kinetic）。
2. 已创建的 Catkin 工作空间。
3. GMapping 和 Navigation Stack。
4. Python 2.7，以及 OpenCV、NumPy、scikit-learn。

以 ROS Kinetic 为例：

```bash
sudo apt-get install ros-kinetic-gmapping ros-kinetic-navigation
sudo apt-get install python-opencv python-numpy python-scikits-learn
```

## 2. 安装

把功能包放入 Catkin 工作空间的 `src` 目录，然后在工作空间根目录执行：

```bash
catkin_make
```

如需快速体验，可结合 [`rrt_exploration_tutorials`](https://github.com/hasauino/rrt_exploration_tutorials) 中的单机器人和多机器人 Gazebo 示例使用。

## 3. 机器人配置

### 3.1 网络

多机器人模式只需要一个 ROS Master。其他机器人应通过 `ROS_MASTER_URI` 指向运行 Master 的计算机，并确保所有主机可相互解析和通信。

### 3.2 TF 坐标系

每台机器人的坐标系必须通过命名空间区分，例如 `robot_1/base_link`、`robot_2/base_link`。所有节点和传感器 TF 都应使用相同的机器人前缀，避免多机器人坐标系冲突。

### 3.3 节点和话题命名

机器人相关节点与话题应放入 `robot_x` 命名空间，其中 `x` 为机器人编号。例如第一台机器人的导航接口为 `robot_1/move_base`。

### 3.4 导航栈

每台机器人都必须启动 Navigation Stack，并能独立接收目标、规划路径和发布全局代价地图。开始自主探索前，建议先在 RViz 中手动下发目标验证导航链路。

### 3.5 建图与地图融合

单机器人可直接使用 GMapping 等节点发布的地图。多机器人需要把各机器人局部地图融合为全局地图，并将该地图配置给检测器、过滤器和任务分配器。

## 4. 节点结构

边界检测器产生候选探索点，`filter` 删除无效或重复点，`assigner` 根据收益和行驶代价把目标分配给各机器人。

![探索策略结构](https://github.com/hasauino/storage/blob/master/pictures/fullSchematic.png)

### 4.1 `global_rrt_frontier_detector`

全局 RRT 检测器在占据栅格中扩展随机树并发布边界点。多机器人模式通常只运行一个实例；需要提高检测速度时也可增加实例。

参数：

- `~map_topic`（字符串，默认 `/robot_1/map`）：输入地图话题。
- `~eta`（浮点数，默认 `0.5` 米）：RRT 单次扩展步长。较大值检测更快，但可能遗漏狭小区域。

订阅：

- `~map_topic`：`nav_msgs/OccupancyGrid` 地图。
- `clicked_point`：由 RViz 的“发布点”工具依次发布 5 个点；前 4 个定义探索区域，第 5 个作为树的起点。

发布：

- `detected_points`：检测到的 `geometry_msgs/PointStamped` 边界点。
- `~shapes`：用于在 RViz 中显示 RRT 的 `visualization_msgs/Marker`。

### 4.2 `local_rrt_frontier_detector`

局部检测器每发现一个边界点就重置树，用于快速搜索机器人附近区域。多机器人模式下每台机器人运行一个实例，并与全局检测器共同向 `/detected_points` 发布。

参数：

- `~robot_frame`（默认 `/robot_1/base_link`）：机器人坐标系，重置后从当前机器人位置开始扩展。
- `~map_topic`（默认 `/robot_1/map`）：地图话题。
- `~eta`（默认 `0.5` 米）：局部 RRT 扩展步长。

订阅和发布接口与全局检测器基本相同。

### 4.3 `frontier_opencv_detector`

该节点使用 OpenCV 从地图中检测边界点，不依赖 RRT。它原本用于和 RRT 方法对比，也可与全局/局部检测器并行运行。多机器人模式通常只需一个实例。

- 参数 `~map_topic`（默认 `/robot_1/map`）：地图话题。
- 订阅 `nav_msgs/OccupancyGrid` 地图。
- 发布 `detected_points` 和用于 RViz 显示的 `shapes`。

### 4.4 `filter`

过滤器汇总所有检测器产生的候选点，删除过期、不可达和重复点，再把有效目标发送给任务分配器。

参数：

- `~map_topic`（默认 `/robot_1/map`）：用于判断边界点是否仍有效的地图。
- `~costmap_clearing_threshold`（默认 `70.0`）：代价值超过此阈值的点视为无效。
- `~info_radius`（默认 `1.0` 米）：计算信息增益时使用的半径。
- `~goals_topic`（默认 `/detected_points`）：候选点输入话题。
- `~n_robots`（默认 `1`）：机器人数量。
- `~rate`（默认 `100` Hz）：节点循环频率。

过滤器还会订阅每台机器人的 `robot_x/move_base_node/global_costmap/costmap`。所有机器人命名空间都必须以 `robot_x` 开头。

发布：

- `frontiers`：全部候选边界点的 RViz 标记。
- `centroids`：过滤后边界点的 RViz 标记。
- `filtered_points`：发送给任务分配器的 `PointArray`。

### 4.5 `assigner`

任务分配器接收过滤后的边界点，通过 Actionlib 向各机器人的 `move_base_node` 下发目标。

参数：

- `~map_topic`（默认 `/robot_1/map`）：单机器人时为本机地图，多机器人时应为融合后的全局地图。
- `~info_radius`（默认 `1.0` 米）：信息增益计算半径。
- `~info_multiplier`（默认 `3.0`）：信息增益相对于预计行驶距离的权重。
- `~hysteresis_radius`（默认 `3.0` 米）和 `~hysteresis_gain`（默认 `2.0`）：滞回区域及增益。
- `~frontiers_topic`（默认 `/filtered_points`）：过滤后边界点话题。
- `~n_robots`（默认 `1`）：机器人数量。
- `~delay_after_assignement`（默认 `0.5` 秒）：每次目标分配后的等待时间。
- `~rate`（默认 `100` Hz）：节点循环频率。

该节点订阅地图和 `PointArray`，本身不发布普通话题，而是作为 Actionlib 客户端向 `move_base` 动作服务器发送目标。
