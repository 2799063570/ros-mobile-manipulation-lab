# AUBO 复合机器人导航分拣任务

该功能包把已有模块组合成一个完整场景：

```text
机械臂收回 down 安全姿态
→ move_base 导航到分拣工位
→ 机械臂进入相机观察位
→ 复用 aubo_mobile_sorting 完成红、绿、蓝方块分拣
→ 发布最终任务结果
```

底层建图、导航、MoveIt、视觉和抓取逻辑仍分别由
`aubo_mobile_navigation`、`aubo_mobile_moveit_config`、
`aubo_mobile_perception` 和 `aubo_mobile_sorting` 提供，本包只负责场景与任务编排。

## 1. 编译

```bash
cd ~/wheeltec_robot
catkin_make --force-cmake
source devel/setup.bash
```

## 2. 仿真建图

```bash
roslaunch aubo_mobile_nav_sorting mapping_gazebo.launch
```

使用键盘或 RViz 控制机器人遍历场景后保存地图：

```bash
rosrun map_server map_saver -f ~/nav_sorting
```

包内已经提供与仿真世界配套的 `maps/nav_sorting.yaml`，因此直接测试任务时不必重新建图。

真实机器人已经启动底盘和激光雷达后，使用以下入口建图：

```bash
roslaunch aubo_mobile_nav_sorting mapping.launch
```

## 3. 一键启动导航分拣场景

```bash
roslaunch aubo_mobile_nav_sorting mission_gazebo.launch
```

系统稳定后执行：

```bash
rosservice call /nav_sorting/start
```

需要启动后自动执行一次：

```bash
roslaunch aubo_mobile_nav_sorting mission_gazebo.launch auto_start:=true
```

停止当前任务：

```bash
rosservice call /nav_sorting/stop
```

可查看两个状态话题：

```bash
rostopic echo /nav_sorting/state
rostopic echo /sorting/state
```

任务状态依次为 `STOWING_ARM`、`NAVIGATING`、`AT_WORKSTATION`、
`SORTING`、`SUCCEEDED`；失败时为 `FAILED`。

## 4. 修改新场景

- 工位导航目标、超时和重试：`config/scenario.yaml`
- 当前工位相对底盘的桌面与放置坐标：`config/sorting.yaml`
- Gazebo 房间、障碍物、桌子和方块：`worlds/nav_sorting.world`
- 配套静态地图：`maps/nav_sorting.yaml` 与 `maps/nav_sorting.pgm`
- 抓取、放置、颜色顺序：`aubo_mobile_sorting/config/sorting.yaml`

也可在启动时临时覆盖工位：

```bash
roslaunch aubo_mobile_nav_sorting mission_gazebo.launch \
  goal_x:=2.15 goal_y:=0.0 goal_yaw:=0.0
```

真实机器人上应先启动底盘驱动、传感器、机械臂控制器与机器人描述，然后执行：

```bash
roslaunch aubo_mobile_nav_sorting navigation_sorting.launch \
  map_file:=/绝对路径/现场地图.yaml sorting_config:=/绝对路径/实机分拣参数.yaml \
  goal_x:=工位X goal_y:=工位Y goal_yaw:=工位朝向
```

实机入口会关闭 Gazebo 专用的临时吸附插件。真实场地必须使用现场生成的地图，并将
工位位姿标定为机械臂能够覆盖工作台、底盘又不会碰撞工作台的位置。
