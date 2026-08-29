# AUBO 复合机器人导航分拣任务

该功能包把已有模块组合成一个完整场景：

```text
机械臂收回 transport（低重心 A 形折叠）运输姿态
→ move_base 导航到分拣工位
→ 机械臂进入相机观察位
→ 通过 aubo_mobile_sorting 场景入口调用通用分拣核心
→ 发布最终任务结果
```

底层建图、导航和 MoveIt 分别由 `aubo_mobile_navigation`、
`aubo_mobile_moveit_config` 提供；视觉检测、抓取状态机和 Gazebo 辅助插件分别由
`aubo_perception`、`aubo_sorting_core`、`aubo_gazebo_plugins` 提供。移动端的参数
和组合入口仍由 `aubo_mobile_perception`、`aubo_mobile_sorting` 提供，本包只负责
导航到指定工位后的场景与任务编排。

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
roslaunch aubo_mobile_bringup simulation.launch mode:=mission
```

调试本场景时仍可直接运行
`roslaunch aubo_mobile_nav_sorting mission_gazebo.launch`。

该仿真场景在源方块正上方 `(x=2.82, y=0, z=2.0 m)` 单独加载一台垂直向下的
RGB-D 相机。相机固定在场景中，不连接移动底盘，也不修改通用机器人 URDF；机器人
导航时，相机通过 `map` 坐标系保持固定。

系统稳定后执行：

```bash
rosservice call /nav_sorting/start
```

需要启动后自动执行一次：

```bash
roslaunch aubo_mobile_bringup simulation.launch mode:=mission auto_start:=true
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

任务状态依次为 `STOWING_ARM`、`NAVIGATING`、`PREPARING_ARM`、`COORDINATING`、
`VALIDATING_DOCK`、`AT_WORKSTATION`、`SORTING`、`SUCCEEDED`；失败时为
`FAILED`。

到达预停靠点后，任务先调用 `/sorting/prepare_work`，刷新桌子碰撞物并将机械臂从
`transport` 运输姿态切换到末端向下的 `work_ready` 工作准备姿态，再进行低速精停。
每段底盘运动开始前机械臂均已停止，两者不会同时运动。精停完成后任务调用
`/sorting/move_to_observation`，机械臂从 `work_ready` 移动到 `observe` 相机观察
姿态，再开始检测。仿真手部相机使用 90° 水平视场，检测节点从
`/hand_camera/camera_info` 实时读取相机内参。

近场协同默认启用。底盘先到 `pre_dock_goal`，再对
`near_field_candidate_x/y/yaw` 的组合进行评分，淘汰桌边间距不足或无法覆盖全部
方块的位姿。每个候选精停后，系统用真实 MoveIt 规划场景执行观察动作，并要求
红、绿、蓝检测同时可见；失败时自动回到 `transport` 并尝试下一个候选。放置点
使用 `map` 坐标，在执行时转换到当前 `base_link`，因此小幅横移或转角不会造成
放置偏差。最终抓取 TCP 位于 40 mm 方块的几何中心；可通过
`config/sorting.yaml` 的 `grasp_height_offset` 做毫米级实机标定。

## 4. 修改新场景

- 工位导航目标、超时和重试：`config/scenario.yaml`
- 当前工位相对底盘的桌面与放置坐标：`config/sorting.yaml`
- Gazebo 房间、障碍物、桌子和方块：`worlds/nav_sorting.world`
- 配套静态地图：`maps/nav_sorting.yaml` 与 `maps/nav_sorting.pgm`
- 当前导航工位的抓取、放置和颜色顺序：本包 `config/sorting.yaml`
- 普通移动分拣场景的默认参数：`aubo_mobile_sorting/config/sorting.yaml`
- 通用分拣动作流程：`aubo_sorting_core`（不保存工位坐标）

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
