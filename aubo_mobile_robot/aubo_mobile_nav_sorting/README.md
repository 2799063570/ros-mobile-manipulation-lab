# AUBO 复合机器人导航分拣任务

该功能包把已有模块组合成单工位或多工位完整场景：

```text
机械臂收回 transport（低重心 A 形折叠）运输姿态
→ move_base 导航到分拣工位
→ 机械臂进入相机观察位
→ 通过 aubo_mobile_sorting 场景入口调用通用分拣核心
→ 发布最终任务结果
```

多工位模式在每张桌子之间使用 `move_base`；只有当分拣核心明确发布
`PLANNING_FAILED` 时，任务节点才停止使用导航数据，向 `/cmd_vel_raw` 发布短距离、
单轴的 `linear.x`/`linear.y` 速度脉冲，并在每一步后重新请求 MoveIt 求解。

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

任务编排节点同时提供 Python 和 C++ 两种实现，对外话题、服务和 YAML 参数兼容。
默认继续使用 Python；选择 C++ 实现时执行：

```bash
roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch \
  mission_implementation:=cpp
```

通用 `mission.launch` 对应参数为 `implementation:=python|cpp`。两个实现不能同时启动，
因为它们有意提供相同的 `/nav_sorting/start`、`/nav_sorting/stop` 和状态话题。
原单桌入口 `mission_gazebo.launch` 和实机入口 `navigation_sorting.launch` 也支持
`mission_implementation:=cpp`。

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

到达预停靠点后，机械臂保持 `transport` 运输姿态，底盘进行低速精停。
精停完成后，任务调用 `/sorting/prepare_work`，刷新桌子碰撞物并将机械臂切换到
末端向下的 `work_ready` 工作准备姿态。每段底盘运动开始前机械臂均已停止。
随后任务调用 `/sorting/move_to_observation`，机械臂从 `work_ready` 移动到 `observe` 相机观察
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

## 5. 四张桌子的第一阶段配置

四桌 Gazebo 场景已经包含四张桌子、每桌三种颜色方块、配套静态地图和连续任务入口：

场地范围为 `14 x 8 m`。机器人从左侧 `(-4, 0)` 出发，纵向隔墙连接北侧边界，
因此前往第一桌时必须从墙体南端绕行；四桌之间保留了足够的局部规划和转向空间。

```bash
roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch
rosservice call /nav_sorting/start
```

四桌场景默认使用适合差速底盘和窄通道轨迹优化的 TEB 局部规划器；需要和原来的
DWA 做对照或回退时，可以在启动时选择：

```bash
roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch local_planner:=teb
roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch local_planner:=dwa
```

TEB 和 DWA 共用相同的全局路径、预停靠点与最终直线精停逻辑，参数分别位于
`config/four_tables_navigation.yaml` 的 `TebLocalPlannerROS` 和 `DWAPlannerROS`
命名空间。

也可以用 `auto_start:=true` 自动开始。每个工位都先由 `move_base` 到预停靠点，随后用
低速 `/cmd_vel_raw` 直行精靠。每桌完成后的固定顺序为：机械臂回到 `transport`
移动姿态、底盘通过 `/cmd_vel_raw` 后退 0.30 m、再导航到下一桌；最后一桌也会后退。
后退过程读取 TF 闭环判断距离，并保留激光安全过滤及无进展超时。
四桌入口使用独立的 `config/four_tables_colors.yaml`，通过轮廓面积上限排除桌面上的
大尺寸红绿蓝放置区，只发布 40 mm 待抓取方块。

`config/four_tables.yaml` 同时可作为实机四工位模板。Gazebo 中的坐标已经标定，迁移到
真机时仍必须按现场 `map` 重新标定：

```bash
roslaunch aubo_mobile_nav_sorting mission.launch \
  config:=$(rospack find aubo_mobile_nav_sorting)/config/four_tables.yaml
rosservice call /nav_sorting/start
```

每个 `workstations` 条目包含：

- `navigation_goal_frame`：预停靠和精停目标的标定坐标系；Gazebo 固定桌面使用
  `odom`，避免 AMCL 的 `map -> odom` 修正改变机器人与桌子的物理间距；
- `pre_dock_goal`：交给 `move_base` 的安全预停靠位姿；
- `navigation_goal`：从预停靠点低速直行到达的最终工作位姿 `[x, y, yaw]`；
- `table_center/table_size/table_z`：桌子碰撞体与抓取高度；
- `place_targets`：该桌红、绿、蓝放置点；
- `objects`：第一阶段的已知物体清单，后续感知模块可更新；
- `grasp_model_names`：Gazebo 中当前桌红、绿、蓝方块的唯一模型名；
- `retreat_enabled/retreat_distance`（可选）：覆盖该桌完成后的后退策略；
- `enabled`（可选）：是否跳过该桌。

当前工位的完整 JSON 会锁存发布到 `/nav_sorting/current_workstation`。任务节点同时把
配置写入 `/sorting/workspace_config` 并调用 `/sorting/configure_workspace`，因此后续
感知模块可以复用同一接口更新桌子/物体信息。物体检测结果仍使用现有强类型接口
`/sorting/detections`（`aubo_perception/DetectedObjectArray`）。

底盘搜索由 `base_recovery_steps` 配置，每项为当前 `base_link` 下的增量 `[dx, dy]`，
必须有且仅有一个非零轴。速度命令经过现有激光安全过滤器；停止、执行失败、检测失败
或夹爪失败都不会触发底盘搜索，只有机械臂规划失败会触发。

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

## 6. 异常退出与验证

Python 和 C++ 任务节点采用相同的取消规则：

- `/nav_sorting/stop` 快速返回“停止请求已接受”；任务线程负责后续取消和确认。
  初始化等待和动作状态等待会响应停止，底盘控制等待不再依赖仿真时钟继续走动。
- `sorting_operation_timeout` 包含分拣服务调用及动作执行等待。超时或停止时，
  尚未确认结束的动作会调用 `/sorting/stop`；只有收到本次动作之后的终态
  （`READY`、`ERROR` 或 `STOPPED`）以及底盘解锁消息，才认为分拣核心已退出动作。
  这是软件接口的确认，不是机械臂硬件急停反馈。
- `sorting_stop_timeout` 默认 5 秒，使用单调时钟计时，涵盖停止服务与退出确认。
  未确认退出时发布 `STOP_UNCONFIRMED`，拒绝新任务，RViz 面板也禁止重新开始。
- ROS 1 服务请求不能由调用方强制撤销。请求在超时后仍未返回时保留启动锁定；
  涉及分拣的迟到请求返回后会再次请求停止。应先排除服务故障、处理悬而未决的
  请求并确认设备停止，再重启任务节点；不要仅靠重启任务节点来清除异常。
- `sorting_base_lock_topic` 默认 `/sorting/base_locked`，必须与分拣核心的
  `base_lock_topic` 一致。终态与本次动作后的解锁消息也用于正常动作完成判断，
  避免把核心尚在清理时发布的 `ERROR` 当作已经可以移动底盘。
- `base_pose_max_age` 默认 0.5 秒。精停和小角度对正会拒绝过旧、未来超过
  0.05 秒、零时间戳或无效坐标的 TF；查找失败也立即发布零速度。本次任务随后
  退出，不再通过其他候选或后退动作继续移动。修复定位后可以重新发起任务。

不依赖 ROS 的 Python 异常路径测试：

```bash
cd $(rospack find aubo_mobile_nav_sorting)
python3 -m unittest discover -s test -v
```

该测试使用模拟的 ROS 服务、状态消息和 TF，不代表 C++ 编译或机器人运动验证。
在 ROS 工作空间编译后，应分别使用 `mission_implementation:=python` 和 `cpp`
运行以下 Gazebo 验收：

1. 正常完成一个工位，确认动作成功路径仍可运行。
2. 等待初始化、等待导航服务、导航途中及机械臂动作中分别请求停止。
3. 缩短 `sorting_operation_timeout`，确认先请求取消，再发布 `FAILED`；
   模拟停止服务无响应或不发布退出确认，确认状态为 `STOP_UNCONFIRMED` 且开始被拒绝。
4. 在精停及航向对正时中断 TF 更新，确认输出零速度、任务失败，并且不尝试其他候选。
5. 底盘运动中暂停 Gazebo，确认墙上时间超时仍能退出控制循环；恢复仿真前检查任务已退出。

本轮没有更改恢复移动的开环距离策略、导航算法或 RViz 服务调用的线程模型。
