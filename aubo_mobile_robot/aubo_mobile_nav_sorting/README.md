# AUBO 复合机器人导航分拣任务

本包源码现位于 `src/aubo_mobile_robot/aubo_mobile_nav_sorting`，与移动平台功能包一起维护。
ROS 包名仍为 `aubo_mobile_nav_sorting`，启动命令与 RViz 插件标识不变；移动平台资源通过包依赖引用。

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
默认使用 C++；显式选择时执行：

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

四桌场景（颜色和 YOLO 配置）已将最终停靠点沿各自朝向前移 5 cm，直接采用原先
前移重试的位置：table_1 `[1.50, 1.20, 1.5708]`、table_2 `[4.20, 2.00, 0]`、
table_3 `[5.00, -1.20, -1.5708]`、table_4 `[2.30, -2.00, 3.14159]`。
桌子、物体和放置点的世界坐标不变，机械臂目标仍通过 TF 转换到当前底座坐标系。

四桌场景最终精靠容差为 0.015 m，避免提前 8 cm 判定到达。
预停点 TEB/DWA 的位置容差同步收紧到 0.01 m，减少后续直线无法修正的侧向误差。
颜色和 YOLO 配置开启 `pregrasp_forward_recovery_enabled: true`（C++/Python 均支持）：
只有 `PLANNING_FAILED | <颜色> pre-grasp` 且操作结束、底盘解锁后才允许恢复。
先收臂至 transport，再读取当前桌坐标系下的实际底盘位姿，每次闭环前移 0.03 m，
最多两次；目标必须更接近桌子且底盘中心距轴对齐桌边至少 0.35 m。
每次重新观察和检测，保留已完成颜色。导航分拣配置关闭旧目标缓存兜底。
执行失败、抓取/放置失败、停止请求及无法收臂均不会触发此恢复。
这是有边界的可达性重试，不是 IK/碰撞根因分类器；仍失败时停止并检查目标与碰撞。
通用旧搜索保持 `base_recovery_enabled: false`、`base_recovery_steps: []`。
全部分拣成功后的收臂及 0.30 m 离桌后退仍然执行。

其他场景若显式启用底盘搜索，可通过 `base_recovery_steps` 配置当前 `base_link`
下的增量 `[dx, dy]`。速度命令经过现有激光安全过滤器；只有机械臂规划失败才可能
触发该恢复流程。

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

两种实现保留基础取消规则；本次执行对象拆分与在线停止恢复仅用于默认 C++ 实现，
Python 保留为旧版对照实现：

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
  请求并确认设备停止。C++ 使用 `/nav_sorting/recover_stop` 重新验证；旧 Python 实现
  仍需确认停止后重启任务节点，不能直接清除异常。
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

### YOLO、真机夹爪及四工位面板

真机入口（移动底盘驱动、机器人模型及 TF 应先启动）：

```bash
roslaunch aubo_mobile_nav_sorting navigation_sorting.launch \
  robot_ip:=192.168.1.2 gripper_port:=/dev/ttyUSB0 camera_serial_no:=""
```

入口默认启动 D435i 可用的 `realsense2_camera/rs_camera.launch`，开启彩色、
深度和深度对齐；相机安装外参使用 `eye_to_hand_camera_real.launch` 中的标定文件。
默认启动现有 `aubo_control.launch`（内部链接 aubo_sdk），MoveIt 执行地址为
`/aubo_i5/aubo_i5_controller/follow_joint_trajectory`。只启动六轴和关节状态控制器，
夹爪通过 Inspire 串口服务控制，不启动夹爪 ros_control 控制器。
已有机械臂/夹爪驱动时分别设置 `start_arm_driver:=false`、`start_gripper:=false`。

YOLO 使用 `/home/zlab/deepL/code/ultralytics-main-modify/weights/GC-yolo.pt`，
启动 Ultralytics 推理和 RGB-D 桥。任务标签是 `bottle / can / box`；默认放置点在
`aubo_mobile_sorting/config/yolo_sorting.yaml`，分别是 base_link 下
`[0.45, -0.20]`、`[0.45, 0.0]`、`[0.45, 0.20]` 米，使用前按实际放置区调整。
保持原来的桌面抓取流程和设定抓取高度，深度用于反投影获得目标位置。

桥保存最近 90 张对齐深度，在 YOLO 推理结束后按原始 RGB 图像时间戳选最近深度，
误差大于 `maximum_depth_age` 则拒绝该检测。真机默认把匹配的原始深度、编码、
相机内参和 RGB/深度时间戳保存到 `~/sorting_depth/*.npz`；
`depth_save_directory:=` 可关闭磁盘保存，内存缓存继续工作。
这些文件用于深度回放；它们不是包含机器人 TF 的完整 rosbag。

```bash
roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch perception_mode:=color
roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch perception_mode:=yolo
```

本仓库的 RealSense 包没有 Gazebo 插件。仿真使用现有
`libgazebo_ros_openni_kinect.so` RGB-D 相机，发布同样的彩色、对齐深度和 CameraInfo。
YOLO 模式使用 `four_tables_yolo.yaml` 的类别及默认放置区；现有 world 是彩色方块，
验证 bottle/can/box 识别需要替换为对应外观模型，不能假定该权重能识别彩色方块。

`NavSortingPanel` 显示各工位的目标坐标和待执行/执行中/已完成/失败状态。
多工位模式禁用单工位坐标编辑，开始按钮按配置顺序执行工位列表。

本次在 `/tmp/sorting_validation` 中编译核心、Inspire 驱动和面板，避免修改原工作空间
的 build/devel。深度缓存/反投影单元测试和原任务安全测试通过；未执行真机运动或
Gazebo 完整抓取验收。

## 7. C++ 执行对象与线程约定

默认启动 C++ 实现。`aubo_mobile_nav_sorting_execution` 导出可复用执行库，
节点入口独立为 `navigation_sorting_mission_node.cpp`：

| 对象 | 负责内容 |
| --- | --- |
| `ArmExecutor` | `home / prepare / observe / sort`、工位配置、状态订阅、停止确认 |
| `BaseExecutor` | move_base 目标、TF、航向校正、直线精靠／退让、零速度与取消 |
| `NavigationSortingMission` | 工位顺序、近场候选选择、机械臂与底盘执行顺序、恢复策略 |
| `MissionContext` | 参数加载与校验、共享状态、取消信号及状态输出 |

其他 C++ 节点可以构造 `MissionContext(nh, private_nh)`，再组合所需执行器，链接
`aubo_mobile_nav_sorting_execution`。调用执行方法的工作线程必须唯一；停止底盘
和 ROS 回调可并发。Context 必须比执行器活得更久，销毁执行器前停止 spinner 并
等待工作线程退出。组合两种执行器时应设置 `context.stop_base` 为底盘的 `stopBase()`；
单独复用机械臂执行器时默认回调为空。可选设置 `context.state_publisher_` 接收任务状态。

任务状态为 `enum class MissionState`，下游分拣状态为 `enum class SortingState`。
字符串只在 ROS 消息边界解析／序列化，未知状态不能作为成功或停止确认。
原有 `STATE | detail` 话题格式保持兼容。

并发访问规则：

- `lifecycle_mutex_` 串行化启动、停止恢复的受理和工作线程替换。
- Context 的 `mutex_` 保护分拣状态、故障、底盘锁消息及序号；动态参数在同一把锁下
  检查 `busy_`。任务／恢复执行期间拒绝修改，工作线程读取参数不必逐项加锁。
- 停止请求、未确认标志和悬而未决 RPC 数量使用原子变量。RPC 线程持有自己的数据，
  不捕获任务对象；迟到请求及其补发停止都结束后才允许恢复。
- 底盘 `command_mutex_` 保护 action client 与速度发布；发送前检查停止和底盘锁。
  与共享状态同时加锁时固定先 Context、再 command；不持锁进行 TF 查询或运动等待。
- `operation_active_`、`base_pose_failed_` 为唯一执行线程的局部执行状态，
  不供订阅回调读写，不需要再添加线程锁。

### 停止未确认的恢复

排除下游服务故障后，点击 RViz 的“重新确认停止”，或执行：

```bash
rosservice call /nav_sorting/recover_stop "{}"
```

服务返回“已接受”只表示开始验证。只有旧 RPC（包括迟到补发的停止）全部结束、
重新调用 `/sorting/stop` 成功，并收到本次验证之后的新终态及新解锁消息，
才清除 `stop_unconfirmed_` 并发布 `STOPPED`。失败仍保持 `STOP_UNCONFIRMED`；
恢复操作不会自动启动任务。分拣核心在已初始化且空闲时也会响应停止并发布新确认，
因此本包和 `aubo_sorting_core` 需要一起重新编译。

### SRDF 与命名姿态

导航分拣配置使用以下映射，关节值统一来自移动机器人 SRDF：

| 调用 | 配置参数 | SRDF 名称 |
| --- | --- | --- |
| `/sorting/home` | `finish_named_target` | `transport`（导航时收拢） |
| `/sorting/prepare_work` | `work_ready_named_target` | `work_ready`（到站后准备） |
| `/sorting/move_to_observation` | `observation_named_target` | `observe` |

`home` 是服务名称，不是要求 SRDF 存在名为 `home` 的姿态。当前 SRDF 的 `work_ready`
与 `down` 关节值相同，是不同任务语义的别名。本次没有修改机器人姿态关节角度。
分拣核心启动时核对实际加载的规划组命名目标；缺失时报告 `CONFIGURATION_FAILED`
并拒绝就绪，执行时也检查 `setNamedTarget` 的返回值。

### 自动验证

```bash
catkin_make --pkg aubo_mobile_nav_sorting aubo_sorting_core
catkin_make run_tests_aubo_mobile_nav_sorting
catkin_test_results
```

新增测试包括状态枚举边界、配置与 SRDF 的名称／关节覆盖，以及真实 C++ 节点配合
模拟 ROS 服务／action 的任务、取消、恢复和并发路径。`rostest` 使用独立 master，
不需要连接机器人；测试日志保存在 `/tmp/nav_mission_test_*.log`。这些测试不代替
Gazebo 中的路径、碰撞和实际到位验证。


四桌仿真中，40 mm 方块使用 `gripper_closed: 0.20` rad，指间几何间隙约 42.45 mm。
世界插件同时将吸附距离限制为 6 cm，并拒绝横向偏心超过 8 mm 的吸附请求；该门限
用于避免明显误抓，不是接触力检测。插件打印实际偏心距离，供后续 TCP/感知标定。
修改后需重启 Gazebo 和任务节点；其他尺寸物体应重新标定闭合角。

### 分拣完成后返回任务起点

默认启用 `return_to_start: true`。每次任务在分拣节点就绪后、首次移动之前，记录
当前底盘在 `return_frame` 中的位置和朝向（默认使用 `navigation_frame`，场景配置
为 `map`）。起点由本次实际位置决定，不硬编码为 Gazebo 出生坐标；新任务重新记录。

四桌流程为：全部工位分拣成功 → 最后一桌收臂并离桌 → 确认机械臂收拢、底盘解锁
→ `move_base` 返回记录的位置和朝向。返航期间状态为 `RETURNING_TO_START`，
RViz 显示“正在返回任务起点”；只有返航成功才发布 `SUCCEEDED`。

起始 TF 缺失或过旧时，在任何任务运动前退出。任务失败、停止未确认和手动停止
不触发自动返航；返航本身失败时报告 `FAILED` 并说明“分拣完成但返航失败”。
返航期间仍可使用原“停止当前任务”按钮或 `/nav_sorting/stop`，并沿用导航超时、
重试和机械臂底盘锁定规则。

参数在启动时读取，修改后重启任务节点生效：

```yaml
return_to_start: true
return_frame: map
```

设置 `return_to_start: false` 可恢复完成最后工位后停止的行为。C++ 与 Python
任务实现均支持以上参数。
