# AUBO 通用分拣核心

该包只维护一份 MoveIt 抓取分拣状态机。固定底座和移动底盘的控制器名称、桌面
坐标、观察姿态、放置区域及 Gazebo 抓取辅助开关都由各场景 YAML 参数传入。

请从 `aubo_sorting` 或 `aubo_mobile_sorting` 的 launch 文件启动，不要直接
启动核心脚本。

核心同时提供 Python 和 C++ 两个等价实现。固定平台和移动平台的分拣 launch 默认
使用 `color_sorting_task_cpp`；如需回退 Python 版本，可在 launch 命令后增加
`task_executable:=color_sorting_task.py`。C++ 类声明位于
`include/aubo_sorting_core/color_sorting_task.hpp`，实现和节点入口分别位于
`src/color_sorting_task.cpp` 与 `src/color_sorting_task_node.cpp`。

## 输入与输出

- 订阅 `/sorting/detections`，类型为
  `aubo_perception/DetectedObjectArray`；
- 连接 MoveIt 的 `aubo_i5` 规划组和平台配置的夹爪轨迹 action；
- 发布 `/sorting/state` 与 `/sorting/detection_summary`；
- 发布 `/sorting/failure`；无法生成非空 MoveIt 轨迹时以
  `PLANNING_FAILED | <目标>` 明确上报，执行器失败不会混为规划失败；
- 提供 `/sorting/move_to_observation`、`/sorting/prepare_work`、
  `/sorting/start`、`/sorting/stop`、`/sorting/open_gripper`、`/sorting/home` 和
  `/sorting/configure_workspace`。

`prepare_work` 主要供移动机器人到达工位后的任务编排使用，固定平台通常不调用。
`configure_workspace` 在节点空闲时从 `/sorting/workspace_config` 读取当前桌子的
碰撞体、抓取高度和放置点。切换桌子会清空本桌完成颜色记录；同一桌规划失败后重试
则跳过已经完成的颜色，避免重复抓取。

## 参数边界

以下差异必须由场景 YAML 提供，不能写进核心包：

- `gripper_action`、规划组和末端链接；
- `table_frame`、桌面碰撞体和目标坐标系；
- 观察、运输、工作准备及结束姿态；
- 抓取高度、放置点、速度和加速度；
- 是否启用 Gazebo 抓取辅助插件。

固定平台加载 `aubo_sorting/config/sorting.yaml`，移动平台加载
`aubo_mobile_sorting/config/sorting.yaml`；导航分拣场景可用
`aubo_mobile_nav_sorting/config/sorting.yaml` 覆盖移动平台默认值。

### 2026-09 分拣调整

C++ 实现内部使用 `enum class State`；`/sorting/state` 仍保留原字符串协议，
导航任务和 RViz 面板无需迁移消息类型。named pose 的关节值由 SRDF 定义，
`observation_named_target` / `work_ready_named_target` / `finish_named_target`
只选择 SRDF 名称。固定底座 SRDF 补齐了 `work_ready`。

只有 pick/place 操作失败才清理它持有的 Gazebo 吸附；观察、检测、准备、回零
失败不触发释放。真机不使用 Gazebo 吸附，规划失败不自动张开夹爪。
工作台通过同一 `sorting_table` ID 同步 ADD 替换，不清空其他节点的障碍物；
工作区配置参数存在时优先使用，只有参数不存在才使用话题缓存，参数非法直接报错。

`gripper_backend=trajectory` 用于 Gazebo，轨迹包含当前关节位置和目标点，
两端速度为零，默认运动时间 0.8 秒，可按仿真控制器响应调节。
`gripper_backend=inspire` 仅支持 `color_sorting_task_cpp`，调用
`/inspire_gripper/move_max`、`move_min`、`get_state`、`set_es`。
参数 `inspire_speed=500`、`inspire_force=100`、`inspire_motion_timeout=5.0` 可调整。
张开等待状态 1；闭合等待状态 2 或 6；超时/异常调用停止服务。
`inspire_gripper/get_state.srv` 新增 `motion_state`，需要一起重新编译驱动和任务节点。

导航分拣停止恢复：已初始化且空闲时再次调用 `/sorting/stop`，会发布新的
`STOPPED` 与 `base_locked=false`，供导航编排确认停止。C++ 初始化时检查配置的
named target 是否存在于实际加载的 SRDF 规划组，配置不一致时不会进入就绪。

`preplace_height` 为放置点上方的接近高度及放置后退离高度，相对 `table_z`；
未配置时继承 `lift_height`，保留原场景行为。抓取后的抬升仍由 `lift_height` 控制。
导航分拣场景设置 `preplace_height: 0.24`，即 `base_link` 下 Z=0.38 m，
抓取抬升保持 Z=0.40 m。

抓取后抬升支持有界高度重试：`lift_height` 是初始桌面相对高度，
`lift_min_height` 是允许的下限（默认等于初始高度，即不自动降低），
`lift_height_step` 是每次降低量（默认 0.02 m），`lift_max_attempts` 是包含首次在内的
最大尝试数（默认 5，范围 1–100）。上下限及步长必须为有限正数，下限不得高于初始
高度，也必须高于抓取 TCP 的桌面相对高度。
导航分拣配置使用 0.26 → 0.24 → 0.22 → 0.20 → 0.18 m；桌面 Z=0.14 m 时，
目标 Z 为 0.40 → 0.38 → 0.36 → 0.34 → 0.32 m。

只在抬升规划失败时降低目标，不重试控制器执行失败、停止或其他故障。抬升要求
100% 笛卡尔路径，部分路径不会执行，也不回退到可能绕行的关节空间运动；每次降低
目标重新规划并检查碰撞。成功后继续分拣，达到下限或次数上限仍失败则退出。
重试过程保留抓取吸附，最终失败沿用现有失败清理。此策略只作用于抓取后抬升，
放置前／放置后退离仍使用独立的 `preplace_height`，不自动改变底盘位置。
