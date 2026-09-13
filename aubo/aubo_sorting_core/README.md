# AUBO 通用分拣核心

该包只维护一份 MoveIt 抓取分拣状态机。固定底座和移动底盘的控制器名称、桌面
坐标、观察姿态、放置区域及 Gazebo 抓取辅助开关都由各场景 YAML 参数传入。

请从 `aubo_sorting` 或 `aubo_mobile_sorting` 的 launch 文件启动，不要直接
启动核心脚本。

核心同时提供 Python 和 C++ 两个实现。固定平台和移动平台的分拣 launch 默认
使用 `color_sorting_task_cpp`。`continuous_sorting=true` 时，历史 Python 入口也转交给
C++ 执行器并保留 ROS 名称/命名空间参数，避免维护两份并发实现。仅当场景 YAML 设置
`continuous_sorting: false` 时，`task_executable:=color_sorting_task.py` 才运行旧 Python 流程。C++ 类声明位于
`include/aubo_sorting_core/color_sorting_task.hpp`，节点入口位于 `src/color_sorting_task_node.cpp`，实现按职责拆分如下。

## 程序职责与维护入口

本包保留为固定和移动平台共用的执行层，`aubo_sorting` 保留为固定平台场景层。
合并会让移动分拣依赖固定平台场景，因此当前不合并。

| 文件 / 程序 | 职责 |
| --- | --- |
| `src/color_sorting_task_node.cpp` / `color_sorting_task_cpp` | 默认 ROS 节点入口，初始化并运行任务对象 |
| `include/aubo_sorting_core/color_sorting_task.hpp` | 分拣任务类、状态及接口声明 |
| `src/color_sorting_task.cpp` | 生命周期、ROS 服务、异步任务调度与状态发布 |
| `src/task_parameters.cpp` | 参数读取、基本参数校验和机械臂关节限制检查 |
| `src/target_tracking.cpp` | 检测订阅、坐标转换、目标缓存及多帧确认 |
| `src/instance_tracking.cpp` | 最新帧后台线程、实例过滤、连续抓取调度、空场景确认 |
| `include/aubo_sorting_core/instance_queue.hpp` | 不依赖 ROS 的实例匹配、状态、预留和失效规则 |
| `src/workspace_manager.cpp` | 工作区配置、桌面碰撞体及 OctoMap 更新 |
| `src/motion_executor.cpp` | MoveIt 规划与执行、轨迹时间参数化、抬升重试和夹爪控制 |
| `src/grasp_attachment.cpp` | Gazebo 抓取辅助插件的状态、吸附和释放 |
| `src/pick_place.cpp` | 单目标抓放顺序及一轮分类分拣流程 |
| `src/task_utils.hpp/.cpp` | 包内部共用的可中断等待、JSON 转义和参数转换 |
| `include/aubo_sorting_core/height_recovery.hpp` | 抓取后抬升的有界高度重试规则 |
| `scripts/color_sorting_task.py` | 可选 Python 实现，用于回退和对照；不与 C++ 节点同时启动 |

`color_sorting_task` 是历史命名，当前任务通过检测消息和类别参数支持颜色及 YOLO
分拣。选择检测器和加载场景由上层 launch 完成，核心不执行图像识别或底盘导航。
这些 C++ 文件共同构建 `color_sorting_task` 库，由一个任务对象共享状态和锁；
它们不是单独启动的节点。公共头文件、节点名及 `/sorting/*` 接口保持不变。

Python 与 C++ 存在重复实现，修改共同抓放逻辑时需要同步检查；Inspire 夹爪仅支持 C++。

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
碰撞体、抓取高度和放置点。切换桌子会清空目标。连续模式每次开始都建立新的采样周期，
从桌面实际剩余物体恢复；旧模式同一桌重试才按完成颜色跳过。

## 连续感知与实例队列

默认 `continuous_sorting: true`。感知回调只投递最新一帧；独立线程维护实例，机械臂
操作线程顺序抓放。队列不积压图像，也不在运动期间持有目标锁。同类多个物体按
类别和 XY 距离一对一匹配，短距离重复检测去重，多帧稳定后可领取。队列最多保留
200 个实例，过期记录在新有效帧到来时清理；ID 在节点生命周期内不复用。

状态为 `candidate / ready / reserved / done / review`。领取时复制完整抓取信息，
执行坐标冻结，其他物体继续更新。预抓取运动前和下降前复核检测流新鲜度及已观测位移，
不把后台坐标直接写入在途轨迹。遮挡短暂保留；明显跳变重新确认；同类新位置出现时，
未匹配旧目标取消就绪以防旧坐标残留。匹配只能依据几何，不保证交叉/重叠物体的身份。
预留后的最后观测允许保留至 `queue_reserved_max_age`：手眼相机在预抓取运动中可能
被夹爪遮挡，原来的 10 秒普通目标有效期可能短于一次预抓取动作。若期间看到目标
明显位移，或有效检测流中断超过 `queue_frame_max_age`，仍拒绝下降。这个宽限只
适用于已预留目标，不允许领取过期的新目标。

抓放完成后等待一帧释放后的有效检测，再领取仍然有效的实例；不强制回观察位。
队列不可用时短暂等待（`target_cache_fallback_delay`，默认 2 秒），随后回观察位并
重新确认。只有在观察位收到至少 `queue_empty_min_frames` 帧有效空检测、持续
`queue_empty_confirmation` 秒且检测仍新鲜，才结束。感知失联、无效深度、无效 TF、
不满足抓取几何的物体均不能作为桌面清空证据；观察超时报告 `DETECTION_FAILED`。
这里的“有效空检测”仍依赖相机覆盖抓取区，不能证明视野外或被其他物体完全遮挡的
区域为空。观察姿态必须在部署时确认覆盖需要处理的桌面。

目标在 `target_frame` 中保存，使用图像时间戳做严格 TF 转换。抓取区域为 `table_frame`
下桌面 XY 边界，排除**所有类别**放置点周围 8 cm；因此源物体不能放在这些排除区中。
夹爪闭合成功后才排除 TCP 邻域及 TCP 沿相机射线投影到桌面的邻域，降低手中物体
重新入队的概率。闭合前保留这些检测，以便预抓取运动期间继续确认预留目标。
桌高投影会受遮挡和深度误差影响，这不是物体分割或抓取成功传感器的替代品。

每次抓取保留 OctoMap（如启用）和桌面碰撞体更新；底盘锁继续由现有操作接口维持。
开始、人工重新观察、切换工作区均重建缓存，不沿用上次停靠的抓取坐标。执行失败或
急停退出当前操作，待复核后重新观察和开始，不自动在夹持状态不明时抓下一个。
`done` 表示抓放动作成功；无夹持反馈时不声称已经通过视觉确认抓取成功。

主要参数（距离单位 m，时间单位 s）：

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `target_cache_min_observations` | 5 | 多帧稳定确认 |
| `queue_match_distance` | 0.04 | 同类实例关联上限 |
| `queue_stable_distance` | 0.015 | 位置稳定阈值 |
| `queue_reserved_distance` | 0.035 | 已预留目标在预抓取运动中的位移判定阈值，须介于稳定和关联阈值之间 |
| `queue_duplicate_distance` | 0.008 | 同帧去重半径，应小于物体间距 |
| `queue_max_age` | 10 | 目标最后可靠观测的有效期 |
| `queue_frame_max_age` | 1 | 源图像及有效检测流的新鲜度上限 |
| `queue_reserved_max_age` | 30 | 已预留目标在预抓取运动中的最后观测有效期，必须不小于 `queue_max_age` |
| `queue_confirmation_gap` | 1 | 超过此间隔重新累计确认 |
| `queue_retention` | 30 | 未预留实例的保留时间 |
| `queue_done_hold` | 2 | 完成后原位置的短暂重入隔离 |
| `queue_empty_confirmation` / `queue_empty_min_frames` | 2 / 5 | 观察位空场景确认条件 |
| `queue_place_exclusion_radius` | 0.08 | 放置点排除半径 |
| `queue_gripper_exclusion_radius` | 0.10 | 夹爪及其桌面投影排除半径 |
| `queue_height_tolerance` | 0.04 | 桌面物体中心高度检查；depth 模式同时应用 `height_tolerance` |

`target_cache_fallback_enabled / frame / max_age / outlier_distance` 仅用于旧模式，不影响
新实例队列。降低 `queue_max_age` 会增加重新观察频率；如果 YOLO 推理超过 1 秒，需要
根据实测延迟设置 `queue_frame_max_age`，不能通过重写图像时间戳规避过期检查。

`/sorting/target_cache` 连续模式输出 `schema_version: 2`，`targets` 的键由类别改成
实例 ID，每项包含 `id/color/status/position/observations/confidence/age/picked`。
`confidence` 是稳定确认进度，不是检测模型概率；第三方按颜色解析的面板需适配 ID。

### 重新编译和验证

`DetectedObjectArray.msg` 新增 `observation_valid` 与 `sensor_frame`，ROS 消息 MD5 会改变。
必须一起重编译并重启感知、分拣、Gazebo 插件及其他使用该消息的节点。自定义感知节点
必须明确设置有效性和相机光学坐标系；默认 false 不会触发抓取或完成判定。

Gazebo 连续模式发送 `nearest:<配置模型名>`，插件在该名称或 `<名称>_<后缀>` 的模型中
选择夹爪附近实例，并保留原来的距离和横向对中检查。例：`red_block / red_block_1`。
需要重编译并重启 Gazebo 加载新插件；实机不使用该协议。放置仍使用现有每类单一位置，
多物体堆放容量和防碰撞摆放不属于此队列改造，需要按实际分拣容器配置。

在 ROS 工作区执行 `catkin_make`，构建测试后运行 `instance_queue_test`（或 CTest），
以及 `test_observation_contract.py` 和原有感知/抓取测试。硬件验证前在仿真中检查：
同类多目标、抓取中加入新目标、遮挡、目标位移、感知断流、停止恢复和切换工作区。
本机可独立编译 `test/instance_queue_test.cpp`，不需要 ROS；感知契约测试同样无需 ROS。

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

固定平台的 `pregrasp_wrist1_limit=1.2`（rad）仅在预抓取姿态规划期间对
`wrist1_joint` 施加相对零位的路径限制，避免相同末端位姿落到约 ±2.8 rad 的
翻腕逆解；抓取下降、放置、观察和回收不沿用这项限制。设为 `0` 可禁用。
限制必须包含当前起始姿态，默认 `observe` 的 wrist1 约为 -0.91 rad。
目标在限制内仍不可达时应报告规划失败，不应放宽约束后自动执行。

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
