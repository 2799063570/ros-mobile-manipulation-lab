# AUBO 通用分拣核心

该包只维护一份 MoveIt 抓取分拣状态机。固定底座和移动底盘的控制器名称、桌面
坐标、观察姿态、放置区域及 Gazebo 抓取辅助开关都由各场景 YAML 参数传入。

请从 `aubo_sorting` 或 `aubo_mobile_sorting` 的 launch 文件启动，不要直接
启动核心脚本。

核心同时提供 Python 和 C++ 两个实现。固定平台和移动平台的分拣 launch 默认
使用通用入口 `sorting_task_cpp`。旧入口 `color_sorting_task_cpp` 继续指向同一实现，
兼容已有 launch 和 `rosrun` 命令。`continuous_sorting=true` 时，历史 Python 入口也转交给
C++ 执行器并保留 ROS 名称/命名空间参数，避免维护两份并发实现。仅当场景 YAML 设置
`continuous_sorting: false` 时，`task_executable:=color_sorting_task.py` 才运行旧 Python 流程。C++ 类声明位于
`include/aubo_sorting_core/sorting_task.hpp`，节点入口位于 `src/sorting_task_node.cpp`，实现按职责拆分如下。

## 程序职责与维护入口

本包保留为固定和移动平台共用的执行层，`aubo_sorting` 保留为固定平台场景层。
合并会让移动分拣依赖固定平台场景，因此当前不合并。

| 文件 / 程序 | 职责 |
| --- | --- |
| `src/sorting_task_node.cpp` / `sorting_task_cpp` | 通用 ROS 节点入口，初始化并运行任务对象 |
| `color_sorting_task_cpp` | 旧 C++ 可执行文件名的兼容入口，与 `sorting_task_cpp` 使用同一源码和状态机 |
| `include/aubo_sorting_core/sorting_task.hpp` | 通用 `SortingTask` 分拣任务类、状态及接口声明 |
| `include/aubo_sorting_core/color_sorting_task.hpp` | 旧 C++ API 兼容头，将 `ColorSortingTask` 映射到 `SortingTask` |
| `src/sorting_task.cpp` | 生命周期、ROS 服务、异步任务调度与状态发布 |
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

`color_sorting_task` 库名、`color_sorting_task_cpp` 可执行文件及
`/color_sorting_task` ROS 节点/参数命名空间均为兼容保留的历史名称；新的默认可执行
文件为 `sorting_task_cpp`，C++ 核心类为 `SortingTask`。当前任务通过检测消息和类别参数支持颜色及 YOLO
分拣。选择检测器和加载场景由上层 launch 完成，核心不执行图像识别或底盘导航。
这些 C++ 文件共同构建 `color_sorting_task` 库，由一个任务对象共享状态和锁；
它们不是单独启动的节点。公共头文件、节点名及 `/sorting/*` 接口保持不变。

Python 与 C++ 存在重复实现，修改共同抓放逻辑时需要同步检查；Inspire 夹爪仅支持 C++。

## 入口命名与兼容性

新代码和新 launch 应使用通用名称：

```bash
rosrun aubo_sorting_core sorting_task_cpp
```

旧命令仍然有效，并运行相同的 `SortingTask` 实现：

```bash
rosrun aubo_sorting_core color_sorting_task_cpp
```

| 项目 | 推荐名称 | 兼容名称／说明 |
| --- | --- | --- |
| C++ 源码入口 | `src/sorting_task_node.cpp` | 旧文件 `src/color_sorting_task_node.cpp` 已更名 |
| C++ 可执行文件 | `sorting_task_cpp` | `color_sorting_task_cpp` 继续构建和安装 |
| C++ 任务类 | `SortingTask` | `ColorSortingTask` 头文件别名继续可用 |
| ROS 节点及私有参数空间 | `/color_sorting_task` | 暂不更名，保证现有 YAML、面板和外部编排兼容 |
| 内部共享库 | `color_sorting_task` | 暂不更名，保证已有链接方兼容 |

launch 参数 `task_executable` 默认是 `sorting_task_cpp`；需要验证旧部署时仍可显式传入
`task_executable:=color_sorting_task_cpp`。颜色与 YOLO 不需要分别创建任务节点，两者通过
统一的 `/sorting/detections` 消息和 `sort_classes` 等参数选择行为。

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

可使用 [rqt 分拣工作台](../aubo_sorting_rqt/README.md) 查看过程和实例队列、调用任务控制、在线调参。
`/sorting/progress` 发布观察、接近、下降、夹紧、抬升、放置、释放及退离的当前步骤。
`target_cache_min_observations` 和 `queue_max_age` 与速度、加速度、XY 修正及检测超时共用
dynamic_reconfigure 接口；执行中拒绝修改，队列设置改变后要求目标重新积累稳定观测。

默认 `continuous_sorting: true`。感知回调只投递最新一帧；独立线程维护实例，机械臂
操作线程顺序抓放。队列不积压图像，也不在运动期间持有目标锁。同类多个物体按
类别和 XY 距离一对一匹配，短距离重复检测去重，多帧稳定后可领取。队列最多保留
200 个实例，过期记录在新有效帧到来时清理；ID 在节点生命周期内不复用。
`sort_classes` 只筛选允许类别；默认 `queue_selection_policy: fifo` 按入队 ID 从小到大选择新鲜的就绪实例，
类别列表顺序不影响领取顺序。未就绪、过期或类别不允许的目标不会阻塞后续就绪实例。

实机场景启用 `queue_selection_policy: top_first`，先剔除 XY 投影重叠且被更高表面覆盖的目标，
再从最高可抓表面以下 `queue_top_height_band`（默认 1 cm）范围内按模型检测置信度选择，ID 打破平分。
覆盖阈值为 `queue_stack_overlap`（默认 0.15）和 `queue_stack_height_gap`（默认 1 cm）。
新鲜的候选/预留/待复核上层也会阻挡下层，完成或过期的上层不再阻挡；下降前再次检查覆盖。
该策略要求 depth 模式，使用表面 Z，并以三维距离关联叠放实例，避免同 XY 的上下层串号。
它依赖可见检测与投影重叠，不是完整的接触关系或手指可达性判断。

重复实例按世界坐标和检测框投影合并：中心三维距离 ≤ `queue_merge_distance`（默认 0.02 m），
且两个凸投影框的交集面积 / 较小框面积 ≥ `queue_merge_overlap`（默认 0.5）。
框顶点来自 `DetectedObject.footprint`，和检测中心使用同一坐标系。
同帧保留高检测置信度的完整数据，一帧最多计一次稳定观测；合并不会改变该组的 FIFO 入队位置。
跨帧类别闪变沿用实例 ID，较低置信度的新类别不能立即取代旧类别；新类别胜出时重新确认，
历史高置信度数据仍受 `queue_max_age` 限制。缺少投影框时不做跨类别合并。
已预留实例的类别和抓放坐标保持冻结；相同物体的重分类不直接取消预留，真实位移仍触发保护。

状态为 `candidate / ready / reserved / done / review`。领取时复制完整抓取信息，
执行坐标冻结，其他物体继续更新。预抓取运动前和下降前复核检测流新鲜度及已观测位移，
不把后台坐标直接写入在途轨迹。遮挡短暂保留；明显跳变重新确认；同类新位置出现时，
未匹配旧目标取消就绪以防旧坐标残留。匹配只能依据几何，不保证交叉/重叠物体的身份。
预留后的最后观测允许保留至 `queue_reserved_max_age`：手眼相机在预抓取运动中可能
被夹爪遮挡，原来的 10 秒普通目标有效期可能短于一次预抓取动作。若期间看到目标
明显位移，或有效检测流中断超过 `queue_frame_max_age`，仍拒绝下降。这个宽限只
适用于已预留目标，不允许领取过期的新目标。

预抓取使用 `APPROACH` 阶段：按检测采集时间查询相机和机器人链路 TF，用
`queue_occlusion_links` 原点间的胶囊包络（半径 `queue_occlusion_radius`，默认 0.06 m）
判断机器人是否挡住相机到冻结目标的可见表面中心、投影角点或边中点的视线。
确认遮挡时冻结整个实例队列，只保留新鲜源帧心跳，
不把遮挡框的类别/位置写回目标，也不把空检测当作桌面清空。可见时仍正常跟踪和检查位移。
遮挡解除要求连续 `queue_occlusion_clear_frames`（默认 3）帧均未被包络遮挡，减少边缘闪烁。
`queue_reserved_anomaly_confirmations` 控制确认预留关联变化所需的异常观测数，
通用默认 1，实机配置为 3。首个异常立即暂停下降，并标记 `association_pending`；
只有原类别、有效几何且位于 `queue_stable_distance` 内连续
`queue_reserved_recovery_observations`（默认 3）次观测才能解除尚未确认的异常。
确认的位移/隔离故障保持锁定，不因恢复或遮挡清除。连续性仍受 `queue_confirmation_gap` 约束。
下降前的关联复核在队列锁外等待，最多 `queue_reserved_recheck_timeout`（默认 2 s），
停止请求可中断。未恢复、无有效数据或已确认变化均拒绝下降；已确认遮挡不能替代异常后的重新确认。
下降前允许已确认遮挡使用源帧心跳，但仍检查遮挡 TF 时效、预留目标最大年龄、已有位移故障和停止请求。
这不能检测完全被挡住期间的外力移动；遮挡例外有 `queue_reserved_max_age` 上限，不会无限延长。
相机帧可能比关节 TF 超前一个反馈周期。跟踪线程在队列锁外最多等待
`queue_tf_wait_timeout`（默认 0.15 s），等同一图像采集时刻所需的相机、手臂及夹爪 TF 齐全。
等待期间只保留当前帧和最新待处理帧；停止、阶段切换或源图像过期会取消等待。
运动部件不回退到最新 TF，不重写图像时间戳；确实缺失的 TF 超时后仍触发原有下降前检查。
下降/夹紧/抬升及放置/释放/退离使用 `MASKED` 阶段；切换阶段按源时间戳建立新纪元，丢弃推理迟到帧。
实测工位配置提供 AUBO 手臂链路列表；其他机器人需配置自己的链路及包络半径。

`DetectedObject.msg` 增加了投影框字段，修改后需重新编译，并同时重启感知、任务和 rqt 进程，
避免旧消息 MD5 与新节点不一致。不会自动重启实机或触发运动。

观察运动开始时建立新队列纪元，因此从初始位到观察位以及到位静止期间的
有效帧都能累积稳定目标；进入连续分拣时不再二次清空。抓放完成后保留本轮安全
时段收集的其他实例；只要检测源仍新鲜且目标未超过 `queue_max_age`，即可直接领取，
不强制回观察位。
队列不可用时短暂等待（`target_cache_fallback_delay`，默认 2 秒），随后回观察位并
重新确认。只有在观察位收到至少 `queue_empty_min_frames` 帧有效空检测、持续
`queue_empty_confirmation` 秒且检测仍新鲜，才结束。感知失联、无效深度、无效 TF、
不满足抓取几何的物体均不能作为桌面清空证据；观察超时报告 `DETECTION_FAILED`。
这里的“有效空检测”仍依赖相机覆盖抓取区，不能证明视野外或被其他物体完全遮挡的
区域为空。观察姿态必须在部署时确认覆盖需要处理的桌面。

目标在 `target_frame` 中保存，使用图像时间戳做严格 TF 转换。抓取区域为 `table_frame`
下桌面 XY 边界，排除**所有类别**放置点周围 8 cm；因此源物体不能放在这些排除区中。
夹爪闭合成功后才排除 TCP 邻域及 TCP 沿相机射线投影到桌面的邻域，降低手中物体
重新入队的概率。观察位到预抓取位、上升位到预放置位允许更新队列；预抓取下降、
抓取上升、预放置下降及退离期间只更新检测源心跳，不改动实例轨迹。
桌高投影会受遮挡和深度误差影响，这不是物体分割或抓取成功传感器的替代品。

每次抓取保留 OctoMap（如启用）和桌面碰撞体更新；底盘锁继续由现有操作接口维持。
人工重新观察、切换工作区均重建缓存，不沿用上次停靠的抓取坐标；从已完成的观察直接
开始分拣则继承当前队列。执行失败或
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
| `queue_merge_distance` / `queue_merge_overlap` | 0.02 / 0.5 | 跨类别合并的中心三维距离上限及交集/较小框面积比例 |
| `queue_occlusion_radius` / `queue_occlusion_links` | 0.06 / [end_effector_link] | 接近阶段遮挡判断的机器人胶囊半径与有序链路列表 |
| `queue_max_age` | 10 | 目标最后可靠观测的有效期 |
| `queue_frame_max_age` | 1 | 源图像及有效检测流的新鲜度上限 |
| `queue_tf_wait_timeout` | 0.15 | 在队列锁外等待源时刻 TF 到齐的墙钟时间上限，不得超过 queue_frame_max_age |
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
实例 ID，每项包含 `id/category/color/status/position/observations/confidence/detection_confidence/stability/age/picked`；
`color` 是为旧面板保留的 `category` 别名。
`detection_confidence` 是所选模型检测概率；`stability` 是稳定确认进度，旧字段 `confidence` 保留该进度含义。
每项还包含缓存检测的 `grasp_geometry_valid/grasp_angle/grasp_width`。
角度以顶层 `frame_id` 的 XY 平面为准，单位 rad；宽度为检测得到的夹取方向尺寸，单位 m。
几何无效、数值非有限或宽度非正时，相应数值输出 JSON `null`。
预留后的角度和宽度随执行快照冻结，与本次抓取使用的缓存检测一致。
顶层还包含 `vision_phase/target_occluded/merged_detections_total`。第三方按颜色解析的面板需适配 ID。

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
`gripper_backend=inspire` 仅支持 C++ 入口（推荐 `sorting_task_cpp`，兼容
`color_sorting_task_cpp`），调用
`/inspire_gripper/move_max`、`move_min`、`get_state`、`get_openlimit`、`get_copen`、`set_es`。
参数 `inspire_speed=500`、`inspire_force=100`、`inspire_motion_timeout=5.0` 可调整。
预抓取、放置释放和手动全开均发送 `move_max`，达到状态 1 后还需回读开度：
实际开度与驱动最大限位的误差不超过 `inspire_open_tolerance`（默认 5，协议单位为 0–1000）才继续。
闭合等待状态 2 或 6；超时/异常调用停止服务。`gripper_open: 0.0` 是任务层的张开端点，
Inspire 后端将它映射到最大开度，并非发送协议开度 0。启动、停止或失败时不自动张开。
`inspire_gripper/get_state.srv` 新增 `motion_state`，需要一起重新编译驱动和任务节点。

`minimum_gripper_height` 是抓取及放置下降终点的最低 TCP 离桌高度，默认 `0.06 m`。
抓取 Z 取 `max(物体中心Z + grasp_height_offset, table_z + minimum_gripper_height)`；
实机可选择 `grasp_height_reference: surface`，改用
`max(可见表面Z - grasp_surface_inset, table_z + minimum_gripper_height)`，要求有效 depth 模式。
`stacked_grasp_offset_z`（默认 0，范围 ±0.05 m）给上层目标额外增加 TCP Z 修正；
判定阈值 `stacked_support_min_height`（默认 0.02 m，可调 0.005–0.30 m）与
`中心Z−自身厚度/2−table_z` 比较，依赖原有厚度先验。只有 depth + allow_stacked_objects 时启用，
中心参考或表面参考抓取均可应用。修正叠加在 TCP 相对中心偏移上，放置沿用同一偏移。
两者通过 dynamic_reconfigure 提供，空闲时允许更新、执行中拒绝并回传原值。
感知中心/表面、上层优先排序不随修正而改变；修正后的抓取/放置仍须通过接近高度检查。
默认 inset 为 1 cm，描述 TCP 相对表面的几何关系；中心 Z 与自身物高仍单独发布。
抓放使用相同 TCP 相对中心偏移，目的放置不继承源物体的支撑高度。
放置还保留 `minimum_place_z` 的绝对高度下限。该参数必须小于预抓取、最低抬升及预放置高度，
可在空闲时通过 dynamic_reconfigure/rqt 修改。它以 `end_effector_link`（实机为 `tcp_link`）
控制点为准，不代表手指最低几何点，也不替代运动路径碰撞检查。已有小物体仿真 YAML
显式设为 `0.0`，保留原来的低物体抓取高度。

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

## 按类别统计已完成分拣

C++ 任务节点通过 latched `std_msgs/String` 话题 `/sorting/completed_counts` 发布统计。
节点启动时按 `sort_classes` 初始化为零；一次抓取、放置及退离流程成功返回后，对该类别加一。
失败、中途停止或仅接收开始请求不计入；统计表示控制流程成功完成，不是额外视觉验收结果。
连续实例队列与按类别任务共用同一完成记录点，实验审计开启时也使用这一记录点。

示例报文（示例数字，不代表当前实际产量）：

```json
{"schema_version":1,"scope":"node_session","session":"任务节点运行标识","counts":{"bottle":1,"can":2,"box":3},"total":6}
```

范围为 **本次任务节点运行期间累计**。重新观察、队列清理、目标过期、重新点击开始和界面重连都不清零；
重启任务节点后清零，运行标识随之改变。没有历史回填或跨重启持久化。
UI 直接显示最新统计快照，重复收到 latched 消息不会再次累加。
`queue_done` / 队列中 `done` 仍是当前保留的队列目标数，与累计完成数量分开显示。
类别沿用当前配置，颜色任务是 red/green/blue，YOLO 示例是 bottle/can/box。
