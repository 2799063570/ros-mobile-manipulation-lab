# AUBO 复合机器人导航分拣 —— 整体控制流程说明

> 本文档面向 `aubo_mobile_nav_sorting` 功能包，说明一台搭载 AUBO i5 机械臂的移动底盘，
> 如何**自主导航到工位 → 精停对位 → 视觉检测 → 分拣方块 → 移动到下一工位 → 返回起点**这一完整任务的编排逻辑。
>
> 配套总图见 [`src/navigation_sorting_mission_flowchart-v2.png`](src/navigation_sorting_mission_flowchart-v2.png)。

---

## 1. 这个包做什么

`aubo_mobile_nav_sorting` 是整个移动分拣系统的**任务编排层（顶层导演）**。它本身不建图、不跑
MoveIt、不做视觉识别，而是把已有模块按固定时序串成一个完整的"导航 + 分拣"任务：

```
机械臂收至 transport 运输姿态（低重心 A 形折叠）
   → move_base 导航到分拣工位的"预停靠点"
   → 底盘低速直行 / 小角度对正，精确靠到工作位
   → 机械臂进入 work_ready 准备姿态 → observe 相机观察姿态
   → 调用 aubo_sorting_core 完成颜色识别与抓放
   → 收臂、后退离桌
   → （多工位）导航到下一桌
   → 全部完成后收臂并返回任务起点
```

底层能力由其他包提供，本包只负责"什么时候调用谁、出错了怎么退"：

| 能力 | 提供方 |
| --- | --- |
| 建图、`move_base` 导航、costmap | `aubo_mobile_navigation` |
| MoveIt 机械臂规划、SRDF 命名姿态 | `aubo_mobile_moveit_config` |
| 视觉检测（颜色 / YOLO） | `aubo_perception` / `aubo_mobile_perception` |
| 抓取状态机、抓放动作 | `aubo_sorting_core` / `aubo_mobile_sorting` |
| Gazebo 世界与吸附插件 | `aubo_gazebo_plugins` |

任务节点同时提供 **C++（默认）** 和 **Python（旧版对照）** 两种实现，对外话题、服务、YAML 参数完全兼容；二者不能同时启动。

---

## 2. 程序对象与线程模型（C++ 实现）

入口节点 `navigation_sorting_mission_node.cpp` 构造一个 `NavigationSortingMission`，其内部由四个对象组成：

| 对象 | 职责 |
| --- | --- |
| `MissionContext` | 加载并校验 YAML 参数；保存任务级共享状态；提供取消信号、状态发布、状态字符串解析 |
| `BaseExecutor` | `move_base` 目标下发、TF 查询、航向小角度校正、低速直行精靠 / 后退、零速度急停 |
| `ArmExecutor` | 调用分拣核心的 `/sorting/*` 服务，订阅 `/sorting/state`、`/sorting/base_locked` 确认动作结束 |
| `NavigationSortingMission` | 工位遍历顺序、近场候选位姿评分、机械臂与底盘的执行时序、恢复策略 |

**线程约定**：

- ROS 主线程跑 `AsyncSpinner(4)`，负责服务、订阅、动态参数回调。
- 收到 `/nav_sorting/start` 后，`submitMission()` 才新建唯一的 `mission_thread_` 执行体；
  `busy_ == true` 时拒绝重复启动。所有"机械臂在做什么、底盘在做什么"的顺序决策都在这一个工作线程里串行完成。
- `lifecycle_mutex_` 串行化启动 / 停止 / 恢复的受理；`mutex_` 保护分拣状态与底盘锁消息；
  停止请求、未确认标志、悬而未决 RPC 计数用原子变量。
- **每一段底盘运动开始前，机械臂一定已经停止并回到 `transport`**；每一段机械臂动作期间，
  底盘由 `/sorting/base_locked` 锁定不动。这是"臂动底盘锁、底盘动臂收拢"的硬安全规则。

---

## 3. 任务状态机

任务级状态定义在 `mission_state.h` 的 `enum class MissionState`，通过 `/nav_sorting/state`
以 `状态 | 详情` 的字符串形式发布：

```
IDLE
 → INITIALIZING            等待分拣节点就绪
 → CONFIGURING_WORKSTATION  配置当前工位（写入 workspace_config）
 → STOWING_ARM              机械臂回零 / 收至 transport
 → NAVIGATING               move_base 前往预停靠点
 → DIRECT_DOCKING           低速直行精靠到最终工作位
   （中途可能出现 ALIGNING_BASE 小角度对正 / ADJUSTING_BASE 候选微调）
 → AT_WORKSTATION           已就位，验证机械臂可达
 → PREPARING_ARM            /sorting/prepare_work → work_ready
   → （分拣核心内部）OBSERVING / DETECTING / PICKING
 → SORTING                  /sorting/start，连续抓放
 → RETREATING_BASE          收臂后底盘后退 0.30 m
 → WORKSTATION_COMPLETE     当前工位完成，继续下一工位
 ... 所有工位完成后 ...
 → STOWING_ARM              最后一次收臂
 → RETURNING_TO_START       move_base 返回记录的起点
 → SUCCEEDED
```

失败 / 中断终态：

- `FAILED`：导航失败、分拣失败、TF 过旧等任何未恢复的错误。
- `STOPPED`：收到 `/nav_sorting/stop` 且下游正常确认退出。
- `STOP_UNCONFIRMED`：停止服务无响应 / 未收到下游解锁确认，此时**拒绝新任务**，需调用
  `/nav_sorting/recover_stop` 重新握手。

下游分拣核心自己的状态（`READY / IDLE / HOMING / PREPARING / OBSERVING / SORTING / DETECTING / PICKING / ERROR / STOPPED`）由 `ArmExecutor` 订阅并翻译成上面的任务状态。

---

## 4. 单工位 / 多工位的执行主循环

核心函数是 `runWorkstationSequence()`。它先从 `workstations` 配置里筛出 `enabled: true` 的工位，
然后**按配置顺序逐个**执行同一个循环：

```text
对第 i 个工位 (table_1 → table_2 → table_3 → table_4)：
  1. publishState(CONFIGURING_WORKSTATION, id)
  2. 若导航前需要收臂（home_before_navigation 或非第一个工位）
        → ArmExecutor.home()           即 /sorting/home → SRDF 姿态 transport
  3. ArmExecutor.configureWorkspace(workspace)
        → 把本工位 JSON（桌子尺寸、放置点、物体清单）写入 /sorting/workspace_config
        → 调用 /sorting/configure_workspace，让分拣核心更新 MoveIt 碰撞场景
  4. 导航到位：
        · 配置了 pre_dock_goal 时：
            BaseExecutor.navigate(pre_dock_goal)        # move_base 安全预停
            BaseExecutor.driveStraightTo(navigation_goal)  # 低速直行精靠（DIRECT_DOCKING）
        · 否则直接 navigate(navigation_goal)
  5. publishState(AT_WORKSTATION, "... validating arm reach")
  6. prepareAndObserveWithRecovery()
        → /sorting/prepare_work    机械臂到 work_ready（末端向下）
        → /sorting/move_to_observation  机械臂到 observe 观察位，触发视觉检测
        → 含近场候选评分与预抓取前移恢复（见第 5、6 节）
  7. publishState(SORTING, id)
  8. sortAtWorkspaceWithRecovery(workspace)
        → /sorting/start，分拣核心按 red → green → blue 连续抓放
        → 本工位所有方块抓完为止
  9. retreatAfterSorting(workspace)
        → ArmExecutor.home() 收臂到 transport
        → BaseExecutor 以 /cmd_vel_raw 后退 0.30 m（TF 闭环读距离）
 10. publishState(WORKSTATION_COMPLETE, id)，回到第 1 步处理下一工位
```

> 单工位场景（无 `workstations` 配置）走 `runMission()` 里的另一条分支：直接 `navigate` +
> `coordinateNearField()`（近场候选打分），然后 `sortWithRecovery()`，结构相同。

---

## 5. 近场协同停靠：为什么不是一步到位

`move_base` 的精度在桌子这种窄环境下不够，所以靠近桌子的最后 ~0.5 m **不走 move_base**，而是三段式：

1. **预停靠（pre_dock_goal）**：`move_base` 把底盘送到桌子前方约 0.5 m 的安全位置，状态 `NAVIGATING`。
2. **航向对正（ALIGNING_BASE）**：在安全距离外用低速 `cmd_vel` 做小角度闭环修正（最大 ±0.12 rad，约 6.9°），
   只校正朝向，不做位置移动。
3. **直行精靠（DIRECT_DOCKING）**：沿当前方向低速（`base_recovery_speed = 0.04 m/s`）直线前进到
   `navigation_goal`，用 TF 闭环判断到位，容差约 1.5 cm。

直行速度命令走 `/cmd_vel_raw` → **激光安全过滤层** → `/cmd_vel`，不会绕过急停；
超距 / 横向误差 / 无进展超时都会安全停下并退出。

在四桌等更复杂场景，还会对 `near_field_candidate_x/y/yaw` 的组合做**评分**：
底盘中心到桌边留出 ≥ 0.35 m 安全距离、相机视场能覆盖全部方块才入选；每个候选精停后用真实
MoveIt 规划场景跑一次观察，要求红 / 绿 / 蓝同时可见，失败则收臂回 `transport` 试下一个候选。

> 放置点坐标在世界坐标系（`map`/`odom`）下配置，执行时由 TF 实时转到当前 `base_link`，
> 因此精停的小偏差不会直接变成放偏。

---

## 6. 异常与恢复策略

任务在任意阶段都可被 `/nav_sorting/stop` 打断；执行过程中按错误类型有边界地重试：

| 异常 | 触发条件 | 恢复动作 |
| --- | --- | --- |
| **导航失败** | move_base 返回失败 / 超时 180 s | `clear_costmaps` 后重试 1 次，仍失败 → `FAILED` |
| **预抓取规划失败** | `/sorting/failure` 报 `PLANNING_FAILED` 且为 `<颜色> pre-grasp` | 收臂回 `transport`，在桌子坐标系下每次闭环前移 3 cm，最多 2 次；要求更靠近桌子且底盘距桌边 ≥ 0.35 m；重新观察检测，保留已完成颜色 |
| **分拣动作超时** | 单次分拣操作超过 `sorting_operation_timeout`（300 s） | 先调 `/sorting/stop`，等待本次动作后的终态与底盘解锁消息，再发布 `FAILED` |
| **TF 过旧 / 无效** | 精停 / 对正读到 > 0.5 s 旧 TF 或未来时间戳 | 立即发零速，本工位放弃，不再尝试其他候选，任务 `FAILED` |
| **停止未确认** | `/sorting/stop` 超时（5 s）未回，或未收到解锁 | 进入 `STOP_UNCONFIRMED`，拒绝新任务；调 `/nav_sorting/recover_stop` 重新握手 |

通用的"底盘在 base_link 下乱挪"搜索（`base_recovery_steps`）在导航分拣场景默认**关闭**，
只保留有明确边界的预抓取前移重试，避免无规则挪动。

---

## 7. 对外 ROS 接口

### 服务（本包提供）

| 服务 | 作用 |
| --- | --- |
| `/nav_sorting/start` (`std_srvs/Trigger`) | 启动一次任务 |
| `/nav_sorting/stop` (`std_srvs/Trigger`) | 请求停止（立即返回"已接受"，实际取消在工作线程完成） |
| `/nav_sorting/recover_stop` (`std_srvs/Trigger`) | 停止未确认时重新握手 |

### 服务 / 动作（本包调用下游）

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/move_base` | action | 导航到预停靠点 / 最终工作位 / 返航 |
| `/move_base/clear_costmaps` | service | 导航失败后清代价图 |
| `/sorting/home` | Trigger | 机械臂到 `transport` 运输姿态 |
| `/sorting/prepare_work` | Trigger | 到 `work_ready` 工作准备姿态，刷新桌面碰撞体 |
| `/sorting/move_to_observation` | Trigger | 到 `observe` 观察位并触发检测 |
| `/sorting/start` | Trigger | 开始连续抓放 |
| `/sorting/stop` | Trigger | 请求分拣核心停止当前动作 |
| `/sorting/configure_workspace` | — | 下发当前工位桌子 / 物体 / 放置点配置 |

### 话题

| 话题 | 方向 | 内容 |
| --- | --- | --- |
| `/nav_sorting/state` | 发布 | `MissionState | 详情` |
| `/nav_sorting/current_workstation` | 发布 | 当前工位完整 JSON |
| `/sorting/state` | 订阅 | 分拣核心状态 |
| `/sorting/failure` | 订阅 | 分拣失败原因（如 `PLANNING_FAILED`） |
| `/sorting/base_locked` | 订阅 | `true` = 机械臂在用、底盘禁止动；`false` = 底盘可动 |
| `/sorting/detections` | 下游发布 | 检测到的物体（`DetectedObjectArray`） |
| `/cmd_vel_raw` | 发布 | 精靠 / 后退 / 恢复的低速速度指令（仍过激光安全层） |

---

## 8. 关键配置文件

| 文件 | 内容 |
| --- | --- |
| `config/scenario.yaml` | 单桌场景：导航目标、预停点、近场候选、精靠 / 对正阈值、超时、返航开关 |
| `config/sorting.yaml` | 单桌机械臂参数：抓取高度、夹爪开合、放置点、观察姿态、lift 高度重试 |
| `config/four_tables.yaml` | 四工位列表：每桌的 `pre_dock_goal` / `navigation_goal` / 桌子 / 放置点 / 物体 / 后退策略 |
| `config/four_tables_colors.yaml`、`four_tables_yolo.yaml` | 四桌场景的颜色 / YOLO 感知参数与预抓取前移恢复开关 |
| `config/four_tables_navigation.yaml` | TEB / DWA 局部规划器参数 |
| `maps/` | 配套静态地图（`nav_sorting`、`four_tables`、`artificial_map`） |
| `worlds/` | Gazebo 仿真世界 |

每个 `workstations` 条目里：

- `navigation_goal_frame`：预停与精靠坐标所在坐标系（Gazebo 固定桌用 `odom`，避免 AMCL 的
  `map→odom` 修正改变桌子与底盘的物理距离；真机一般用 `map`）。
- `pre_dock_goal`：交给 `move_base` 的安全预停位姿。
- `navigation_goal`：从预停点低速直行到达的最终工作位姿 `[x, y, yaw]`。
- `table_center / table_size / table_z`：桌子碰撞体与抓取高度。
- `place_targets`：该桌红 / 绿 / 蓝（或 YOLO 的 bottle/can/box）放置点。
- `enabled`：置 `false` 可跳过该工位。

---

## 9. 一次完整任务的时序串讲（以四桌 Gazebo 为例）

```text
 rosservice call /nav_sorting/start
        │
        ▼
 [INITIALIZING]  等分拣节点 READY（60 s 超时）；若 return_to_start，记录当前 base 在 map 下的位姿
        │
 ┌──────┴─────────────── table_1 ───────────────────────┐
 │ CONFIGURING_WORKSTATION  写 table_1 配置 → /sorting/configure_workspace
 │ STOWING_ARM              /sorting/home → transport
 │ NAVIGATING               move_base → pre_dock (1.50, 0.80, 90°)
 │ ALIGNING_BASE            小角度对正
 │ DIRECT_DOCKING           低速直行 → (1.50, 1.20, 90°)
 │ AT_WORKSTATION           验证机械臂可达
 │ PREPARING_ARM            /sorting/prepare_work → work_ready
 │ (OBSERVING)              /sorting/move_to_observation → observe，红绿蓝检测
 │ SORTING                  /sorting/start：抓红→放红，抓绿→放绿，抓蓝→放蓝
 │ RETREATING_BASE          收臂 transport → 底盘后退 0.30 m
 │ WORKSTATION_COMPLETE
 └──────────────────────────────────────────────────────┘
        │  table_2 → table_3 → table_4 重复同一循环
        ▼
 [STOWING_ARM]             最后一桌收臂
 [RETURNING_TO_START]      move_base 回到任务开始时记录的位姿
        ▼
 [SUCCEEDED]
```

任何一步失败：`/nav_sorting/stop` 取消导航、发零速、调 `/sorting/stop`，然后落到
`FAILED` / `STOPPED` / `STOP_UNCONFIRMED`。

---

## 10. 快速运行

```bash
# 编译
cd ~/wheeltec_robot && catkin_make --force-cmake && source devel/setup.bash

# 四桌仿真（C++ 默认实现）
roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch
rosservice call /nav_sorting/start

# 单桌仿真
roslaunch aubo_mobile_nav_sorting mission_gazebo.launch

# 真机（底盘 / 雷达 / 机械臂 / 相机先启动）
roslaunch aubo_mobile_nav_sorting navigation_sorting.launch \
  map_file:=/abs/path/map.yaml sorting_config:=/abs/path/sorting.yaml \
  goal_x:=X goal_y:=Y goal_yaw:=Yaw

# 查看状态
rostopic echo /nav_sorting/state
rostopic echo /sorting/state
```

---

## 11. 一句话总结

> **底盘用 `move_base` 走到桌边，机械臂用 `transport` 收拢；最后半米用低速 `cmd_vel` 自己精停，
> 机械臂再到 `observe` 看一眼、连续抓完这一桌；然后收臂、后退、去下一桌；
> 全部抓完后导航回起点。** 底盘和机械臂通过 `/sorting/base_locked` 互锁，
> 任务节点只负责编排顺序和异常恢复，具体怎么抓、怎么识别全交给 `aubo_sorting_core`。
