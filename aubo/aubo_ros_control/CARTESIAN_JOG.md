# 末端 XYZ 连续点动（C++）

2026-10-25：SDK启动时增加`WAITING_FOR_SDK`就绪阶段，未接受运动请求时等待连续有效空MAC诊断；
首次使能/home后仍执行原有严格超时保护。启动156 ms诊断退出的证据及验证见
[启动就绪修正](TIMING_DIAGNOSIS.md)。

2026-10-24 后续排查改为先采集完整时序、冻结运动参数，见
[SDK 时间线、MAC 水位与反向平滑分析](TIMING_DIAGNOSIS.md)。

这是独立的 ROS 1 点动控制器，面向按住按钮移动、松手减速停止的操作。
不调用 `computeCartesianPath()`，不启动 MoveIt 轨迹执行器。使用 KDL 雅可比阻尼逆解，
将末端线速度变成六轴关节速度，经速度/加速度/jerk 平滑后积分成 5 ms 关节位置点。
核心、中文 Qt 窗口和平滑器测试均使用 C++14。

本实现使用 ROS 1 / MoveIt 1 的接口；完整 ROS 构建和运行需要 Linux ROS 工作空间。
当前实机默认使用与视觉伺服共用的直接 SDK 后端：50 Hz 更新关节速度目标，
按固定 5 ms 周期平滑积分成六轴位置点，SDK 按需批量取点下发，不经过 JTC。
此前 ros_control 路径的真机 home / 低速 +X 记录属于历史验证，不能作为此版本的真机通过依据。
SDK 同步回包仍可能耗时 60–90 ms；本版改为按 MAC 实际缺口补点并保留有界前瞻，
不是消除网络/控制柜等待。离线测试不能代替本版真机连续运动验证。
2026-10-24 14:49，同连接补点版已完成一次近 home 的真机回位并只读确认停稳；
初始最大关节误差约 0.0215 rad，最终约 0.00086 rad。长时间 XYZ 点动尚未验证。
随后 14:54 用户使能并点击 X+ 时，第二批同步发送耗时 118.73 ms，超过保留的 100 ms
发送预算并停流；因此本版 XYZ 实机问题仍未解决，不能将上述 home 成功视为点动通过。

## 编译与启动

在已有依赖的 catkin 工作空间根目录运行：

```bash
catkin_make
source devel/setup.bash

# 空场景仿真，同时打开中文点动窗口
roslaunch aubo_ros_control cartesian_jog_gazebo.launch

# 无桌面仿真（通过自定义 ROS 客户端操作）
roslaunch aubo_ros_control cartesian_jog_gazebo.launch gui:=false panel:=false
```

Gazebo 入口从非零 home 姿态 `[0, 0, 1.6292, 0, 1.6628, 0]` rad 开始，
顺序为肩、上臂、前臂、腕 1、腕 2、腕 3，并加载六个独立关节位置控制器。
该姿态的 TCP 雅可比最小奇异值约为 `0.213`，高于默认减速阈值 `0.06`；
全零姿态约为 `2.1e-7`，会触发 `singularity_stop`。

启动时先暂停 Gazebo 物理，由 `spawn_model -J ... -unpause` 设置完关节值后再恢复。
如果在物理运行时生成模型，控制器可能先读到零位并保持零位，或在初始值跳变时触发
`following_error`。修改启动文件后需要退出旧仿真，再重新启动。
可通过 `initial_shoulder`、`initial_upper_arm`、`initial_fore_arm`、`initial_wrist1`、
`initial_wrist2`、`initial_wrist3` 六个 launch 参数覆盖初始值，单位 rad。

同一模型只保留一个上层运动指令源；点动运行期间不得另行执行 MoveIt 轨迹、视觉伺服
或其他关节指令发布者。默认入口直接管理 SDK 的运动写入连接。

真机使用专用入口：

```bash
roslaunch aubo_ros_control cartesian_jog_real.launch robot_ip:=192.168.1.2
```

该入口默认 `backend:=sdk`，不启动 `aubo_hw_node`、控制器管理器或 JTC。
节点复用视觉伺服的 `DirectSdkBackend`，执行 SDK startup/握手并进入 TCP2CANBUS，
可能上电和松刹车；点动运动默认仍为禁用。启动前应使机械臂停稳。不要同时运行
`aubo_real_bringup.launch` 或视觉伺服真机入口，以免重复启动硬件驱动。
该入口不启动 move_group 执行器，也不使用 MoveIt Servo。启动文件默认使用带腕部相机的模型；
如果实际没有相机，传入 `robot_xacro:=aubo_i5.xacro robot_srdf:=aubo_i5.srdf`。

也可显式指定相同的 SDK 后端：

```bash
roslaunch aubo_ros_control cartesian_jog_real.launch backend:=sdk
```

旧的 `backend:=ros_control` 选项保留供回归对照，以下对应章节明确标为历史路径。

### 实机初始姿态与可取消回 home

实机不会自动回 home。窗口收到当前关节反馈、控制器就绪后显示当前六轴位置与 home
的最大关节误差；如果误差超过 0.005 rad，会询问是否移动到 home。选择“否”保持当前位置，
之后可手动使能点动；如果当前位置接近奇异位形，笛卡尔运动仍会被奇异保护停止。
已经位于 home 附近时不弹出询问。无界面运行也不会自行回 home。

“移动到 home（关节运动）”通过 `/cartesian_jog/move_home` action
（`aubo_ros_control/MoveHomeAction`，空 goal）发起。该按钮在仿真中也可用于验证。
home 目标由 `home_joint_positions` 配置，默认与仿真初始 home 一致；
`home_joint_velocity` 默认 0.10 rad/s，并同时受各关节速度、加速度、jerk 限制。
回 home 使用 50 Hz 关节空间误差控制和同一个 5 ms 平滑点生成器，默认经 SDK 下发，
可以从无碰撞且限位内的奇异位形出发；不会调用 MoveGroup 规划或执行。
运动期间检查候选点及其制动尾段；遇到已知碰撞或限位会停止，不提供全局绕障规划。

必须先禁用点动且停稳，再请求回 home；运动中拒绝点动使能和第二个 home 目标。
action 以 10 Hz 反馈当前位置、目标位置、最大关节误差和减速状态。
点击“取消回 home 并减速停止”，或由客户端发送 cancel，可以随时请求减速；
“停止并禁用”和正常关闭窗口也会请求取消。取消不是瞬时停止，也不能撤回控制柜已有点。
只有指令平滑器停稳、反馈新鲜、跟随误差不超过 0.01 rad、反馈关节速度不超过 0.01 rad/s
时，才返回 `PREEMPTED`；到达目标且最大反馈误差不超过 0.005 rad 时返回 `SUCCEEDED`。
故障返回 `ABORTED`，未就绪或忙碌时返回 `REJECTED`。
成功、取消后均保持禁用，需要手动重新使能点动。

`home_timeout` 默认 90 秒，超时触发故障并减速。反馈丢失时不能确认停稳，action 会等待反馈恢复；
控制器退出时会返回未确认停止的失败结果。客户端进程崩溃或网络断开本身不会自动发送 cancel，
可由其他客户端取消目标或调用“停止并禁用”。不要使用 action 取消代替硬件急停。

```python
import actionlib
from aubo_ros_control.msg import MoveHomeAction, MoveHomeGoal

client = actionlib.SimpleActionClient('/cartesian_jog/move_home', MoveHomeAction)
client.wait_for_server()
client.send_goal(MoveHomeGoal())  # 用户确认后才调用
# 需要取消时调用 client.cancel_goal()；等待结果确认停稳。
```

窗口中先点击“使能点动”，选择基坐标系或工具坐标系，再按住 `X± / Y± / Z±`。
默认速度 10 mm/s，可调范围 0.1–100 mm/s；界面 mm/s 自动转换为消息中的 m/s。
范围上限由 `max_linear_velocity: 0.10` 控制，仿真与真机入口共用此配置。
实际速度仍受关节速度、加速度、jerk、奇异位形及限位约束，可能低于设定值。
松手、窗口失焦/隐藏或关闭时发布零指令；控制器没有收到新指令时也会超时减速。
“停止并禁用”使用同一减速路径。GUI 服务调用阻塞时，控制器自身的超时机制仍生效。

## 接口与使能流程

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/cartesian_jog/command` | `geometry_msgs/TwistStamped` | 持续发送末端线速度；建议 30–50 Hz |
| `/cartesian_jog/set_enabled` | `std_srvs/SetBool` | 使能或减速禁用 |
| `/cartesian_jog/reset` | `std_srvs/Trigger` | 停稳、反馈正常且停止位姿安全时清除故障；不自动使能 |
| `/cartesian_jog/state` | `std_msgs/String` | 10 Hz 状态心跳，含故障原因 |
| `/cartesian_jog/move_home` | `aubo_ros_control/MoveHomeAction` | 显式请求回 home，可取消；不自动使能点动 |

`header.frame_id` 仅接受配置中的 `base_link` 或 `tool_link`，默认分别为
`base_link`、`tcp_link`。工具坐标系方向按当前指令位姿旋转到基坐标系。
`linear` 单位 m/s；`angular` 必须全部为零，本版只开放 XYZ 平移。
控制器在开始移动时记录末端朝向，并在移动期间加入姿态误差修正。

每条有效消息都需要当前 ROS 时间戳，且时间戳递增。相同时间戳的重复消息丢弃且
不刷新超时计时，兼容 Gazebo 时钟分辨率低于 GUI 事件频率的情况。过期、未来偏差过大、乱序、
非有限值、不支持的坐标系和角速度输入会触发故障锁存。多个指令发布者不应混用。
无论输入时间戳如何，指令与反馈超时都使用本机单调时钟，防止 ROS 时间暂停绕过超时。

每次使能后，必须先发送零速度，再发送非零速度；使能前的命令不会积存。
如果一直没有发送零指令，会在 `command_timeout` 到期后进入故障。
中文窗口会自动持续发送带新时间戳的零指令，满足此要求。

```bash
rosservice call /cartesian_jog/set_enabled "data: true"
rosservice call /cartesian_jog/set_enabled "data: false"
rosservice call /cartesian_jog/reset "{}"
rostopic echo /cartesian_jog/state
```

`DISABLED` 表示已禁用并停稳；`ENABLED_IDLE` 表示已使能、无运动；
`HOME_MOVING` / `HOME_STOPPING` 表示回 home 运动 / 减速停止中；
`JOGGING` 表示运动或仍在减速；`STOPPING` 表示禁用后的减速阶段；
`FAULT: 原因 | STOPPING/STOPPED` 表示锁存故障。故障不会自动重启运动。
仿真暂停/恢复可能触发调度间隙或陈旧时间戳故障，恢复后需检查状态并复位。

## 平滑、缓存与停止

`VelocitySmoother` 对每个关节采用两层状态：第一层把目标速度变成加速度受限的斜坡，
第二层滤波该斜坡。固定步长 `h=0.005 s`，系数
`alpha=min(1, J*h/(2*A))`。从静止开始且目标速度始终限制在 `[-V,V]` 时，
生成点对应的离散速度序列满足 `|v|<=V`、`|Δv/h|<=A`、`|Δ²v/h²|<=J`。
反向输入同样经过此过程，不会直接翻转关节速度。滤波有响应延迟，不能既要求任意小 jerk
又要求立即停止；默认等效滤波时间约为 0.13–0.15 s。

此界限针对离散指令序列，不代表电机实际速度/加速度一定满足相同界限，也不保证
控制柜内部插值具有连续 jerk。零指令下速度渐近归零，以 1e-6 的内部状态阈值判定停稳，
不通过强制清零速度制造突变。有限的姿态误差、阻尼逆解和滤波会造成末端偏离理想直线。

### 当前默认 SDK：50 Hz 目标与 5 ms 插补

`control_frequency: 50.0` 控制目标更新周期，即每 20 ms 计算一次雅可比逆解/回 home
关节速度目标；独立控制线程用单调时钟运行，延迟后不突发追赶。
周期须为 5 ms 的整数倍（1..10 点）。SDK 和 Gazebo 的输出点周期仍为 5 ms，
不会把积分步长直接改成 20 ms，也不会按某次 SDK RPC 的耗时拉伸点周期。

SDK 在需要路点时串行调用点生成器，生成器读取最新目标快照，以 5 ms 的速度/加速度/jerk
平滑和积分产生下一点，并检查候选点与制动尾段的限位、碰撞及跟随误差。
名义上一个 20 ms 目标周期对应四个 5 ms 点；通信批次不再固定为四点。
SDK 首批最多取 50 点，之后先查询 MAC，按本次缺口取点。批次中的每个点仍代表 5 ms。
没有新增 PC 轨迹 FIFO，也不发布 `JointTrajectory`。六轴位置点是 SDK 透传协议的数据，
不是交给 ROS 关节轨迹控制器的执行目标。

禁用、取消、故障、零输入立即让点生成器采用零速度目标；重新使能/开始 home 会使旧快照失效。
运动中目标快照超过两个控制周期（默认 40 ms）未更新会锁存 `control_target_timeout` 并减速。
原有反馈/输入超时、跟随误差、MAC 断流和下发阻塞保护继续生效。
SDK 的同步调用仍可能超出补点预算，降低目标计算频率不能保证消除这类通信阻塞。

### ros_control 历史对照路径（显式 `backend:=ros_control`）

此显式选项保留原先的 KDL 阻尼逆解、5 ms 速度/加速度/jerk 平滑积分、
限位及候选点/制动尾段碰撞检查。输出改为向
`/aubo_i5/aubo_i5_controller/command` 发布六轴 `trajectory_msgs/JointTrajectory`：
每条只有一个位置目标，`header.stamp=0`，`time_from_start=0.01 s`，不填速度和加速度。
JTC 从当前指令轨迹桥接到新目标，位置型目标采用线性插补。指令位置只在初始化时取自
反馈，之后连续积分，不会每周期回到滞后的反馈位置重新计算增量。
使能或发起 home 前，通过 `/aubo_i5/controller_manager/list_controllers` 确认
`aubo_i5_controller` 处于 `running`；查询失败或控制器未运行时拒绝运动请求。

专用 launch 对底层使用与正常 `aubo_real_bringup.launch` 相同的默认参数：
`control_frequency=200.0`、`mac_buffer_target=300`（50 点 / 名义 250 ms）、
`trajectory_preload_points=20`（启动预装 100 ms）。MAC 前瞻按本机记录的 167 ms 最长供点间隔留出余量，
并保持低于 SDK 缓冲上限；MAC 查询、批量补点及 SDK 连接由已有硬件节点负责；
点动节点不再实例化 `DirectSdkBackend`。JTC 的额外插补和底层预装/排队都会增加延迟，
因此不能把点生成器的离散 V/A/J 界限直接当成 JTC 输出或电机实际运动的同等保证。
底层原有同步 SDK 调用仍然存在，切换输出路径没有消除它们的实际耗时。

此历史路径的反馈为 `/aubo_i5/aubo_hw_node/raw_joint_states`：专用入口开启
`use_realtime_feedback=true`，硬件节点从 SDK 实时路点回调更新六轴状态并发布原始样本。
同步诊断只在定时器中运行，不再让“路点查询 + 诊断查询”的耗时决定反馈间隔。
推送使用启动完成后空闲的控制连接；诊断和 MAC 下发分别使用另外两个连接。
专用入口另建第四个只读连接：60 ms 内没有新采样时，由独立线程查询真实路点，
补充偶发的推送间隙。查询回复使用请求开始时的时间戳，迟到回复不能刷新成当前时间；
若请求早于已接收的新推送，则丢弃回复。查询失败不刷新反馈，两条来源都中断时仍触发保护。
原始反馈订阅使用独立 ROS 回调队列，避免与场景更新、逆解、输出和服务查询排队。
进入控制循环前须在 2 秒内收到第一份有效推送；未收到就拒绝启动。
普通 `aubo_real_bringup.launch` 仍默认使用原有轮询方式；底层
`aubo_control.launch` 可显式传入 `use_realtime_feedback:=true` 使用推送。
节点检查原始采样年龄，并将消息传输年龄计入单调时钟
反馈超时；普通 `/joint_states` 的缓存重发或重新打时间戳不能代替新硬件采样。
TF 和可视化可继续使用汇聚后的 `/joint_states`。默认反馈超时仍为 150 ms，
200 Hz 是指令生成/控制周期，不代表实机反馈频率。

实机入口的夹爪模型采用固定 `joint1=0.0`、`joint2=0.0` rad
（SRDF 的 open 姿态），由 `aubo_joint_state_relay` 的 `zeros` 参数提供。
规划场景通过 `scene_joint_states_topic=/joint_states` 接收完整模型状态，避免缺失夹爪告警；
运动控制仍只订阅原始六轴反馈，不用该汇聚话题的缓存或时间戳判断硬件是否正常。

启动时应看到 `实时关节推送已就绪；诊断查询不阻塞反馈发布`。
随后应看到 `独立反馈查询已启动：推送间隔达 60 ms 时查询真实路点，保留请求时间戳`。
`feedback_timeout` 仍为 150 ms；推送丢失或消息过期仍锁存故障，不以重复发布缓存掩盖中断。

### ros_control 历史路径的轨迹点存储与下发顺序

此历史路径在线生成点，没有先把整个 XYZ 路径保存成文件或完整轨迹再执行：

1. 点动节点每 5 ms 把 XYZ 速度经雅可比逆解、速度/加速度/jerk 平滑并积分，
   得到一个六轴绝对位置目标。每条 `JointTrajectory` 只有这个点，目标时间为 10 ms。
   新消息滚动更新 JTC 的短轨迹，并不是把所有消息追加成一条长轨迹。
2. 硬件主循环以 200 Hz 执行 `read → cm.update → write`，JTC 插补输出六轴位置。
   `write()` 将变化达到约 `1e-6 rad` 的命令放入 `ros_motion_queue_` 内存 FIFO。
   该队列按 4096 点初始化预分配，每个元素只有六个位置值，没有单独的执行时间戳。
   当前 `try_enqueue` 失败时报警并丢弃新点；没有额外按点龄或毫秒数设置较小的积压硬上限。
3. 独立喂点线程先同步查询 MAC 缓冲，再检查本地队列。首段至少积累 20 点才发送；
   正常 200 Hz 下约需要 100 ms。日志中的“预装 N 点”是检查时队列实际已有 N 点。
4. 默认 MAC 目标是 300 个关节标量，即 50 个六轴点、名义 250 ms；不是 300 个路点。
   当前读数为 B 时，按 `ceil((300-B)/6)` 取点，去重并执行原有速度检测/插值后，
   同步调用 `robotServiceSetRobotPosData2Canbus(vector)` 批量发送。
   控制柜按每点 5 ms 执行。MAC 和本地队列持续排空 20 次检查后，下一段重新预装。

喂点循环末尾的 1 ms 是休眠时间；完整周期还包括同步 MAC 查询、取点和同步发送耗时，
因此它不保证每 1 ms 完成一次补点。PC FIFO 与控制柜缓冲都是积压来源，
软件队列超过 MAC 目标 50 点时，诊断会提示队列积压；名义 MAC 250 ms 也不是总指令延迟的上限。
加大 FIFO 或 MAC 缓冲不能补出新的硬件反馈。

### 默认直接 SDK 后端的供点与保护

下面的 `sdk_*` 缓冲参数和供点诊断用于当前默认 SDK 路径，不控制历史 ros_control 硬件节点。
SDK 点动后端采用与常规视觉伺服相同的“查询 MAC → 补足缺口 → 等待 SDK 返回”顺序，
查询和下发均使用 `mac_service_`，与已正常运动的 `aubo_real_bringup.launch` 底层一致；
控制连接只负责 startup/握手/进退透传和事件，实时反馈由独立连接推送。
保留独立的 50 Hz 目标更新线程和 SDK 实时关节反馈回调。MAC 查询和发送在同一个供点线程
串行执行；反馈回调不执行诊断查询，也不等待这个线程。点动不使用视觉伺服的 80 点软件队列。

默认 `sdk_mac_buffer_target: 300`：按六轴标量解释为 50 个点 / 名义 250 ms 前瞻。
收到 MAC 读数 B 后，将 B 向上取整为完整六轴点，计算 `max(0, 50-ceil(B/6))` 的缺口。
缺口不足 4 点（20 ms）时等待，避免为一两个点的波动频繁调用同步 SDK。首批按缺口预装；
之后还受独立 `SdkPointPacer` 约束：首个非零 MAC 的缺口作为启动额度，后续每 5 ms
最多积累一个新点的生产额度，单次最多保留 50 点额度。MAC 接近目标时丢弃未用额度；
有缺口时本批不超过“MAC缺口”和“生产额度”两者的较小值。生产额度不是已消费或有空间的证据，
仍须通过实测 MAC/预留限额，未收到新诊断不补点。
例如读数 48 标量表示 8 点，缺口为 42 点，实际批量还受生产额度限制。批内路点连续，每点对应 5 ms，
不会因为 SDK 阻塞 60 ms 就将一个积分周期改成 60 ms，也不丢旧点或追赶一个墙钟发送周期。
点动模式接受 24–300 的六倍数作为目标。减小目标会降低可覆盖的通信抖动。

之前固定“初始 10 点 / 50 ms，之后 4 点 / 20 ms”的供点方式，无法覆盖用户实测的
59.65 ms 第二批发送。只提高看门狗也不能解决每 20 ms 必须完成一次同步 RPC 的吞吐要求。
本版将前瞻改为 250 ms，同时按 MAC 缺口动态改变批量；上层目标仍每 20 ms 更新，
已提交到控制柜的点不能用后来的新目标改写，因此操作响应会受这段前瞻影响。

启动后首个运动增量出现前，不发送精确相同的静止保持点，空缓冲不作为运行中欠载。
运动开始后持续补点，包含平滑器停稳后的保持点；已建立的流读到零 MAC 或控制柜报告
`macDataInterruptWarning` 时立即停流，不自动重新启动。首次发送后、还未观察到非零 MAC 时，零读数只允许继续查询等待，不允许再发第二批；
300 ms 首次确认期限达到后仍未见非零则停流。SDK 返回并不证明缓存诊断已更新，
因此不能把启动旧零立即当成已建立流的欠载。首次确认后读到零仍立即停流。

`SdkFlowControl` 的当前点动模式按串行 MAC 观测准入：查询开始前必须没有未返回的发送，
查询在途时禁止预留发送点。只有收到新读数后才计算缺口，不按经过时间推算消费；
SDK 成功返回不表示执行完成，也不会自行归还额度。

2026-10-24 的单次实机 home 测试实际运动后，在第 42 批成功返回之后读到 306 标量（51 点），
旧账本仍推算 300 标量（50 点），触发了“计数矛盾”。这只能证明本地模型与观测不符，
不能直接归因为无害的一个点误差。这次最后一批发送仅 21.32 ms，
反馈年龄 11.26 ms，诊断 4.86 ms，不属于之前的发送超时。SDK 字段只定义有效数据长度，
没有硬件采样时间戳或点序号，不能将本地推算值当成每次读数必须满足的协议保证。
计数差异的底层来源尚未确定，不能据此声称控制柜一定额外执行了一个点。
退出后只读查询确认 `RobotStopped`、MAC=0，500 ms 间隔关节读数一致；最终最大 home
误差约 0.0327 rad，该次 home 未完成。早期计数修正版的 ROS 回归曾在第 42 批后注入
306 标量并通过，但随后的实机仍有超限，见下文记录。该注入只改读数、未同步改变模拟柜内队列，
不适合用来证明连续供点。当前连续通信测试由模拟柜内真实收点/消费历史生成读数；
306 标量计数分支单独在 `sdk_jog_timing_test` 中回放，验证暂停补点和限额，不能证明硬件队列。

随后计数修正版单次实机测试在第 6 批后读到 372 标量（62 点），正确触发 360 标量限额，
home 仍未完成；末批发送 24.20 ms、诊断 4.53 ms、反馈年龄 12.47 ms。只读查询确认停稳，
MAC=0、500 ms 间隔关节位置一致，最大 home 误差约 0.0215 rad。
因此仅允许计数上升并不足以解决实际积压，不能继续以增大限额代替定位。

本次将发送恢复到查询所用的同一 MAC 连接，并增加上述独立生产速率约束；
这避免把反复收到的旧 MAC 缺口当作无限制生成新点的依据。故障时输出最近 64 次 RPC 的
返回时序、MAC 标量、发送批次/点数及耗时；正常循环只写固定大小内存，不逐批打印。
同一连接能消除客户端跨连接的顺序差异，但不证明控制柜内部采样或入队已同步。

当前串行模式采用新 MAC 实测值：达到或超过 300 标量目标时暂停补点，不因此退出；
低于目标时才按差额取点。`sdk_mac_buffer_limit: 360` 仍限制新观测与本批预留，
按完整六轴点向上取整，361 标量会触发超限并停流。迟到/重复回复、RPC 重叠、
超时或发送失败仍锁存故障。历史异步模式的严格账本保留供测试，但当前默认不使用。
该限额不认证两个诊断之间任意时刻真实物理队列的上界，SDK 的采样/内部入队延迟仍无法
由这些字段确认；不能将“360 标量限额”当作已证明的总停止延迟上界。

单次同步发送达到 `min(100 ms, 名义前瞻)` 时停流；默认为 100 ms，**不是放宽到 300 ms**。
`sdk_diagnosis_timeout` 仍为 150 ms，同时限制在途查询和距上次有效响应的时间；
它也会限制“生成 + 发送 + 下次诊断”过慢的组合。反馈保护仍为 150 ms，且缓存读取不刷新年龄。
SDK按MAC需要生成点，柜内有点时不套用旧的固定生成间隔检查；停滞由独立诊断/发送看门狗监测。
非SDK路径保留80ms间隔检查，SDK单点计算仍有20ms上限；50 Hz 控制目标超过 40 ms
未更新、碰撞、限位、跟随误差和输入超时保护仍生效。批次生成不完整、发送失败或迟到成功
都不会恢复供点。大于预算或突发组合延迟仍可能停机，250 ms 前瞻不是无限制容错。

松手、取消和软件故障不能撤回已提交点。实际停止还包括输入心跳、PC/网络延迟、
控制柜已有前瞻和 A/J 减速尾段；250 ms 不是总停止时间上限。

停流时 `[sdk/transport]` 输出当前批次、点数、阻塞时长、生成耗时、批内最大关节增量、
最近成功的 MAC 读数及其请求/响应年龄、诊断在途时长和实时反馈年龄。
SDK 调用随后返回时，`[sdk/send]` 仍记录该批次的返回码及完整耗时，迟到成功不会恢复供点。
MAC 的两个年龄只描述请求和响应的时间。缓存快照可能早于请求，不能据此推定硬件采样年龄的区间。
后端故障会保留首个原因，不能通过 reset 恢复已停止的 SDK 连接，需要重启节点。

2026-10-24 14:49 的同连接补点版单次真机 home 已通过，速度上限 0.10 rad/s，
客户端只发送一个目标，45 秒超时取消且不重试；本次正常返回 `SUCCEEDED` / `Home reached; jog remains disabled`。
初始最大关节误差 0.02149956 rad，退出后只读测得 0.00086035 rad（约 0.0493°）。
只读查询四项返回码均为 0、`RobotStopped`、MAC=0，间隔 500 ms 的关节读数一致。
本次日志未出现发送超时、缓冲超限、反馈超时或缺失 `joint1/joint2` 告警；
仍有慢诊断及 MAC 高于前次账面的警告，不能据一次成功确定控制柜计数变化的底层原因。
本次覆盖接近 home 时的单次回位，不能替代远距离回位或长时间 XYZ 点动验证。
原始记录位于 `logs/sdk_mac_validation_20261024/real_home_20261024_144920/`。

14:54 用户 X+ 失败记录：首批 50 点返回 21.73 ms，其后先读到缓存 MAC=0，再读到
276 标量（46 点，查询 23.95 ms），补发 4 点时等待 118.73 ms。点生成仅 0.26 ms，
最先触发的是 100 ms 发送看门狗，随后才退出；不是首先触发逆解、碰撞或夹爪缺失。
此次 SDK 返回码仍为 0，但在停流后返回，不能恢复供点。
离线补充首批 22 ms、第二批 119 ms、诊断 24 ms 的反例：若只将发送预算放宽到
130 ms，在后续 60 ms 发送、20 ms 缓存滞后且控制柜晚接收的模型中会欠载。
该反例证明只调整超时不能保证连续供点，不表示已测得真实控制柜的接收时刻。
当前生产参数仍为 100 ms / 300 标量目标 / 360 标量限额，准备进一步采集 X+ 系统调用时序。
15:04 只读确认 `RobotStopped`、MAC=0，500 ms 最大关节变化约 0.0000037 rad。
记录位于 `logs/sdk_mac_validation_20261024/xplus_1454_analysis/`。

15:10 授权的一次基坐标系 +X 采集（1 mm/s、非零指令最多 2 秒）完成并返回 DISABLED，
只读确认 RobotStopped、MAC=0、500 ms 关节读数一致，但没有复现 118.73 ms 发送退出。
此次 strace 捕获一次 98.58 ms 慢诊断：请求 sendto 仅 0.009 ms；接收线程等待数据，
对应响应收到后约 0.007 ms 即唤醒供点线程，主要耗时在响应到达前。
没有数据包 ACK/服务端记录，不能进一步断定是网络还是控制柜内部处理，
也不能用本次慢诊断证明另一次发送故障的具体来源。
用户随后确认原故障为 **10 mm/s、基坐标系**，与本次采集速度不同。
10 mm/s 的离线 XYZ/home/取消测试已通过（90 ms 发送、20 ms 诊断、20 ms 缓存滞后）；
15:24 对应速度的单次真机采集再次退出：首批50点发送16.28 ms返回，
随后MAC诊断约160.7 ms才唤醒，上次有效诊断响应年龄达到165.56 ms时已触发保护；
此次没有发送第二批。15:26只读确认停稳、MAC=0。
故障尚未解决，生产参数未放宽；后续按 `TIMING_DIAGNOSIS.md` 的固定步骤采集分析。
完整分析：`logs/sdk_mac_validation_20261024/real_xplus_trace_20261024_151059/analysis.md`。

排查时显式指定后端并保留完整控制台日志：

```bash
roslaunch aubo_ros_control cartesian_jog_real.launch backend:=sdk 2>&1 | tee /tmp/aubo_jog_sdk.log
```

需要定位 SDK 是卡在 socket 发送、等待响应还是线程唤醒时，可运行：

```bash
rosrun aubo_ros_control capture_cartesian_jog_sdk.sh
```

该脚本显式传入 `backend:=sdk`，采集直接 SDK 路径。
脚本启动后默认禁用，不自动发送 home 或点动目标；在现场确认可运动后
通过窗口手动操作。完整控制台及各线程的系统调用时序默认保存在本包
`logs/sdk_capture_时间_进程号/`，可用 `AUBO_JOG_CAPTURE_DIR` 指定可写目录。
网络收发参数按原始指针记录，不转储数据载荷。`strace` 会扰动调度，采集结果只能用于
定位等待阶段，不能作为实时性能通过的依据。

ROS 定时器以 50 Hz 读取 SDK 缓存，实际推送频率与间隔以控制柜和实测为准；
200 Hz 是路点消费/生成节拍，不是 200 Hz 实机反馈闭环。
关节反馈订阅 SDK 实时路点推送，由 ROS 定时器读取缓存，不再每 20 ms 发起同步查询。
缓存用单调时钟记录收到新样本的时间；重复读取不会刷新样本年龄，超过
`feedback_timeout`（默认仍为 150 ms）或尚无反馈时读状态失败。
进入透传前必须在 2 秒内收到第一帧反馈。
轨迹点下发和诊断均由同一 MAC 连接串行执行；独立状态连接继续接收实时反馈。
连接建立后只对本进程到控制柜地址/端口的 TCP socket 设置 `TCP_NODELAY`，
同步查询/下发前重新启用 Linux `TCP_QUICKACK`；不修改系统网络参数。
控制柜消费周期需要与本仓库使用的 5 ms SDK 透传协议一致。

### 停止延迟与写入互斥

两种真机后端的停止命令都不会撤回已有驱动队列或已经进入控制柜的点。直接 SDK 路径
停止时正在执行的调用也可能随后完成，其预留点数仍计入积压。历史 ros_control 路径
还经过 JTC 插补和原硬件节点预装/排队；实际停止延迟包括输入采样/网络、这些已有点和减速尾段。
正常 SIGINT/SIGTERM 会尝试先减速、等待缓冲排空；ROS 被直接 shutdown、
断线、急停、SDK 失败或缓冲欠载时无法保证平滑退出。软件停止不能替代硬件急停。

同一 Linux 主机上的本包三个真机写入路径（ros_control、视觉伺服、点动）共享
`/tmp/aubo_motion_writer.lock` 文件锁。只读示教器模式不持锁。锁文件不应手动删除；
进程结束会由内核释放锁。该锁不能约束其他主机、示教器或仓库外的 SDK 客户端，
也不支持同一主机并行控制多台机械臂。Gazebo 输出不使用此文件锁。

## 限位、奇异位形和碰撞

节点验证关节名称与 KDL 链顺序一致，且 YAML 限制不超过模型限制。
雅可比最小奇异值接近阈值时整体缩放六轴速度；达到停止阈值时锁存故障并减速。
接近关节限位时还考虑当前平滑器的停止距离，不单独截断某个关节。
机器人处于奇异位形或关节限位之外时，本版不会自动选择逃逸方向。

碰撞检查使用独立 `PlanningSceneMonitor`，检查候选位姿及其零指令减速尾段，
而非每次规划完整路径。尾段以 5 ms 推演关节限位，以 `collision_resolution` 指定的
最大关节增量采样自碰撞/环境碰撞。默认推演最多 3 秒，不能在此时限停稳的配置拒绝运动。
候选点不安全时接续上一周期已验证的减速尾段，不清队列、不把指令位置重置为反馈位置。
过长计算、反馈超时、跟随误差过大或缓冲断流均会触发停止/退出。

启动场景默认只含机器人，**不会自动把 Gazebo 地面、桌子或其他模型加入 MoveIt 场景**。
外部可向 `/collision_object`、`/planning_scene_world`、`/planning_scene` 发布障碍物，
向 `/attached_collision_object` 发布附着物。节点不自行运行点云建图。
如果已有纯规划 MoveIt 场景，使用自定义配置设置：

```yaml
planning_scene_topic: /move_group/monitored_planning_scene
initial_scene_service: /get_planning_scene
```

此时完整场景快照获取失败会拒绝启动，后续更新由指定话题同步。已有 MoveIt 必须关闭
轨迹执行，且没有其他硬件控制进程占用 SDK。默认独立入口不需要启动 move_group。
场景访问按 MoveIt 的
[PlanningSceneMonitor 锁接口](https://github.com/moveit/moveit/blob/noetic-devel/moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h)
保护，避免读场景时与更新线程并发修改。

碰撞检测是基于已知几何的离散预测，不覆盖采样点间薄障碍、突然进入的动态障碍、
错误 TCP/模型、未发布的环境物体或反馈跟随误差。模型检查不能构成真机安全保证。
复杂网格可能使预测耗时超出预算；需要测量实际计算耗时，再决定是否简化碰撞模型。

## 配置与代码入口

参数位于 `config/cartesian_jog.yaml`；两套完整入口均支持 `config:=自定义YAML路径`。
启动时读取参数，本版不提供运动期间动态改参。

| 文件 | 职责 |
| --- | --- |
| `src/cartesian_jog.cpp` | 输入、状态、KDL 逆解、限位/碰撞预测、ros_control/Gazebo/SDK 输出 |
| `include/aubo_ros_control/cartesian_jog_common.h` | 可独立测试的平滑器和停止距离上界 |
| `src/cartesian_jog_panel.cpp` | 中文 XYZ 按钮、速度/坐标系、失焦释放和状态心跳 |
| `src/aubo_hardware_interface.cpp` | 历史 ros_control 对照路径的硬件连接与批量供点 |
| `src/direct_sdk_backend.cpp` | SDK 连接、独立诊断与定时供点、下发边界校验 |
| `include/aubo_ros_control/sdk_flow_control.h` | 诊断快照、在途点预留与保守积压上界 |
| `include/aubo_ros_control/sdk_jog_timing.h` | MAC 缺口批量、250 ms 前瞻和发送预算 |
| `include/aubo_ros_control/motion_lease.h` | 同机运动写入互斥 |

## 验证与调试

### 默认真机入口与验证范围

先退出旧 launch，重新编译点动节点并加载工作空间：

```bash
cd ~/aubo/ros_mobile_manipulation_lab
source ~/moveit_ws/devel/setup.bash
catkin_make aubo_cartesian_jog_node -j2
source devel/setup.bash
roslaunch aubo_ros_control cartesian_jog_real.launch
```

默认启动应显示 `sdk 后端已启动` 和 `目标更新 50.0 Hz，按 5 ms 周期平滑插补`。
用 `rostest aubo_ros_control cartesian_jog_sdk50.test` 离线验证 SDK 分支：测试程序在链接时
替换 SDK 边界，不调用 SDK login/startup/透传，不启动 JTC 或硬件节点；真实 AUBO 相机模型
使用 100 ms 延迟的模拟反馈，检查 XYZ 六方向位移、每个 5 ms 点的速度/加速度/jerk、
反馈中断、home/取消及结束保持禁用。当前同连接/生产限速版本分别通过 60 ms 发送/20 ms 诊断/提前接收，
以及 90 ms 发送/20 ms 诊断/发送返回时接收两组测试（各 1 项，0 failures / errors），
两组均额外模拟了从首批起就存在的 20 ms MAC 缓存滞后。
这两组故意让同步发送耗时大于旧版 20 ms 补点周期和 50 ms 前瞻。

`sdk_jog_timing_test` 对每组模拟 30 秒，覆盖 60/89/90 ms 下发、20/25/60 ms 诊断，
诊断采用请求开始时或更早 20 ms 的缓存快照，控制柜分别在发送开始/结束接收，以及停止消费和迟到结果；
另回放 300→306→288→360→361 标量，检查高于目标时不再补点、超限仍停流及 RPC 串行约束。
在这些模型场景中检查不断流、排队计数与观测限额及故障不恢复。
真实后端的 `direct_sdk_backend_feedback_test` 验证 49/50 ms（小前瞻）和 99/100 ms
（当前默认）的发送看门狗边界、迟到成功/失败、首个故障原因保留和 299/300 ms 首次 MAC 确认期限。
30 万步数值平滑测试也通过。这些离线测试不能证明实际控制柜回包、反馈或电机停止性能，
首轮真机 home 在 306 标量处因本地账本计数矛盾退出；计数修正版再次实测仍因 372 标量超限退出；
当前同连接/生产限速版本已完成上述 14:49 单次近 home 回位。
测试同时发现并修复了 SDK 样本已过期时复位仍可能接受前端缓存的问题：使能、复位和
home 现在直接检查 SDK 缓存采样年龄，不以最近一次 ROS 定时读取代替新硬件反馈。

### 历史 ros_control 验证记录

以下记录针对显式 `backend:=ros_control`，底层为原 ros_control 硬件通道的
120 标量目标、20 点预装。当前新增离线集成测试
`rostest aubo_ros_control cartesian_jog_ros_control.test`，使用 100 ms 延迟的六轴模拟反馈，
检查绝对目标累积、home/取消和原始反馈中断、陈旧采样时的保护；测试已通过（1 项集成测试，含上述故障注入）。
该模型不加载真实 JTC 或 SDK，不能证明 JTC 插补、实际排队和真机停止行为。

`rostest aubo_ros_control aubo_hardware_feedback.test` 直接测试硬件接口的反馈回调，
在诊断 RPC 阻塞超过 240 ms 时仍发布新的六轴反馈；同时检查诊断完成、空样本、NaN
和已停止的回调不会刷新旧采样；新增推送停顿时独立查询补充、查询阻塞时推送继续、
旧回复丢弃、查询失败不更新时间和 220 ms 迟到回复保留请求时间戳的测试（共 2 项，已通过）。
该测试不调用 SDK login/startup，不连接或移动机械臂。

2026-10-24 将默认点动入口改为独立连接上的 SDK 实时反馈后，按现场授权完成一次
真机回 home：关节速度上限 0.10 rad/s，客户端最长等待 45 秒，不自动重试。
约 32 秒后 action 返回 `SUCCEEDED`，消息为 `Home reached; jog remains disabled`，
最终状态 `DISABLED`，最大关节误差约 0.00087 rad（阈值 0.005 rad）。客户端收到
1072 个原始反馈样本，观测到的最大到达间隔约 128.3 ms，低于保留的 150 ms 超时阈值；
运动期间未触发 `feedback_timeout`。这是一轮实测，不能保证后续所有反馈间隔。
关闭 launch 时，硬件和点动节点同时收到 SIGINT，反馈发布停止后点动节点仍在退出等待，
产生一条 `feedback_timeout`；该条发生在 home 成功之后，各节点正常退出，未屏蔽保护。
完整记录见 `logs/feedback_push_validation_20261024/validation_summary.md`。
当时尚未验证真机 XYZ 点动；后续记录见下方。取消过程、停止延迟和平滑性仍未验证。

同日 09:27 用户在 XYZ 点动开始透传约 167 ms 后再次触发 `feedback_timeout`，
说明上述单次 home 成功不能代表稳定解决。后续不运动的原生 SDK 只读测试也观测到
230–237 ms 推送间隔；另一次带系统调用采集的复测未出现长间隔，尚不能确定抖动的底层来源。
因此新增上述独立真实路点查询及独立 ROS 反馈队列，保留 150 ms 保护。
只读故障注入测试主动忽略 3 秒推送，补充查询得到的最大采样年龄约 100.4 ms；
没有 startup、进入透传或发送运动命令。完整 AUBO 相机模型加 100 ms 延迟模拟反馈的
XYZ 正反六方向及反馈中断保护测试 `cartesian_jog_real_model.test` 已通过。
随后按现场授权执行唯一一次真机基坐标 +X 点动：配置将最大线速度限制为 1 mm/s，
发送 2 秒后归零并禁用。客户端返回 `XYZ_TEST_SUCCESS`、结束为 `DISABLED`，
131 个原始反馈样本的最大观测到达间隔约 86.0 ms，运动期间没有反馈超时。
根据前后真实关节反馈和同一 URDF 正解计算，TCP 位移约为 `[2.0045, 0.0019, -0.0002]` mm；
该数值不是外部定位设备测量。退出时驱动停止反馈后仍有一条超时告警，各节点正常退出。
两次先行启动在参数/控制器未就绪校验处退出，均未发送使能；实际点动只执行一次，无自动重试。
完整日志、指标和单次标记见 `logs/feedback_fallback_validation_20261024/`。
本次只覆盖该低速 +X 动作；其余真机方向、速度、长期抖动和停止距离仍未验证。

后续用户再次点动的日志显示两条反馈通路同时延迟：SDK 推送间隔 272.73 ms，
独立查询耗时 213.40 ms；超时前一帧 ROS 传输/回调排队约 0.05 ms、锁等待约 0 ms。
150 ms 保护据此触发，本次现象仍未解决；上一轮短时 +X 成功不代表长期稳定。
按用户要求将实机夹爪模型固定为 `joint1=joint2=0` 后，完整模型离线测试通过，
未再出现缺失夹爪告警；汇聚状态持续发布时，原始反馈中断仍在约 150 ms 触发保护。
本次未执行新的真机运动，也未修改轨迹点存储/下发算法或放宽反馈超时。

默认入口出错时保留 `[AuboHW]`、JTC 和 `[cartesian_jog]` 日志；不要只查直接 SDK 的日志标签。

### 直接 SDK 旧版故障记录与诊断

以下历史记录使用名义目标 60 标量，保守积压硬上限 360 标量；当前默认目标已改为 300 标量。使用自定义 YAML 时检查
`sdk_mac_buffer_target`、`sdk_mac_buffer_limit` 和 `sdk_diagnosis_timeout`。
保护触发后停止节点，不自动重启运动。启动日志显示初始批次、补点节拍、名义时长、
积压硬上限和诊断看门狗。慢查询会记录耗时和积压保守上界，生成超时会额外区分锁等待
与计算耗时。若仍退出，保留 `[sdk/jog]`、`[sdk/diagnosis]`、`[sdk/feedback]` 和
`[sdk/event]` 报错前后的日志：

- 诊断或下发耗时高：检查有线连接、丢包、网卡节能以及其他 SDK 客户端。
- 生成计算耗时高：确认 Release 构建，检查碰撞模型与场景复杂度。
- 锁等待或供点调度间隙高：检查 CPU 负载与回调阻塞。

`Missing joint1, joint2` 对应夹爪反馈缺失，与六轴 SDK 缓冲欠载分别排查。
本机曾在有线 ping 0.12–0.29 ms、零丢包时测到 SDK 接口数十毫秒延迟；
单独启用 TCP 选项未稳定消除抖动。在三连接、80 次诊断查询的只读对比中，
将反馈由同步轮询改为实时推送后，诊断平均耗时从 13.9 ms 降至 7.0 ms，
最大值从 41.8 ms 降至 15.5 ms。这是一次测量，不是最坏耗时保证，且未发送运动点。
后续透传启动日志仍出现诊断查询 310 ms 和实时反馈超时，因此上述只读结果
不能证明进入 TCP2CANBUS 后通信正常，也不能据此屏蔽超时保护。
`[sdk/event]` 记录控制柜事件类型/错误码，`[sdk/feedback]` 记录样本年龄、
累计帧数与最大帧间隔，`[sdk/diagnosis]` 记录慢查询及失败时的 MAC 中断、CAN 和保护状态。
2026-10-23 对比正常入口后，修复了启动等待阶段发送重复静止点、误判运行中欠载和
调度间隙的问题。将缓冲临时改为正常入口的 120 标量仍有停顿，未保留此修改。
此前名义目标为 60 标量时，12 秒无面板真机启动测试读取到连续 `DISABLED`，
由测试超时正常关闭。该测试没有使能、点动或请求回 home，只验证静止启动，不能验证
运动连续供点。后续回 home 仍出现零缓冲、约 104.51 ms 同步诊断阻塞；首批 10 点
仅覆盖 50 ms，原先的首批等待逻辑没有解决供点阻塞。

直接 SDK 路径改为上述独立诊断、定时供点和保守额度控制后，完成 12 秒无面板真机禁用启动检查，
读取到连续三条 `DISABLED`，无故障并由测试超时正常关闭；未发起使能、home 或实际位移。
`sdk_flow_control_test` 在离线消费模型中
模拟反复 100 / 105 ms 查询延迟，各持续 2.5 秒，检查仍消费时不断流、积压不被低估；
同时覆盖查询开始时已有在途下发、旧零读数、停止消费后的硬上限、150 ms 看门狗、
失败下发、迟到响应和协议计数异常。这些是模拟验证，不是实际 SDK/网络的运行保证。
后续用户运动日志显示第二批 4 点下发在停流后才返回成功，完整耗时 **65.26 ms**；
之后的复现又测到 **89.24 ms** 同步下发耗时。成功返回不等于已经执行完毕，两个耗时都
超过 20 ms 补点周期，说明独立诊断没有解决下发接口自身阻塞。直接 SDK 路径仍未修复，
也不能把上述静止测试或离线额度模型当成连续运动通过的依据。

```bash
catkin_make tests
# 当前默认SDK分支的离线替身测试，不连接控制柜
catkin_make aubo_cartesian_jog_fake_sdk_node cartesian_jog_ros_control_integration_test -j2
rostest aubo_ros_control cartesian_jog_sdk50.test
rosrun aubo_ros_control cartesian_jog_common_test
rosrun aubo_ros_control sdk_tcp_transport_test
rosrun aubo_ros_control direct_sdk_backend_feedback_test
rosrun aubo_ros_control sdk_flow_control_test
rostest aubo_ros_control cartesian_jog.test
# 历史ros_control路径的100ms延迟反馈集成测试（已通过）
rostest aubo_ros_control cartesian_jog_ros_control.test
# 硬件反馈在诊断阻塞期间继续更新的离线测试（已通过，不连接真机）
rostest aubo_ros_control aubo_hardware_feedback.test
# 完整 AUBO 相机模型，XYZ 正反六方向和原始反馈中断（无真机连接）
rostest aubo_ros_control cartesian_jog_real_model.test
```

独立数值测试覆盖 30 万步输入、启停/反向/随机目标/NaN、速度/加速度/jerk 上界、
停止距离上界、停稳和参数拒绝。ROS 集成测试使用满秩 XYZ+RPY 合成模型及模拟反馈，
覆盖禁用不运动、使能前零指令、基/工具坐标系、输入/反馈超时、过期时间戳、
非法坐标系/角速度/NaN 和故障复位，无需连接机械臂。
同一集成测试还覆盖 home action：点动使能时拒绝目标、回 home 时拒绝点动使能和重复目标、
取消后停稳、反馈中断后等待恢复并返回失败，以及重新回 home 成功后保持禁用。

2026-10-20 在 ROS Noetic 下复现并修复了 Gazebo 启动姿态竞争：修复后反馈保持 home，
初始状态为 `DISABLED`。使用已有编译产物，在基坐标系和工具坐标系分别验证 XYZ 正负方向，
每次以 10 mm/s 点动 1 秒后发送零速度至停稳，检查 TCP 位移方向、姿态保持和无故障停止。
这不代表已验证 SDK 后端或真机。真机的按下/松手延迟、低速匀速性、反向平滑性、
TCP 偏差和停止距离仍需在目标环境验证。
新增 home action 后已重新编译节点和 Qt 窗口，并在 AUBO Gazebo 模型上验证从零位出发、
途中取消后返回 `PREEMPTED`、再次执行到达 home 后返回 `SUCCEEDED` 且保持 `DISABLED`。
不要把 200 Hz 配置值当作硬实时或真机平滑性的证明。

### 历史直接 SDK 故障日志（2026-10-23）

以下保留的是旧版 `backend:=sdk` 的失败记录，不是本轮 50 Hz 目标更新版本的通过结果。

```text
    aubo_cartesian_jog (aubo_ros_control/aubo_cartesian_jog_node)
    aubo_cartesian_jog_panel (aubo_ros_control/aubo_cartesian_jog_panel)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)

auto-starting new master
process[master]: started with pid [97809]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 2f45835e-cec9-11f1-ac07-2f9159443fdb
process[rosout-1]: started with pid [97834]
started core service [/rosout]
process[robot_state_publisher-2]: started with pid [97841]
process[aubo_cartesian_jog-3]: started with pid [97842]
process[aubo_cartesian_jog_panel-4]: started with pid [97843]
[INFO] [1792749890.698104612]: Loading robot model 'aubo_i5'...
[INFO] [1792749890.698298679]: No root/virtual joint specified in SRDF. Assuming fixed joint
[INFO] [1792749891.015764696]: Starting planning scene monitor
[INFO] [1792749891.016336594]: Listening to '/planning_scene'
[INFO] [1792749891.016350091]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1792749891.016769095]: Listening to '/collision_object'
[INFO] [1792749891.017103229]: Listening to '/planning_scene_world' for planning scene world geometry
[INFO] [1792749891.017839614]: Listening to '/attached_collision_object' for attached collision objects
[INFO] [1792749891.017849835]: Updating internal planning scene state at most every 0.020000 seconds
log4cplus: Just opened file: logfiles/trace_all.log
10/23/2026 18:04:51,020 [139657963875264] [INFO ] - log4cplus started.
10/23/2026 18:04:51,020 [139657963875264] [INFO ] - sdk log: Connect robot server success.
10/23/2026 18:04:51,090 [139657963875264] [INFO ] - sdk log: set dhparam success. type:0, DhParam:0.408000,0.376000,0.098500,0.121500,0.102500,0.094000
10/23/2026 18:04:51,091 [139657963875264] [INFO ] - sdk log: login  success.
10/23/2026 18:04:51,105 [139657963875264] [INFO ] - sdk log: enable joint6 360.
10/23/2026 18:04:51,105 [139657963875264] [INFO ] - sdk log: login completed, set joint6 retate 360 falg success. joint6Rot360Enable=1
10/23/2026 18:04:51,106 [139657963875264] [INFO ] - sdk log: Connect robot server success.
10/23/2026 18:04:51,153 [139657963875264] [INFO ] - sdk log: login  success.
10/23/2026 18:04:51,167 [139657963875264] [INFO ] - sdk log: enable joint6 360.
10/23/2026 18:04:51,167 [139657963875264] [INFO ] - sdk log: login completed, set joint6 retate 360 falg success. joint6Rot360Enable=1
10/23/2026 18:04:51,167 [139657963875264] [INFO ] - sdk log: Connect robot server success.
10/23/2026 18:04:51,216 [139657963875264] [INFO ] - sdk log: login  success.
10/23/2026 18:04:51,230 [139657963875264] [INFO ] - sdk log: enable joint6 360.
10/23/2026 18:04:51,230 [139657963875264] [INFO ] - sdk log: login completed, set joint6 retate 360 falg success. joint6Rot360Enable=1
[INFO] [1792749891.230915864]: [sdk] 已为 3 个控制柜 TCP 连接启用 TCP_NODELAY / TCP_QUICKACK
[INFO] [1792749891.319310736]: [sdk/event] type=19 code=1 content={"code":0,"text":"RobotStartupState."}
[INFO] [1792749891.920429718]: [sdk/event] type=19 code=2 content={"code":0,"text":"RobotStartupState."}
[INFO] [1792749891.970620658]: [sdk/event] type=19 code=3 content={"code":0,"text":"RobotStartupState."}
[INFO] [1792749892.020775602]: [sdk/event] type=19 code=4 content={"code":0,"text":"RobotStartupState."}
[INFO] [1792749892.171055654]: [sdk/event] type=19 code=5 content={"code":0,"text":"RobotStartupState."}
[INFO] [1792749893.924353891]: [sdk/event] type=34 code=8 content={"code":8,"text":"Safe io event:release safeguard mode in manual mode."}
[INFO] [1792749894.024594592]: [sdk/event] type=34 code=7 content={"code":7,"text":"Safe io event:release safeguard mode in automatic mode."}
[INFO] [1792749894.074761019]: [sdk/event] type=19 code=6 content={"code":0,"text":"RobotStartupState."}
[INFO] [1792749894.124925351]: [sdk/event] type=20 code=2 content={"code":2,"text":"RobotStartupDone."}
10/23/2026 18:04:54,165 [139657790592768] [INFO ] - sdk log: enable joint6 360.
10/23/2026 18:04:54,165 [139657790592768] [INFO ] - sdk log: init ik param success.
10/23/2026 18:04:54,167 [139657304078080] [INFO ] - sdk log: enable joint6 360.
10/23/2026 18:04:54,167 [139657304078080] [INFO ] - sdk log: init ik param success.
10/23/2026 18:04:54,201 [139657798985472] [INFO ] - sdk log: enable joint6 360.
10/23/2026 18:04:54,201 [139657798985472] [INFO ] - sdk log: init ik param success.
[INFO] [1792749894.225177973]: [sdk/event] type=42 code=1 content={"code":1,"text":"UpdateJoint6Rot360Flag."}
[INFO] [1792749894.525776952]: [sdk/event] type=35 code=0 content={"code":0,"text":"Robot hand shake Succ."}
[INFO] [1792749894.565582858]: [sdk] 已收到实时关节反馈；反馈使用推送，控制连接下发，MAC 连接仅查询诊断
[INFO] [1792749894.571541911]: [visual_servo/sdk] 已连接 192.168.1.2:8899，启动直接 SDK 队列下发
[INFO] [1792749894.571591580]: [sdk/jog] 独立供点：初始 10 点，之后每 20.0 ms 补 4 点；名义 50.0 ms，保守积压硬上限 60 点 / 300.0 ms，诊断看门狗 150.0 ms
[INFO] [1792749894.575004319]: [cartesian_jog] sdk 后端已启动，默认禁用；先发送零指令，再按住点动
[WARN] [1792749894.591734566]: The complete state of the robot is not yet known.  Missing joint1, joint2
[INFO] [1792749894.676034617]: [sdk/event] type=39 code=0 content={"addr":24,"status":1.0}
[INFO] [1792749894.676056440]: [sdk/event] type=39 code=0 content={"addr":25,"status":0.0}
[WARN] [1792749895.691691878]: The complete state of the robot is not yet known.  Missing joint1, joint2
[WARN] [1792749896.711673021]: The complete state of the robot is not yet known.  Missing joint1, joint2
[WARN] [1792749897.731699041]: The complete state of the robot is not yet known.  Missing joint1, joint2
[INFO] [1792749897.821068830]: [sdk/jog] 首个非零 MAC 缓冲已观察到：30 标量
[ERROR] [1792749897.875179944]: [sdk] SDK 下发阻塞超过名义前瞻时间；停止供点，需人工重启
[ERROR] [1792749897.875210354]: [sdk/transport] 已开始=2 批，已返回=1 批；在途批次=2，点数=4，已阻塞=52.03 ms，生成=0.27 ms，最大关节增量=0.000221241 rad；积压上界=14 点
[ERROR] [1792749897.875223711]: [sdk/transport] 最近返回批次=1 ret=0 耗时=24.45 ms；诊断在途=49.05 ms，最近MAC=30 标量（响应年龄=54.16 ms，请求年龄=58.75 ms，RPC=4.59 ms）；反馈年龄=39.92 ms，最大帧间隔=70.91 ms，累计=111 帧；-1 表示无样本/无在途
[INFO] [1792749897.880204255]: [sdk/event] type=39 code=0 content={"addr":1,"status":1.0}
[WARN] [1792749897.888449892]: [sdk/send] 批次=2 SDK已返回 ret=0 实际耗时=65.26 ms 停流后返回=是；返回不代表轨迹已执行完毕
[ERROR] [1792749897.891664023]: [cartesian_jog] sdk_backend_failed: SDK 下发阻塞超过名义前瞻时间；后端已停流，无法继续生成减速点，需重启节点
[INFO] [1792749897.980375631]: [sdk/event] type=39 code=0 content={"addr":9,"status":1.0}
[INFO] [1792749898.080544131]: [sdk/event] type=39 code=0 content={"addr":24,"status":0.0}
[INFO] [1792749898.130641815]: [sdk/event] type=39 code=0 content={"addr":25,"status":1.0}
[INFO] [1792749898.134963709]: Stopping world geometry monitor
[INFO] [1792749898.135140070]: Stopping planning scene monitor
log4cplus: Entering ConsoleAppender::close()..
log4cplus: Destroying appender named [ALL_MSGS].
log4cplus: Destroying appender named [STDOUT].
================================================================================REQUIRED process [aubo_cartesian_jog-3] has died!
process has died [pid 97842, exit code 1, cmd /home/zlab/aubo/ros_mobile_manipulation_lab/devel/lib/aubo_ros_control/aubo_cartesian_jog_node __name:=aubo_cartesian_jog __log:=/home/zlab/.ros/log/2f45835e-cec9-11f1-ac07-2f9159443fdb/aubo_cartesian_jog-3.log].
log file: /home/zlab/.ros/log/2f45835e-cec9-11f1-ac07-2f9159443fdb/aubo_cartesian_jog-3*.log
Initiating shutdown!
================================================================================
[aubo_cartesian_jog_panel-4] killing on exit
[aubo_cartesian_jog-3] killing on exit
[robot_state_publisher-2] killing on exit
QFileSystemWatcher::removePaths: list is empty
QFileSystemWatcher::removePaths: list is empty
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```
