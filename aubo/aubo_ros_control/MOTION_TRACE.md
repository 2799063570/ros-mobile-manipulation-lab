# Recording a real-arm trajectory

中文调试过程、实测对照、最终 JTC 配置与残余问题汇总见
[AUBO MoveIt 运动抖动与通信调试复盘](../../docs/AUBO_MoveIt运动抖动与通信调试复盘.md)。

Current configuration (2026-10-30): real-arm trajectory execution uses only
`aubo_ros_control/GuardedJointTrajectoryController`. The
`trajectory_execution_mode` launch argument and the experimental precomputed
executor have been removed. Use the recording command below without a mode
argument. The JTC fault latch, 40-point software queue limit, feedback timeout,
MAC buffer target and motion recording remain in place. Teach-pendant/read-only
mode is still available through `motion_source:=teach_pendant`.

The dated precomputed sections below are historical test records; their old
launch commands no longer apply. The offline analyzer still recognizes those
older bags so their different timing semantics are not mistaken for JTC data.

JTC-only cleanup validation: normal workspace build, 33 ROS regression tests
(27 streaming/controller and 6 feedback), and 8 Python analysis tests passed.
Both motion and teach-pendant launch configurations were parsed without
starting hardware processes. No real-arm motion was performed for this cleanup.

`aubo_real_bringup.launch` can optionally record the MoveIt plan, both action
results, controller desired/actual/error state, joint feedback, hardware MAC
flow diagnostics, TF, and ROS logs to a compressed rosbag. The hardware node
publishes a 10 Hz summary of MAC occupancy, queue depth, diagnosis/send RPC
latency, feed-loop timing, controller-loop period, and detected queue/SDK
errors on `/aubo_i5/aubo_hw_node/flow_diagnostics`.

Enable it when starting the real-arm bringup:

```bash
roslaunch aubo_ros_control aubo_real_bringup.launch \
  record_motion:=true
```

After the run, stop roslaunch normally. The bag name has a timestamp, for
example `$HOME/aubo_motion_2026-10-26-20-30-00.bag`. The default prefix is now
`$HOME/aubo_motion`, so captures survive temporary-directory cleanup. Inspect its topics with:

```bash
rosbag info "$HOME/aubo_motion_2026-10-26-20-30-00.bag"
/usr/bin/python3 $(rospack find aubo_ros_control)/scripts/analyze_moveit_motion.py \
  "$HOME/aubo_motion_2026-10-26-20-30-00.bag" \
  --json /tmp/aubo_motion_report.json
```

The main comparison is `/aubo_i5/aubo_i5_controller/state`: compare
`desired.positions` with `actual.positions`, and inspect `error.positions` at
the moment motion starts. `/move_group/display_planned_path` contains the
planned trajectory. Controller and `/execute_trajectory` action results show
whether execution completed, was rejected, or aborted. `/rosout` preserves the
corresponding node log messages.

The analyzer reports the largest observed command velocity/acceleration,
tracking error by joint, and hardware-flow counters. MAC values are cabinet
diagnostic samples, not hardware-timestamped execution positions. A low/empty
sample or long RPC is evidence to investigate; it does not alone prove a
physical CAN-bus underflow. Compare it with the controller error and action
result at the same time. Desired acceleration is measured from MoveIt/controller
trajectory data and can be compared with the tracking error to assess whether
the trajectory dynamics are too demanding.

This recorder observes ROS-level commands and feedback. It does not capture
servo-side motor current, torque, or internal controller telemetry from the
robot cabinet. If desired and actual joint positions already diverge in the
bag, collect the robot controller's own diagnostic/alarm log as a second trace.

## Continuous 5 ms streaming

The hardware driver now streams stationary holds as well as moving samples.
With the default MAC target of 300 scalars, the initial preload is 50 points
(250 ms). A target of 300 does not provide 250 ms of protection when only
20 points are actually preloaded: a realtime producer cannot fill the remaining
30 points while the cabinet consumes at the same rate. This was a weakness in
the previous setup. The preload default now follows the configured MAC target.

Every generated sample represents exactly 5 ms. Duplicate positions and tiny
steps must not be deleted. After a late host wakeup, the controller samples
the missing intermediate times using a fixed step instead of jumping forward
in the trajectory; a backlog beyond the MAC target horizon terminates the node.
Queue overflow and velocity/acceleration violations latch a stop. The driver
does not insert extra points to hide an oversized step. A signed velocity
difference detects reversal accelerations that the previous absolute-speed
difference missed. Existing MoveIt Ruckig settings and path tolerances remain
in effect.

SDK sockets use the existing `SdkTcpTransport` implementation (`TCP_NODELAY`
and re-armed `TCP_QUICKACK`). MAC queries and sends remain serialized because
the SDK supplies neither a sample timestamp nor a point sequence. Moving a
query to another thread without accounting for concurrent sends can overfill
the cabinet using a stale observation. These changes reduce avoidable latency
and protect trajectory timing; they do not provide a hard realtime guarantee
against a prolonged SDK/network/OS stall. A confirmed continuous stream
observed empty, or a cabinet `macDataInterruptWarning`, latches a stop and
requires a node restart. Stationary holds mean there is no intended empty tail.

New diagnostics distinguish these cases:

| Field | Meaning |
| --- | --- |
| `mac_active_empty_samples` | Empty observations after a nonzero active MAC sample, including producer starvation |
| `mac_data_interrupt_warning_samples` | Cabinet MAC interruption flag observed |
| `stream_fault_latched` | Stream or command validation fault requires restart |
| `command_velocity_max_rad_s` | Peak discrete velocity before sending |
| `command_acceleration_max_rad_s2` | Peak signed-difference acceleration magnitude before sending |
| `command_jerk_max_rad_s3` | Peak signed-difference jerk magnitude before sending |

Command peaks include rejected candidates so a violation remains visible. They
describe driver samples, not measured motor dynamics. Compare these peaks with
the planned/controller trajectory and with low-water, query/send latency and
feedback at the same time. A 50 Hz controller state topic can miss a 5 ms spike;
the 10 Hz flow messages retain maxima from each reporting window.

The default preload introduces about 250 ms of nominal command delay, plus any
software/network backlog. Verify tracking error against the existing 0.15 rad
path tolerance; increasing that tolerance would mask the delay. Test first in
a cleared workspace at conservative speed, with motion recording enabled.

Offline regression (no SDK login/startup or real-arm commands):

```bash
rostest aubo_ros_control aubo_hardware_streaming.test
rostest aubo_ros_control aubo_hardware_feedback.test
```

The streaming test covers a deterministic cabinet model with an isolated
167 ms query and 20 ms sends, holds/tiny moves, a 50 ms producer pause, signed
reversals, queue overflow, partial-point capacity, RPC failures, and refusing
to resume after an empty/interrupted stream. This model does not certify the
timing or freshness behavior of the physical cabinet.

## 2026-10-27 recorded vibration investigation

Two physical runs are available under `/tmp`:

| Bag start time (Asia/Shanghai) | Executions | MAC evidence | Largest measured query / send |
| --- | --- | --- | --- |
| 21:12:35 | 3 succeeded, 1 path-tolerance abort | One confirmed empty observation with 112 host points pending; stream stop latched | 351.620 / 205.990 ms |
| 21:14:31 | 3 succeeded, 2 path-tolerance aborts | No sampled empty/interruption flag; minimum observed active MAC 6 points | 223.578 / 208.194 ms |

The first bag demonstrates that even the 250 ms continuous-stream preload can
be exhausted by a sufficiently slow synchronous query. The second bag's maximum
feed-loop interval is 255.211 ms. Absence of an empty observation in that run
does not rule out a short interruption between observations. No RPC returned a
failure code; an SDK success result alone does not establish timely delivery.

All recorded RViz MoveGroup requests used velocity and acceleration scaling
`1.0`. The handeye verification node's `velocity_scaling` launch argument does
not change RViz's MotionPlanning scaling controls.

There is also a separate, reproducible trajectory problem. The first wrist1
goal in the 21:14:31 bag has 9 sparse P/V/A knots. Their maximum acceleration
is 0.5921 rad/s². Reconstructing the exact quintic interpolation used by
`JointTrajectoryController` gives:

| Quantity | Continuous quintic peak | Configured limit |
| --- | --- | --- |
| Velocity | 0.50999 rad/s | 0.5 rad/s |
| Acceleration | 1.49844 rad/s² | 0.6 rad/s² |
| Jerk | 42.28659 rad/s³ | 2.5 rad/s³ |

The driver observed 1.49709 rad/s² and 36.07073 rad/s³ in its 5 ms candidates;
the 50 Hz controller state also captured an acceleration peak of 1.49460 rad/s².
These values agree with an interpolation overshoot. Sparse knot differences
previously reported only 2.2061 rad/s³ and did not bound the executed curve.
The installed MoveIt1 Ruckig adapter does not transmit its full piecewise jerk
profile to ros_control; checking only the waypoint boundary values is insufficient.

`aubo_moveit_config/LimitSplineDynamics` now runs after Ruckig (outermost/first
entry in the adapter list). It reconstructs each final P/V/A quintic, checks
endpoints and interior derivative extrema, and computes a common time dilation:
`max(1, V/V_limit, sqrt(A/A_limit), cbrt(J/J_limit))`. Durations multiply by this
factor, velocities divide by it, and accelerations divide by its square.
The interpolated geometric path is preserved. Request V/A scaling is included;
jerk uses the same `ruckig/<joint>/jerk_limit` parameters as MoveIt1.
An extra 0.1% timing margin covers duration rounding. Missing limits, invalid
segments or a nonstationary start requiring dilation cause planning to fail.

Offline reconstruction of all 9 recorded goals requires about 2.30–2.57 times
their original durations at the recorded request scaling. The first wrist1
goal becomes about 5.48 s instead of 2.13 s. This is a prediction, not a new
physical run. Tests compare against the installed ROS controller spline, replay
the recorded wrist1 goal, check request scaling and path preservation, reject
invalid input and verify plugin discovery.

This limiter covers planned trajectories. It does not certify the controller's
generated stop/bridge segments, internal robot servo behavior, or communication
timing. It does not repair the demonstrated 352 ms query stall. Cabinet logs
and per-connection packet timing are needed to attribute that stall. The robot
route uses wired interface `enp11s0`; current interface counters show no RX/TX
errors or drops. System-wide cumulative TCP retransmission counts cannot be
attributed to the robot connection.

Restart MoveIt to load the new adapter, replan, and record another short run.
For a low-speed comparison, explicitly set **both RViz scaling controls to 0.1**.
Look for `Quintic spline dynamics retimed ... (V/A/J limits)` when dilation is
needed. Keep the path tolerance unchanged and compare flow diagnostics, command
peaks and residual vibration. Preserve the current two bags as baselines.

## Balanced profile and straight-path timing optimization

Both supplied RViz configurations now default to velocity scaling **0.5** and
acceleration scaling **0.4**. These are saved GUI defaults, not a replacement for
the limits in `joint_limits.yaml`. The handeye verification node keeps its own
low-speed configuration. Restart RViz/MoveIt and replan to use new settings.

The recorded paths are monotonic straight lines in joint space, including the
old controller interpolants. For this case the final dynamics adapter now uses
a single scalar Ruckig S-curve: `q(t) = q_start + delta_q * s(t)`. Scalar V/A/J
limits are the minimum of each moving joint's scaled limit divided by its joint
travel. All joints share the same progress, so the geometric path is retained.
The optimizer retains every original waypoint and inserts jerk-phase boundaries.
Each constant-jerk cubic phase is represented with P/V/A states for ros_control;
timestamp rounding introduces only small numerical perturbations, and the
complete serialized spline is checked again against V/A/J limits.

Eligibility requires stationary endpoints, strictly monotonic progress, no
intermediate stop/dwell or initial delay, unchanged non-group positions, and a
maximum lateral error of 1e-9 rad over the complete original spline. Collinear
knots alone are insufficient. Curved paths, moving boundaries, failed numerical
checks or an optimization saving less than 1% use the existing checked uniform
retiming fallback. `optimize_straight_paths=false` in the planning adapter's
namespace disables the new timing optimizer for an A/B comparison.

Offline replay of all 22 goals in the four October 27 bags passed the controller
spline V/A/J checks, original-waypoint retention and straight-line checks. Replay
used the recorded scaling capped at 0.5 / 0.4. For the five goals already recorded
at exactly that profile in the 21:43:42 bag:

| Execution | Recorded duration | New predicted duration | Reduction |
| --- | --- | --- | --- |
| 1 | 9.753 s | 5.562 s | 43.0% |
| 2 | 10.023 s | 6.346 s | 36.7% |
| 3 | 11.378 s | 7.135 s | 37.3% |
| 4 | 11.631 s | 6.790 s | 41.6% |
| 5 | 11.860 s | 7.036 s | 40.7% |

These are offline predictions, not physical measurements. The first five
predictions reach approximately 0.1748 rad/s, 0.1598 rad/s² and 1.998 rad/s³ on
the limiting arm joint, inside its scaled 0.175 / 0.16 / 2.0 limits. A faster
command also increases error caused by transport delay; retain the existing
0.15 rad path tolerance and repeat a short recorded physical trial.

The hardware feeder now waits for at least four points and four **measured**
free MAC slots before a refill at the default target. This amortizes synchronous
RPC overhead over 20 ms of trajectory without deleting samples. The first
50-point preload is unchanged. MAC queries start no faster than every 5 ms;
time spent inside an RPC counts toward this interval. Elapsed time never grants
buffer capacity. Small custom targets use a proportionally smaller batch.
The fast-link cabinet model verifies point conservation, no starvation, bounded
refills and batches of at least four points after preload.

`flow_diagnostics` additionally reports `minimum_send_batch_points`,
`small_batch_wait_samples`, `minimum_mac_query_interval_ms`, `tcp_info_valid`,
`tcp_connections`, `tcp_total_retransmissions`, `tcp_retransmissions_window`,
`tcp_max_rtt_ms`, `tcp_max_rto_ms` and `tcp_unacknowledged_segments`.
TCP snapshots cover only this process's sockets to the robot endpoint. RTT/RTO
are the maximum kernel estimates across those sockets at reporting time; RTO
is a configured timeout estimate, not evidence that a timeout occurred.
Client-side retransmission counts do not reveal every cabinet-to-client lost
packet. `analyze_moveit_motion.py` summarizes the new metrics and still accepts
old bags without TCP fields.

Batching/polling reduces avoidable request overhead, but it does not establish
that the observed 200–350 ms stalls are fixed. The next physical bag must compare
query/send latency, queue depth, MAC watermarks and per-connection TCP metrics.
No kernel/network tuning or automatic robot movement is part of this change.

## Startup reference and stationary trajectory joins

The hardware producer now waits until `controller_manager` has started a
controller claiming all six position joints. A joint-state controller alone
does not authorize a command stream. This avoids queuing the older SDK-login
hold before JointTrajectoryController initializes its hold from newer feedback.
The feeder seeds its finite-difference reference from that first controller
command. It must agree with current feedback within 0.001 rad on every joint;
larger differences latch a stop. Command positions are not snapped or filtered.
This readiness flag concerns initial startup; it is not a hot-switch recovery
mechanism and faults still require restart.

The final planning adapter gives a stationary first waypoint a default
`time_from_start` of 0.2 s. This lets JTC connect its preceding hold to the new
feedback-based start over a finite interval, rather than compressing a few
microradians of discrepancy into one 5 ms slot. The S-curve motion after that
first waypoint is unchanged. Existing longer delays remain; moving starts are
not modified by this feature. Parameter `stationary_start_transition_duration`
in the planning pipeline namespace accepts 0..2 s; zero disables the addition.
This adds at most 0.2 s per ordinary stationary-start plan.

The test uses the actual controller_manager and position JointTrajectoryController
with in-memory hardware, verifying both startup claims and a future first
waypoint. A 10-microradian, rest-to-rest position difference over 0.2 s has a
quintic jerk peak of approximately 0.075 rad/s³. This is an example for a small
stationary discrepancy, not a bound on arbitrary preemption, large tracking
errors or unknown moving controller states. Generated JTC bridge/stop segments
still need checking in recorded driver samples.

## Detailed command and TCP/RPC capture without root privileges

With `record_motion:=true`, bringup also prepares a unique
`$HOME/aubo_motion_hardware_<capture>` prefix. No SDK thread writes files during
motion. On orderly shutdown, after feeder shutdown and leaving stream mode,
the node exports:

* `<prefix>.commands.csv`: every successfully queued 5 ms command, its ID,
  steady-clock recording time, ROS recording time and controller sample time.
  `q0..q5` follow the hardware `joint_names` order. The first row is the seeded
  reference; these are queued commands, not motor measurements.
* `<prefix>.rpc.csv`: each query/send begin and return time, result, MAC count,
  point count and first queued-point ID, plus before/after TCP_INFO for each
  socket to the configured cabinet endpoint. Port, byte counters, retransmissions,
  RTT/RTO, unacknowledged/notsent bytes and last payload age are retained.

The MAC connection's local-port hint is inferred from the one socket added by
its login; no SDK object internals are accessed. Ambiguous discovery or a later
SDK reconnect makes attribution uncertain, and the analyzer reports that.
Extended TCP_INFO fields are length-checked for kernel compatibility.

The default `motion_trace_capacity=120000` retains that many commands and up to
15000 RPCs, using preallocated bounded memory. The files report dropped records
if a limit is reached. Existing captures are never overwritten at initialization.
Export uses temporary files and renames them only after completion. If the
process is killed during export, the pair retains an incomplete marker rather
than presenting a partially written capture as complete.
The `motion_trace_prefix` launch argument can override the unique prefix;
`analyze_moveit_motion.py` also lists recorded prefixes from flow diagnostics.

Analyze the complete pair with:

```bash
rosrun aubo_ros_control analyze_hardware_trace.py \
  "$HOME/aubo_motion_hardware_<capture>" --slow-ms 50 \
  --json /tmp/aubo_hardware_report.json
```

For a slow RPC, payload received well before its return suggests investigating
SDK processing/wakeup or host scheduling. Payload received near return keeps
cabinet processing and inbound-network delay as candidates. A client TCP
retransmission is recorded separately. TCP_INFO identifies socket payload timing,
not application RPC payloads: asynchronous data can affect these observations,
and zero outbound retransmissions does not exclude inbound loss. These records
improve attribution; they do not prove the 200 ms stalls have been eliminated.

## 2026-10-28：运动中抖动的修改计划与实现

本次以 14:19:33 的五趟真机记录为依据。完整的 23,985 个命令没有
5 ms 网格缺失，V/A/J 峰值分别为 0.174825/0.159840/1.998008；
查询最长 213 ms，MAC 最低观测为六点，软件队列峰值 95 点。
第 4 趟存在旧推送帧集中补发，不能把由此产生的反馈速度尖峰当作真实机械振动。

### 1. 独立调整 jerk，保留运动速度与加速度

`ruckig_limits.yaml` 将前三轴 jerk 从 2.0 调到 1.0 rad/s³，后三轴从
2.5 调到 1.25 rad/s³。RViz 的速度/加速度比例继续为 0.5/0.4。
MoveIt Ruckig 和最终样条校验器使用同一组限制，原有 0.2 s 静止起步
连接保留。S 曲线仍为分段常值 jerk，不能宣称 jerk 连续。

同路径标量 Ruckig 离线复算预测：五条轨迹分别由
8.240910/19.411622/4.900300/4.107046/11.091758 s
变为 8.320910/19.491622/4.980300/4.187046/11.171758 s。
这不是新的真机测量，实际减振效果需要复测。

### 2. 消除查询与旧推送交替覆盖

正常时继续接收 SDK 推送。当有效推送间隔达到 60 ms，独立查询线程在
发出第一个请求前，持锁将来源切换为 `query_latched`；此后直到节点重启
只接受查询结果。即使查询失败或阻塞，后续推送也不能刷新反馈时间。
SDK 推送没有柜端采样时间戳，因此不能仅凭恢复后的回调到达时间判断
它是否新于已查询的位置，不自动切回推送。

接管后查询按请求起点限频到最多 25 Hz，避免不受限地增加柜端 RPC
压力；慢调用不再额外等待一个完整周期。查询保留请求时的 ROS 与
steady-clock 时间，普通非推送模式的查询也采用相同处理。
这会改变反馈的传递方式和更新节拍，仍需对照真机查询耗时验证。

### 3. 约束反馈年龄与积压，保留通信证据

新增两个启动参数，并在连接 SDK 前验证：

| 参数 | 默认值 | 范围 | 行为 |
|---|---:|---:|---|
| `feedback_timeout` | 0.2 s | 0.1..0.5 s | 生产和消费端检查有效样本年龄，超时锁定停流 |
| `max_command_queue_duration` | 1.0 s | 0.5..5 s | 限制待发点数的名义时长，以及最旧待发命令实际等待年龄 |

每个队列元素保存入队的 steady-clock 时间；不依赖可选 CSV 记录才能
执行积压保护。达到上限时不丢弃旧点后追赶，不重新插值，不自动续发。
队列上限仅约束软件待发部分，已经下发到 MAC 的点不能撤回。
这些是驱动故障处理，不是安全认证急停或保证实时完成的减速停止。

诊断新增反馈来源、忽略推送累计数、有效采样年龄、查询耗时/失败累计、
反馈超时锁定、最旧命令等待年龄、队列上限和积压锁定状态。
另记录 MAC 慢查询次数（>50 ms）、最近查询耗时，以及查询耗时超过
返回水位名义覆盖时间的次数。后者只是诊断比较，不能证明水位已干涸。
分析脚本支持这些字段，并保持旧 bag 和无 flow 记录可读。

MAC 查询与发点仍在同一专用 SDK 对象上串行执行。诊断没有柜端采样
时间戳，目前无法从旧回复证明并发发送可用的容量；本次不以墙钟时间
推算空位，不增大 MAC 目标，也不根据约 199.55 点/s 的推算改变固定
5 ms 采样或删除保持点。213 ms 响应等待的柜端/入站网络原因尚未消除，
本次新增保护和定位证据，不能宣称通信延迟已修复。

### 4. 验证与真机复测

离线回归覆盖：记录中的七帧旧位置补发、查询尚未返回时的来源竞争、
首次查询失败、慢查询保留旧请求时间、反馈超时禁止入队/发送、积压数量
和实际年龄超限、原有断流不续发、实际 ros_control 控制器的启动/轨迹连接，
以及降低 jerk 后的完整控制器样条 V/A/J 边界和峰值 V/A 保持。
所有测试使用模拟 SDK 或内存模型，不执行真机运动。

停止旧进程后，用正常工作区重新启动：

```bash
source /home/zlab/aubo/ros_mobile_manipulation_lab/devel/setup.bash
roslaunch aubo_ros_control aubo_real_bringup.launch record_motion:=true
```

保持 RViz 0.5/0.4，先重跑相同起止姿态的短路径，再比较较长路径；
尽量记录抖动所在趟次。正常 Ctrl+C 退出保存 bag 和配套两个 CSV。
若出现 `query_latched`，下一次分析应检查采样年龄是否稳定、旧帧是否被
忽略，再对照 MAC 慢响应与队列增长；若触发上述上限，按首条故障记录
定位后重启，不通过放宽路径误差或自动恢复继续旧轨迹。

本次离线验证结果：正常工作区两包编译通过；35 项 ROS 回归全部通过
（反馈 3、流控/真实控制器 19、轨迹样条 13），5 项 Python 分析测试
及 TCP 传输测试通过。只解析启动参数验证了新阈值和 jerk 配置传递，
没有启动真机节点。`git diff --check` 通过。首次测试发现原控制器测试
人为积压整秒后一次性消费会触发新的实际年龄限制，现已改为模拟真实
喂点线程定期消费，并单独保留超龄拒发用例。

## 2026-10-28：预生成轨迹执行对照模式

新增 `trajectory_execution_mode:=precomputed`。默认仍为 `jtc`，用于保留原模式对照。
预生成模式不创建 controller_manager，也不启动 controller_spawner；相同的
`/aubo_i5/aubo_i5_controller/follow_joint_trajectory` action 由独立执行器提供，
状态话题和关节状态继续发布，MoveIt 的 action 配置不变。

收到完整轨迹后，按关节名重排，使用与 JTC 相同边界条件的五次多项式，
在统一 5 ms 网格上预生成全部位置/速度/加速度。校验每个完整多项式的
位置与 V/A/J 极值，以及采样点有限差分的 V/A/J（含静止起点、终点过渡）。
限值从当前 robot_description_planning 与 /ruckig 读取；不重新缩短轨迹时间。
末点不在网格时，最后一个网格点保持精确终点，增加不足 5 ms。

整条轨迹存放在有界、不可变的轨迹存储中，最多 300 s / 60001 个输入路点。
发送线程仅在 MAC 查询确认有空间时取下一批点，立即通过原有 FIFO 校验并发送。
因此无需 200 Hz 主循环持续供点，也不把整条未来轨迹塞入受 1 s 年龄限制的 FIFO。
停顿后保留全部点和顺序，不丢点赶时间。初次预装、MAC 目标、反馈查询策略保持原值；
TCP 长延迟和反馈超时保护仍然存在，这个改动不代表通信根因已解决。

### 使用

在工作空间根目录（只构建/加载不会发起运动）：

```bash
source devel/setup.bash
```

用户现场准备好后，以以下命令启动新模式；启动连接真机，接受轨迹后才进入透传：

```bash
roslaunch aubo_ros_control aubo_real_bringup.launch \
  trajectory_execution_mode:=precomputed record_motion:=true
```

RViz 继续固定速度/加速度缩放 `0.5 / 0.4`。本轮保持 `mac_buffer_target=300`，
不同时提高速度或改水位。对照旧模式只改为 `trajectory_execution_mode:=jtc`。
两种模式必须分别启动，不能同时控制同一机械臂。

### 当前接口边界与终止行为

- 接受完整六轴 P/V/A 轨迹，要求静止始末状态；不支持 effort、部分关节轨迹、
  非零 header.stamp 的定时启动，均明确拒绝。并发新目标也拒绝，不隐式拼接/抢占。
- 起点必须距当前保持命令不超过 0.001 rad，实际关节距保持命令不超过 0.01 rad，
  且速度不超过 0.05 rad/s。t=0 起点以保持命令为准，仅首段发生至多 1 mrad 的调整，
  仍执行完整多项式限值校验；t>0 则添加 t=0 的静止边界。不满足就拒绝，不突然跳到起点。
- 支持逐关节位置 path/goal tolerance，默认仍为 0.15 / 0.05 rad；正的速度/加速度
  容差暂不支持并明确拒绝。终点必须满足速度不超过 0.05 rad/s。
- 只有终点批次成功发送后，等待完整 MAC 目标覆盖时长，再取得时间不同、覆盖至少
  100 ms 的新鲜静止到位反馈，再确认柜端停止和退出透传，才返回成功；
  预生成结束或发送 SDK 返回都不算到位。
- 取消、超差、反馈超时和通信故障会锁停。主线程停止并等待发送线程退出，然后调用
  柜端 RobotMoveStop 和 LeaveTcp2Canbus，最后才结束 action；SDK 未确认停止时返回
  aborted，不报告取消成功。取消/故障后需重启节点，当前不支持在线恢复。
- SDK 调用不能被这个执行器中途打断，取消延迟仍受正在执行的 RPC 影响。
  停止 RPC 的实际制动行为未做真机验证，不能把测试中的模拟成功视为实机停止保证。

### 对照记录

flow_diagnostics 新增执行模式、已生成/已提交点数和 MAC 推算消费序号。
预生成模式的 controller/state 目标相位由 MAC 水位估计，SDK 没有柜端时间戳，
该相位不是真实采样时刻；不可直接将其跟踪误差与旧 JTC 的墙钟目标误差比较。
分析脚本已标记这一区别。重点比较原始轨迹点连续性、MAC 最低水位、RPC 长尾、
重传与现场体感，不能把跟踪误差降低当成振动减小。

### 离线验证

新增 11 项测试：与实际 JTC 五次插值逐点比较、非整数网格终点、名字重排和保持段、
完整曲线和边界限值、错误输入、供点/成功判定、无控制器生产线程且注入 150 ms
停顿的喂点、失败发送、陈旧反馈、真实机器人模型限值加载、取消/拒绝替换目标、
柜端停止未确认时拒绝取消成功，以及静止到位必须等待新鲜采样。
另运行现有 19 项流控、3 项反馈测试，以及 5 项 Python 记录分析测试。
所有 SDK 运动调用均为模拟，未登录或移动真机。

## 2026-10-29：预生成模式到位后退出透传

现场反馈为运动中仍有抖动，随后到位退出。对应 ROS 日志目录
`~/.ros/log/bf8519b2-d2c9-11f1-a245-471320312239/` 中有 6 次轨迹成功记录。
最后一次 controller 成功时间为 1793190045.6399，反馈超时锁停为
1793190047.9875，相差约 2.35 s；随后独立反馈查询返回，耗时 207.05 ms。
本次 `/tmp` 下的 bag 和逐点/RPC CSV 已不可用，不能据此判定运动中的
MAC 是否干涸、具体抖动段或通信长尾的根因，也不能把执行成功解释为无振动。

此前预生成模式一直发送终点保持点，到位后仍处于连续透传，200 ms
反馈保护因而仍会触发。现在正常执行按以下状态转换：

1. 启动后空闲，不发送保持点、不查询 MAC；安全诊断和关节反馈继续运行。
2. 新目标必须通过新鲜反馈、静止、起点和全轨迹校验，发送线程进入透传并预装。
3. 满足原有终点/静止条件后，请求发送线程执行 RobotMoveStop、LeaveTcp2Canbus；
   两者成功后确认空闲，再返回 action 成功。等待确认期间仍视为执行中。
4. 空闲反馈过期只暂停状态发布并拒绝新目标，后续新鲜反馈可重新允许接收目标。
   下一趟重新进入透传，并清除上一趟 MAC 非零确认、启动计时和差分参考。

运动、取消、通信失败和停机确认失败的锁停规则保持；200 ms 阈值没有放宽。
该改动解决的是到位后仍持续透传的生命周期问题，尚不能证明运动中抖动已改善。
诊断新增 `prepared_idle_requested` / `prepared_idle_confirmed`，用于对齐动作状态。
默认记录位置改为 `$HOME/aubo_motion*`，仍需正常退出才能导出内存中的 CSV。

复测继续使用上述 precomputed 启动命令，速度/加速度保持 0.5/0.4，MAC 目标 300。
每次到位应出现“已停止供点并退出透传，进入空闲”，之后可再执行下一条轨迹。
完成后正常 Ctrl+C 保存记录，再分析运动段的发送间隔、水位、逐点连续性与现场体感。

离线验证：14 项预生成测试、19 项流控测试、3 项反馈测试全部通过，另有
6 项 Python 分析测试通过。新增覆盖空闲反馈过期、停止失败不确认空闲、
成功等待停止确认、拒绝陈旧反馈的新目标，以及同一发送线程连续完成两趟、
中间不供点且第二趟重新预装。正常工作区编译通过，launch 只做参数/节点解析，
确认持久路径、200 ms 阈值和 MAC 目标 300；未连接或运动真机。

## 2026-10-29：连接真机的静止对照、工具参数与反馈 RPC

实测记录保存在 `/home/zlab/aubo/network_diagnostics/live_debug_20261029/REPORT.md`。
原 100 Mbps 半双工与临时全双工各做 5400 次并发 SDK 查询：前者最长 226.594 ms、
两次超过 200 ms、两次 TCP 重传；后者最长 80.986 ms、未记录到重传。
所有查询成功，未发送运动指令。试验结束已恢复原网络设置，IP/掩码/网关保持原值。
这是静止条件下的对照证据，尚未读取柜端双工配置，亦不代表真实运动下振动已解决。

柜端读取工具负载为 0 kg，用户确认实际末端装有工具，负载/重心尚未标定。
驱动原先在 RobotStartup 时将工具动力学参数清零；现改为读取、校验并保留柜端已有参数。
读取失败、非有限值或负质量拒绝启动，不以零值代替。允许真实空载的零质量。
此修改不会推算或自动设置实际工具参数；现场需先配置并启用正确负载/重心，
再观察启动日志中的“保留柜端工具参数”是否与实际值相符。

详细 CSV 新增 `kind=feedback_query`，记录独立路点查询的 SDK 耗时与调用前后
TCP_INFO，并使用该反馈连接登录时识别的端口提示。`query` 仍专指 MAC 诊断查询，
`send` 专指轨迹发送。使用当前 `analyze_hardware_trace.py`，其报告分别给出三类统计；
`is_rpc_login_hint` 为通用归属提示，旧 `is_mac_login_hint` 仅对 MAC query/send 为真。
导出仍在生产线程 join 后进行；反馈时间戳仍取请求时间，不因日志记录或 SDK 返回而续期。

验证：39 项 ROS 回归（预生成 14、流控 19、反馈/工具启动 6）和 7 项 Python 分析测试通过，
正常工作区编译通过。新增覆盖慢反馈返回后仍可导出记录、不混淆 MAC/反馈归属、
保留非零工具负载/重心、空载、读取失败/非法参数时禁止启动，以及 SDK 启动失败的传播。

## 2026-10-29：全双工后的 JTC 运动记录与实际缩放参数

上述临时对照之后，按用户要求已将电脑直连网口持久配置为 100 Mbps 全双工；
IP、掩码和网关保持原值。用户要求暂时保持现有工具设置，未写入猜测的负载参数。

13:50 的完整记录及逐趟分析位于
`/home/zlab/aubo/network_diagnostics/jtc_full_20261029_1350/REPORT.md`。
5 趟 JTC 轨迹均成功，但用户仍感觉抖动。bag 中实际请求均为速度/加速度
`0.7 / 0.4`，并非 RViz 文件默认的 `0.5 / 0.4`。运动窗口最低采样 MAC
为 25～32 点，未采样到断粮或反馈超时；运动期间仍观察到一次 TCP 出站重传，
不能据此认定通信完全稳定，也不能将跟踪误差等同于振动幅度。

`analyze_moveit_motion.py` 新增 `motion_entrypoints.move_group_requests`，
按请求保存 goal ID、接收时间、规划组和原始速度/加速度缩放值，避免用配置默认值
推断实际运行参数。新增离线 bag 回归覆盖请求间缩放变化；3 项该脚本测试通过。
这些字段记录的是请求值，不推断 MoveIt 对特殊默认值的最终解释。
后续对照应固定起终点并检查实际规划路径，分别记录 `0.5 / 0.4` 和 `0.7 / 0.4`；
本轮未修改运动算法、反馈保护阈值或 MAC 水位。

## 2026-10-29 晚间：JTC 超差锁停与队列限量生成

20:30 记录的第一趟为 0.5/0.4，后六趟为 0.8/0.4。最后三趟返回
PATH_TOLERANCE_VIOLATED 后仍持续输出运动目标约 5、4、3 秒。软件队列逐趟
增至 84 点，指令与反馈的拟合滞后从约 380 ms 增至 555 ms；不是同路径的严格速度对照，
这些数也不是振动幅度。详细证据位于
`/home/zlab/aubo/network_diagnostics/jtc_full_20261029_2030/REPORT.md`。

### 超差后的行为

真机配置改用 `aubo_ros_control/GuardedJointTrajectoryController`，仍复用上游 JTC
五次插值、路径/到位容差和 action 接口。上游 update 返回执行错误时，在同一个控制周期
锁定硬件输出；随后的 write 不入队，主循环退出并关闭发送线程。无效目标被拒绝、
正常成功不会触发该锁定。故障后不可直接恢复运动，必须重启节点。

两种执行模式的 shutdown 都在发送线程退出后调用柜端 RobotMoveStop 和
LeaveTcp2Canbus；两者都成功才记录“已应答停止和退出透传”。停止失败仍记录错误，
不会将 SDK 断开日志描述为已经安全停止。停止调用安排在等待独立反馈线程之前，
避免无关反馈 RPC 额外延迟停机；日志导出在停止和连接关闭之后进行。
已经进入 SDK 的发送不能撤销或中断，必须等待返回后再停止，后续不再提交新批次。
action 的 ABORTED 仍表示执行失败，不能用作柜端停止确认；SDK 应答也不替代现场制动验证。

### 生产节拍与记录

首次仍预装 50 点、MAC 目标仍为 300 个关节标量（50 点）。开始供点后，
主控制循环在生成下一个采样之前检查软件队列：达到 40 点（200 ms）时暂缓推进
JTC，腾出空间后生成原本的下一个 5 ms 采样。不删除静止点、不跳过运动点、不根据
估算的柜端空位冒险补发。实际耗时可能稍长于名义规划时间，但不会靠增加软件积压
保持电脑墙钟进度。反馈新鲜度检查在等待期间继续运行，200 ms 保护和原队列年龄硬上限保留。

JTC uptime 按成功生成的固定 5 ms 样本推进，ROS 时间戳使用当前时间；逐点 CSV 的
sample_ns 仍是连续 5 ms 逻辑网格，ros_ns/steady_ns 记录真正生成时刻。
有意等待不计入下一轮的追赶欠账，真正的主机调度停顿仍保留原追赶上限保护。
诊断和分析报告新增 `jtc_queue_target_points`、`jtc_backpressure_waits_total`。

### 离线验证与复测

使用真实 controller_manager、插件加载和 JTC action 接口配合模拟反馈，验证路径和
到位超差当周期禁止入队、失败后新目标不能恢复输出、正常成功和拒绝非法目标不误锁。
模拟慢查询/慢发送过程中请求停止，验证先等待在途 RPC 返回、无后续批次，再调用
停止和退出；分别注入停止失败、退出失败，均不得报告停机获确认。
模拟十分钟 199.48 Hz 消费对 200 Hz 生产，队列不超过 40 点，指令顺序完整；
另覆盖限量生产下首次 50 点预装、167 ms 查询延迟及等待期间陈旧反馈锁停。

共 47 项 ROS 回归（流控/JTC 27、反馈 6、预生成 14）与 8 项 Python 分析测试通过，
正常工作区编译通过。所有运动 SDK 调用均为模拟，未连接或移动机械臂。
现场复测仍由用户在 RViz 选择短路径，固定 0.5/0.4，开启 record_motion，正常退出后
核对队列是否有界、等待次数、MAC 最低水位、执行状态和现场体感。暂不继续测试 0.8。
