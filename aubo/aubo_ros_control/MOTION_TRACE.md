# Recording a real-arm trajectory

`aubo_real_bringup.launch` can optionally record the MoveIt plan, both action
results, controller desired/actual/error state, joint feedback, hardware MAC
flow diagnostics, TF, and ROS logs to a compressed rosbag. The hardware node
publishes a 10 Hz summary of MAC occupancy, queue depth, diagnosis/send RPC
latency, feed-loop timing, controller-loop period, and detected queue/SDK
errors on `/aubo_i5/aubo_hw_node/flow_diagnostics`.

Enable it when starting the real-arm bringup:

```bash
roslaunch aubo_ros_control aubo_real_bringup.launch \
  record_motion:=true motion_bag_prefix:=/tmp/aubo_motion
```

After the run, stop roslaunch normally. The bag name has a timestamp, for
example `/tmp/aubo_motion_2026-10-26-20-30-00.bag`. Inspect its topics with:

```bash
rosbag info /tmp/aubo_motion_2026-10-26-20-30-00.bag
/usr/bin/python3 $(rospack find aubo_ros_control)/scripts/analyze_moveit_motion.py \
  /tmp/aubo_motion_2026-10-26-20-30-00.bag \
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
`/tmp/aubo_motion_hardware_<capture>` prefix. No SDK thread writes files during
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
  /tmp/aubo_motion_hardware_<capture> --slow-ms 50 \
  --json /tmp/aubo_hardware_report.json
```

For a slow RPC, payload received well before its return suggests investigating
SDK processing/wakeup or host scheduling. Payload received near return keeps
cabinet processing and inbound-network delay as candidates. A client TCP
retransmission is recorded separately. TCP_INFO identifies socket payload timing,
not application RPC payloads: asynchronous data can affect these observations,
and zero outbound retransmissions does not exclude inbound loss. These records
improve attribution; they do not prove the 200 ms stalls have been eliminated.
