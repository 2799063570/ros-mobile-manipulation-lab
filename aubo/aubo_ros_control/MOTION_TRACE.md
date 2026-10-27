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
