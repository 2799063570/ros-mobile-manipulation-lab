# aubo_sdk

This package vendors the headers, controller libraries and runtime config from
[`aubo_perception_planning/aubo_sdk`](https://github.com/2799063570/aubo_perception_planning/tree/main/aubo_sdk),
snapshot commit `e7d7558cb907f4a6cef632e302d612b7157172ec`. The vendored assets are unchanged;
the catkin metadata and demos are adapted for this workspace.

## Tests

The connection test is read-only: it logs in and prints controller diagnostics,
joint positions and TCP pose without starting or moving the arm.

```bash
rosrun aubo_sdk sdk_test 192.168.1.2
```

The motion demo is intentionally guarded and requires an explicit flag plus all six
joint targets in radians:

```bash
rosrun aubo_sdk joint_move_demo --execute 192.168.1.2 q1 q2 q3 q4 q5 q6
```

Run motion only after checking the workcell, teach pendant mode and E-stop.
