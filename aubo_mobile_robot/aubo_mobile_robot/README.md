# AUBO mobile manipulator

This ROS 1 package combines the existing AUBO i5 model with a circular
differential-drive base for Gazebo Classic.

## Model

- 0.68 m diameter circular chassis, two powered wheels and two passive supports
- AUBO i5 arm and the existing two-finger gripper from `aubo_description`
- front lidar: `front_laser_link`, topic `/front/scan`, forward 180 degrees
- rear lidar: `rear_laser_link`, topic `/rear/scan`, rearward 180 degrees
- hand RGB camera: `hand_camera_optical_frame`, topics
  `/hand_camera/image_raw` and `/hand_camera/camera_info`
- visible collision-enabled mounting posts for both lidars and a side-mounted
  L bracket for the hand camera
- differential drive command: `/cmd_vel`
- wheel odometry: `/odom` and TF `odom -> base_footprint`
- arm trajectory action: `/aubo_i5_controller/follow_joint_trajectory`
- gripper trajectory action: `/gripper_controller/follow_joint_trajectory`

The two lidar scans are deliberately separate so that downstream navigation can
either use them independently or merge them with the scan-merger package chosen
for the target robot.

## Run in Gazebo

Build and source the catkin workspace, then run:

```bash
roslaunch aubo_mobile_robot gazebo.launch
```

Drive the base from another terminal:

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.15}, angular: {z: 0.0}}'
```

Stop it with `Ctrl-C`, then publish a zero command if necessary.

Send a simple arm pose:

```bash
rostopic pub -1 /aubo_i5_controller/command trajectory_msgs/JointTrajectory \
  "joint_names: [shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint]
points:
- positions: [0.0, -0.8, 1.0, 0.0, 0.8, 0.0]
  time_from_start: {secs: 4}"
```

Close both gripper fingers:

```bash
rostopic pub -1 /gripper_controller/command trajectory_msgs/JointTrajectory \
  "joint_names: [joint1, joint2]
points:
- positions: [0.35, 0.35]
  time_from_start: {secs: 2}"
```

To inspect only the kinematic model in RViz:

```bash
roslaunch aubo_mobile_robot display.launch
```

## Useful checks

```bash
rostopic echo /front/scan
rostopic echo /rear/scan
rostopic echo /hand_camera/camera_info
rostopic echo /odom
rosservice call /controller_manager/list_controllers
```

The original stand-alone AUBO launch files still work as before. The small
change in `aubo_description/urdf/arm.xacro` only makes its world joint and
ros_control namespace configurable; both retain their original defaults.

## MoveIt Setup Assistant

Create a separate `aubo_mobile_moveit_config` package instead of overwriting the
existing arm-only `aubo_moveit_config` package:

```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```

In the assistant, choose **Create New MoveIt Configuration Package**, browse to
`aubo_mobile_robot/urdf/aubo_mobile_robot.xacro`, and use these settings:

1. Generate the self-collision matrix with a high sampling density. Review
   arm-to-chassis pairs carefully; do not disable a pair that can collide.
2. Add a planar virtual joint named `world_joint`, with parent frame `odom` and
   child link `base_footprint`. This lets MoveIt track the mobile base pose but
   does not make MoveIt drive the differential base.
3. Create arm group `aubo_i5` as a kinematic chain from `base_link` to
   `tcp_link`, using the KDL solver. `tcp_link` is the grasp tool centre point;
   keep `gripper_link` only as the compatible intermediate frame.
4. Create gripper group `gripper` from joints `joint1` and `joint2`.
5. Add end effector `aubo_gripper`: group `gripper`, parent link
   `gripper_base_link`, parent group `aubo_i5`.
6. Mark `left_wheel_joint` and `right_wheel_joint` as passive joints. Navigation
   owns `/cmd_vel`; arm planning should not include either wheel joint.
7. Add useful poses such as arm `zero`/`home` and gripper `open`/`closed`.
8. Use `/aubo_i5_controller` for the six arm joints and
   `/gripper_controller` for `joint1` and `joint2`.
9. Generate the package beside the inner robot package, inside the common
   collection directory: `aubo_mobile_robot/aubo_mobile_moveit_config`.

The current hand camera is RGB-only, so leave the 3D perception section empty.
Configure an OctoMap sensor there only after changing the hand sensor to a depth
camera or adding a point-cloud source.
