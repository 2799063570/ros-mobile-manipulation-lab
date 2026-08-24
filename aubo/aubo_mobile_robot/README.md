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
