# aubo_ros_control

ROS 1 hardware interface for a real AUBO i5. It adapts the referenced AUBO SDK
driver pattern from
[`aubo_ros_control`](https://github.com/2799063570/aubo_perception_planning/tree/main/aubo_ros_control)
to this workspace's existing joint names and MoveIt controller:
`/aubo_i5/aubo_i5_controller/follow_joint_trajectory`.

## Safety model

- Position commands are accepted only after all three SDK sessions are connected,
  the controller reports a real robot, startup and handshake succeed, and
  TCP2CANBUS mode is active.
- Initial hardware joints must be read successfully before controllers are loaded.
- Emergency stop, collision/protective stop, invalid numeric commands, send failure,
  or socket disconnect stops further command output. A disconnect requires inspection
  and a node restart; the driver deliberately does not resume motion automatically.
- The feed thread applies per-joint velocity step limits and fills the controller MAC
  buffer independently of the ROS controller loop.

## Build

```bash
cd ~/catkin_ws
catkin_make --pkg aubo_sdk aubo_ros_control
source devel/setup.bash
```

For ROS Melodic, install `ros-control`, `ros-controllers`,
`joint-state-controller`, and `position-controllers` if they are not present.

## Verify before motion

First check SDK connectivity without starting or moving the arm:

```bash
rosrun aubo_sdk sdk_test 192.168.1.2
```

Then start state-only mode. This does not call robot startup and does not enter
TCP2CANBUS mode:

```bash
roslaunch aubo_ros_control aubo_state.launch robot_ip:=192.168.1.2
rostopic echo /aubo_i5/joint_states
```

## Real robot control

Keep the teach pendant and E-stop available, clear the workcell, and verify RViz
joint directions against the physical robot before executing a trajectory.

Start only the hardware controller:

```bash
roslaunch aubo_ros_control aubo_control.launch robot_ip:=192.168.1.2
```

Or start hardware, MoveIt, robot state publisher and RViz together:

```bash
roslaunch aubo_ros_control aubo_real_bringup.launch robot_ip:=192.168.1.2
```

Important topics/actions:

- State: `/aubo_i5/joint_states`
- MoveIt/TF merged state: `/joint_states`
- Trajectory action: `/aubo_i5/aubo_i5_controller/follow_joint_trajectory`
- Controller manager: `/aubo_i5/controller_manager`

Parameters such as `server_port`, credentials, collision class, command smoothing,
and control frequency are exposed by `aubo_control.launch` rather than hard-coded.

The real-robot launch uses `aubo_i5_with_camera.xacro` by default. To load the
arm without the hand camera, pass:

```bash
roslaunch aubo_ros_control aubo_real_bringup.launch \
  robot_ip:=192.168.1.2 \
  robot_xacro:=aubo_i5.xacro robot_srdf:=aubo_i5.srdf
```

The eye-in-hand visual-servo launch files use the same RealSense-style RGB-D
topics in Gazebo and on hardware. `eye_in_hand_visual_servo_real.launch` can start the physical
RealSense driver with aligned depth; the robot URDF owns the calibrated mount TF
and the common optical frame is `camera_color_optical_frame`.

`eye_in_hand_visual_servo_gazebo.launch` relays `/aubo_i5/joint_states` to the global
`/joint_states` topic consumed by `robot_state_publisher`. If RViz reports that
all arm and camera links have no transform to `base_link`, verify that both
topics are publishing before debugging the camera TF itself.

`visual_servo_gazebo.launch` and `visual_servo_real.launch` are deprecated
compatibility wrappers for existing deployments.
