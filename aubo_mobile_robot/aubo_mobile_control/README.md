# AUBO mobile control

This ROS 1 package provides manual base control and a simple sequential
navigation/manipulation coordinator for the AUBO mobile robot.

## Keyboard control

Start the robot or mapping stack first, then run this in an interactive terminal:

```bash
rosrun aubo_mobile_control keyboard_teleop.py
```

`keyboard_teleop.launch` is also provided for terminals that pass an interactive
TTY through roslaunch. Prefer `rosrun` if roslaunch reports that no TTY exists.

Keys:

- `w/s`: forward/backward
- `a/d`: rotate left/right
- `q/e`: forward arcs
- `z/c`: backward arcs
- `space` or `x`: stop
- `+/-`: adjust speed
- `Ctrl-C`: quit and publish a final zero velocity

The command watchdog stops the base if keyboard input ceases. Default speeds are
conservative for a robot carrying an arm.

## Navigation followed by arm planning

`nav_arm_coordinator.py` executes one task:

```text
MoveIt named target "down"
        -> send map-frame goal to /move_base
        -> after navigation succeeds, plan/execute MoveIt target "up"
```

With navigation, MoveIt and robot controllers already running:

```bash
roslaunch aubo_mobile_control nav_arm_coordinator.launch \
  goal_x:=1.0 goal_y:=0.5 goal_yaw:=1.57 \
  pre_navigation_target:=down post_navigation_target:=up
```

To start map-based navigation and MoveIt as well (robot/Gazebo already running):

```bash
roslaunch aubo_mobile_control navigation_arm.launch \
  map_file:=/absolute/path/to/map.yaml \
  goal_x:=1.0 goal_y:=0.5 goal_yaw:=1.57
```

For an all-in-one Gazebo test using the training world:

```bash
roslaunch aubo_mobile_control navigation_arm_gazebo.launch \
  map_file:=$(rospack find aubo_mobile_navigation)/maps/map.yaml \
  goal_x:=1.0 goal_y:=0.5 goal_yaw:=1.57
```

The map must already exist and match the selected Gazebo world. Initialize AMCL
in RViz if its starting pose is not known.

The post-navigation arm target can also be a Cartesian TCP pose by running the
node directly or adding parameters to a custom launch:

```text
arm_target_type: pose
arm_pose_frame: base_link
arm_x, arm_y, arm_z
arm_roll, arm_pitch, arm_yaw
```

This is sequential task coordination, not simultaneous whole-body planning.
The differential base remains controlled by `move_base`; MoveIt controls only
the `aubo_i5` arm group and uses `tcp_link` as the end effector.
