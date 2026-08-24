# AUBO mobile navigation

ROS 1 mapping, localization and navigation for the circular AUBO mobile
manipulator. The package is designed for the frames and topics published by
`aubo_mobile_robot`.

## Data flow

```text
/front/scan ─┐
             ├─ dual_laser_merger ─ /scan ─ GMapping or AMCL + move_base
/rear/scan  ─┘

odom ───────── differential-drive plugin
map -> odom ── GMapping while mapping, AMCL while navigating a saved map
```

The merger transforms both scans into `base_footprint` before producing a
720-sample, 360-degree `/scan`. It accounts for the front/rear sensor offsets
and orientations rather than simply concatenating the two range arrays.

## Build

From the catkin workspace root:

```bash
catkin_make
source devel/setup.bash
```

Keep the arm in a folded/home pose before driving. The configured `0.40 m`
navigation radius covers the folded robot, not a fully extended arm.

Do not run the fake-controller `aubo_mobile_moveit_config/demo.launch` together
with navigation: that demo publishes a static `odom -> base_footprint` transform,
which conflicts with differential-drive odometry. For combined arm and base
simulation, use the Gazebo/real-controller MoveIt launch path instead.

## Mapping in Gazebo

Start the robot, scan merger, GMapping and RViz together:

```bash
roslaunch aubo_mobile_navigation mapping_gazebo.launch
```

Pass another world when required:

```bash
roslaunch aubo_mobile_navigation mapping_gazebo.launch \
  world:=/absolute/path/to/site.world
```

Drive the base by publishing `/cmd_vel`. If `teleop_twist_keyboard` is
installed, one option is:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
  _speed:=0.20 _turn:=0.60
```

When the map is complete, save it:

```bash
roslaunch aubo_mobile_navigation map_saver.launch
```

This creates `maps/map.yaml` and `maps/map.pgm`. An absolute writable path can
also be supplied:

```bash
roslaunch aubo_mobile_navigation map_saver.launch \
  map_name:=/absolute/writable/path/site_map
```

## Navigation with a saved map

Start Gazebo, map server, AMCL, move_base, scan merger and RViz:

```bash
roslaunch aubo_mobile_navigation navigation_gazebo.launch \
  map_file:=$(rospack find aubo_mobile_navigation)/maps/map.yaml
```

In RViz:

1. Use **2D Pose Estimate** to initialize AMCL.
2. Use **2D Nav Goal** to send a goal.

For a real robot or when Gazebo is already running, omit the simulator:

```bash
roslaunch aubo_mobile_navigation navigation.launch \
  map_file:=/absolute/path/to/map.yaml
```

To navigate while building a live map, first start the robot and then run:

```bash
roslaunch aubo_mobile_navigation mapping_navigation.launch
```

To start Gazebo, GMapping and move_base together—matching the structure of the
reference `simple_diff_robot_gazebo/launch/mapping_nav.launch`—run:

```bash
roslaunch aubo_mobile_navigation mapping_nav.launch
```

Run `rosrun aubo_mobile_control keyboard_teleop.py` from another interactive
terminal to explore the environment.

## Checks

```bash
rostopic hz /front/scan
rostopic hz /rear/scan
rostopic hz /scan
rosrun tf tf_echo odom base_footprint
rostopic echo -n 1 /map
rostopic echo -n 1 /move_base/status
```

Expected TF ownership:

- the differential-drive plugin publishes `odom -> base_footprint`;
- GMapping publishes `map -> odom` during mapping;
- AMCL publishes `map -> odom` during saved-map navigation.

Never run GMapping and AMCL together because both would try to own the same
`map -> odom` transform.
