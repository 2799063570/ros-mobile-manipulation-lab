AUBO Robot Description

A description package for the aubo_i3/aubo_i5/aubo_i7/aubo_e3/aubo_e5 robot arms from AUBO Robots. The 3D model files should be simplified to improve the computation efficiency in robot planning and collision detection.

We will add the aubo_i10 package soon.

## AUBO i5 hand camera

`urdf/aubo_i5_with_camera.xacro` adds an eye-in-hand RGB-D camera beside the
gripper with RealSense-compatible topic and frame names. Its main TF chain is:

```text
gripper_base_link -> hand_camera_mount_link -> hand_camera_link -> camera_link
                  -> camera_color_frame -> camera_color_optical_frame
                  -> camera_depth_frame -> camera_depth_optical_frame
```

Display the model with:

```bash
roslaunch aubo_description arm_with_camera_display.launch
```

In Gazebo the camera publishes:

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/depth/color/points
```

For a physical camera, the RealSense driver publishes the actual streams. The
robot URDF owns the calibrated eye-in-hand TF, so launch the driver with
`publish_tf:=false` and use `camera_color_optical_frame` for aligned RGB-D data.




