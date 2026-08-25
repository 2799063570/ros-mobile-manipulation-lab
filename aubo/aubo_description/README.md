AUBO Robot Description

A description package for the aubo_i3/aubo_i5/aubo_i7/aubo_e3/aubo_e5 robot arms from AUBO Robots. The 3D model files should be simplified to improve the computation efficiency in robot planning and collision detection.

We will add the aubo_i10 package soon.

## AUBO i5 hand camera

`urdf/aubo_i5_with_camera.xacro` adds an eye-in-hand RGB camera beside the
gripper, based on the mobile manipulator model. Its TF chain is:

```text
gripper_base_link -> hand_camera_mount_link -> hand_camera_link
                  -> hand_camera_optical_frame
```

Display the model with:

```bash
roslaunch aubo_description arm_with_camera_display.launch
```

In Gazebo the camera publishes:

```text
/hand_camera/image_raw
/hand_camera/camera_info
```

For a physical camera, this URDF provides the mounting and optical TF only. The
camera vendor driver must publish the actual image and camera-info topics, and
its frame must be configured as `hand_camera_optical_frame`.




