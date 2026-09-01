# AUBO 机器人描述

该功能包提供 AUBO Robots 的 `aubo_i3`、`aubo_i5`、`aubo_i7`、`aubo_e3` 和 `aubo_e5` 机械臂描述。三维模型经过适当简化，以提高运动规划和碰撞检测的计算效率。

## AUBO i5 手眼相机

`urdf/aubo_i5_with_camera.xacro` 在夹爪旁安装眼在手上的 RGB-D 相机，并使用与 RealSense 兼容的话题和坐标系名称。主要 TF 链如下：

```text
gripper_base_link -> hand_camera_mount_link -> hand_camera_link -> camera_link
                  -> camera_color_frame -> camera_color_optical_frame
                  -> camera_depth_frame -> camera_depth_optical_frame
```

显示模型：

```bash
roslaunch aubo_description arm_with_camera_display.launch
```

Gazebo 中的相机发布以下话题：

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/depth/color/points
```

使用实体相机时，由 RealSense 驱动发布实际数据流。机器人 URDF 负责提供标定后的眼在手 TF，因此应以 `publish_tf:=false` 启动驱动，并对齐 RGB-D 数据到 `camera_color_optical_frame`。
