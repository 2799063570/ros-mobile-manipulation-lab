# 移动抓取中的眼在手外 RGB-D

## 标定结果进入抓取、位置伺服和 OctoMap

先用 `roslaunch aubo_mobile_control handeye_calibration.launch motion_mode:=teach`
或 `motion_mode:=moveit` 完成眼在手外标定，保存 easy_handeye YAML。
标定时不要启动旧外参发布器；采样期间底盘保持静止。保存文件中的
`parameters.robot_base_frame` 必须是 `base_link`，
`parameters.tracking_base_frame` 必须是
`workspace_camera_color_optical_frame`。相机重新安装后需重新标定。

真机分拣入口现在要求显式传入该文件，并启动点云过滤和 MoveIt OctoMap：

```bash
roslaunch aubo_mobile_nav_sorting navigation_sorting.launch \
  calibration_file:=/absolute/path/to/saved_handeye.yaml \
  camera_serial_no:=<serial>
```

只运行位置伺服时使用同一个文件：

```bash
roslaunch aubo_mobile_bringup mobile_manipulation_visual_servo.launch \
  calibration_file:=/absolute/path/to/saved_handeye.yaml \
  camera_serial_no:=<serial>
```

不要同时启动上述两个入口：分拣使用 ros_control 轨迹控制，位置伺服使用
SDK 控制，两者都会连接同一机械臂。分拣入口的 `use_octomap:=true`
和 `require_octomap:=true` 默认开启；若仅排查相机或感知，可显式传
`use_octomap:=false require_octomap:=false`，同时关闭任务自动启动。

运行前核对同一条坐标链：

```bash
rosrun tf tf_echo base_link workspace_camera_color_optical_frame
rostopic echo -n1 /workspace_camera/color/camera_info/header
rostopic echo -n1 /workspace_camera/depth/color/points/header
rostopic hz /workspace_camera/points_for_moveit
rostopic echo -n1 /sorting/detections/header
rostopic echo -n1 /visual_servo/target_pose/header
rostopic echo -n1 /move_group/monitored_planning_scene/world/octomap
```

相机内参和深度图用于抓取目标三维重建；标定 TF 将结果转换到
`base_link`，并供位置伺服变换目标位姿。点云过滤器也用该 TF 把点云
转换到 `base_link`，MoveIt 再从
`/workspace_camera/points_for_moveit` 更新 OctoMap。若点云的
`header.frame_id` 与标定的光学帧不同，必须先确认 RealSense 发布了
两帧间的 TF，不能用重命名点云 frame 的方式替代坐标变换。

移动机器人不再维护独立的相机、感知或视觉伺服实现。通用能力来自：

- `aubo_description`：外部相机 TF；
- `aubo_perception`：RGB-D/YOLO 三维目标与 OctoMap 点云过滤；
- `aubo_ros_control`：统一视觉伺服核心及 Gazebo/SDK 后端；
- `aubo_mobile_*`：导航、底盘与机械臂协调、移动抓取任务参数。

真机移动抓取组合入口：

```bash
roslaunch aubo_mobile_bringup mobile_manipulation_visual_servo.launch \
  robot_ip:=192.168.1.2 camera_serial_no:=<serial>
```

该入口假设移动底盘、机器人模型和状态发布已启动；它只组合外部相机、移动场景
YOLO 参数以及 `aubo` 的眼在手外控制核心。默认不自动运动。

启动移动机器人模型时应传入 `enable_hand_camera:=false`；该 Xacro 开关会同时移除
手部相机模型、传感器和话题，避免眼在手外任务产生重复数据流。

主要话题：

```text
/workspace_camera/color/image_raw
/workspace_camera/aligned_depth_to_color/image_raw
/workspace_camera/depth/color/points
/workspace_camera/points_for_moveit
/visual_servo/target_pose
/visual_servo/state
```

OctoMap 过滤算法位于 `aubo_perception/workspace_cloud_filter_node`，移动端只保留工作
空间边界等场景参数。固定工位相机相对移动机器人时，TF 的父坐标应是随底盘运动的
`base_link`；世界固定相机则应使用 `map`/`odom`，不能混用。

眼在手外任务默认不启动手部相机。若后续任务需要眼在手上精定位，应在任务状态机
明确切换相机和 `servo_mode`，且任一时刻只能有一个机械臂伺服控制器输出命令。
