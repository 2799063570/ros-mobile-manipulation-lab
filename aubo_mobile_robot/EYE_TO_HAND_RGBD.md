# 移动抓取中的眼在手外 RGB-D

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
