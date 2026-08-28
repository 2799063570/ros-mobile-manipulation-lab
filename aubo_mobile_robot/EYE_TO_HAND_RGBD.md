# 移动 AUBO 眼在手外 RGB-D、OctoMap、YOLO 与视觉伺服

本方案保留原腕部 RGB 相机，并新增安装在车体立柱上的
`workspace_camera` RGB-D 相机。两类相机职责分离：

- `workspace_camera`：全局目标检测、三维定位、MoveIt OctoMap；
- `hand_camera`：近距离观察以及后续眼在手上精定位。

通用模型、感知和控制代码放在顶层 `aubo` 包中；移动平台包仅保存安装姿态、
话题、工作区和启动配置，避免复制算法代码。

## 效果演示

[▶ 预览分拣流程（MP4，约 3.9 MB）](../aubo/video_or_img/preview.mp4)

[下载高清版本（MP4，约 22.7 MB）](../aubo/video_or_img/sorting_process.mp4)

该视频用于展示目标识别、机械臂规划、抓取和分类放置的整体效果。它不能替代下面的
点云、TF、OctoMap和仅规划安全检查。

## 关键接口

| 接口 | 类型 | 用途 |
|---|---|---|
| `/workspace_camera/depth/color/points` | `sensor_msgs/PointCloud2` | 相机原始注册点云 |
| `/workspace_camera/points_for_moveit` | `sensor_msgs/PointCloud2` | 去除已建模桌面薄层后的避障点云 |
| `/sorting/detections` | `aubo_perception/DetectedObjectArray` | 与检测器无关的抓取目标 |
| `/visual_servo/target_pose` | `geometry_msgs/PoseStamped` | 与检测器无关的视觉伺服目标 |
| `/sorting/base_locked` | `std_msgs/Bool` | 机械臂运行期间锁住底盘速度输出 |

## 仿真验证

```bash
catkin_make
source devel/setup.bash
roslaunch aubo_mobile_bringup simulation.launch mode:=sorting auto_start:=false
```

检查以下数据后，才在 RViz 面板中开始任务：

```bash
rostopic hz /workspace_camera/depth/color/points
rostopic hz /workspace_camera/points_for_moveit
rostopic echo -n 1 /move_group/filtered_cloud
rostopic echo /sorting/base_locked
rosservice call /clear_octomap
```

RViz 的 MotionPlanning 显示中需要启用 `Scene Geometry`。点云过滤器只剔除
PlanningScene 已精确建模的桌面薄层和当前 4 cm 方块高度范围；高于该范围的障碍物
仍应进入 OctoMap。更换桌子或物体尺寸后，必须同步修改
`aubo_mobile_perception/config/workspace_cloud_filter.yaml`。

## 真机相机和仅规划验证

真实相机入口默认使用 RealSense 风格话题。当前配置关闭相机驱动自身 TF，由机器人
URDF发布完整相机坐标树：

```bash
roslaunch aubo_mobile_robot eye_to_hand_rgbd_real.launch serial_no:=<serial>
```

在机械臂驱动、`joint_states` 和 `robot_state_publisher` 已启动后，使用禁止轨迹执行的
MoveIt入口：

```bash
roslaunch aubo_mobile_moveit_config octomap_validation.launch \
  load_robot_description:=false
```

该入口会加载 RGB-D 点云过滤和 OctoMap，但 `move_group` 不允许执行轨迹。完成
相机内参、`mobile_base_link -> workspace_camera_link` 外参、点云方向、OctoMap和
RViz轨迹检查之前，不得切换到真机执行模式。

当前 Xacro 中的安装位置和俯角只是结构初值，不是标定结果。真机必须用实际测量或
手眼标定结果更新 `aubo_mobile_robot.xacro` 中 `workspace_camera` 的 `xyz/rpy`。

当前 Xacro 对 RealSense 深度和彩色光学坐标系之间的内部偏移采用近似值，适合模型
联调，不适合作为高精度抓取外参。真机推荐只在URDF中保存
`mobile_base_link -> workspace_camera_link` 标定结果，由 RealSense 驱动发布设备
内部深度/彩色TF；切换前必须移除重复坐标变换并在RViz中检查整棵TF树。

## YOLO 抓取适配

YOLO网络保持独立运行，只需发布标准的 `darknet_ros_msgs/BoundingBoxes`。适配器将
检测框与对齐深度结合，输出共用三维目标接口：

```bash
roslaunch aubo_mobile_perception yolo_rgbd_detector.launch
```

网络类别名到分拣标签的映射位于 `config/yolo_rgbd.yaml`。例如网络输出
`red_block`，分拣核心收到的稳定标签为 `red`。切换分拣前端时使用：

```bash
roslaunch aubo_mobile_sorting sorting.launch perception_mode:=yolo
```

适配器不启动或绑定某个 YOLO 网络，因此以后换成其他检测器时，只需继续发布
`/sorting/detections` 和 `/visual_servo/target_pose`，MoveIt和分拣状态机无需修改。

## 眼在手外视觉伺服

眼在手外控制器使用低频 MoveIt PBVS，小步逼近固定相机给出的三维目标。它默认
只规划、不执行：

```bash
roslaunch aubo_mobile_bringup eye_to_hand_yolo_servo.launch \
  plan_only:=true start_enabled:=false
rosservice call /visual_servo/set_enabled "data: true"
```

只有完成 RViz 检查后才能显式传入 `plan_only:=false`。该节点与现有
`eye_in_hand_visual_servo_node` 使用相同公共服务和目标接口，设计上只能二选一启动；
同时也不能运行分拣状态机，否则两个上层控制器会争用机械臂。

建议最终任务流程为：

```text
导航（机械臂 transport，底盘解锁）
→ 到站并停止
→ 清空并重建 OctoMap
→ 底盘锁定
→ 眼在手外 YOLO 粗定位 / MoveIt 规划
→ 可选腕部相机精定位
→ 抓取和放置
→ 机械臂回 transport
→ 底盘解锁并继续导航
```
