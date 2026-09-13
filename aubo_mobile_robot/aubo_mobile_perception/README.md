# AUBO 移动机器人视觉感知

## 移动颜色分拣 RGB-D 定位

移动机器人腕部 Gazebo 相机现在发布同光心、同分辨率的 RGB 与对齐深度：
`/hand_camera/image_raw`、`/hand_camera/camera_info` 和
`/hand_camera/aligned_depth_to_color/image_raw`。默认颜色配置及四工作台配置
开启 `use_depth: true`、`require_depth: true`。共享 `aubo_perception` 节点
反投影轮廓内的深度像素到 `base_link`，保留距预期顶面 4 mm 以内的点，
以 XY 稳健边界的中点估计中心。发布的 Z 仍为配置的方块中心高度。

图像时刻 TF 尚未到达时，按原时间戳重试最多 `tf_wait_timeout` 秒（默认 0.2，
使用墙钟计时，仿真暂停也会超时），不会使用最新 TF 代替历史 TF。
深度与 RGB 时间戳为零、时间差超过 0.06 s、顶面点少于 30 个或占有效深度点
不足 15%，以及等待后图像时刻 TF 仍不可用时跳过目标，
不会退回纯 RGB 质心定位。调试图像中的 `red-D/green-D/blue-D` 表示使用了深度。
修改相机类型后必须完全重启 Gazebo、重新生成机器人，并重启感知节点；
仅重启感知无法让已经加载的 RGB 相机产生深度。

```bash
rostopic hz /hand_camera/aligned_depth_to_color/image_raw
rosparam get /color_object_detector/require_depth
```

参数可通过 `rosparam get /color_object_detector/use_depth` 与
`rosparam get /color_object_detector/require_depth` 检查。保留抓取对中阈值 8 mm，
需要在实际仿真中对照 Gazebo 方块实时位姿验证误差，深度开启并不保证零误差。

下文纯 RGB 平面投影说明仅适用于显式设置 `use_depth: false`、
`require_depth: false` 的旧相机配置。

该功能包保存移动平台专用的视觉参数和启动入口。通用 OpenCV/YOLO RGB-D适配节点
及消息定义由 `aubo_perception` 提供，识别结果统一转换为机器人坐标系下的目标位置。

## 话题接口

腕部RGB颜色检测订阅：

- `/hand_camera/image_raw`：手部相机图像
- `/hand_camera/camera_info`：相机内参

眼在手外RGB-D适配订阅：

- `/darknet_ros/bounding_boxes`：YOLO检测框
- `/workspace_camera/aligned_depth_to_color/image_raw`：对齐深度图
- `/workspace_camera/color/camera_info`：彩色相机内参

发布：

- `/sorting/detections`：目标颜色、像素位置及三维位置，消息类型为
  `aubo_perception/DetectedObjectArray`
- `/sorting/debug_image`：带识别框和目标坐标的调试图像
- `/visual_servo/target_pose`：与检测器无关的眼在手外三维目标

## 定位原理

当前腕部相机的 Gazebo 颜色检测链路使用 RGB 与对齐深度。节点先用 HSV 和轮廓
生成候选，再将轮廓内的深度点变换到 `base_link`，只保留方块预期顶面附近的点
估计 X/Y；发布位姿的 Z 使用 `object_center_z` 方块几何中心高度。

显式关闭 `use_depth` 时才使用旧的 RGB 像素射线与固定顶面求交定位。

## 单独启动

请先启动机器人和手部相机，然后执行：

```bash
roslaunch aubo_mobile_perception color_detector.launch
rostopic echo /sorting/detections
rosrun image_view image_view image:=/sorting/debug_image
```

ROS Melodic 默认使用 Python 2，运行环境需要安装 `python-opencv`、`python-numpy`
和 `ros-melodic-cv-bridge`。修改 `aubo_perception` 的自定义消息后，必须重新执行
`catkin_make` 并重新加载 `devel/setup.bash`。

## 参数调整

识别参数位于 `config/colors.yaml`，主要包括：

- 红、绿、蓝三种颜色的 HSV 范围
- 最小和最大轮廓面积
- 最大轮廓长宽比，用于排除画面边缘的分类色块
- `table_z` 桌面在 `base_link` 下的高度
- `object_height` 当前抓取物体高度
- `projection_plane_z` 图像射线使用的方块顶面高度
- `object_center_z` 检测位姿发布的方块中心高度
- 允许抓取的 X、Y 工作范围
- `position_offset_x/y` 固定观察姿态下的平面定位标定偏移

连接真实腕部相机后，应重新标定 HSV 阈值、相机内参以及相机到夹爪的固定变换。
连接眼在手外RealSense后，应标定底盘到相机的外参，并检查深度/彩色内部TF、点云
方向和时间戳同步，再执行机械臂运动。

## 眼在手外 RGB-D 与 YOLO

`yolo_rgbd_detector.launch` 将 `darknet_ros_msgs/BoundingBoxes`、对齐深度和
相机内参转换成统一的 `/sorting/detections`，同时发布
`/visual_servo/target_pose`。网络类别映射和深度范围位于 `config/yolo_rgbd.yaml`。

检测网络与RGB-D定位适配器保持隔离。当前ROS Melodic入口优先兼容
`darknet_ros`；若后续使用Ultralytics YOLO11，应将Python 3推理环境作为独立后端，
继续输出稳定检测接口，而不要让分拣状态机直接依赖模型框架。

`workspace_cloud_filter.launch` 将原始点云变换到 `base_link`，剔除已经由
PlanningScene 建模的桌面薄层后发布 `/workspace_camera/points_for_moveit`。过滤范围
属于平台/工位参数，不能写死到通用感知节点中。
