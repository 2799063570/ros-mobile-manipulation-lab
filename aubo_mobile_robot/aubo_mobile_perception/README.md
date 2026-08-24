# AUBO 移动机器人视觉感知

该功能包使用 OpenCV 对手部 RGB 相机图像进行颜色分割，并将识别结果转换为
机器人坐标系下的桌面目标位置。

## 话题接口

订阅：

- `/hand_camera/image_raw`：手部相机图像
- `/hand_camera/camera_info`：相机内参

发布：

- `/sorting/detections`：目标颜色、像素位置及三维位置，消息类型为
  `aubo_mobile_perception/DetectedObjectArray`
- `/sorting/debug_image`：带识别框和目标坐标的调试图像

## 定位原理

当前 Gazebo 手部相机只有 RGB 图像，没有深度信息。节点首先使用 HSV 阈值和
轮廓面积识别目标，然后根据相机内参生成像素射线，再将射线变换到 `base_link`
坐标系，与已知的桌面平面求交，得到抓取坐标。

该方法适用于目标都放在经过标定的平整桌面上。如果目标高度未知，建议将手部
传感器更换为深度相机，并直接使用深度图或点云定位。

## 单独启动

请先启动机器人和手部相机，然后执行：

```bash
roslaunch aubo_mobile_perception color_detector.launch
rostopic echo /sorting/detections
rosrun image_view image_view image:=/sorting/debug_image
```

ROS Melodic 默认使用 Python 2，运行环境需要安装 `python-opencv`、`python-numpy`
和 `ros-melodic-cv-bridge`。添加或修改本功能包的自定义消息后，必须重新执行
`catkin_make` 并重新加载 `devel/setup.bash`。

## 参数调整

识别参数位于 `config/colors.yaml`，主要包括：

- 红、绿、蓝三种颜色的 HSV 范围
- 最小和最大轮廓面积
- 最大轮廓长宽比，用于排除画面边缘的分类色块
- `table_z` 桌面在 `base_link` 下的高度
- 允许抓取的 X、Y 工作范围
- `position_offset_x/y` 固定观察姿态下的平面定位标定偏移

连接真实相机后，应重新标定 HSV 阈值、相机内参以及相机到夹爪的固定变换，再
执行机械臂运动。
