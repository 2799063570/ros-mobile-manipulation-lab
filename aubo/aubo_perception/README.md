# AUBO 通用视觉感知

该包提供固定机械臂和移动机械臂共用的目标消息、RGB/RGB-D检测节点和点云过滤。
相机话题、工作空间、投影平面和是否使用对齐深度均由调用方参数配置；平台场景
参数继续分别保存在 `aubo_color_sorting` 和 `aubo_mobile_perception` 中。

`color_object_detector.py` 优先使用对齐深度提取物体顶面；未启用深度或深度数据
不可用时，自动退回相机射线与已知平面的交点计算。

## 提供内容

- `DetectedObject.msg`：颜色名称、目标位姿、轮廓面积和像素中心；
- `DetectedObjectArray.msg`：同一图像帧中的目标数组；
- `color_object_detector.py`：HSV 分割、轮廓过滤、TF 坐标转换和调试图发布。
- `rgbd_visual_target_node.py`：两种相机安装共用的同步 RGB-D 目标位姿前端。
- `yolo_rgbd_target_node.py`：YOLO 检测框与深度图到统一三维目标的适配器。
- `workspace_cloud_filter_node`：MoveIt OctoMap 使用的工作区点云过滤。
- `ultralytics_yolo_node.py`：在独立 Conda Python 中执行 YOLO/OBB 推理，且不加载
  与 Conda ABI 冲突的 Noetic `cv_bridge`。

## Ultralytics YOLO

默认订阅 `/camera/color/image_raw`，发布：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/yolo/detections` | `aubo_perception/YoloDetectionArray` | 类别、置信度、中心、尺寸和 OBB 弧度角 |
| `/yolo/annotated_image` | `sensor_msgs/Image` | 带检测结果的 BGR 图像 |

环境定义保存在 `environment/yolo_ros.yml`。已有的 `yolo` 环境也可以直接使用。
节点通过 NumPy 解码 ROS 图像，不调用 `cv_bridge`，因此 ROS Noetic 保持系统
Python 3.8，YOLO 可使用 Conda Python 3.10。

需要从零重建环境时执行：

```bash
conda env create -f src/aubo/aubo_perception/environment/yolo_ros.yml
```

```bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
catkin_make
source devel/setup.bash
roslaunch aubo_perception ultralytics_yolo.launch
```

若使用新建的环境，则启动命令增加
`python:=/home/zlab/anaconda3/envs/yolo_ros/bin/python`。

模型、话题、阈值和设备在 `config/ultralytics_yolo.yaml` 中配置。当前机器的
NVIDIA 驱动不可用，所以默认 `device: cpu`；驱动恢复且
`torch.cuda.is_available()` 返回 `True` 后再改为 `device: "0"`。

默认发布：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/sorting/detections` | `aubo_perception/DetectedObjectArray` | 抓取目标坐标 |
| `/sorting/debug_image` | `sensor_msgs/Image` | 带检测框的调试图像 |

## 参数归属

通用节点不保存平台默认场景。调用方需要加载自己的 YAML：

- 固定 RGB-D 相机：`aubo_color_sorting/config/colors.yaml`；
- 移动平台 RGB 相机：`aubo_mobile_perception/config/colors.yaml`。

移动平台配置设置 `use_depth: false`，采用射线和平面求交；固定平台启用对齐深度，
深度不可用时仍会自动回退。通常应从对应场景 launch 启动，而不是直接运行脚本。

## 消息迁移

重构前的 `aubo_color_sorting/DetectedObjectArray` 和
`aubo_mobile_perception/DetectedObjectArray` 已停止维护。外部 Python 节点应改为：

```python
from aubo_perception.msg import DetectedObject, DetectedObjectArray
```
