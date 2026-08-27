# AUBO 通用视觉感知

该包提供固定机械臂和移动机械臂共用的颜色目标消息及手部相机检测节点。
相机话题、工作空间、投影平面和是否使用对齐深度均由调用方参数配置；平台场景
参数继续分别保存在 `aubo_color_sorting` 和 `aubo_mobile_perception` 中。

`color_object_detector.py` 优先使用对齐深度提取物体顶面；未启用深度或深度数据
不可用时，自动退回相机射线与已知平面的交点计算。

## 提供内容

- `DetectedObject.msg`：颜色名称、目标位姿、轮廓面积和像素中心；
- `DetectedObjectArray.msg`：同一图像帧中的目标数组；
- `color_object_detector.py`：HSV 分割、轮廓过滤、TF 坐标转换和调试图发布。

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
