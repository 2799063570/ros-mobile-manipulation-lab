# Launch 功能说明

本目录提供独立感知仿真和已有相机环境下的感知入口。颜色与 YOLO 检测共用三维几何计算，输出供分拣或视觉伺服使用的目标信息。

## 各文件实现的功能

| 文件 | 功能 | 使用场景 |
| --- | --- | --- |
| `color_eye_in_hand_gazebo.launch` | 启动带末端 RGB-D 相机的机械臂、颜色方块场景、MoveIt 与 HSV 检测，默认移动到观察位 | 仿真验证眼在手上的颜色识别 |
| `color_eye_to_hand_gazebo.launch` | 启动颜色方块场景、固定 RGB-D 相机与 HSV 检测，不加载机械臂 | 仿真验证固定相机的颜色识别 |
| `yolo_eye_in_hand_gazebo.launch` | 启动带末端相机的机械臂、YOLO 场景、MoveIt 与模型检测，默认移动到观察位 | 仿真验证眼在手上的 YOLO 检测，需匹配的权重与 Python 环境 |
| `yolo_eye_to_hand_gazebo.launch` | 启动 YOLO 场景、固定 RGB-D 相机与模型检测，不加载机械臂 | 仿真验证固定相机的 YOLO 检测 |
| `perception_gazebo.launch` | 统一选择检测方法、相机安装方式，加载仿真环境并调用感知流水线 | 上述四个快捷入口共用的实现，也可直接启动 |
| `grasp_perception.launch` | 将颜色或 YOLO 二维框结合内参、深度和 TF，生成分拣/伺服目标 | 已有图像、深度、内参和 TF 的仿真或真实机器人环境 |
| `ultralytics_yolo.launch` | 对本地图片或 ROS 图像话题进行 YOLO 二维检测，输出检测框及可选标注图 | 单独调试模型，不进行三维定位 |

## 调用关系与数据流

四个仿真快捷入口调用 `perception_gazebo.launch`，再由它调用 `grasp_perception.launch`。
后者根据 `detector` 选择 HSV 检测节点或 `ultralytics_yolo.launch`，两种检测结果都交给 `grasp_geometry` 节点处理。

```text
RGB 图像 → HSV / YOLO → /perception/boxes
                              ↓ 结合相机内参、对齐深度及 TF
                       grasp_geometry
                        ├─ /sorting/detections
                        └─ /visual_servo/target_pose、target_label、target_confidence
```

这些入口提供感知结果，不启动自动分拣任务或视觉伺服控制器。眼在手上的仿真入口默认会通过 MoveIt 将机械臂移动到 `observe` 观察位，可用 `auto_move_to_observation:=false` 关闭。

## 常用参数

| 参数 | 含义 |
| --- | --- |
| `detector` | `color` 使用 HSV 颜色检测；`yolo` 使用模型检测。四个快捷入口已固定此值 |
| `camera_mount` | `eye_in_hand` 为末端相机；`eye_to_hand` 为固定相机。四个快捷入口已固定此值 |
| `task_mode` | 默认 `sorting` 只发布分拣结果；`servo` 只发布伺服结果；`both` 同时发布 |
| `selected_class` | `servo` 和 `both` 必填；颜色可用 `red`、`green`、`blue`，YOLO 使用模型类别名或配置后的别名 |
| `height_mode` | `depth` 使用深度估计高度；`table` 使用桌高与已知物高投影。仿真入口默认 `depth`，纯感知入口默认 `table`；伺服输出始终需要有效深度 |
| `table_z` / `object_height` | 桌面高度与物体高度，单位米，应与场景及目标坐标系一致 |
| `target_frame` | 分拣输出坐标系；仿真眼在手上默认 `base_link`，眼在手外默认 `world`。纯感知入口默认 `base_link` |
| `gui` / `rviz` / `paused` | 仿真窗口、RViz 开关与初始暂停状态；仅仿真入口提供 |
| `camera_x/y/z`、`camera_roll/pitch/yaw` | 仿真固定相机位姿，位置单位米，角度单位弧度；仅眼在手外分支使用 |
| `python` / `model_path` | YOLO 的解释器与模型权重路径，需匹配本机环境 |

眼在手上默认订阅 `/camera/color/image_raw`、`/camera/aligned_depth_to_color/image_raw`、`/camera/color/camera_info`；眼在手外的前缀为 `/workspace_camera`。纯感知入口可通过 `image_topic`、`depth_topic`、`camera_info_topic` 覆盖。

## 启动示例

默认颜色感知，查看分拣候选目标：

```bash
roslaunch aubo_perception color_eye_in_hand_gazebo.launch
# 在另一个终端查看
rostopic echo /sorting/detections
```

同时输出红色目标的伺服信息：

```bash
roslaunch aubo_perception color_eye_in_hand_gazebo.launch task_mode:=both selected_class:=red
# 在其他终端查看
rostopic echo /visual_servo/target_pose
rostopic echo /visual_servo/target_label
```

伺服位姿使用相机光学坐标系。只有指定类别具有有效深度时才发布位姿；无有效目标时标签为空，不重发旧位姿。默认 `sorting` 模式不会向这些伺服话题发送消息。

单独调试 YOLO 实时图像检测：

```bash
roslaunch aubo_perception ultralytics_yolo.launch input_mode:=topic \
  image_topic:=/camera/color/image_raw \
  python:=/path/to/python model_path:=/path/to/model.pt
```

单独启动 `ultralytics_yolo.launch` 时默认是 `input_mode:=image`，可用 `image_path:=/path/to/test.jpg` 指定图片。接入共用几何流水线时会强制使用实时话题，以匹配采集时刻的深度与 TF。
