# AUBO 感知到分拣

本次入口先面向固定机械臂。颜色识别和 YOLO 共用几何层，移动机器人场景暂不迁移。

```text
ROS RGB ── HSV 旋转框 / Ultralytics YOLO、OBB ── YoloDetectionArray
                                                     │
                     RGB 内参 + 对齐深度缓存 + TF ──────┤
                                                     ▼
                                   yolo_rgbd_target_node.py
                                      │              │
                     DetectedObjectArray       PoseStamped + 类别 + 置信度
                                      │              │
                          aubo_sorting_core       视觉伺服控制器
```

## 为什么原来启动 sh，为什么仍保留两个进程

原 `.sh` 只是选择 Conda Python，再执行 `.py`，不负责检测。
`ultralytics_yolo.launch` 现在直接启动 `ultralytics_yolo_node.py`，通过
`launch-prefix="$(arg python)"` 选择解释器。旧 shell 保留供原有外部入口兼容。

两步不是算法上的硬性要求，但当前环境值得保留：YOLO 使用 Conda/Torch/OpenCV，
几何节点使用 ROS 系统 Python/cv_bridge/TF。强行放入一个进程可能引入 OpenCV ABI 冲突。
两步不会阻止保存深度：几何节点有有界帧缓存，按检测结果携带的 **RGB 采集时间戳**
找最近的深度，而非按推理完成时间配对。`maximum_depth_age` 限制配对误差；
`depth_cache_size` 要覆盖最长推理延迟。默认只保存在内存；配置
`depth_save_directory` 后，把配对深度、原始单位、内参、两帧时间戳保存为 `.npz`。
相机驱动须提供已经对齐 RGB 的深度；此节点不负责把原始深度注册到 RGB。

## YOLO 输入模式

```bash
# 在线：订阅图像话题
roslaunch aubo_perception ultralytics_yolo.launch input_mode:=topic \
  image_topic:=/camera/color/image_raw

# 离线：读取当前运行主机能够访问的单张图像
roslaunch aubo_perception ultralytics_yolo.launch input_mode:=image \
  image_path:=/absolute/path/test.jpg
```

离线模式推理一次，锁存发布 `/yolo/detections` 和 `/yolo/annotated_image`，供后启动的
查看器读取。图片使用零时间戳，不允许与实时深度配对，也不会启动分拣任务。
Windows 的 `D:\...` 地址必须换成 ROS 运行主机实际可访问的路径。
`python:=...`、`model_path:=...` 可覆盖机器相关路径；项目路径、阈值和设备见
`config/ultralytics_yolo.yaml`，Conda 环境定义见 `environment/yolo_ros.yml`。

## 共用模式入口

```bash
# 颜色识别，眼在手上，按已知桌高求交
roslaunch aubo_perception grasp_perception.launch detector:=color height_mode:=table

# YOLO，眼在手外，用深度估计物体中心
roslaunch aubo_perception grasp_perception.launch detector:=yolo \
  camera_mount:=eye_to_hand height_mode:=depth

# 指定类别的视觉伺服目标；不启动机械臂控制器
roslaunch aubo_perception grasp_perception.launch detector:=yolo \
  task_mode:=servo selected_class:=bottle
```

| 参数 | 行为 |
| --- | --- |
| `detector=color/yolo` | 两个 2D 前端，使用同一套 3D 计算 |
| `task_mode=sorting` | 发布所有通过置信度、深度、工作区校验的目标；不按 selected_class 筛掉分拣类别 |
| `task_mode=servo` | 必须设置 selected_class；仅发布该类别最高置信度有效目标 |
| `task_mode=both` | 同时发布上述两种输出，仍须指定 selected_class |
| `height_mode=table` | 相机射线与“桌高 + 物高”的顶面求交，中心 Z=桌高+半物高；分拣不需要深度 |
| `height_mode=depth` | 对齐深度反投影、TF 转到基座，校验顶面 Z≈桌高+物高，再减半物高作为中心 Z |
| `camera_mount=eye_in_hand` | 默认 `/camera` 话题；需要腕部到相机的外参和采集时刻的机械臂 TF |
| `camera_mount=eye_to_hand` | 默认 `/workspace_camera` 话题；需要基座到固定相机的外参 |

`camera_mount` 不会自动标定或凭空发布 TF。两种模式都用采集时刻的 TF；眼在手上时
不会用“最新 TF”替代过期变换。可通过 `camera_namespace`，或独立的 `image_topic`、
`depth_topic`、`camera_info_topic` 覆盖话题。配置 `grasp_pipeline.yaml` 中的工作区、
`height_tolerance`、`class_heights`、`class_aliases`；场景参数 `table_z/object_height/target_frame`
通过 launch 参数传入。按类别物高与桌高必须在该基座坐标系中有意义。

伺服始终需要深度，输出相机光学坐标系中的可见表面点，不套用分拣桌高校验。
`/visual_servo/target_pose` 保留 RGB 时间戳，另发 `target_label`、`target_confidence`。
目标丢失时标签为空、置信度归零，不重复旧 Pose；控制器必须检查 Pose 时间戳超时。
这里的“跟踪”是逐帧选择指定类别的最高分目标，没有跨帧实例 ID；同类目标分数交替时可能切换。
HSV 的置信度是轮廓对旋转框的填充率，不等同于神经网络概率。

## 通用消息与抓取约定

`darknet_ros/BoundingBoxes` 只是可选的旧检测输入适配器，当前 Ultralytics 链路不依赖
`darknet_ros_msgs`。只有手动选择 `backend=darknet` 时才需要安装它。
`DetectedObjectArray` 用于隔离感知与分拣接口，不是 darknet 必须发布的格式，也不是时间同步机制。

| DetectedObject 字段 | 含义 |
| --- | --- |
| `class_name`、`color` | 相同类别键；color 保留兼容原分拣，可写 bottle 等任意类别 |
| `confidence` | 检测质量分数 |
| `pose.position` | 数组 header.frame_id 下的物体中心，单位 m |
| `grasp_width` | 顶面投影得到的米制窄边宽度 |
| `grasp_angle` | 基座 XY 平面内窄边方向，弧度，按平行夹爪 π 周期归一化 |
| `grasp_geometry_valid` | OBB/颜色旋转框可用于角度和宽度抓取；普通水平框为 false |
| `depth_valid` | 深度中心高度已通过桌高+物高校验；table 模式为 false |
| `object_height` | 此类别的已知物高，用于中心/顶面换算 |

图像角度必须先投影到基座再计算 yaw，不能直接当作 TCP 旋转。
当前假定物体顶面近似水平、夹爪竖直向下；宽度是旋转框投影的估计，斜面、遮挡或框不贴合时
不等于真实物宽。普通水平框不能恢复真实物体角度；需使用 OBB 模型，或明确关闭任务的
`use_detected_angle/use_detected_width` 使用固定夹持参数。

## 接入机械臂分拣

见 [分拣运行说明](../aubo_color_sorting/README.md)。默认 C++ 和备选 Python 任务均支持：

- `height_mode=table/depth`；深度模式在动作前再校验中心高度，失败即拒绝目标。
- `use_detected_angle=true`；保持 `grasp_rpy` 的竖直向下姿态，yaw 增加检测角度。
  `grasp_rpy` 的原始 yaw 用于补偿 TCP 与夹爪闭合轴的标定偏差；整个抓取、抬升、放置期间保持该角度。
- `use_detected_width=true`；闭合开口=窄边宽度×`width_close_scale`，再按标定开口端点
  `gripper_width_open/gripper_width_closed` 映射到 `gripper_open/gripper_closed` 关节位置。
  默认不开启宽度控制，端点 -1 表示未标定；超出标定范围会拒绝目标，不截断伪装成可抓取。
  此宽度控制目前仅支持 trajectory 后端；Inspire 原有开/合服务模式仍使用固定闭合，开启自动宽度会明确报错。

旧缓存仅保存 X/Y。启用检测角度、宽度或 depth 模式后不再用旧缓存兜底，避免丢掉 Z 和抓取参数。
当前分拣仍按 `sort_colors` 顺序每类取一个目标；`place_targets` 的键改为 YOLO 类别即可复用，
不等同于“同类任意数量物体”的持续清空任务。

## 编译、兼容与验证

本次扩展了 ROS 消息，MD5 会改变。必须重新编译所有使用这些消息的包并重启相关节点，
旧 rosbag 中同名旧消息不能直接假定兼容。

```bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
catkin_make
source devel/setup.bash
catkin_make run_tests_aubo_perception run_tests_aubo_sorting_core
catkin_test_results
```

测试覆盖离线图片输入、OBB 标记、延迟推理配对、深度缓存、顶面/中心校验、相机旋转后的
米制短边、模式输出隔离，以及分拣在发动作前拒绝非法高度/宽度/角度。不替代相机外参、
夹爪宽度标定和实机闭环测试。

`color_object_detector.py` 的 `output_mode=legacy` 仍保留原有颜色顶面修正和平面回退行为，
用于尚未迁移的入口；新的机械臂入口使用 `output_mode=boxes` + 共用几何层。
`rgbd_visual_target_node.py` 原有伺服入口和 `workspace_cloud_filter_node` 点云过滤保持独立可用。
