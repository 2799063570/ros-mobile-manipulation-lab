# AUBO 固定机械臂颜色 / YOLO 抓取分拣

正式包名为 `aubo_sorting`，同时支持颜色和 YOLO。旧 `aubo_color_sorting` 包已移除，
自定义 launch 和脚本请改用 `aubo_sorting`。

```bash
# 两种入口共用 sorting.launch 和 aubo_sorting_core
roslaunch aubo_sorting color_sorting.launch
roslaunch aubo_sorting yolo_sorting.launch model_path:=/实际路径/obb.pt task_config:=/实际路径/task.yaml
```

也可继续使用 `sorting.launch detector:=color/yolo`。`config/sorting.yaml` 是共用的机械臂、
桌面和夹爪任务配置，内含红绿蓝方块示例；YOLO 应通过 `task_config` 指定实际
`sort_colors`（该旧参数名表示通用类别列表）和 `place_targets`，不要将示例颜色当成模型类别。
`colors_config` 指定 HSV 阈值，`yolo_config` 指定模型推理配置，`perception_config` 指定共用 3D 规则。
仿真入口同样支持这些配置以及 `model_path/python`；切换 detector 不会自动改变仿真物体或训练模型。


## 通用颜色 / YOLO 抓取入口

感知规则与消息定义见 [aubo_perception](../aubo_perception/README.md)。
`sorting.launch` 现在通过共用几何层接收颜色旋转框或 YOLO OBB，任务默认为 C++，
备用 `task_executable:=color_sorting_task.py` 使用相同参数。

```bash
# 机械臂控制器、MoveIt、相机和 TF 已运行；启动后仍等待 /sorting/start。
roslaunch aubo_sorting sorting.launch detector:=color height_mode:=table

# 深度高度 + 眼在手外 + YOLO；自定义配置须设置类别及放置位置。
roslaunch aubo_sorting sorting.launch detector:=yolo height_mode:=depth \
  camera_mount:=eye_to_hand model_path:=/absolute/path/obb.pt \
  task_config:=/absolute/path/task.yaml

# 仿真可以在原入口选择高度模式
roslaunch aubo_sorting sorting_gazebo.launch height_mode:=depth
```

`table_z` 与 `object_height` 由 launch 同时传给感知和任务；`perception_config` 提供
工作区、类别物高与深度误差容限，`task_config` 提供 `sort_colors/place_targets`、
任务侧 `height_tolerance` 和夹爪配置。实机请设置 `use_grasp_attachment:=false`。
眼在手外的真实相机话题与默认值不同可通过 `camera_namespace:=/实际相机` 覆盖。

默认启用检测角度、保持夹爪向下。普通水平框模型须显式设置
`use_detected_angle:=false` 使用固定角度；不能将水平框假装成可靠 OBB。
自动宽度默认关闭；实测 `gripper_width_open/gripper_width_closed` 与
`gripper_open/gripper_closed` 对应关系后，使用 `use_detected_width:=true`。
`width_close_scale` 默认 0.90。当前宽度换算只用于 trajectory 夹爪后端。

原颜色节点的 `use_depth` 不再控制新入口的抓取 Z；统一使用 `height_mode`。
旧场景配置里 `projection_plane_z/object_center_z` 等仅属于旧节点的参数也不再决定新入口几何。


`aubo_sorting` 是不带移动底盘的 AUBO i5 颜色 / YOLO 分拣场景包。它保存固定平台的
参数、world 和启动入口，并组合 `aubo_perception`、`aubo_sorting_core` 与
`aubo_gazebo_plugins`。腕部 RGB-D 相机利用对齐深度图计算方块顶面的三维中心，
MoveIt 负责抓取和放置，Gazebo 通用插件提高小物体夹持稳定性。

## 场景约束

- 机械臂底座固定在世界原点，不加载底盘、里程计或导航节点。
- 桌面高度为 `0.10 m`，桌体中心为 `[0.70, 0.0, 0.0]`，尺寸为
  `0.70 x 1.00 x 0.20 m`。
- 红、绿、蓝放置区域由 `sorting_gazebo.launch` 单独生成，不写入公共 world；因此
  视觉伺服复用同一场景时只会看到真实物块。分拣仿真可用
  `show_sorting_pads:=false` 隐藏放置区域。
- `upperArm_joint` 的 URDF 与 MoveIt 位置范围均为
  `[-1.0471976, 1.0471976] rad`，即 `[-60 deg, 60 deg]`。
- 启动时任务节点还会读取 `/robot_description` 和 MoveIt 关节限制做一次一致性检查；
  如果仍加载旧模型，任务进入 `ERROR`，不会执行运动。

通用检测器、消息、分拣状态机和插件不在本包内修改；本包只维护固定平台差异。

## 构建与启动

在工作空间根目录构建并加载环境后运行：

```bash
catkin_make
source devel/setup.bash
roslaunch aubo_sorting sorting_gazebo.launch
```

启动完成后，机械臂先进入 `observe` 观察姿态。确认调试图像和目标位置正常，再开始：

```bash
rosservice call /sorting/start
```

也可以启动后自动执行：

```bash
roslaunch aubo_sorting sorting_gazebo.launch auto_start:=true
```

如需在没有彩色放置区域干扰的情况下调试分拣识别：

```bash
roslaunch aubo_sorting sorting_gazebo.launch show_sorting_pads:=false
```

无界面运行时可关闭 Gazebo、RViz 和图像窗口：

```bash
roslaunch aubo_sorting sorting_gazebo.launch \
  gui:=false rviz:=false debug_view:=false
```

## 主要接口

| 接口 | 用途 |
| --- | --- |
| `/camera/color/image_raw` | 腕部 RealSense 彩色图像 |
| `/camera/color/camera_info` | 彩色相机内参 |
| `/camera/aligned_depth_to_color/image_raw` | 与彩色图像对齐的深度图 |
| `/sorting/debug_image` | 带颜色框、定位来源和坐标的调试图像 |
| `/sorting/detections` | 目标颜色及 `base_link` 坐标 |
| `/sorting/state` | 状态机状态 |
| `/sorting/start` | 开始一轮红、绿、蓝分拣 |
| `/sorting/stop` | 停止当前任务 |
| `/sorting/move_to_observation` | 回到相机观察姿态 |
| `/sorting/open_gripper` | 打开夹爪 |
| `/sorting/home` | 回到 `down` 姿态 |

`/sorting/detections` 的消息类型统一为 `aubo_perception/DetectedObjectArray`，固定与
移动平台不再各自维护一套消息定义。

颜色阈值、工作区和投影高度在 `config/colors.yaml` 中修改；抓取高度、放置点、速度
和桌面碰撞体在 `config/sorting.yaml` 中修改。若调整桌面高度，必须同步修改这两个配置
以及 `worlds/sorting.world`，避免视觉投影面、MoveIt 碰撞体和 Gazebo 实体不一致。

## 顶面深度定位

彩色方块位于画面两侧时，相机会同时看到顶面和侧面。如果直接使用整个彩色轮廓的
二维质心，侧面会把中心点拉偏，形成“中央目标准确、两侧目标有误差”的现象。

检测器默认使用以下流程改善该问题：

1. 使用彩色图像确定每个方块的轮廓。
2. 读取与彩色图像对齐的深度值，将轮廓内像素转换到 `base_link` 三维坐标系。
3. 只保留高度接近 `projection_plane_z` 的顶面点，排除较低的侧面点。
4. 使用顶面点云稳健边界的中点作为抓取 X/Y 坐标。
5. 深度图缺失、过期或有效顶面点不足时，自动退回原来的像素射线与顶面求交方法。

调试图像中的标签会显示定位来源：

- `red-D`、`green-D`、`blue-D`：使用深度顶面定位（Depth）。
- `red-R`、`green-R`、`blue-R`：使用射线投影后备方法（Ray）。

正常运行时三个目标应显示 `-D`。如果持续显示 `-R`，应检查深度话题是否发布、深度
图尺寸是否与彩色图一致，以及图像时间戳是否相差过大。

相关参数位于 `config/colors.yaml`：

| 参数 | 默认值 | 用途 |
| --- | ---: | --- |
| `use_depth` | `true` | 启用顶面深度定位 |
| `max_depth_age` | `0.25` | 彩色图与最近深度图允许的最大时间差（秒） |
| `top_surface_tolerance` | `0.008` | 顶面高度筛选容差（米） |
| `min_top_surface_points` | `30` | 使用深度定位所需的最少顶面点数 |
| `top_surface_percentile` | `5.0` | 计算稳健边界时排除两端异常点的百分比 |
| `projection_plane_z` | `0.14` | 方块顶面在 `base_link` 下的高度（米） |

如果更换方块高度或桌面高度，需要首先更新 `projection_plane_z`；不要优先使用
`position_offset_x/y` 修正随画面位置增大的误差，因为固定偏移只能补偿所有目标
方向和大小一致的误差。

## 笛卡尔轨迹执行

抓取下降和抬升使用笛卡尔路径。任务节点会对 `compute_cartesian_path` 生成的轨迹再次
进行时间参数化，确保 `velocity_scaling` 和 `acceleration_scaling` 对这类轨迹同样
生效。日志中的 `Retimed Cartesian path ...` 会显示重新定时后的执行时长。

如果路径显示 `100%`，随后控制器报告 `GOAL_TOLERANCE_VIOLATED`，表示路径规划已经
成功，但仿真关节没有在规定时间内稳定到终点，并非视觉代码中的深度有效值检查失败。
仿真控制器的目标时间和关节容差位于 `aubo_gazebo/config/controllers.yaml`；修改后必须
完整重启 Gazebo，使控制器重新加载参数。

## 接入真实机械臂

真实机械臂由 `aubo_ros_control` 和真实相机驱动提供控制器、关节状态及相机话题后，
只启动算法节点：

```bash
roslaunch aubo_sorting sorting.launch use_grasp_attachment:=false
```

首次在真实设备运行时应保持 `auto_start:=false`，降低速度比例，并先检查相机外参、
桌面高度、抓取偏移和所有规划轨迹。真实相机必须发布已经对齐到彩色图的深度图；
如果话题名称不同，应修改 `launch/sorting.launch` 中的 `depth_topic`。Gazebo 的吸附
插件不能用于真实机械臂。
