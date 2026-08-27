# AUBO 固定机械臂颜色抓取分拣

`aubo_color_sorting` 是不带移动底盘的 AUBO i5 颜色分拣功能包。它使用腕部 RGB-D
相机识别红、绿、蓝方块，并利用对齐深度图计算方块顶面的三维中心，通过 MoveIt
规划抓取和放置轨迹，并在 Gazebo 中用世界插件提高小物体夹持的稳定性。

## 场景约束

- 机械臂底座固定在世界原点，不加载底盘、里程计或导航节点。
- 桌面高度为 `0.10 m`，桌体中心为 `[0.70, 0.0, 0.0]`，尺寸为
  `0.70 x 1.00 x 0.20 m`。
- `upperArm_joint` 的 URDF 与 MoveIt 位置范围均为
  `[-1.0471976, 1.0471976] rad`，即 `[-60 deg, 60 deg]`。
- 启动时任务节点还会读取 `/robot_description` 和 MoveIt 关节限制做一次一致性检查；
  如果仍加载旧模型，任务进入 `ERROR`，不会执行运动。

## 构建与启动

在工作空间根目录构建并加载环境后运行：

```bash
catkin_make
source devel/setup.bash
roslaunch aubo_color_sorting sorting_gazebo.launch
```

启动完成后，机械臂先进入 `observe` 观察姿态。确认调试图像和目标位置正常，再开始：

```bash
rosservice call /sorting/start
```

也可以启动后自动执行：

```bash
roslaunch aubo_color_sorting sorting_gazebo.launch auto_start:=true
```

无界面运行时可关闭 Gazebo、RViz 和图像窗口：

```bash
roslaunch aubo_color_sorting sorting_gazebo.launch \
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

## 接入真实机械臂

真实机械臂由 `aubo_ros_control` 和真实相机驱动提供控制器、关节状态及相机话题后，
只启动算法节点：

```bash
roslaunch aubo_color_sorting sorting.launch use_grasp_attachment:=false
```

首次在真实设备运行时应保持 `auto_start:=false`，降低速度比例，并先检查相机外参、
桌面高度、抓取偏移和所有规划轨迹。真实相机必须发布已经对齐到彩色图的深度图；
如果话题名称不同，应修改 `launch/sorting.launch` 中的 `depth_topic`。Gazebo 的吸附
插件不能用于真实机械臂。
