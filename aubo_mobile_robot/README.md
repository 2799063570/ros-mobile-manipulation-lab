# AUBO 复合移动机器人功能包集合

眼在手外 RGB-D、MoveIt OctoMap、YOLO 抓取适配和视觉伺服的集成说明见
[`EYE_TO_HAND_RGBD.md`](EYE_TO_HAND_RGBD.md)。

## 演示视频

[▶ 预览目标识别、规划、抓取与分类放置流程（MP4，约 3.9 MB）](../aubo/video_or_img/preview.mp4)

[下载高清版本（MP4，约 22.7 MB）](../aubo/video_or_img/sorting_process.mp4)

GitHub 若未直接显示播放器，可点击链接打开或下载原视频。视频及后续演示素材统一
保存在 [`aubo/video_or_img/`](../aubo/video_or_img/README.md)。

## 当前集成能力

- 车体固定式 RealSense RGB-D 相机模型和真机启动入口；
- 对齐深度、注册点云以及工作区点云过滤；
- `PointCloudOctomapUpdater` 与独立的 `use_sensor_manager` 开关；
- YOLO检测框、深度图和相机内参到三维抓取目标的通用适配；
- 分拣初始化阶段的点云与OctoMap就绪检查；
- 底盘移动后清空OctoMap，机械臂规划和执行期间锁住底盘；
- 默认只规划不执行的眼在手外MoveIt小步逼近模式。

该目录用于统一存放 AUBO 复合移动机器人相关的 ROS 功能包。外层目录没有
`package.xml`，它只是功能包集合，因此 catkin 会继续发现下面的各个功能包。

## 当前目录结构

```text
aubo_mobile_robot/
├── aubo_mobile_robot/          # 机器人 Xacro、Gazebo、传感器及控制器
├── aubo_mobile_moveit_config/  # MoveIt 运动规划配置
├── aubo_mobile_navigation/     # 建图、定位、move_base 导航与 RRT 自主探索
├── aubo_mobile_bringup/        # 机器人、导航、分拣和任务的统一仿真入口
├── aubo_mobile_control/        # 键盘控制及导航/机械臂协同
├── aubo_mobile_follower/       # 激光跟随、颜色跟随与视觉循线
├── aubo_mobile_perception/     # 移动平台视觉参数与检测启动入口
├── aubo_mobile_sorting/        # 移动分拣参数、场景、面板与启动入口
└── aubo_mobile_nav_sorting/    # 建图、导航到工位并自动分拣的场景任务
```

## 分层结构

```text
aubo_mobile_robot/
├── aubo_mobile_robot/          # 核心机器人模型与 Gazebo 仿真
├── aubo_mobile_moveit_config/  # MoveIt Setup Assistant 生成的配置
├── aubo_mobile_navigation/     # 建图、定位及 move_base 参数
├── aubo_mobile_bringup/        # 统一仿真启动入口
├── aubo_mobile_control/        # 复合控制与指令分发
├── aubo_mobile_follower/       # 跟随/循迹与机械臂相机姿态准备
├── aubo_mobile_perception/     # 移动平台视觉配置
├── aubo_mobile_sorting/        # 移动平台分拣场景配置
└── aubo_mobile_nav_sorting/    # 导航分拣任务编排
```

AUBO 机械臂的通用网格模型和原有机械臂功能包继续保留在顶层 `aubo/` 目录中。
移动机器人相关功能包应通过依赖复用这些资源，不要重复复制模型文件。

颜色检测、分拣状态机和 Gazebo 抓取插件也已分别下沉到顶层的
`aubo_perception`、`aubo_sorting_core` 和 `aubo_gazebo_plugins`。移动端包只保存
底盘相关模型、参数、界面和场景编排。

## 依赖方向

```text
aubo_description
       ↑
aubo_mobile_robot ── aubo_mobile_moveit_config
       │                         │
       ├── aubo_mobile_navigation│
       ├── aubo_mobile_control   │
       └── aubo_mobile_follower  │
                                 ▼
aubo_mobile_perception ──→ aubo_perception
aubo_mobile_sorting ──────→ aubo_sorting_core
          │                → aubo_gazebo_plugins
          ▼
aubo_mobile_nav_sorting
          ↑
aubo_mobile_bringup（统一组合启动）
```

`aubo_mobile_bringup` 只组合已有功能包，不保存机器人模型、算法或场景参数。原有
功能包 launch 保留用于模块调试；完整仿真优先使用：

```bash
roslaunch aubo_mobile_bringup simulation.launch mode:=sorting
```

检测消息统一为 `aubo_perception/DetectedObjectArray`。`aubo_mobile_perception` 不再
定义另一套同名消息，也不再维护检测器副本。
