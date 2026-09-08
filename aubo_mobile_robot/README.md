# AUBO 复合移动机器人功能包集合

眼在手外 RGB-D、MoveIt OctoMap、YOLO 抓取适配和视觉伺服的集成说明见
[`EYE_TO_HAND_RGBD.md`](EYE_TO_HAND_RGBD.md)。

## 演示视频

[▶ 预览目标识别、规划、抓取与分类放置流程（MP4，约 3.9 MB）](../aubo/video_or_img/preview.mp4)

[下载高清版本（MP4，约 22.7 MB）](../aubo/video_or_img/sorting_process.mp4)

GitHub 若未直接显示播放器，可点击链接打开或下载原视频。视频及后续演示素材统一
保存在 [`aubo/video_or_img/`](../aubo/video_or_img/README.md)。

## 当前集成能力

- 复用 `aubo` 的 RealSense RGB-D 真机启动入口；
- 复用 `aubo_perception` 的对齐深度适配和工作区点云过滤；
- `PointCloudOctomapUpdater` 与独立的 `use_sensor_manager` 开关；
- YOLO检测框、深度图和相机内参到三维抓取目标的通用适配；
- 分拣初始化阶段的点云与OctoMap就绪检查；
- 底盘移动后清空OctoMap，机械臂规划和执行期间锁住底盘；
- 复用统一视觉伺服核心的眼在手外 SDK 控制模式。

通用机器人 URDF 不安装眼在手外相机或相机立柱，以免占用机械臂工作空间并增加
碰撞风险。固定相机应按任务需要安装在对应场景中，其外参也由该场景负责定义。

该目录存放 AUBO 移动平台基础功能，以及移动分拣与导航分拣应用。外层目录没有
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
├── aubo_mobile_sorting/        # 单工位分拣场景、参数和 RViz 面板
└── aubo_mobile_nav_sorting/    # 多工位导航与分拣任务编排
```

分拣应用见 [aubo_mobile_sorting](aubo_mobile_sorting/README.md) 和
[aubo_mobile_nav_sorting](aubo_mobile_nav_sorting/README.md)。只迁移源码目录，
ROS 包名、RViz 插件名和 `roslaunch aubo_mobile_*` 命令不变。

## 源码阅读入口

| 关注的问题 | 主要入口 | 阅读要点 |
| --- | --- | --- |
| 多工位任务怎样串联 | `aubo_mobile_nav_sorting/src/navigation_sorting_mission.cpp` | 导航、观察、分拣、撤离和停止恢复 |
| 底盘和机械臂怎样分工 | 同包 `src/base_executor.cpp`、`src/arm_executor.cpp` | 动作完成条件、重试和取消确认 |
| 服务超时为什么不能直接重启 | 同包 `include/aubo_mobile_nav_sorting/bounded_rpc.h` | 结束本地等待不等于取消远端请求；延迟返回后还需补偿停止 |
| 双雷达如何合并 | `aubo_mobile_navigation/scripts/dual_laser_merger.py` | 坐标变换、角度栅格取近点和时间戳去重 |
| 底盘何时被禁止移动 | `aubo_mobile_navigation/scripts/laser_safety_filter.py` | 机械臂互锁、数据超时、制动距离与连续障碍点簇 |
| 跟随速度怎样计算 | `aubo_mobile_follower/src/aubo_mobile_follower/pid.py` | 测量时间戳、抗积分饱和和动态输出限幅 |

修改任务编排时要分别检查“请求已受理”“动作已完成”和“停止已确认”，三者不能互相替代。
跟随 PID 的重复或乱序测量不会再次积分，但仍应用最新速度上下限；仿真重置或切换任务时，
调用方应通过 `reset()` 清除旧的积分和测量时间。

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
├── aubo_mobile_sorting/        # 单工位分拣场景
└── aubo_mobile_nav_sorting/    # 导航分拣任务编排
```

AUBO 机械臂的通用网格模型和原有机械臂功能包继续保留在顶层 `aubo/` 目录中。
移动机器人相关功能包应通过依赖复用这些资源，不要重复复制模型文件。

颜色/RGB-D检测、视觉伺服、OctoMap点云过滤、分拣状态机和Gazebo抓取插件已分别
下沉到顶层的 `aubo_perception`、`aubo_ros_control`、`aubo_sorting_core` 和
`aubo_gazebo_plugins`。此目录保留底盘模型、导航、平台参数和 bringup；分拣场景与任务编排包位于 `aubo/`。

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
aubo_mobile_bringup ─────→ aubo_ros_control
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

移动导航抓取中的视觉伺服入口为：

```bash
roslaunch aubo_mobile_bringup mobile_manipulation_visual_servo.launch
```

检测消息统一为 `aubo_perception/DetectedObjectArray`。`aubo_mobile_perception` 不再
定义另一套同名消息，也不再维护检测器副本。
