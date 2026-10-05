# 功能包总览与运行指南

本文回答三个问题：每个包负责什么、应该从哪里启动、启动前还需要什么。
包名以 `package.xml` 为准，而不是目录名。例如 `wheeltec_joy_control` 目录中的包名是 `wheeltec_joy`。
当前清单覆盖 **46 个未被 `CATKIN_IGNORE` 屏蔽的包**，并在末尾单列保留的第三方源码。
“参与构建”不等于“所有功能均已验证可用”；硬件、模型权重和外部依赖仍需自行准备。

按功能查找：[运行约定](#1-阅读与运行约定) · [固定机械臂](#2-aubo-固定机械臂11-个包) ·
[复合机器人](#3-aubo-复合移动机器人9-个包) · [差速机器人](#4-简单差速机器人1-个包) ·
[通用算法](#5-通用标定与算法组件5-个包) · [设备驱动](#6-传感器与夹爪驱动10-个包) ·
[WheelTec](#7-wheeltec-参考应用与仿真10-个包) · [保留源码](#8-保留但默认不构建的第三方源码25-个包)。

## 1. 阅读与运行约定

- **完整入口**：通常会启动场景、机器人和相关算法，第一次体验优先选它。
- **组件入口**：只启动局部节点，必须先准备相机、TF、控制器等；不能当作完整演示运行。
- **模型 / 配置 / 接口 / 插件**：可能没有独立节点，正常用法是由上层 launch 加载。
- **实机 / 旧功能**：必须核对设备型号、参数和依赖。命令只是入口说明，不代表连接后可直接运动。

下文命令用于 **Linux Bash + ROS1 + Catkin**，不是 Windows PowerShell 命令。
先按照[根目录安装说明](../README.md#安装编译与环境加载)准备 Melodic 或 Noetic、Gazebo、MoveIt、导航和 Python 依赖。
假设仓库放在 `~/catkin_ws/src`；若位置不同，请替换工作空间路径。每个新终端都要加载环境：

```bash
# 在工作空间根目录编译，不是在仓库 src 目录编译。
cd ~/catkin_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source src/setup_ros.sh

# 确认使用当前工作空间中的包，而不是另一个 overlay 中的同名包。
rospack find aubo_mobile_bringup
```

一个 ROS Master 下尽量只启动一套机器人、Gazebo、MoveIt 和导航；不要同时执行下文所有示例。
对同一底盘，只保留一个主动速度来源，避免导航、键盘、跟随节点争用 `/cmd_vel`。
实机实验应先确认急停、工作空间、夹爪方向、标定和限速，且仿真入口不能接入真机控制链。

### 最适合展示的入口

```bash
# 三条命令任选一条；默认不自动启动分拣任务。
roslaunch aubo_mobile_bringup showcase.launch scenario:=perception
roslaunch aubo_mobile_bringup showcase.launch scenario:=sorting
roslaunch aubo_mobile_bringup showcase.launch scenario:=nav_sorting
```

任务启动按钮、讲解顺序和重置方式见[演示指南](SHOWCASE.md)。
学习底盘基础则从 `simple_diff_robot_gazebo` 开始。

## 2. AUBO 固定机械臂（11 个包）

### `aubo_description`：机械臂模型

- 作用：AUBO i5 的 URDF/Xacro、网格、夹爪与相机安装关系；供 Gazebo、MoveIt 和 RViz 共用。
- 类型：模型 / 可视化入口，不连接真机，也不执行轨迹。
- 运行：`roslaunch aubo_description arm_with_camera_display.launch`。
- 前置与观察：需要 RViz、Xacro、joint_state_publisher；在 RViz 中检查 RobotModel 和 TF，拖动关节仅改变显示。
- 源码与细节：[aubo_description](../aubo/aubo_description/README.md)。

### `aubo_gazebo`：机械臂仿真及控制器

- 作用：加载 Gazebo 世界、机械臂模型和 ros_control 关节轨迹控制器。
- 类型：仿真基础入口。
- 运行：`roslaunch aubo_gazebo aubo_gazebo_bringup.launch`。
- 前置与观察：需要 Gazebo、gazebo_ros_control、MoveIt；观察关节状态和控制器是否正常加载。仅加载世界可用 `aubo_world_gazebo.launch`，但世界入口本身不等于完整规划链。
- 注意：不要再同时启动其他完整 AUBO 仿真入口，避免重复生成机器人和控制器。
- 配置位置：[launch](../aubo/aubo_gazebo/launch)。

### `aubo_gazebo_plugins`：仿真抓取附着插件

- 作用：在 Gazebo 中将被抓物体附着到夹爪、再释放；供分拣状态机调用。
- 类型：共享库插件，没有独立 `roslaunch` / `rosrun` 入口。
- 使用：先编译本包，再运行 `roslaunch aubo_sorting sorting_gazebo.launch auto_move_to_observation:=false auto_start:=false`，由世界文件加载插件。
- 前置与观察：查看 `/sorting/grasp/attach`、`/sorting/grasp/detach` 服务及 `/sorting/grasp/status`。附着是仿真辅助机制，不代表实机抓取成功或真实摩擦接触模型。
- 细节：[插件说明](../aubo/aubo_gazebo_plugins/README.md)。

### `aubo_moveit_config`：固定机械臂运动规划配置

- 作用：SRDF 规划组、运动学、碰撞矩阵、规划器、轨迹控制器和 RViz 配置。
- 类型：MoveIt 配置入口。
- 运行：`roslaunch aubo_moveit_config demo.launch`，用 fake controller 观察规划；实际 Gazebo 规划用 `roslaunch aubo_moveit_config demo_gazebo.launch`。
- 前置与观察：需要 MoveIt；在 MotionPlanning 中设置目标并 Plan。fake controller 的 Execute 不会驱动实机。
- 注意：`move_group.launch` 是组件入口，须已有机器人描述、关节状态与控制器；真机优先使用 `aubo_ros_control` 的完整入口。
- 配置位置：[aubo_moveit_config](../aubo/aubo_moveit_config)。

### `aubo_perception`：通用视觉感知

- 作用：颜色 / YOLO 二维检测、RGB-D 抓取几何和点云处理；固定臂与移动臂共用。
- 类型：完整仿真入口 + 组件入口。
- 运行：`roslaunch aubo_perception color_eye_to_hand_gazebo.launch`；眼在手上用 `color_eye_in_hand_gazebo.launch`。
- 前置与观察：颜色入口不需要 YOLO 权重；检查图像、深度、CameraInfo、TF 与检测结果。`grasp_perception.launch` 只接入现有相机，不启动完整机器人。
- YOLO：需要额外 Python 环境和模型；默认 `/home/zlab/...` 是开发机路径，必须替换。例如已有 RGB 图像时：

```bash
# 以下 /absolute/... 是占位符，换成你机器上的解释器和权重。
roslaunch aubo_perception ultralytics_yolo.launch \
  input_mode:=topic image_topic:=/camera/color/image_raw \
  python:=/absolute/yolo_env/bin/python model_path:=/absolute/models/model.pt
```

- 细节：[感知说明](../aubo/aubo_perception/README.md)、[launch 参数](../aubo/aubo_perception/launch/README.md)。

### `aubo_planning`：运动规划实验

- 作用：笛卡尔轨迹、夹爪、抓放与 OctoMap 避障示例。
- 类型：实验组件，部分 launch 包含完整仿真。
- 运行：`roslaunch aubo_planning cartesian_path_demo.launch execute:=false` 默认同时启动 MoveIt fake demo，只规划不执行轨迹。若机械臂 Gazebo / MoveIt 已运行，则追加 `start_demo:=false`，避免重复启动。
- 完整避障实验：`roslaunch aubo_planning octomap_planning_gazebo.launch execute:=false`，不要同时保留上一套仿真。
- 前置与观察：检查规划轨迹、笛卡尔路径完成率和 PlanningScene；确认安全后才考虑 `execute:=true`。夹爪和 pick/place 示例会运动，参数需按场景配置。
- 细节：[规划示例](../aubo/aubo_planning/README.md)。

### `aubo_sdk`：厂商通信与底层 SDK

- 作用：封装厂商 SDK、提供连接状态诊断及底层关节运动示例，供硬件接口链接。
- 类型：底层库 / 诊断工具，不启动 MoveIt。
- 运行：`rosrun aubo_sdk sdk_test 192.168.1.2`，将 IP 换成实际控制柜 IP；此工具用于读取状态。
- 前置与观察：需 SDK 动态库、匹配的 protobuf ABI 和可达控制柜；检查连接、关节及 TCP 状态。
- 注意：`joint_move_demo` 是另一个带执行确认的运动工具，不作为默认检查命令；读完其安全要求再使用。
- 细节：[SDK 说明](../aubo/aubo_sdk/README.md)。

### `aubo_ros_control`：真机控制与视觉伺服

- 作用：AUBO hardware_interface、状态反馈、MoveIt 真机轨迹执行，以及眼在手上 / 手外视觉伺服、混合控制。
- 类型：实机入口 + 独立伺服仿真入口。
- 首次检查：`roslaunch aubo_ros_control aubo_state.launch robot_ip:=192.168.1.2`，先验证状态通信。
- 完整真机：`roslaunch aubo_ros_control aubo_real_bringup.launch robot_ip:=192.168.1.2`。这会建立运动控制链，必须完成安全检查。
- 伺服仿真：`roslaunch aubo_ros_control eye_to_hand_visual_servo_gazebo.launch auto_start:=false`。
- 前置与观察：SDK ABI、网络、控制柜模式、相机话题和相机外参；不要将仿真外参直接用于真机。
- 细节：[控制说明](../aubo/aubo_ros_control/README.md)、[视觉伺服](../aubo/aubo_ros_control/VISUAL_SERVO.md)、[混合控制](../aubo/aubo_ros_control/HYBRID_CONTROL.md)。

### `aubo_sorting_core`：共用分拣状态机

- 作用：统一抓取 / 放置流程、目标实例队列、失败恢复、机械臂与底盘锁定；主要 C++ 任务程序为 `sorting_task_cpp`。
- 类型：算法库 / 任务组件，无独立完整场景入口。
- 使用：通过 `aubo_sorting` 或 `aubo_mobile_sorting` 的 launch 加载；不建议直接 `rosrun`，否则缺少任务 YAML 和接口配置。
- 前置与观察：需要检测、TF、MoveIt、夹爪和抓取后端；关注 `/sorting/state`、`/sorting/detection_summary`、`/sorting/base_locked`，以及 `/sorting/start`、`/sorting/stop` 服务。
- 细节：[核心状态机与接口](../aubo/aubo_sorting_core/README.md)。

### `aubo_sorting`：固定机械臂分拣应用

- 作用：组合机器人、感知、规划和分拣核心，实现颜色或 YOLO 目标分拣；支持不同相机安装方式。
- 类型：完整仿真 / 真机应用入口。
- 运行：`roslaunch aubo_sorting sorting_gazebo.launch detector:=color auto_move_to_observation:=false auto_start:=false`。
- 前置与观察：默认颜色场景不需外部权重；使用 RViz 分拣面板或服务手动移至观察位、启动任务，观察识别、抓取、放置和状态变化。
- 注意：常规入口默认 `auto_move_to_observation:=true`，本示例显式关闭启动后自动移臂；真机改用 `sorting_real.launch`，另需机械臂 IP、夹爪串口与外参标定。
- 细节：[固定臂分拣](../aubo/aubo_sorting/README.md)。

### `aubo_vla`：OpenVLA 观察桥接

- 作用：ROS 图像采集 / 时效检查、HTTP 模型请求及动作结果观察，用于研究 VLA 接入。
- 类型：观察组件，**不向机械臂或底盘发送运动指令**。
- 运行：先准备相机图像和模型服务，再运行 `roslaunch aubo_vla observe.launch image_topic:=/workspace_camera/color/image_raw`。
- 前置与观察：模型服务使用单独环境；可先使用 README 中的 mock 模式验证 HTTP 链路。检查 `/openvla_observe/diagnostics`、服务连接和图像时效，暂停的仿真不会提供持续新图像。
- 注意：真实模型权重、GPU 与 Python 依赖不包含在普通 ROS 编译流程中；不要将观察结果称为已实现闭环机器人控制。
- 细节：[OpenVLA 包说明](../aubo/aubo_vla/README.md)、[部署记录](OpenVLA_Ubuntu20.04_部署实测记录.md)。

## 3. AUBO 复合移动机器人（9 个包）

### `aubo_mobile_bringup`：统一启动与展示

- 作用：组合各子系统，提供统一实验和展示入口，以及移动操作视觉伺服入口。
- 类型：完整应用编排。
- 运行：`roslaunch aubo_mobile_bringup showcase.launch scenario:=nav_sorting`；基础场景用 `roslaunch aubo_mobile_bringup simulation.launch mode:=robot`。
- 前置与观察：展示场景默认颜色检测、手动启动任务；`simulation.launch` 支持 `robot / navigation / exploration / sorting / mission`，不能把 `scenario` 与 `mode` 混用。
- 细节：[统一启动](../aubo_mobile_robot/aubo_mobile_bringup/README.md)、[演示指南](SHOWCASE.md)。

### `aubo_mobile_robot`：复合机器人模型与 Gazebo

- 作用：底盘、AUBO、夹爪、相机与双激光模型；加载仿真传感器、关节控制器和里程计。
- 类型：完整基础仿真 / 模型入口。
- 运行：`roslaunch aubo_mobile_robot gazebo.launch`；仅看模型用 `roslaunch aubo_mobile_robot display.launch`。
- 前置与观察：检查 `/odom`、前后激光、相机与 TF；基础模型入口不等于导航 / 自动分拣已启动。
- 细节：[模型与仿真](../aubo_mobile_robot/aubo_mobile_robot/README.md)。

### `aubo_mobile_moveit_config`：复合机器人机械臂规划

- 作用：复合平台的 SRDF、机械臂规划组、控制器、碰撞与传感器配置。
- 类型：MoveIt 配置入口。
- 运行：`roslaunch aubo_mobile_moveit_config demo.launch`，使用 fake controller；真实仿真执行用 `demo_gazebo.launch`。
- 前置与观察：需要 MoveIt；检查规划组、末端和轨迹。机械臂规划不负责底盘全局导航，后者由 `aubo_mobile_navigation` 承担。
- 配置位置：[aubo_mobile_moveit_config](../aubo_mobile_robot/aubo_mobile_moveit_config)。

### `aubo_mobile_navigation`：建图、定位、导航与探索

- 作用：Gmapping、AMCL、move_base、双激光合并、激光速度安全过滤及 RRT 探索编排。
- 类型：完整仿真入口 + 算法组件。
- 运行：

```bash
# 先建图；另一个终端运行键盘控制，并在地图完成后保存。
roslaunch aubo_mobile_navigation mapping_gazebo.launch
roslaunch aubo_mobile_control keyboard_teleop.launch
roslaunch aubo_mobile_navigation map_saver.launch map_name:=/absolute/maps/lab

# 关闭建图与键盘进程后，再启动导航。map_file 指向已保存的 YAML。
roslaunch aubo_mobile_navigation navigation_gazebo.launch map_file:=/absolute/maps/lab.yaml

# 自主探索是另一套完整入口，不与上面的 Gazebo 同时启动。
roslaunch aubo_mobile_navigation rrt_exploration_gazebo.launch
```

- 前置与观察：地图保存目录需存在且可写；导航先在 RViz 设置初始位姿，再设置目标。探索需用 Publish Point 指定四个边界角点和第五个自由空间种子点。
- 组件：`scan_merger.launch`、`laser_safety_filter.launch` 只处理已有传感器 / 速度；`mapping.launch`、`navigation.launch` 不应与完整仿真入口重复加载。
- 细节：[导航说明](../aubo_mobile_robot/aubo_mobile_navigation/README.md)、[地图说明](../aubo_mobile_robot/aubo_mobile_navigation/maps/README.md)。

### `aubo_mobile_control`：底盘与机械臂协调

- 作用：键盘遥控、导航前后机械臂姿态协调、手眼标定与相关控制组件。
- 类型：组件 + 协调实验入口。
- 运行：已有底盘时 `roslaunch aubo_mobile_control keyboard_teleop.launch`；完整仿真协调实验用 `roslaunch aubo_mobile_control navigation_arm_gazebo.launch send_navigation_goal:=false`。
- 前置与观察：键盘需要可交互终端，会发布速度；协调实验显式关闭自动发送导航目标，但仍需核对机械臂姿态流程。
- 标定：`roslaunch aubo_mobile_control handeye_calibration.launch motion_mode:=teach` 是实机链，需相机、标记和机械臂；先阅读标定说明，不要无准备启动自动采样运动。
- 细节：[协调与标定](../aubo_mobile_robot/aubo_mobile_control/README.md)。

### `aubo_mobile_follower`：激光 / 颜色跟随与循线

- 作用：跟随控制器、调试输出、目标交互及传感器超时 / 激光安全处理；也供简单差速机器人复用。
- 类型：完整仿真实验，启动后可能主动控制底盘。
- 运行：`roslaunch aubo_mobile_follower laser_follow.launch`、`color_follow.launch`、`line_follow.launch` 三选一。
- 前置与观察：不要同时启动导航或遥控速度源；观察目标位置、调试图像、跟随状态和丢目标停车。外部相机需校验画面方向、HSV 和 TF。
- 注意：`semantic_line_follow.launch` 为进阶入口，另需其语义检测后端配置；不属于基础颜色循线的零配置承诺。
- 细节：[跟随说明](../aubo_mobile_robot/aubo_mobile_follower/README.md)。

### `aubo_mobile_perception`：移动平台相机适配

- 作用：为移动臂配置颜色 / RGB-D YOLO 检测及工作空间点云过滤，复用通用感知算法。
- 类型：组件入口，不启动机器人 / 相机。
- 运行：已有对应相机后 `roslaunch aubo_mobile_perception color_detector.launch`；首次展示更推荐上层 `showcase.launch scenario:=perception` 自动配好相机话题。
- 前置与观察：默认颜色组件使用手部相机话题；眼在手外输入必须对齐 RGB、深度、CameraInfo、TF。观察 `/sorting/detections` 和 `/sorting/debug_image`。
- 注意：YOLO 入口需要本地权重和解释器配置；不能直接使用开发机绝对路径。
- 细节：[移动感知说明](../aubo_mobile_robot/aubo_mobile_perception/README.md)。

### `aubo_mobile_sorting`：移动平台工位分拣

- 作用：静止工位的感知、机械臂抓取和放置编排；提供 RViz SortingPanel，分拣时锁定底盘。
- 类型：完整仿真入口 + 已有机器人上的任务组件。
- 运行：`roslaunch aubo_mobile_sorting sorting_gazebo.launch perception_mode:=color auto_move_to_observation:=false auto_start:=false`。
- 前置与观察：检查检测、机械臂规划、夹爪及状态；通过面板手动准备观察姿态并启动。`sorting.launch` 只接入已有机器人与 MoveIt。
- 细节：[移动臂分拣](../aubo_mobile_robot/aubo_mobile_sorting/README.md)。

### `aubo_mobile_nav_sorting`：导航与分拣完整任务

- 作用：多工位导航、到站分拣、状态切换、停止恢复及返航；提供 RViz NavSortingPanel。
- 类型：完整任务入口。
- 运行：`roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch perception_mode:=color auto_start:=false`。
- 前置与观察：需要导航地图、MoveIt、感知和分拣；观察 `/nav_sorting/state`、`/sorting/state` 和全局 / 局部路径；使用面板或 `/nav_sorting/start`、`/nav_sorting/stop`。
- 注意：另一入口 `mission_gazebo.launch` 默认自动启动任务；首次运行务必显式指定 `auto_start:=false`。实机 `navigation_sorting.launch` 需真实底盘、机械臂、相机和夹爪配置。
- 细节：[任务说明](../aubo_mobile_robot/aubo_mobile_nav_sorting/README.md)、[工位配置](../aubo_mobile_robot/aubo_mobile_nav_sorting/config/README.md)。

## 4. 简单差速机器人（1 个包）

### `simple_diff_robot_gazebo`：底盘学习与独立演示

- 作用：两轮差速模型、激光 / 相机、键盘控制、建图导航、RRT、跟随循线和单障碍绕行。
- 类型：独立完整仿真，无需 AUBO SDK 或机械臂。
- 运行：

```bash
# 两个终端：基础仿真 + 键盘；W/S 前后，A/D 转向，空格停车。
roslaunch simple_diff_robot_gazebo gazebo.launch
roslaunch simple_diff_robot_gazebo teleop.launch

# 以下是其他完整场景，先关闭上一套仿真再选择一个。
roslaunch simple_diff_robot_gazebo mapping.launch
roslaunch simple_diff_robot_gazebo laser_follow.launch
roslaunch simple_diff_robot_gazebo color_follow.launch
roslaunch simple_diff_robot_gazebo line_follow.launch
roslaunch simple_diff_robot_gazebo single_obstacle_demo.launch auto_start:=false
```

- 导航：先通过 `map_saver.launch` 保存地图，再运行 `roslaunch simple_diff_robot_gazebo navigation.launch map_file:=/absolute/maps/lab.yaml`。
- 前置与观察：需要 Gmapping、Navigation Stack、GlobalPlanner、TEB；检查 `/scan`、`/odom`、`/camera/image_raw` 和 TF。跟随会自行运动，不能与键盘 / 导航抢占速度。
- 细节：[完整说明](../simple_diff_robot_gazebo/README.md)、[排错](../simple_diff_robot_gazebo/TROUBLESHOOTING.md)。

## 5. 通用标定与算法组件（5 个包）

| 包名 | 作用 | 运行 / 使用方式 | 前置条件与检查 |
| --- | --- | --- | --- |
| `easy_handeye` | 采集机器人 / 标记 TF 并求解手眼外参，保存与发布标定结果 | 推荐通过 `aubo_mobile_control handeye_calibration.launch` 完整配置；已有 TF 时可用下方组件命令 | 机器人基座、末端、相机、标记四组 frame 必须正确且持续更新；标定后验证外参而不是只看求解成功 |
| `easy_handeye_msgs` | 标定服务 / 消息定义 | 无独立节点；编译后供标定服务和 GUI 使用 | 不单独 `rosrun`；接口定义见 [srv](../easy_handeye/easy_handeye_msgs/srv) |
| `rqt_easy_handeye` | 标定采样、运动和评估 GUI 插件 | 由 easy_handeye 标定入口加载；也可在 rqt 的 Robot Tools 菜单打开 | 先启动标定服务；仅打开 GUI 不会自动建立全部 TF 和硬件链 |
| `robot_pose_ekf` | 融合底盘里程计与 IMU，发布融合位姿 / TF | 已被 WheelTec 底盘入口包含；独立组件入口见下方 | 需要 `/odom`、`/imu`；不要同时启动两个同名 EKF 或重复 TF 发布者 |
| `rrt_exploration` | frontier 检测、过滤与探索目标分配 | 优先 `aubo_mobile_navigation rrt_exploration_gazebo.launch` 或差速机器人同名入口；`roslaunch rrt_exploration simple.launch` 仅算法 | 已有 `/map`、TF、move_base 与路径规划服务；先用 RViz 点击探索边界及种子点 |

```bash
# 已有机械臂状态 / TF 时的标定组合；本入口会启动 RealSense 和 ArUco，勿重复开相机。
# 需要系统安装 realsense2_camera、aruco_ros，并准备 10 cm、ID 30 的标记。
roslaunch easy_handeye aubo_realsense_calibration.launch

# 已有 WheelTec /odom 与 /imu 时单独启动融合；完整底盘入口已包含它，不要重复启动。
roslaunch turn_on_wheeltec_robot include/robot_pose_ekf.launch
```

详细参考：[easy_handeye](../easy_handeye/README.md)、[标定 launch](../easy_handeye/easy_handeye/launch/README.md)、[RRT](../rrt_exploration/README.md)、[EKF](../robot_pose_ekf/README.md)。

## 6. 传感器与夹爪驱动（10 个包）

这些入口面向**真实设备**，通常只启动驱动，不包含机器人、地图或任务。
先核对设备型号、串口别名、波特率、串口权限或网卡 / IP。不要给同一个串口启动两个驱动。

| 包名 | 作用 | 运行 / 使用方式 | 前置条件与检查 |
| --- | --- | --- | --- |
| `fdilink_ahrs` | FDI IMU / AHRS / GNSS 串口解析 | `roslaunch fdilink_ahrs ahrs_data.launch` | 实际默认端口 `/dev/wheeltec_FDI_IMU_GNSS`、921600；参数位于 launch 内，不是可直接传入的 arg；看 `/imu`，其他输出取决于设备能力 |
| `inspire_gripper` | 因时夹爪串口控制 | `roslaunch inspire_gripper gripper_control.launch port:=/dev/ttyUSB0 test_flag:=0` | 核对 ID / 波特率；关闭内置测试标志，连接前确认夹爪无人手 / 障碍；上层分拣负责发送动作 |
| `ldlidar_14` | LD14 激光驱动 | `roslaunch ldlidar_14 ld14.launch` | 默认设备 `/dev/wheeltec_lidar`，查看 `/scan`；端口配置在 launch 中 |
| `ldlidar_19` | LD19 激光驱动 | `roslaunch ldlidar_19 ld19.launch` | 同上，务必使用匹配型号；查看 `/scan` |
| `rplidar_ros` | 思岚 RPLIDAR 驱动 | `roslaunch rplidar_ros rplidar.launch` | 基础型号入口；A3 / S1 / S2 使用对应型号 launch，核对端口、波特率和 frame；查看 `/scan` |
| `lslidar_driver` | 乐动系列激光驱动（位于 `lsx10`） | `roslaunch lslidar_driver lslidar_serial.launch` 或 `lslidar_net.launch`，二选一 | 核对 YAML 中设备 / 网络参数；构建需要 libpcap，未安装时此驱动可能被跳过；按配置检查扫描输出 |
| `lslidar_msgs` | 上述驱动使用的消息定义 | 无独立入口，编译后由 `lslidar_driver` 使用 | 消息类型包不产生传感器数据 |
| `lslidar_x10_driver` | X10 系列激光驱动及多雷达入口 | `roslaunch lslidar_x10_driver lslidar_x10_serial.launch` 或 `lslidar_x10_net.launch` | 核对型号、串口 / IP、frame 和配置话题；双 / 四雷达入口需分别配置设备，不能照搬默认地址 |
| `lslidar_x10_msgs` | X10 驱动消息定义 | 无独立入口，编译后由 X10 驱动使用 | 不单独 `rosrun` |
| `lslidar_x10` | X10 元包，组织驱动与消息依赖 | 无独立节点；运行 `lslidar_x10_driver` 的入口 | 元包本身不是驱动程序 |

驱动细节：[FDI](../fdilink_ahrs/README.md)、[LD19](../ldlidar_19/README.md)、[RPLIDAR](../rplidar_ros/README.md)、[乐动 / X10](../lsx10/README.md)。
个别上游 README 的旧 launch 名可能与源码不一致，以上入口以当前实际文件为准。

```bash
# 启动雷达 / IMU 后检查是否持续发布，而不是只看到进程存在。
rostopic hz /scan
rostopic hz /imu
rostopic echo -n 1 /scan/header
```

## 7. WheelTec 参考应用与仿真（10 个包）

这部分保留教学 / 厂商参考功能。部分入口会调用外部包或已经迁出的相机 / 语音资产，
必须按所选功能核对依赖，不代表所有历史入口仍可开箱即用。

### `turn_on_wheeltec_robot`：实机底盘与功能编排

- 作用：串口底盘通信、车型模型、TF、EKF，以及建图 / 导航和传感器组合入口。
- 类型：实机基础入口。
- 运行：`roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch car_mode:=mini_mec usart_port_name:=/dev/wheeltec_controller`。
- 前置与观察：车型必须与实际底盘一致；此基础入口不自动提供所有相机 / 雷达。核对 `/odom`、`/imu` 和融合 TF，再单独接入正确传感器。
- 注意：导航 / 建图选择需要额外算法包和匹配地图；语音、旧相机等历史功能的依赖未必仍在主仓库。
- 位置：[launch](../turn_on_wheeltec_robot/launch)、[设备切换说明](车型_雷达_相机的切换.txt)。

### `wheeltec_description`：WheelTec 多车型模型

- 作用：多种底盘的 URDF、网格、传感器安装和显示配置。
- 类型：模型 / 可视化入口。
- 运行：`roslaunch wheeltec_description wheeltec_robot_display.launch car_mode:=mini_mec`。
- 前置与观察：需要 RViz / Xacro；检查车体、关节与 TF；显示入口不会启动真实串口或自动导航。
- 位置：[模型包](../wheeltec_robot_gazebo/wheeltec_description)。

### `wheeltec_gazebo_control`：WheelTec 仿真底盘

- 作用：Gazebo、底盘控制器、仿真里程计以及可选导航配置。
- 类型：完整基础仿真入口。
- 运行：`roslaunch wheeltec_gazebo_control wheeltec_gazebo_control.launch car_mode:=mini_mec`。
- 前置与观察：核对模型、控制器、激光和 TF；具体车型依赖其描述文件。导航控制器加载不代表已提供正确静态地图和 AMCL。
- 位置：[仿真控制包](../wheeltec_robot_gazebo/wheeltec_gazebo_control)。

### `wheeltec_gazebo_function`：WheelTec 仿真建图与导航

- 作用：组合 WheelTec 仿真与 Gmapping / 其他建图、地图服务、AMCL 和 RRT。
- 类型：完整仿真应用入口。
- 运行：`roslaunch wheeltec_gazebo_function mapping.launch mapping_mode:=gmapping`；导航用 `roslaunch wheeltec_gazebo_function navigation.launch map_file:=/absolute/maps/lab.yaml`。
- 前置与观察：建图 / 导航二选一；其他算法选项需要对应外部包。核对实际 launch 转发的车型参数，不能假定所有入口都支持 `car_mode`。
- 位置：[功能包](../wheeltec_robot_gazebo/wheeltec_gazebo_function)。

### `wheeltec_robot_rc`：键盘遥控

- 作用：终端键盘速度输入，发布 `/cmd_vel`。
- 类型：控制组件，不启动底盘。
- 运行：已有底盘 / 仿真时 `roslaunch wheeltec_robot_rc keyboard_teleop.launch`。
- 前置与观察：可交互终端、已启动底盘；按程序提示控制，测试停车和输入超时。停止其他主动速度源。
- 细节：[遥控说明](../wheeltec_robot_rc/README.md)。

### `wheeltec_joy`：手柄遥控

- 作用：读取手柄并映射底盘速度；目录名为 `wheeltec_joy_control`。
- 类型：控制组件。
- 运行：已有底盘时 `roslaunch wheeltec_joy joy_control.launch`。
- 前置与观察：需要 ROS joy 和 `/dev/input/js0` 等实际设备；先核对轴 / 按键、死区与停车行为，再允许底盘运动。
- 位置：[手柄包](../wheeltec_joy_control)。

### `wheeltec_multi`：多机器人编队跟随

- 作用：根据主车位姿、编队偏置和激光避障计算从车速度，支持全向 / 非全向控制。
- 类型：实机 / 多机系统组件，启动后可能运动。
- 运行：完成主从机器人网络、命名空间和定位配置后，使用 `roslaunch wheeltec_multi wheeltec_slave.launch slave_x:=-0.8 slave_y:=0.8`。
- 前置与观察：先验证主车与从车位姿在一致坐标系中、激光有效、底盘型号匹配；默认全向控制不适用于所有差速 / Ackermann 底盘。观察跟随误差、避障与停车。
- 细节：[编队说明](../wheeltec_multi/README.md)。

### `kcf_track`：视觉目标跟踪与跟随参考

- 作用：KCF 图像目标跟踪，结合 PID 输出跟随控制。
- 类型：旧实机应用入口，可能主动控制底盘。
- 运行入口：`roslaunch kcf_track kcf_tracker.launch`，**仅在核对相机和底盘依赖后使用**。
- 前置与观察：入口会组合底盘 / 相机，而不只是一个图像算法；迁出的相机包或不同话题可能需要适配。检查图像、目标框和速度输出，不与导航同时运行。
- 位置：[跟踪包](../kcf_track)。

### `simple_follower`：厂商跟随功能参考

- 作用：激光跟随、颜色 / 视觉跟随、循线及 AR 等参考实现。
- 类型：旧实机应用入口，可能主动控制底盘。
- 运行入口：`roslaunch simple_follower laser_follower.launch`，入口会包含底盘和雷达。
- 前置与观察：需要匹配 WheelTec 底盘与雷达；视觉 / AR 功能另需相机及检测依赖。纯仿真体验优先使用 `simple_diff_robot_gazebo` 或 `aubo_mobile_follower`。
- 位置：[参考跟随包](../simple_follower)。

### `wheeltec_yolo_action`：旧 YOLO 动作应用

- 作用：将检测结果接入跟随 / 手势等动作决策，是历史参考链路。
- 类型：**待补外部依赖的旧应用，不作为默认可运行演示**。
- 历史入口：`roslaunch wheeltec_yolo_action dp_drive.launch` 或 `gesture.launch`；不要在依赖未满足时直接运行。
- 前置与观察：当前主仓库未提供其所需的 `darknet_ros_msgs`；还要准备兼容检测节点、权重、相机 / 雷达和底盘，核对检测消息、话题及动作安全。
- 位置：[旧动作包](../wheeltec_yolo_action)。

## 8. 保留但默认不构建的第三方源码（25 个包）

以下目录或其父目录有 `CATKIN_IGNORE`。`catkin_make` 不会因此编译它们，
`roslaunch` / `rospack` 找到的同名包可能来自 `/opt/ros` 或其他工作空间。
保留源码用于参考和必要时适配；正常运行优先使用与 ROS 发行版匹配的系统包。
不要为了“能找到包”直接删除忽略标记，否则可能引入重复包名、旧 API 或依赖冲突。

### Navigation：`navigation-melodic`

| 包名 | 作用与使用方式 |
| --- | --- |
| `amcl` | 激光地图定位；由导航应用 launch 加载，依赖地图、扫描与 TF |
| `map_server` | 加载地图，提供 map_saver；通过上层 navigation / map_saver 入口使用 |
| `move_base` | 导航 action 服务与规划调度；通过机器人导航入口使用 |
| `costmap_2d` | 全局 / 局部代价地图库及层插件，由 move_base 参数配置 |
| `nav_core` | 全局 / 局部规划器与恢复行为接口，无独立运行入口 |
| `navfn` | Navfn 全局路径规划插件，在 move_base YAML 中选择 |
| `global_planner` | 全局规划插件，在 move_base YAML 中选择 |
| `base_local_planner` | 局部轨迹规划库 / 插件，由 move_base 加载 |
| `dwa_local_planner` | DWA 局部规划插件，与 TEB 按需二选一 |
| `carrot_planner` | 简单目标投影式全局规划插件，供实验配置使用 |
| `clear_costmap_recovery` | 清理代价地图恢复插件，由 move_base 的 recovery_behaviors 加载 |
| `rotate_recovery` | 原地旋转恢复插件，执行时会使底盘运动 |
| `move_slow_and_clear` | 限速 / 清障恢复插件，由导航配置加载 |
| `fake_localization` | 仿真假定位节点，需要匹配真值输入与 TF；不能替代实机定位 |
| `voxel_grid` | 三维体素网格支持库，无独立任务入口 |
| `navigation` | 导航元包，组织上述组件，没有独立节点 |

运行方式：使用前文的 `aubo_mobile_navigation navigation_gazebo.launch`、
`simple_diff_robot_gazebo navigation.launch` 等完整入口，而不是直接运行某个插件库。
配置全局 / 局部规划器前先用 `rospack find global_planner`、`rospack find teb_local_planner` 确认实际来源。

### 其他保留源码

| 目录 / 包名 | 作用 | 运行与限制 |
| --- | --- | --- |
| `realsense-ros-development` / `realsense2_camera` | RealSense 真实 RGB-D 驱动 | 安装兼容的系统版本后可用 `roslaunch realsense2_camera rs_camera.launch`；需要真实设备及 librealsense，保留快照不默认构建 |
| 同目录 / `realsense2_description` | 相机模型 / 网格 | 由相机 / 机器人模型加载，无独立采集节点 |
| 同目录 / `ddynamic_reconfigure` | 动态参数支持库 | RealSense 驱动依赖，无独立任务入口 |
| `slam_karto` / `slam_karto` | Karto 二维激光 SLAM 节点 | 通过选择 Karto 的建图 launch 使用；需系统安装兼容版本及扫描、里程计 TF |
| 同目录 / `open_karto` | Karto 算法库 | 由 slam_karto 链接，无独立节点 |
| 同目录 / `sparse_bundle_adjustment` | 稀疏优化支持库 | 由 SLAM 后端调用，无独立任务入口 |
| `teb_local_planner-melodic-devel` / `teb_local_planner` | TEB 局部路径与速度规划 | 由 move_base 加载插件，需要系统包和正确机器人运动学 / footprint 配置 |
| `depthimage_to_laserscan-melodic-devel` / `depthimage_to_laserscan` | 深度图转二维 LaserScan | 用兼容系统包节点接入已有深度图与 CameraInfo，核对光学 frame、扫描高度和量程；不是完整导航入口 |
| `ros_tensorflow` / `ros_tensorflow` | 旧 TensorFlow ROS 图像推理示例 | 历史源码参考；当前默认屏蔽，旧 Python / TensorFlow 环境与模型未纳入支持，不提供开箱即用承诺 |

## 9. 已迁出资产与非功能包目录

`qt_ros_test`、两套 `xf_mic_asr_offline`、`bodyreader`、`tts_make`、`ros_astra_camera`
已从主仓库迁出，不再计入上述功能包。需要语音、体感或旧 Astra 功能时，应先恢复对应外部资产并明确维护方式，
不能把依赖它们的旧 launch 当作当前完整入口。迁移范围见[清理记录](repository_cleanup_audit.md)。

`aubo/video_or_img` 是演示素材，`robot-system-wiki` 是资料，`docs` 是文档，不是 ROS 包。
没有本地 `package.xml` 的空目录 / Git 子模块占位也不计为可运行功能包；缺少外部源码时应先查清来源，不能仅凭目录名给出运行承诺。

## 10. 通用验收与排错

```bash
# 包发现与节点：入口不存在时先检查是否加载了正确工作空间。
rospack find aubo_mobile_navigation
rosnode list
rosparam get /use_sim_time

# 仿真时间必须在推进；Gazebo 暂停时图像 / 任务超时不等于算法崩溃。
rostopic hz /clock
rostopic hz /joint_states
rostopic hz /scan

# 查看坐标系；底盘导航与机械臂感知的 frame 应连通。
rosrun tf tf_echo map base_footprint

# 在工作空间根目录执行已登记的测试，并检查结果文件。
catkin_make run_tests
catkin_test_results --verbose build/test_results
```

- 模型可见但不动：检查控制器、MoveIt 执行配置和任务是否手动启动；fake controller 不驱动硬件。
- 图像有但没有目标：检查 RGB / 深度 / CameraInfo 对齐、frame、HSV / 模型类别及工作空间过滤。
- 导航不走：检查地图路径、AMCL 初始位姿、TF、规划器插件和底盘锁；不要通过关闭安全过滤掩盖故障。
- 真机驱动起不来：先检查串口别名 / 权限、IP 和 SDK ABI，不要直接开启运动来验证通信。
- “找不到包”：区分未安装的系统依赖、已迁出资产、被忽略的旧源码和未加载工作空间。

本文命令已对照当前仓库的 launch、包名及参数进行静态核查；本次文档整理环境没有 ROS / Gazebo，
尚未逐条进行运行验收。实机入口尤其必须在目标设备上验证，不能把文件存在当作硬件验证通过。

## 11. 后续维护约定

新增 / 删除包时同步更新本页分组与数量；每个条目至少写明职责、入口、前置条件和观察方式。
增加完整入口时注明是否自动启动运动、是否包含机器人 / 相机 / MoveIt，避免组件被误用。
具体参数与算法细节放在包内 README，本页保留索引和最小运行路径；展示流程集中维护在 [SHOWCASE.md](SHOWCASE.md)。
