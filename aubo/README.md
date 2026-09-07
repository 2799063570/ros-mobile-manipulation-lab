# AUBO 功能包分层说明

顶层 `aubo/` 保存 AUBO 机械臂本体能力，以及固定底座和移动底盘都可以复用的
通用能力。`aubo_mobile_robot/` 只保存移动底盘带来的模型、导航、平台参数和场景编排。

## 演示素材

[▶ 预览 AUBO 分拣流程（MP4，约 3.9 MB）](video_or_img/preview.mp4)

[下载高清版本（MP4，约 22.7 MB）](video_or_img/sorting_process.mp4)

视频和后续截图统一收录在 [`video_or_img/`](video_or_img/README.md)，避免把演示素材
散落到算法、驱动和平台配置包中。

## 包职责

```text
硬件与模型
├── aubo_sdk                 # 厂商 SDK 头文件、运行库及诊断示例
├── aubo_description         # AUBO i5、夹爪及两种相机安装的模型/TF
├── aubo_ros_control         # RobotHW、统一视觉伺服及 Gazebo/SDK 输出
├── aubo_gazebo              # 固定机械臂 Gazebo 启动及控制器
└── aubo_moveit_config       # 固定机械臂 MoveIt 配置

平台无关能力
├── aubo_perception          # RGB-D/YOLO 检测、目标位姿及 OctoMap 点云过滤
├── aubo_sorting_core        # MoveIt 分拣状态机
└── aubo_gazebo_plugins      # Gazebo 抓取辅助插件

固定机械臂应用
├── aubo_planning            # 抓放、夹爪及深度相机 OctoMap 避障示例
└── aubo_color_sorting       # 固定场景参数、world 和启动入口
```

依赖只能由场景层指向通用层。例如 `aubo_color_sorting` 和
`aubo_mobile_sorting` 都依赖 `aubo_sorting_core`，通用核心不能反向依赖任一平台
场景。两套 MoveIt 配置对应不同机器人模型，应继续独立维护。

## SDK 与上游参考

`aubo_sdk` 中的控制器头文件、运行库和配置来自
[`aubo_perception_planning`](https://github.com/2799063570/aubo_perception_planning)，
本地保留厂商SDK接口，并对Catkin导出、只读诊断、受保护运动示例和
`aubo_ros_control`安全状态处理进行了适配。远程仓库中的硬编码手眼标定结果、
控制器名称和视觉伺服启动文件不能直接覆盖本工程；移动平台通过通用目标消息、TF、
MoveIt和独立启动入口复用这些能力。

## 分拣调用关系

```text
平台 launch 与 YAML
        │
        ├── aubo_perception/color_object_detector.py
        │       └── aubo_perception/DetectedObjectArray
        │
        ├── aubo_sorting_core/color_sorting_task.py
        │       ├── MoveIt
        │       ├── 夹爪 FollowJointTrajectory
        │       └── /sorting/* 服务和状态话题
        │
        └── aubo_gazebo_plugins/libaubo_grasp_attach_plugin.so
                └── 仅在 Gazebo 中辅助保持小物体接触
```

固定平台参数位于 `aubo_color_sorting/config/`；移动平台参数位于
`aubo_mobile_perception/config/` 和 `aubo_mobile_sorting/config/`。不要在三个通用
包中写死桌面高度、控制器命名或具体 world。

## 兼容性说明

原有场景启动命令继续有效：

```bash
roslaunch aubo_color_sorting sorting_gazebo.launch
roslaunch aubo_mobile_sorting sorting_gazebo.launch
```

固定机械臂的两种视觉位置伺服仿真入口为：

```bash
# 腕部相机（眼在手上）
roslaunch aubo_ros_control eye_in_hand_visual_servo_gazebo.launch

# 场景固定 RGB-D 相机（眼在手外）
roslaunch aubo_ros_control eye_to_hand_visual_servo_gazebo.launch
```

两种安装方式使用同一个 `aubo_visual_servo_node`；只在目标坐标变换和误差模型上分支，
Gazebo 与真机只在输出后端上分支。详细设计与真机入口见
[`aubo_ros_control/VISUAL_SERVO.md`](aubo_ros_control/VISUAL_SERVO.md)。

重构后检测消息统一为 `aubo_perception/DetectedObjectArray`。仓库外部节点如果曾
导入 `aubo_color_sorting.msg` 或 `aubo_mobile_perception.msg`，需要改为
`aubo_perception.msg` 并重新编译工作空间。

对于现在的感知部分有几个问题 尤其是对于YOLO检测部分
1、为啥yolo启动的是sh文件 为啥不直接通过py呢
2、我希望D:\files\codeStore\ros-mobile-manipulation-lab\aubo\aubo_perception\launch\ultralytics_yolo.launch有两种模式 一种是订阅话题 一种是读取提供的本地的图像地址
3、我们现在采用了ultralytics_yolo_node进行检测 yolo_rgbd_target_node进行深度对齐求解3D坐标 的形式    我在想两步到底有没有必要 我们是否合并成一个程序 在进行预测的时候将附近帧的深度图像保存起来直接进行映射
4、darknet → 接 ROS 老牌包 darknet_ros 的 BoundingBoxes这个到底有用没 没用 为啥要发布DetectedObjectArray格式的数据是为了和sorting同步吗
  我们应该设置不同的模式   对于视觉伺服模式我们要设置一种跟踪的类别 然后发布这个类别置信度最高的目标的信息
对于分拣我们就是发布 YOLO的分拣抓取参数 三维坐标  宽度（窄边）作为夹爪的宽度 当然后续的抓取注意闭合乘以一个小数即可 类别 角度（对应一个抓取姿态 这里夹爪还是垂直向下 但是有个角度旋转）所以分拣程序也要做调整 可以添加或者新写一个程序
当前的颜色检测是不是也用了深度图像识别三维坐标 但是目前的抓取好像并没有使用z轴坐标 而是利用了桌子的高度  
这样吧 我们设置两种模式 一种就是使用桌子高度的模式 另一种 就是使用深度图像得到的三维坐标的模式 但是需要注意的是这个模式是需要进行校验的 我们可以利用物体的高度结合桌子的高度进行故意的判断
5、还有就是对于感知 我们先把实现放到aubo目录下 我们先不要考虑复合机器人的抓取 我们要先考虑只是对于机械臂的分拣抓取 同时需要注意的是 我们最好不管是对于yolo还是OpenCV的颜色识别 最好都弄一个眼在手上和眼在手外的模式
6、增加一些中文注释方便我们进行理解


现在还是最好检查一下感知到分拣 我们要让程序有更强的复用性

 