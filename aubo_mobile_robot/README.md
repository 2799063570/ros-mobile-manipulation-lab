# AUBO 复合移动机器人功能包集合

该目录用于统一存放 AUBO 复合移动机器人相关的 ROS 功能包。外层目录没有
`package.xml`，它只是功能包集合，因此 catkin 会继续发现下面的各个功能包。

## 当前目录结构

```text
aubo_mobile_robot/
├── aubo_mobile_robot/          # 机器人 Xacro、Gazebo、传感器及控制器
├── aubo_mobile_moveit_config/  # MoveIt 运动规划配置
├── aubo_mobile_navigation/     # 建图、定位与 move_base 导航
├── aubo_mobile_control/        # 键盘控制及导航/机械臂协同
├── aubo_mobile_perception/     # 基于 OpenCV 的手部相机目标定位
├── aubo_mobile_sorting/        # Gazebo 视觉抓取与颜色分拣
└── aubo_mobile_nav_sorting/    # 建图、导航到工位并自动分拣的场景任务
```

## 后续推荐结构

```text
aubo_mobile_robot/
├── aubo_mobile_robot/          # 核心机器人模型与 Gazebo 仿真
├── aubo_mobile_moveit_config/  # MoveIt Setup Assistant 生成的配置
├── aubo_mobile_navigation/     # 建图、定位及 move_base 参数
├── aubo_mobile_bringup/        # 仿真与真实机器人统一启动入口
├── aubo_mobile_control/        # 复合控制与指令分发
├── aubo_mobile_perception/     # 相机和激光雷达感知
└── aubo_mobile_sorting/        # 抓取、放置及分拣任务
```

AUBO 机械臂的通用网格模型和原有机械臂功能包继续保留在顶层 `aubo/` 目录中。
移动机器人相关功能包应通过依赖复用这些资源，不要重复复制模型文件。
