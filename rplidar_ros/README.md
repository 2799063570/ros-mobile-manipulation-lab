# RPLIDAR ROS 功能包

该功能包提供 RPLIDAR 的 ROS 节点和测试程序。

## 相关资料

- [ROS Wiki](http://wiki.ros.org/rplidar)
- [RPLIDAR 官网](http://www.slamtec.com/en/Lidar)
- [RPLIDAR SDK](https://github.com/Slamtec/rplidar_sdk)
- [使用教程](https://github.com/robopeak/rplidar_ros/wiki)

## 编译

1. 把本项目放入 Catkin 工作空间的 `src` 目录。
2. 在工作空间根目录运行 `catkin_make`，编译 `rplidarNode` 和 `rplidarNodeClient`。

## 运行

### 在 RViz 中查看

根据雷达型号选择启动文件：

```bash
roslaunch rplidar_ros view_rplidar.launch     # RPLIDAR A1/A2
roslaunch rplidar_ros view_rplidar_a3.launch  # RPLIDAR A3
roslaunch rplidar_ros view_rplidar_s1.launch  # RPLIDAR S1
```

RViz 中应能看到激光扫描结果。

### 使用测试程序查看

先启动对应型号的雷达节点：

```bash
roslaunch rplidar_ros rplidar.launch     # RPLIDAR A1/A2
roslaunch rplidar_ros rplidar_a3.launch  # RPLIDAR A3
roslaunch rplidar_ros rplidar_s1.launch  # RPLIDAR S1
```

再启动客户端：

```bash
rosrun rplidar_ros rplidarNodeClient
```

扫描结果会显示在终端中。A1/A2 与 A3/S1 的主要配置差异是串口波特率 `serial_baudrate`。

## 坐标系

RPLIDAR 坐标系必须按照 `rplidar-frame.png` 所示方向发布。
