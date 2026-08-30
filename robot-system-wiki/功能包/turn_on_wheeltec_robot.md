---
tags: [核心包, 底盘, bringup]
---

# turn_on_wheeltec_robot

## 定位

整个项目的核心集成包，负责底盘串口节点、车型选择、机器人模型、TF、位姿融合，以及建图导航的统一启动入口。

## 关键入口

- `launch/turn_on_wheeltec_robot.launch`：底盘与模型总入口。
- `launch/include/base_serial.launch`：串口底盘节点。
- `launch/wheeltec_lidar.launch`：雷达型号选择。
- `launch/wheeltec_camera.launch`：相机型号选择。
- `launch/mapping.launch`：2D 建图。
- `launch/navigation.launch`：2D 导航。
- `src/wheeltec_robot.cpp`：底盘核心实现。

## 关键接口

| 方向 | 接口 | 作用 |
|---|---|---|
| 订阅 | `/cmd_vel` | 接收线速度和角速度 |
| 发布 | `/odom` | 发布轮式里程计 |
| 发布 | `/imu` | 发布 IMU |
| 发布 | `/PowerVoltage` | 发布电池电压 |

## 精读顺序

1. 构造函数中的参数、Publisher 和 Subscriber。
2. `Cmd_Vel_Callback`。
3. 串口发送协议。
4. 串口接收、校验与解析。
5. 里程计积分和协方差。
6. IMU 消息与 TF。

## 关联

- [[分类/01-底盘与控制]]
- [[分类/03-建图定位与导航]]
- [[04-系统数据流]]

