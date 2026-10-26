# D435i 实机调试记录（2026-10-26）

## 环境与结论

- 设备：Intel RealSense D435I，序列号 `335222076553`，固件 `05.17.00.10`。
- Ubuntu 20.04，内核 `5.15.0-139-generic`，ROS Noetic。
- USB：经 Hub 连接，实测链路速率 5000M，SDK 报告 USB 3.2。
- 原 ROS 包：`/opt/ros/noetic/share/realsense2_camera`，版本 2.3.2，链接 SDK 2.50.0。
- 最终驱动：官方 ROS 驱动 2.3.2 源码重新链接 SDK 2.56.5，保存在独立工作区
  `/home/zlab/aubo/ros_mobile_manipulation_lab/realsense_noetic_ws`。
  SDK 使用 V4L2/HID 后端；原系统库、固件和设备标定均未覆盖。
- 仓库内的 `realsense-ros-development` 是 2.2.18，目前不在 catkin 构建白名单中。
- 调试使用独立 master `http://localhost:11319`，未启动机械臂控制。

RGB、RGB 对齐深度、点云、IMU 和现有 YOLO → RGB-D 几何程序已经通过实机验证。
采集入口默认关闭感知程序不需要的 IMU；需要时使用 README 中的完整启动命令。

## 实测

参数：RGB 与深度均为 640×480、30 Hz，`align_depth=true`、`enable_sync=true`、
`enable_pointcloud=true`、`enable_imu=true`、`initial_reset=true`。

最终 20 秒采样结果（同时运行 YOLO）：

| 数据 | 消息数 | 实测频率 |
| --- | ---: | ---: |
| 彩色图像 | 600 | 29.98 Hz |
| 对齐深度 | 600 | 29.98 Hz |
| 彩色内参 | 600 | 29.98 Hz |
| 彩色点云 | 600 | 29.98 Hz |
| 合并 IMU | 8007 | 400.28 Hz |

RGB 和对齐深度时间戳差的 P95 为 0 ms，均使用 `camera_color_optical_frame`。
当前场景有效深度像素占比 96.66%，非零深度中位数 0.906 m；这些值取决于当前视野，
并非相机测距精度指标。深度编码 `16UC1`，程序按毫米转换为米。

静止检查：加速度模长均值 9.84386 m/s²，角速度模长均值 0.00335034 rad/s。
检查脚本增加 `--stationary`，用于发现“话题频率正常但 IMU 量级异常”的情况。

用户放置易拉罐后，最终 10 秒收到 85 次 YOLO 输出和 85 次三维目标位置。
`can` 最高置信度 0.9040，最后一帧目标位置约 `(0.07094, 0.03146, 0.84100)` m，
坐标轴为光学坐标系的右、下、前。模型还输出了 `bottle` 类别；调试几何节点仅选择 `can`。
该位置是可见表面点，尚未验证机器人基座坐标、手眼外参或抓取中心。

## 根因与剩余限制

原 ROS SDK 2.50.0 的 RSUSB 后端没有持续输出 IMU。改用同版本系统内核后端并重置后，
虽收到约 400 Hz 数据，但静止时陀螺仪出现约 1–2 rad/s 的异常量级。仅检查话题频率
不足以判断驱动正常，因此没有采用该组合，也没有给角速度硬编码缩放补丁。

固件从 5.16 起调整了 IMU 数据格式，旧 SDK 不兼容；5.17.0.10 随 SDK 2.56.5 发布。
官方说明见 [SDK 发布记录](https://github.com/realsenseai/librealsense/wiki/Release-Notes#release-2551)。
最终使用与固件配套的 SDK 2.56.5 重编译 ROS 驱动，静止量级和完整感知链路均通过。

当前仍有 `IMU Calibration is not available` 告警，SDK 使用默认内外参，见
[官方标定解析源码](https://github.com/realsenseai/librealsense/blob/master/src/ds/ds-calib-parsers.cpp)。
最后一次启动还记录过一次 `Motion Module failure` 通知，之后 20 秒采样没有断流，
静止量级检查通过。需继续关注长时间运行及设备标定，不能把本次验收当作 IMU 精密标定
或视觉惯性融合精度验证；硬件时钟复位、相机/IMU 同步精度也尚未量测。

## 独立驱动构建

`realsense_noetic_ws` 是本仓库外的本机依赖工作区，源码、SDK 和构建结果都保存在该目录。
通过它的 `devel/setup.bash` 加载新驱动和原主工作区；仅加载原主工作区会回到旧驱动。
不需要修改主工作区的构建白名单，也没有修改 `.bashrc` 或系统 APT 安装。

依赖来源：

- ROS 驱动官方 tag `2.3.2`，归档 SHA256：
  `18c0f90eeea2b64889388e2e441221931e220d1fea06fe6eff8d70442c456459`。
- SDK 官方 APT 包 `2.56.5-0~realsense.17053`，下载前在临时目录验证了官方源签名。
  运行库包 SHA256：`3d7102572f70ab0221f1c6340ec3f1390e51683672105e092cc787fe87e668ee`；
  开发包 SHA256：`a937b2c8e55032327a5d7b7c600be2262520129dc3343f5b9089c21b18f5108a`。
- SDK 解包到 `sdk/usr`。`cmake/realsense2` 只引用共享库及匹配头文件，避免官方包导出的
  未使用 DDS 静态目标缺少 fastcdr/fastrtps 导致 CMake 失败；ROS 驱动源码未修改。

本机重新构建命令：

```bash
source /home/zlab/aubo/ros_mobile_manipulation_lab/devel/setup.bash
catkin_make -C /home/zlab/aubo/ros_mobile_manipulation_lab/realsense_noetic_ws \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DEMPY_SCRIPT=/home/zlab/.local/bin/empy \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCATKIN_ENABLE_TESTING=OFF \
  -Drealsense2_DIR=/home/zlab/aubo/ros_mobile_manipulation_lab/realsense_noetic_ws/cmake/realsense2 -j4
```

## 代码验证

感知包 38 项单元/launch 测试、真机分拣入口 4 项 launch 测试通过。
两个机器人相机入口继续禁用驱动 TF，沿用各自的安装外参来源。
新增独立采集入口发布相机内部 TF；纯感知调试输出位于 `/camera_debug`。

启动及复测命令见 [README.md](README.md#d435i-真机采集与识别调试)。
原始验收数据见 [validation/d435i_20261026.json](validation/d435i_20261026.json)。
