# Intel RealSense 设备的 ROS 驱动

本目录包含 D400 系列、SR300 和 T265 跟踪模块的 ROS 1 功能包。ROS 2 用户应使用对应的 [ROS 2 分支](https://github.com/IntelRealSense/realsense-ros)。当前快照支持的 librealsense 版本为 v2.39.0，详情见 [`realsense2_camera` 发布记录](https://github.com/IntelRealSense/realsense-ros/releases)。

## 安装

以下步骤适用于 Ubuntu 16.04 + ROS Kinetic，也可将发行版名称替换为 `melodic`，用于 Ubuntu 18.04 + ROS Melodic。Windows 10 请参考 [ROS Windows 安装文档](https://wiki.ros.org/Installation/Windows)。

### 方法一：安装 ROS 二进制包

```bash
export ROS_VER=melodic  # Kinetic 环境改为 kinetic
sudo apt-get install ros-$ROS_VER-realsense2-camera
sudo apt-get install ros-$ROS_VER-realsense2-description
```

该方式会安装 `realsense2_camera`、依赖项和 librealsense2。ROS 仓库中的 librealsense2 版本通常落后于 RealSense 官方仓库，且部分发行包使用兼容性更广但稳定性略低的 RS-USB 后端。`realsense2_description` 包含设备三维模型，运行带模型的启动文件时必须安装。

### 方法二：安装官方 SDK 并从源码编译

1. 根据 [librealsense Linux 安装说明](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)安装 RealSense SDK 2.0、`librealsense2-dev` 和 `librealsense2-dkms`，或从 [v2.39.0 源码](https://github.com/IntelRealSense/librealsense/releases/tag/v2.39.0)编译。Windows 可使用 `vcpkg install realsense2:x64-windows`。
2. 创建工作空间并获取 ROS 驱动：

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone https://github.com/IntelRealSense/realsense-ros.git
   cd realsense-ros
   git checkout "$(git tag | sort -V | grep -P '^2\.\d+\.\d+' | tail -1)"
   ```

3. 确保安装 `ddynamic_reconfigure`。如果软件源中没有，可把 [0.2.2 版本源码](https://github.com/pal-robotics/ddynamic_reconfigure/tree/kinetic-devel)放入工作空间。
4. 编译并加载环境：

   ```bash
   cd ~/catkin_ws/src
   catkin_init_workspace
   cd ..
   catkin_make clean
   catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
   catkin_make install
   source devel/setup.bash
   ```

## 使用方法

### 启动相机

```bash
roslaunch realsense2_camera rs_camera.launch
```

启动文件会打开可用传感器并发布相应 ROS 话题。分辨率和帧率可通过启动参数调整。

### 常见话题

实际话题取决于设备型号和启动参数。D435i 的常见输出包括：

- `/camera/color/camera_info`、`/camera/color/image_raw`
- `/camera/depth/camera_info`、`/camera/depth/image_rect_raw`
- `/camera/infra1/image_rect_raw`、`/camera/infra2/image_rect_raw`
- `/camera/gyro/sample`、`/camera/accel/sample`
- `/camera/extrinsics/depth_to_color`
- `/diagnostics`

用 `rostopic list` 查看完整列表。默认前缀 `/camera` 可以修改；D415/D435 等不带 IMU 的型号不会发布陀螺仪和加速度计话题。

### 主要启动参数

- `serial_no`、`usb_port_id`、`device_type`：分别按序列号、USB 端口或型号正则表达式选择设备。
- `rosbag_filename`：从 rosbag 文件回放并发布数据。
- `initial_reset`：启动前复位异常退出的设备。
- `align_depth`：发布对齐到其他图像流的深度图，如 `/camera/aligned_depth_to_color/image_raw`。
- `filters`：以逗号分隔启用 `colorizer`、`pointcloud`、`disparity`、`spatial`、`temporal`、`hole_filling`、`decimation` 等滤波器。
- `enable_sync`：同步不同传感器中时间最接近的帧；点云等滤波器启用时会自动同步。
- `<stream>_width`、`<stream>_height`、`<stream>_fps`：设置各数据流格式；不支持的组合不会发布。
- `enable_<stream>`：启用或禁用 `infra1`、`infra2`、`color`、`depth`、`fisheye`、`gyro`、`accel`、`pose` 等数据流。
- `tf_prefix`、`base_frame_id`、`odom_frame_id`：配置 TF 坐标系名称。
- `unite_imu_method`：以 `linear_interpolation` 或 `copy` 方式融合加速度计与陀螺仪，并发布统一的 `imu` 话题。
- `clip_distance`：删除超过指定距离（米）的深度值，负数表示禁用。
- `linear_accel_cov`、`angular_velocity_cov`：设置 IMU 测量协方差。
- `hold_back_imu_for_frames`：图像处理期间暂存 IMU 消息，以保持发布顺序与采集顺序一致。
- `topic_odom_in`、`calib_odom_file`：为 T265 输入轮式里程计及其标定文件。
- `publish_tf`、`tf_publish_rate`、`publish_odom_tf`：控制 TF 是否发布及发布频率。

全部坐标系参数见 [`nodelet.launch.xml`](./realsense2_camera/launch/includes/nodelet.launch.xml)，后处理滤波说明见 [librealsense 文档](https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md)。

### 点云、深度对齐和动态参数

```bash
# 发布点云
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

# 发布对齐深度图
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# 打开相机动态参数界面
rosrun rqt_reconfigure rqt_reconfigure
```

点云默认仅覆盖深度与纹理视场重叠的区域。设置 `allow_no_texture_points:=true` 可保留无纹理区域，并用零值着色。

### 多相机

当前快照不支持同时启动多台 T265。其他型号可使用：

```bash
roslaunch realsense2_camera rs_multiple_devices.launch \
  serial_no_camera1:=<第一台相机序列号> \
  serial_no_camera2:=<第二台相机序列号>
```

获取序列号：

```bash
rs-enumerate-devices | grep Serial
```

也可以在不同终端中用不同命名空间分别启动：

```bash
roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=<序列号1>
roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=<序列号2>
```

## T265

轮式机器人要获得稳定准确的 T265 跟踪结果，应输入轮式里程计。启动命令：

```bash
roslaunch realsense2_camera rs_t265.launch
```

常见输出包括 `/camera/odom/sample`、`/camera/accel/sample`、`/camera/gyro/sample` 和两个鱼眼图像话题。使用以下命令在 RViz 中查看位姿和坐标系：

```bash
roslaunch realsense2_camera demo_t265.launch
```

坐标系分三类：符合 ROS 约定的 `camera_<stream>_frame`、使用设备原始光学约定的 `camera_<stream>_optical_frame`，以及设备参考坐标系 `camera_link`。

## 模型与测试

显示 D415 模型：

```bash
roslaunch realsense2_description view_d415_model.launch
```

单元测试基于预先录制的 bag 文件，准备好测试数据后运行：

```bash
python src/realsense/realsense2_camera/scripts/rs2_test.py --all
```

## 已知限制

- 此快照不支持 ROS Lunar Loggerhead 和 ROS 2。
- 此快照不支持同时运行多台 T265。

## 许可证

Copyright 2018 Intel Corporation。本项目采用 Apache License 2.0，完整条款见 [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0)。其他名称和品牌可能归各自所有者所有。
