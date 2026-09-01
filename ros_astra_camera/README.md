# astra_camera

这是 Orbbec 3D 相机的 ROS 驱动，支持 ROS Kinetic 和 Melodic。

## 安装

1. 安装 [ROS](http://wiki.ros.org/ROS/Installation)。
2. 安装依赖：

   ```bash
   sudo apt install ros-$ROS_DISTRO-rgbd-launch \
     ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera \
     ros-$ROS_DISTRO-libuvc-ros
   ```

3. 创建 Catkin 工作空间（已有工作空间可跳过）：

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. 获取源码、创建 udev 规则并编译：

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/orbbec/ros_astra_camera
   roscd astra_camera
   ./scripts/create_udev_rules
   cd ~/catkin_ws
   catkin_make --pkg astra_camera
   ```

### 深度滤波

驱动提供普通模式和滤波模式。滤波可提高深度数据精度，但会占用更多计算资源，嵌入式平台建议使用普通模式。通过以下参数选择：

```bash
catkin_make --pkg astra_camera -DFILTER=OFF  # 或 FILTER=ON
```

该选项计划弃用，建议优先参考上游 `master` 分支的实现。

## 启动相机

新终端若未在 `.bashrc` 中加载工作空间，应先执行 `source <工作空间>/devel/setup.bash`。

```bash
# Astra
roslaunch astra_camera astra.launch

# Astra Stereo S（使用 UVC）
roslaunch astra_camera stereo_s.launch
```

可使用 RViz 或 `image_view` 检查输出。

## 重要话题

- `*/image_raw`：深度、RGB 或红外原始图像。显示红外图像时，建议把 16 位数据归一化到 0～255。
- `*/image_rect_raw`：根据内外参校正后的图像。
- `*/camera_info`：相机内外参。
- `/camera/depth/points`：不含颜色的点云。
- `/camera/depth_registered/points`：XYZRGB 点云。

## 常用服务

该驱动通过 [ROS 服务](http://wiki.ros.org/Services)读取设备信息和修改设置：

- `/camera/get_device_type`、`/camera/get_serial`：设备类型和序列号。
- `/camera/get_ir_exposure`、`/camera/set_ir_exposure`、`/camera/reset_ir_exposure`：红外曝光。
- `/camera/get_ir_gain`、`/camera/set_ir_gain`、`/camera/reset_ir_gain`：红外增益。
- `/camera/get_uvc_exposure`、`/camera/set_uvc_exposure`：RGB 曝光；设置为 `0` 表示自动模式。
- `/camera/get_uvc_gain`、`/camera/set_uvc_gain`：RGB 增益。
- `/camera/get_uvc_white_balance`、`/camera/set_uvc_white_balance`：白平衡；设置为 `0` 表示自动模式。
- `/camera/set_laser`、`/camera/set_ldp`、`/camera/set_ir_flood`：激光、LDP 和红外补光开关。
- `/camera/switch_ir_camera`：Stereo S 系列左右红外相机切换。

调用示例：

```bash
rosservice call /camera/get_ir_exposure
rosservice call /camera/set_ir_exposure "{exposure: 50}"
rosservice call /camera/set_laser "{enable: true}"
rosservice call /camera/switch_ir_camera "camera: 'left'"
```

## 多相机

按实际设备修改 `multi_astra.launch` 中的 `device_x_id`（序列号）、`3d_sensor`（启动文件名）和 `has_uvc_serial`（UVC 是否有序列号）。如果出现 USB 缓冲区错误，可适当增大 USBFS 缓冲区：

```bash
echo 64 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

必要时可把 `64` 调整为 `128`。

## 许可证

Copyright 2019 Orbbec Ltd. 本项目采用 Apache License 2.0，完整条款见 [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0)。其他名称和品牌可能归各自所有者所有。
