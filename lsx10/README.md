# lslidar_x10

## 功能说明

`lslidar_x10` 是镭神智能 M10、M10_GPS、M10_P、M10_PLUS 和 N10 激光雷达的 Linux ROS 驱动。上游功能包曾在 Ubuntu 20.04 环境中测试。

## 编译与启动

这是一个标准 Catkin 功能包。将其放入工作空间并确认该工作空间位于 `ROS_PACKAGE_PATH` 后执行：

```bash
cd <工作空间>
catkin_make
source devel/setup.bash

# 网口版雷达
roslaunch lslidar_x10_driver lslidar_x10_net.launch

# 串口版雷达
roslaunch lslidar_x10_driver lslidar_x10_serial.launch
```

通过控制话题开启或关闭雷达：

```bash
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 1"  # 开启
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 0"  # 关闭
```

查看设备信息：

```bash
rostopic echo /difop_information
```

## 节点接口

### `lslidar_m10_driver`

参数：

- `device_ip`（字符串，默认 `192.168.1.222`）：雷达 IP 地址。
- `frame_id`（字符串，默认 `lslidar`）：输出消息使用的坐标系 ID。

发布话题：

- `lslidar_packets`（`lslidar_m10_msgs/LslidarM10Packet`）：设备通过以太网发送的原始雷达数据包，每条消息对应一个数据包。

### `lslidar_m10_decoder`

参数：

- `min_range`（浮点数，默认 `0.3` 米）和 `max_range`（浮点数，默认 `100.0` 米）：删除范围以外的点。
- `frequency`（默认 `20.0` Hz）：期望扫描频率；驱动不会主动改变传感器自身频率。
- `publish_point_cloud`（布尔值，默认 `false`）：设为 `true` 后额外发布每圈扫描的局部点云。

发布话题：

- `lslidar_sweep`（`lslidar_m10_msgs/LslidarM10Sweep`）：按扫描索引和方位角组织的一圈点数据。
- `lslidar_point_cloud`（`sensor_msgs/PointCloud2`）：仅在 `publish_point_cloud:=true` 时发布。

以下启动文件会同时启动驱动和解码器，通常只需运行这一项：

```bash
roslaunch lslidar_x10_driver lslidar_x10_net.launch
```

## 问题反馈

建议优先在上游仓库提交 Issue，也可发送邮件至 `shaohuashu@lslidar.com`。

## 版本记录

- V0.1：每圈 2000 个点。
- V0.2：每圈 1000 个点。
