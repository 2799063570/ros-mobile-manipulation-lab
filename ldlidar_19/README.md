# LDROBOT LD06/LD19 ROS 驱动

> 此 SDK 仅适用于深圳乐动机器人有限公司销售的 LDROBOT LiDAR LD06 和 LD19 激光雷达。

## 0. 获取 ROS 功能包

```bash
cd ~
mkdir -p ldlidar_ros_ws/src
cd ldlidar_ros_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros.git
# 国内镜像：git clone https://gitee.com/ldrobotSensorTeam/ldlidar_stl_ros.git
```

## 1. 系统设置

1. 通过板载串口或 USB 转串口模块（如 CP2102）把雷达连接到计算机。
2. 根据实际挂载设备设置串口权限。可先用 `ls -l /dev` 查看设备；以下以 `/dev/ttyUSB0` 为例：

   ```bash
   sudo chmod 777 /dev/ttyUSB0
   ```

3. 修改 `launch/` 目录中对应型号启动文件的 `port_name`。`ld06.launch` 的关键配置如下：

   ```xml
   <node name="LD06" pkg="ldlidar_stl_ros"
         type="ldlidar_stl_ros_node" output="screen">
     <param name="product_name" value="LDLiDAR_LD06"/>
     <param name="topic_name" value="scan"/>
     <param name="port_name" value="/dev/ttyUSB0"/>
     <param name="frame_id" value="base_laser"/>

     <!-- true 表示逆时针扫描，false 表示顺时针扫描。 -->
     <param name="laser_scan_dir" type="bool" value="true"/>

     <!-- 是否启用角度裁剪。裁剪区间内的距离和强度会被置为 0。 -->
     <param name="flag_parted" type="bool" value="false"/>
     <param name="angle_crop_min" type="double" value="135.0"/>
     <param name="angle_crop_max" type="double" value="225.0"/>
   </node>

   <!-- 发布 base_link 到 base_laser 的静态 TF。参数依次为：
        x y z yaw pitch roll 父坐标系 子坐标系 周期（毫秒）。 -->
   <node name="base_to_laser" pkg="tf" type="static_transform_publisher"
         args="0.0 0.0 0.18 0 0.0 0.0 base_link base_laser 50"/>
   ```

## 2. 编译

```bash
cd ~/ldlidar_ros_ws
catkin_make
```

## 3. 运行

### 3.1 加载工作空间

当前终端临时加载：

```bash
cd ~/ldlidar_ros_ws
source devel/setup.bash
```

如需每次打开终端时自动加载：

```bash
echo "source ~/ldlidar_ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.2 启动 LD06

```bash
# 仅启动雷达节点
roslaunch ldlidar_stl_ros ld06.launch

# Kinetic 或 Melodic：同时在 RViz 中显示
roslaunch ldlidar_stl_ros viewer_ld06_kinetic_melodic.launch

# Noetic：同时在 RViz 中显示
roslaunch ldlidar_stl_ros viewer_ld06_noetic.launch
```

### 3.3 启动 LD19

```bash
# 仅启动雷达节点
roslaunch ldlidar_stl_ros ld19.launch

# Kinetic 或 Melodic：同时在 RViz 中显示
roslaunch ldlidar_stl_ros viewer_ld19_kinetic_melodic.launch

# Noetic：同时在 RViz 中显示
roslaunch ldlidar_stl_ros viewer_ld19_noetic.launch
```

## 4. 验证

该驱动已在 Ubuntu 16.04 + ROS Kinetic、Ubuntu 18.04 + ROS Melodic 和 Ubuntu 20.04 + ROS Noetic 环境下测试。启动 RViz：

```bash
rviz
```

| 产品型号 | Fixed Frame | 话题 |
| --- | --- | --- |
| LDROBOT LiDAR LD06 | `base_laser` | `/scan` |
| LDROBOT LiDAR LD19 | `base_laser` | `/scan` |
