# 故障排查

## 1. 找不到功能包或节点

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rospack find simple_diff_robot_gazebo
rosdep install --from-paths ~/catkin_ws/src --ignore-src -r -y
```

确认安装了 `gazebo_ros_pkgs`、`gmapping`、Navigation Stack 和 `xacro`。键盘节点已包含在本包中。

## 2. Gazebo 中没有机器人

检查 `/robot_description`、生成节点和 Gazebo 日志：

```bash
rosparam get /robot_description | head
rosnode info /spawn_simple_diff_robot
rostopic echo -n 1 /gazebo/model_states
```

模型默认以 `z=0.01` 生成。不要把高度调得过高，否则机器人落地时可能抖动或侧翻。

## 3. 键盘控制无反应

```bash
rostopic info /cmd_vel
rostopic echo /cmd_vel
rosnode info /gazebo
```

键盘节点需要终端保持焦点。确认 `/cmd_vel` 有数据，Gazebo 未暂停，并且插件成功加载 `libgazebo_ros_diff_drive.so`。

## 4. 没有雷达或相机数据

```bash
rostopic hz /scan
rostopic hz /camera/image_raw
rosrun tf tf_echo base_link laser_link
rosrun tf tf_echo base_link camera_optical_frame
```

缺少话题时查看 Gazebo 是否成功加载 `libgazebo_ros_laser.so` 和 `libgazebo_ros_camera.so`。

## 5. Gmapping 不出地图

必须同时满足：

- `/scan` 持续发布；
- `/odom` 持续发布；
- `odom → base_footprint → laser_link` TF 连通；
- 机器人发生足够的平移或旋转。

```bash
rosrun tf tf_echo odom laser_link
rostopic hz /scan
rostopic hz /odom
rosnode info /slam_gmapping
```

## 6. 地图保存失败

包安装目录可能不可写。改用用户目录：

```bash
mkdir -p ~/maps
roslaunch simple_diff_robot_gazebo map_saver.launch map_name:=$HOME/maps/my_map
```

## 7. 导航启动后立即退出

默认需要 `maps/my_map.yaml` 和其中引用的 PGM 文件。检查：

```bash
ls -l $(rospack find simple_diff_robot_gazebo)/maps
```

或显式传入地图：

```bash
roslaunch simple_diff_robot_gazebo navigation.launch map_file:=$HOME/maps/my_map.yaml
```

## 8. RViz 报 TF 错误

导航时应存在：

```text
map → odom → base_footprint → base_link → laser_link/camera_link
```

```bash
rosrun tf view_frames
rosrun tf tf_echo map base_footprint
rosrun tf tf_echo odom base_footprint
```

`map → odom` 由 Gmapping 或 AMCL 发布；两者不应在同一导航会话中同时运行。

## 9. move_base 不动或振荡

依次检查：

1. 是否用 **2D Pose Estimate** 设置了正确初始位姿；
2. 激光是否与地图重合；
3. 局部代价地图是否把机器人自身标成障碍；
4. `/move_base/DWAPlannerROS/local_plan` 是否存在；
5. `/cmd_vel` 是否由 `move_base` 发布。

```bash
rostopic echo /move_base/status
rostopic info /cmd_vel
rosservice call /move_base/clear_costmaps
```

## 9.1 导航时局部地图与雷达出现旋转偏移

现象：转弯/旋转过程中，RViz 里 local costmap 与 LaserScan 的角度逐渐错开，直线行驶不明显。

原因：local costmap 建在 `odom` 帧并随时间累积障碍物，使用的是**每一帧当时的 TF**；LaserScan 是实时数据，用**当前** TF 显示。当 odom 航向发生漂移（编码器里程计误差 / 车轮打滑 / TF 延迟）时，历史栅格停在原位，实时雷达被"更新更偏"的 TF 放过去，就出现旋转错位。AMCL 只修正 `map→odom`，管不到 odom 帧内的 local costmap。

本仓库为演示模式已做如下处理（见 `urdf/simple_diff_robot.xacro` 与 `config/dwa_local_planner.yaml`）：

- `<odometrySource>world</odometrySource>`：里程计改用 Gazebo 真值，消除编码器漂移（根治）；
- 驱动轮摩擦 `mu1/mu2` 提高到 `2.0`：减少打滑；
- DWA `acc_lim_x: 0.4`、`acc_lim_theta: 1.0`、`max_vel_theta: 1.0`：运动更平滑。

若想恢复"更贴近真实机器人的编码器里程计"（用于学习 SLAM），把 xacro 中 `odometrySource` 改回 `encoder` 并降低轮子摩擦即可；届时轻微的局部地图/雷达角度错位属正常现象。

## 10. 当前验证边界

仓库中的 XML、Xacro、YAML 和 RViz 配置可进行静态语法检查；完整物理效果和插件兼容性仍需在 Ubuntu 18.04、ROS Melodic 与对应 Gazebo 版本中启动验证。
