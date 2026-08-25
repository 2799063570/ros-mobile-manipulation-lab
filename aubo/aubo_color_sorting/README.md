# AUBO 固定机械臂颜色抓取分拣

`aubo_color_sorting` 是不带移动底盘的 AUBO i5 颜色分拣功能包。它使用腕部 RGB
相机识别红、绿、蓝方块，通过 MoveIt 规划抓取和放置轨迹，并在 Gazebo 中用世界
插件提高小物体夹持的稳定性。

## 场景约束

- 机械臂底座固定在世界原点，不加载底盘、里程计或导航节点。
- 桌面高度为 `0.10 m`，桌体中心为 `[0.70, 0.0, 0.0]`，尺寸为
  `0.70 x 1.00 x 0.20 m`。
- `upperArm_joint` 的 URDF 与 MoveIt 位置范围均为
  `[-1.0471976, 1.0471976] rad`，即 `[-60 deg, 60 deg]`。
- 启动时任务节点还会读取 `/robot_description` 和 MoveIt 关节限制做一次一致性检查；
  如果仍加载旧模型，任务进入 `ERROR`，不会执行运动。

## 构建与启动

在工作空间根目录构建并加载环境后运行：

```bash
catkin_make
source devel/setup.bash
roslaunch aubo_color_sorting sorting_gazebo.launch
```

启动完成后，机械臂先进入 `observe` 观察姿态。确认调试图像和目标位置正常，再开始：

```bash
rosservice call /sorting/start
```

也可以启动后自动执行：

```bash
roslaunch aubo_color_sorting sorting_gazebo.launch auto_start:=true
```

无界面运行时可关闭 Gazebo、RViz 和图像窗口：

```bash
roslaunch aubo_color_sorting sorting_gazebo.launch \
  gui:=false rviz:=false debug_view:=false
```

## 主要接口

| 接口 | 用途 |
| --- | --- |
| `/camera/color/image_raw` | 腕部 RealSense 彩色图像 |
| `/sorting/debug_image` | 带颜色框和坐标的调试图像 |
| `/sorting/detections` | 目标颜色及 `base_link` 坐标 |
| `/sorting/state` | 状态机状态 |
| `/sorting/start` | 开始一轮红、绿、蓝分拣 |
| `/sorting/stop` | 停止当前任务 |
| `/sorting/move_to_observation` | 回到相机观察姿态 |
| `/sorting/open_gripper` | 打开夹爪 |
| `/sorting/home` | 回到 `down` 姿态 |

颜色阈值、工作区和投影高度在 `config/colors.yaml` 中修改；抓取高度、放置点、速度
和桌面碰撞体在 `config/sorting.yaml` 中修改。若调整桌面高度，必须同步修改这两个配置
以及 `worlds/sorting.world`，避免视觉投影面、MoveIt 碰撞体和 Gazebo 实体不一致。

## 接入真实机械臂

真实机械臂由 `aubo_ros_control` 和真实相机驱动提供控制器、关节状态及相机话题后，
只启动算法节点：

```bash
roslaunch aubo_color_sorting sorting.launch use_grasp_attachment:=false
```

首次在真实设备运行时应保持 `auto_start:=false`，降低速度比例，并先检查相机外参、
桌面高度、抓取偏移和所有规划轨迹。Gazebo 的吸附插件不能用于真实机械臂。
