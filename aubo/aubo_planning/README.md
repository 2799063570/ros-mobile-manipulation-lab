# AUBO Planning

该包提供 AUBO i5 + 双指夹爪的 MoveIt 1 C++ 接口示例：

- `gripper_control_demo`：按 MoveIt 命名状态执行夹爪打开、闭合或循环动作。
- `pick_place_demo`：向 PlanningScene 加入工作台和物体，使用 MoveIt `pick()` / `place()` 完成抓取与放置。
- `octomap_planning_demo`：等待深度点云生成非空 OctoMap，再进行带环境碰撞检测的机械臂规划。

## 深度相机 OctoMap 避障示例

完整 Gazebo 示例只需一个入口：

```bash
roslaunch aubo_planning octomap_planning_gazebo.launch
```

场景中的灰色工作台和橙色立柱没有通过代码直接加入 PlanningScene；它们由固定的
眼在手外 `workspace_camera` 观测。MoveIt 订阅
`/workspace_camera/depth/color/points`，经 TF 变换和机器人自过滤后生成 4 cm
OctoMap体素。启动入口以 100 Hz 中继 Gazebo 关节状态，并允许 MoveIt 最多等待
0.25 s 获取与点云时间戳匹配的机器人 TF，避免相机与控制器更新周期不同导致的
毫秒级外推错误。

眼在手外仿真使用不含手部相机的 `aubo_i5.xacro`，因此不会同时发布腕部相机数据；
固定相机无需机械臂移动到 `observe` 姿态。程序确认收到点云和非空八叉树后直接规划
到 `home`。仿真默认执行轨迹；只看规划可使用：

```bash
roslaunch aubo_planning octomap_planning_gazebo.launch execute:=false
```

在 RViz 的 MotionPlanning 显示中勾选 `Scene Geometry`，即可看到八叉树障碍物。
也可以用下面的命令检查数据链路：

```bash
rostopic hz /workspace_camera/depth/color/points
rostopic echo -n 1 /move_group/filtered_cloud
rosservice call /clear_octomap
```

### RealSense + 真机

先启动相机，再让真机 MoveIt 加载 OctoMap 更新器。真实机械臂示例默认只规划，
请先在 RViz 检查点云、TF、八叉树和轨迹，确认安全后才传入 `execute:=true`：

```bash
roslaunch realsense2_camera rs_camera.launch \
  align_depth:=true enable_pointcloud:=true publish_tf:=false
roslaunch aubo_ros_control aubo_real_bringup.launch \
  robot_ip:=192.168.1.2 use_sensor_manager:=true
roslaunch aubo_planning octomap_planning_demo.launch \
  move_to_sensor_pose:=false execute:=false target_pose:=home
```

眼在手外真机应使用 `sensors_3d_eye_to_hand.yaml` 和外部相机点云
`/workspace_camera/depth/color/points`。眼在手上通用入口仍使用 `sensors_3d.yaml` 和
`/camera/depth/color/points`。两者的点云
`frame_id` 都必须能通过 TF 变换到 `base_link`。

## 编译

在包含本仓库源码的 catkin 工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

## 快速验证（MoveIt fake controller）

```bash
roslaunch aubo_planning gripper_control.launch command:=cycle
roslaunch aubo_planning pick_place.launch
```

单独控制夹爪时，`command` 可设为 `open`、`close` 或 `cycle`。

## Gazebo 或真机已有 MoveIt 时

先启动机器人、控制器和 `move_group`，再关闭示例 launch 对 MoveIt demo 的重复启动：

```bash
roslaunch aubo_planning gripper_control.launch start_demo:=false command:=open
roslaunch aubo_planning pick_place.launch start_demo:=false
```

抓取点和放置点可通过 launch 参数调整：

```bash
roslaunch aubo_planning pick_place.launch \
  object_x:=0.45 object_y:=-0.15 object_z:=0.15 \
  place_x:=0.45 place_y:=0.15 place_z:=0.15
```

## 机构参数

- `joint1`、`joint2` 的零位为张开，`0.45 rad` 为默认闭合状态。
- URDF 安全范围暂设为 `0.0–0.55 rad`，速度上限为 `1.0 rad/s`。
- 两个关节使用同一正方向命令；两侧关节坐标系的镜像朝向使手指对称闭合。
- 真机运行前应低速标定零位、闭合角和最大无干涉角，并同步修改
  `jiazhua.urdf`、`aubo_i5.srdf` 以及抓取程序的 `gripper_closed` 参数。
- MoveIt 和抓取示例使用 `tcp_link` 作为末端链接；它位于
  `gripper_base_link` 局部 Z 轴前方 `0.145 m`，约在指尖中心稍外侧。
- `grasp_offset_z` 默认为 `0.12 m`，表示俯抓时物体中心到 TCP 目标位姿的
  竖直偏移；若更换指尖或 TCP，需要同步标定该值。
- 当前仓库已配置 ros_control transmission、Gazebo 控制器和 MoveIt 控制器映射。
  真机驱动仍须把 `joint1`、`joint2` 注册为 PositionJointInterface，并把轨迹命令
  转换为实际夹爪电机协议；仅有 YAML 配置不会自动驱动物理电机。

## 运行前检查

```bash
rostopic echo /aubo_i5/joint_states
rostopic list | grep gripper_controller
rosservice call /aubo_i5/controller_manager/list_controllers
```

控制器正常时，应看到 `aubo_i5_controller` 和 `gripper_controller` 均为 `running`。
