# AUBO Planning

该包提供 AUBO i5 + 双指夹爪的 MoveIt 1 C++ 接口示例：

- `gripper_control_demo`：按 MoveIt 命名状态执行夹爪打开、闭合或循环动作。
- `pick_place_demo`：向 PlanningScene 加入工作台和物体，使用 MoveIt `pick()` / `place()` 完成抓取与放置。
- `octomap_planning_demo`：等待深度点云生成非空 OctoMap，再进行带环境碰撞检测的机械臂规划。
- `cartesian_path_demo`：以当前 TCP 位姿为起点，规划直线或圆形笛卡尔路径。
- `publish_camera_stand.py`：把相机支架与相机本体加入 MoveIt 碰撞场景。
- `publish_sorting_box.py`：把桌面收纳盒的底板和四面侧壁加入 MoveIt，保留上方和内部开口。

## 真机桌面收纳盒

`config/sorting_box_reference.yaml` 是待测量的示例，默认所有真机入口均不发布盒子。
量好后复制并修改该 YAML：`outer_size` 为外长、外宽、总高；`wall_thickness` 和
`bottom_thickness` 为壁厚、底板厚度；所有长度单位 m。
`bottom_center` 是**盒子外底面的中心**，不是盒体的体积中心。直接放桌上时，
其 Z 应为所选 `frame_id` 下的桌面高度。`yaw` 为绕该坐标系 +Z 的旋转角，单位 rad。

已有 MoveIt 时单独添加：

```bash
roslaunch aubo_planning publish_sorting_box.launch \
  config:=/absolute/path/to/measured_sorting_box.yaml
```

像相机支架一样，通过真机 bringup 加载：

```bash
roslaunch aubo_ros_control aubo_real_bringup.launch \
  publish_sorting_box:=true \
  sorting_box_config:=/absolute/path/to/measured_sorting_box.yaml
```

固定机械臂实机分拣的 `sorting_real.launch` 和四个 `eye_*_*_sorting_real.launch`
也透传这两个参数：

```bash
roslaunch aubo_sorting eye_to_hand_yolo_sorting_real.launch \
  publish_sorting_box:=true \
  sorting_box_config:=/absolute/path/to/measured_sorting_box.yaml \
  task_config:=/absolute/path/to/measured_sorting_task.yaml
```

盒子使用一个 `sorting_box` 世界碰撞物体，包含五个 BOX primitive；内部不是实心障碍物。
节点通过 `/apply_planning_scene` 发布，并查询 `/get_planning_scene` 确认五个部分均存在，
之后 `/publish_sorting_box/ready` 才变为 true。先在 RViz 的 PlanningScene 中核对实物位置，
再启动分拣。此节点不产生 Gazebo 实体，也不发布机械臂运动命令。

**盒子碰撞模型与分拣放置点分别配置**，不会自动修改 `place_targets` 或放置高度。
在正在使用的完整 `task_config` 中按实际盒内位置调整，例如下列片段对应参考盒中心：

```yaml
place_targets:
  can: [0.60, -0.30]
place_clearance: 0.03
preplace_height: 0.30
lift_min_height: 0.24
```

此片段不能代替完整任务配置，类别键也须与模型一致。放置 XY 应落在盒内，
并为张开夹爪和物体预留空间；旋转盒要按 `yaw` 将盒内点换算到 `place_frame`。
若 `place_frame` 不同于盒子坐标系，还须通过有效 TF 换算。

当前分拣核心使用：

```text
盒内底面 Z = bottom_center.z + bottom_thickness
盒沿 Z     = bottom_center.z + outer_size.z
放置 TCP Z = table_z + object_height/2 + grasp_height_offset + place_clearance
放置前 Z   = table_z + preplace_height
```

`table_z` 仍是实际桌面高度，不能为了盒内放置而改成盒内底高，否则会影响抓取与感知。
在理想抓取偏移保持不变时，物体底面释放高度约为 `table_z + place_clearance`，
因此它应高于盒内底面并留出释放间隙。横移前的 `lift_min_height`、盒上方的
`preplace_height` 还要保证所携带物体的底面及夹爪越过盒沿。
上面的数值只是对应参考尺寸的示例，实际应按物高、夹爪、TCP 和盒子测量值设置。
无法进入开口时应调整几何/放置姿态，不要把盒壁从碰撞检测中排除。

节点退出不会自动清除已发布的碰撞物体。移动盒子后，用新参数重新发布同一个
`object_id` 会替换旧模型；MoveIt 重启后须重新启动发布节点。
关闭发布开关只是不再添加物体，不会删除已存在的对象。

## 真机相机支架碰撞物体

`config/camera_stand_reference.yaml` 记录了参考工作空间
`pub_camera_frame.cpp` 的四个盒体：下支架、立柱、横梁和相机。
其中位置是参考装置数据，不是当前机器的实测值。先复制该文件，
测量每个盒体相对所选 `frame_id` 的中心与尺寸并修改，再开启发布。
固定机械臂可使用 `base_link`；移动机器人旁的固定支架应使用具有
有效 TF 的固定世界坐标系，并在每次底盘换位后核对规划场景中的位置。

已有 `move_group` 时单独发布：

```bash
roslaunch aubo_planning publish_camera_stand.launch \
  config:=/absolute/path/to/measured_camera_stand.yaml
```

独立真机 MoveIt 启动时可一次完成：

```bash
roslaunch aubo_ros_control aubo_real_bringup.launch \
  publish_camera_stand:=true \
  camera_stand_config:=/absolute/path/to/measured_camera_stand.yaml
```

移动机器人真机入口 `aubo_mobile_nav_sorting/navigation_sorting.launch`
和 `aubo_mobile_control/navigation_arm.launch` 也接受相同两个参数。
三个入口默认都不发布支架，以免未经测量的参考几何阻塞规划。
发布节点使用 `/apply_planning_scene` 服务添加一个包含四个盒体的
`camera_stand` 碰撞物体，并从 `/get_planning_scene` 验证结果。
在 RViz 的 PlanningScene 中确认位置，也可运行
`rosservice call /get_planning_scene "{components: {components: 16}}"` 查询世界碰撞物体。支架属于固定环境障碍物，
不应同时用其他节点发布同名物体。

## 笛卡尔路径：直线与画圈

默认打开 MoveIt fake controller，规划当前 `tcp_link` 在 XY 平面的 3 cm 半径圆。
路径先显示在 RViz，`execute` 默认关闭：

```bash
roslaunch aubo_planning cartesian_path_demo.launch
```

直线示例从当前 TCP 沿规划坐标系移动 5 cm：

```bash
roslaunch aubo_planning cartesian_path_demo.launch shape:=line dx:=0.05 dy:=0 dz:=0
```

已有真机或 Gazebo 的 `move_group` 时，用 `start_demo:=false` 避免重复启动控制器。
确认 RViz 轨迹、环境碰撞和末端活动空间后，才传 `execute:=true`：

```bash
roslaunch aubo_planning cartesian_path_demo.launch \
  start_demo:=false shape:=circle plane:=yz radius:=0.03 execute:=false
```

圆形轨迹的起点就是当前 TCP 位姿，末点回到起点；`plane` 可选
`xy`、`xz`、`yz`。姿态全程保持不变。程序只接受完整、无碰撞的笛卡尔路径，
并检查相邻关节路点的最大跳变，随后按速度和加速度比例生成时间戳。
`radius`、`samples`、`eef_step`、`max_joint_step`、
`velocity_scaling` 和 `acceleration_scaling` 都可从 launch 调整。
起始姿态附近需留有足够空间；如果部分圆弧无逆解或会碰撞，程序停止且不执行。

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
