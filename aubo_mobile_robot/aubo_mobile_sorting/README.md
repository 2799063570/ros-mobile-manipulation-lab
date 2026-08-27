# AUBO 移动机器人 Gazebo 视觉抓取分拣

该功能包组合手部相机、OpenCV、MoveIt 和 Gazebo 夹爪控制器，实现红、绿、蓝
三种方块的自动识别、抓取和分类放置。

## 编译与启动

通用感知包包含自定义消息，本包包含 RViz C++ 面板，因此修改后必须重新编译：

```bash
catkin_make --force-cmake \
  -DCATKIN_WHITELIST_PACKAGES="aubo_perception;aubo_sorting_core;aubo_gazebo_plugins;aubo_mobile_perception;aubo_mobile_sorting" \
  -j2
source devel/setup.bash
roslaunch aubo_mobile_sorting sorting_gazebo.launch
```

该启动文件会同时运行：

- `worlds/sorting.world` 分拣场景和复合机器人
- AUBO 机械臂及夹爪轨迹控制器
- 使用 Gazebo 真实控制器执行轨迹的 `move_group`
- OpenCV 颜色识别和标注图像发布节点
- 等待面板命令的红、绿、蓝分拣状态机
- 带“AUBO 视觉分拣”控制面板的 RViz 和图像调试窗口

不需要图形窗口时可以执行：

```bash
roslaunch aubo_mobile_sorting sorting_gazebo.launch rviz:=false debug_view:=false
```

## 任务流程

默认采用“先观察、后确认、再分拣”的流程：

```text
启动 Gazebo、控制器和 MoveIt
→ 机械臂自动移动到相机观察位
→ 状态变为 READY
→ 检查调试图像及 RViz 面板中的识别数量
→ 点击“开始分拣”
→ 依次抓取红、绿、蓝方块
→ 机械臂回到 down 姿态，节点继续待命
```

每种颜色的动作顺序为：

```text
移动到观察位姿 → 获取最新识别坐标 → 张开夹爪 → 移动到预抓取点
→ 笛卡尔下降 → 闭合夹爪 → 抬升 → 移动到分类区域
→ 笛卡尔下降 → 张开夹爪 → 回退
```

## RViz 控制面板

默认 RViz 配置会自动加载 `AUBO 视觉分拣` 面板，提供：

- **移动到相机观察位**：移动到命名姿态 `observe`，或配置的 `observation_pose`
- **开始分拣**：只有成功到达观察位后才接受命令
- **停止当前任务**：取消夹爪目标并停止当前机械臂运动
- **张开夹爪**：在待命状态下手动打开夹爪
- **机械臂归位**：移动到 `finish_named_target`，默认是 `down`
- 运行状态以及红、绿、蓝目标识别数量

如果面板没有自动显示，在 RViz 菜单中选择：

```text
Panels → Add New Panel → aubo_mobile_sorting/SortingPanel
```

不使用 RViz 时，也可以调用相同的服务：

```bash
rosservice call /sorting/move_to_observation
rosservice call /sorting/start
rosservice call /sorting/stop
rosservice call /sorting/open_gripper
rosservice call /sorting/home
```

状态和识别数量分别发布到：

```text
/sorting/state
/sorting/detection_summary
```

## 场景尺寸与坐标

- Gazebo 桌面世界坐标高度：`z=0.45 m`
- 机器人落地后 `base_link` 的世界坐标高度：`z=0.31 m`
- 桌面在 `base_link` 坐标系下的高度：`table_z=0.14 m`
- 方块尺寸：`0.04 × 0.04 × 0.04 m`，三块均以 `yaw=0` 水平摆正
- 方块中心高度：`z=0.16 m`；可见顶面高度：`z=0.18 m`
- MoveIt 末端：`tcp_link`
- 抓取时 TCP 的 Z 轴朝向桌面

桌面高度、机器人生成高度和感知参数必须保持一致，否则识别坐标会出现系统性
偏差。

启动文件中的 `z=0.02 m` 只是防止模型生成时嵌入地面的初始间隙。机器人受重力
落地后，坐标计算使用 `base_footprint z=0`，不能把该生成间隙继续计入
`base_link` 高度。

分拣节点在允许机械臂运动前会将 `sorting_table` 加入 MoveIt PlanningScene，并
持续查询 `get_known_object_names()`。只有规划场景确认收到桌子后才会移动到观察
位；超过 `scene_update_timeout` 仍未收到确认时，节点进入 `ERROR`，不会执行轨迹。

## 参数文件

- `aubo_mobile_perception/config/colors.yaml`：HSV 范围、轮廓面积、桌面高度和
  识别工作区域
- `aubo_mobile_sorting/config/sorting.yaml`：观察位姿、抓取高度、夹爪开合量、
  自动观察/自动开始开关、运动速度和各颜色放置位置

抓取前默认对连续 `8` 帧同色目标坐标求平均，减少单帧轮廓中心抖动。基础相机
偏差在感知包 `colors.yaml` 中校准；如果夹爪中心仍有少量固定误差，可通过
`grasp_offset_x` 和 `grasp_offset_y` 微调，无需修改相机模型。

默认优先使用 MoveIt 命名姿态 `observe`，它把手部相机移动到方块上方并使光轴
朝向桌面。只有将 `observation_named_target` 设为空字符串时，程序才会改用
`observation_pose` 的在线笛卡尔逆解。

`shoulder_joint` 是从 `base_link` 出发的第一可动关节，保持原始范围；第二个可动
关节是 `upperArm_joint`。为减少机械臂上下摆动时与桌面碰撞，`upperArm_joint` 在
共享 URDF 以及纯机械臂、移动机械臂两套 MoveIt 配置中均限制为
`-60°～+60°`（`±1.0471976 rad`）。
4 cm 方块约在夹爪关节 `0.24 rad` 时接触。默认闭合位置设为
`gripper_closed: 0.28`，只保留少量预紧量；旧值 `0.42` 会要求手指继续压入方块约
16 mm，容易导致控制器失败、接触抖动或方块弹飞。夹爪开合时间为
`gripper_motion_time: 2.5` 秒。闭合目标还会携带
`gripper_contact_tolerance: 0.30`，使手指被实体方块挡住时被视为正常夹持，而不是
因无法严格到达预紧角度而返回 action 状态 `ABORTED`。

`tcp_link` 与 `gripper_link` 现在重合在手指有效接触高度。这样将 TCP 规划到方块
中心附近时，手指会夹住方块侧面，而不是只搭在方块顶边。默认再通过
`grasp_height_offset: 0.01` 将抓取位提高 1 cm，避免指尖在闭合过程中碰到桌面。

夹爪闭合属于接触动作：手指碰到方块后本来就无法继续严格跟随自由空间轨迹。
因此闭合目标会将 `path_tolerance` 显式设为禁用（ROS 消息中的 `-1`），同时保留
最终位置容差。这样不会再因正常接触返回 `PATH_TOLERANCE_VIOLATED (-4)`。

## Gazebo 抓取固定插件

仅依赖摩擦力时，小尺寸方块可能在抬升瞬间丢失接触。通用插件包提供
`libaubo_grasp_attach_plugin.so`：夹爪闭合后，分拣节点通过
`/sorting/grasp/attach` 通知 Gazebo，在末端的 `wrist3_Link`（与固定夹爪基座同位姿）
和对应颜色方块之间创建临时固定关节；到放置点张开夹爪后，再通过
`/sorting/grasp/detach` 解除。插件状态
发布在 `/sorting/grasp/status`，正常启动时应首先看到 `ready`。

这个插件不替代视觉定位或运动规划。只有机械臂已经移动到识别出的方块位置并完成
夹爪闭合后才会固定方块，因此仍能暴露相机坐标或抓取位姿明显错误的问题。

插件由 `aubo_gazebo_plugins` 提供，不需要安装第三方 link-attacher；修改插件后必须重新运行
`catkin_make --force-cmake` 并重新 `source devel/setup.bash`。如果插件没有成功加载，
分拣节点会进入 `ERROR | Gazebo grasp plugin unavailable`，不会假装抓取成功。

分拣世界中的红、绿、蓝方块不是仅用于显示的模型：每个方块均启用重力，且
`kinematic=false`，并设置了碰撞体、质量和惯量。方块与夹爪手指的接触面使用较高
摩擦系数（`mu1/mu2=20.0`）、接触刚度和阻尼；每个方块最多保留 `20` 个接触点。
修改这些物理参数后必须完全退出并重新启动 Gazebo，已生成的旧模型不会自动更新。

分拣节点启动时还会同时读取 `/robot_description` 和
`/robot_description_planning/joint_limits/upperArm_joint`。只有两者都确认为
`-60°～+60°` 才允许开始分拣；如果 Ubuntu 加载了旧包，RViz 面板会进入 `ERROR`，
终端会打印实际读取到的上下限。

## 调试方法

```bash
rostopic hz /hand_camera/image_raw
rostopic echo /hand_camera/camera_info
rostopic echo /sorting/detections
rostopic hz /sorting/debug_image
rostopic list | grep follow_joint_trajectory
rosservice call /controller_manager/list_controllers
```

如果图像识别正确但抓取位置存在偏差，应先检查调试图像中的目标中心，然后标定
`table_z` 和手部相机固定变换。如果夹爪已经闭合但方块容易滑落，应降低机械臂
速度和加速度，并调整手指摩擦、接触刚度以及 `gripper_closed`，不要无限增加夹紧
力。如果手指仍然明显穿过方块，应进一步检查 Gazebo 的实际碰撞接触；这属于物理
接触或控制器问题，而不是 OpenCV 识别问题。必要时可再引入 grasp-fix/吸附插件，
但应在确认系统已安装相应 Gazebo 插件后再启用。

本示例假定抓取期间移动底盘保持静止。导航与分拣应按顺序执行，在机械臂使用
相机生成的 `base_link` 目标坐标时，不要继续移动底盘。
