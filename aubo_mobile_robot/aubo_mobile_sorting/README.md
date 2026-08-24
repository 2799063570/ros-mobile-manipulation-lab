# AUBO 移动机器人 Gazebo 视觉抓取分拣

该功能包组合手部相机、OpenCV、MoveIt 和 Gazebo 夹爪控制器，实现红、绿、蓝
三种方块的自动识别、抓取和分类放置。

## 编译与启动

感知包包含自定义消息，分拣包包含 RViz C++ 面板，因此修改后必须重新编译：

```bash
catkin_make --force-cmake \
  -DCATKIN_WHITELIST_PACKAGES="xf_mic_asr_offline_circle;aubo_mobile_perception;aubo_mobile_sorting" \
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
- 机器人生成后 `base_link` 的世界坐标高度：`z=0.33 m`
- 桌面在 `base_link` 坐标系下的高度：`table_z=0.12 m`
- 方块尺寸：`0.05 × 0.05 × 0.05 m`
- MoveIt 末端：`tcp_link`
- 抓取时 TCP 的 Z 轴朝向桌面

桌面高度、机器人生成高度和感知参数必须保持一致，否则识别坐标会出现系统性
偏差。

## 参数文件

- `aubo_mobile_perception/config/colors.yaml`：HSV 范围、轮廓面积、桌面高度和
  识别工作区域
- `aubo_mobile_sorting/config/sorting.yaml`：观察位姿、抓取高度、夹爪开合量、
  自动观察/自动开始开关、运动速度和各颜色放置位置

默认优先使用 MoveIt 命名姿态 `observe`，它把手部相机移动到方块上方并使光轴
朝向桌面。只有将 `observation_named_target` 设为空字符串时，程序才会改用
`observation_pose` 的在线笛卡尔逆解。

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
力。

本示例假定抓取期间移动底盘保持静止。导航与分拣应按顺序执行，在机械臂使用
相机生成的 `base_link` 目标坐标时，不要继续移动底盘。
