# AUBO 移动机器人 Gazebo 视觉抓取分拣

该功能包组合手部相机、OpenCV、MoveIt 和 Gazebo 夹爪控制器，实现红、绿、蓝
三种方块的自动识别、抓取和分类放置。

## 编译与启动

由于感知功能包包含自定义消息，首次使用前需要重新编译工作空间：

```bash
catkin_make
source devel/setup.bash
roslaunch aubo_mobile_sorting sorting_gazebo.launch
```

该启动文件会同时运行：

- `worlds/sorting.world` 分拣场景和复合机器人
- AUBO 机械臂及夹爪轨迹控制器
- 使用 Gazebo 真实控制器执行轨迹的 `move_group`
- OpenCV 颜色识别和标注图像发布节点
- 单次红、绿、蓝自动分拣任务
- RViz 和图像调试窗口

不需要图形窗口时可以执行：

```bash
roslaunch aubo_mobile_sorting sorting_gazebo.launch rviz:=false debug_view:=false
```

## 任务流程

每种颜色依次执行：

```text
移动到观察位姿 → 获取最新识别坐标 → 张开夹爪 → 移动到预抓取点
→ 笛卡尔下降 → 闭合夹爪 → 抬升 → 移动到分类区域
→ 笛卡尔下降 → 张开夹爪 → 回退
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
  运动速度和各颜色放置位置

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
