# AUBO 移动机器人跟随与循迹

本功能包参考 `simple_follower` 的激光跟随、颜色目标跟随和循线思路，针对 AUBO
复合移动机器人重新实现。三个模式都先处理机械臂姿态，再允许底盘运动：

| 模式 | 传感器 | 机械臂命名姿态 | 作用 |
| --- | --- | --- | --- |
| 激光跟随 | 合并后的 `/scan` | `transport` | 收拢机械臂后跟随前方目标点簇 |
| 颜色跟随 | `/hand_camera/image_raw` | `follow_forward` | 机械臂低位折叠，相机保持水平朝前，跟随指定 HSV 色块 |
| 循线 | `/hand_camera/image_raw` | `observe` | 相机向下，沿地面引导线行驶 |

`prepare_arm_pose.py` 会先检查命名姿态是否存在，再要求 MoveIt 从当前状态规划出
非空、无碰撞轨迹。只有执行成功（相机模式还必须收到有效图像）才发布
`/aubo_mobile_follower/arm_ready=true`。因此机械臂不可达、规划失败或相机未启动时，
底盘始终保持停止。

`follow_forward` 保留 `transport` 的低位 A 形折叠构型，调整腕部使
相机光轴水平对准车体正前方，同时用 `wrist3_joint` 校正相机横滚，
保持图像地平线水平。这会将相机从高位观测
收至靠近底盘的低位观测位置。六个关节值均在 URDF 限位内，并且不进行
未经验证的笛卡尔目标直达。

## 使用方法

新增功能包后先在 catkin 工作空间根目录重新编译并加载环境：

```bash
catkin_make
source devel/setup.bash
```

颜色跟随和循线启动文件默认会各自启动专用 Gazebo 世界、机器人、MoveIt、控制器
以及对应的 RViz 配置；不要同时运行 `move_base`。直接选择一种模式：

```bash
# 激光目标跟随
roslaunch aubo_mobile_follower laser_follow.launch

# 激光跟随 + RViz 诊断（默认同时启动 Gazebo）
roslaunch aubo_mobile_follower laser_follow_debug.launch

# 红色色块跟随：红色目标场景 + 前视相机 + RViz 目标控制/轨迹/调试图像
roslaunch aubo_mobile_follower color_follow.launch

# 黑线循迹：弯曲黑线场景 + 下视相机 + RViz 轨迹/调试图像
roslaunch aubo_mobile_follower line_follow.launch

# YOLO交通标志控制循线（检测器需另外启动）
roslaunch aubo_mobile_follower semantic_line_follow.launch
```

需要运行时调参时，在对应启动命令后增加：

```bash
roslaunch aubo_mobile_follower color_follow.launch start_rqt_reconfigure:=true
roslaunch aubo_mobile_follower line_follow.launch  start_rqt_reconfigure:=true
```

在 `rqt_reconfigure` 左侧选择 `/aubo_mobile_follower`。颜色模式可以动态调整 HSV、
目标面积、死区、速度和比例增益；循线模式可以动态调整 HSV、ROI、最小掩码面积、
速度及 PD 增益。修改会立即作用于控制器，不需要重启节点。RViz 中的相机调试图像
可用于观察阈值和 ROI 调整结果。

无桌面环境或只做自动验证时，可以关闭 Gazebo 客户端和 RViz，物理仿真仍会运行：

```bash
roslaunch aubo_mobile_follower color_follow.launch gui:=false start_rviz:=false
roslaunch aubo_mobile_follower line_follow.launch  gui:=false start_rviz:=false
```

若机器人或 Gazebo 已经由其他入口启动，应关闭重复的仿真、MoveIt 和传感器节点。
颜色目标控制节点依赖专用世界里的 `color_target`，外部场景没有这个模型时也应关闭：

```bash
roslaunch aubo_mobile_follower color_follow.launch start_robot:=false \
  start_moveit:=false start_scan_merger:=false start_target_control:=false
roslaunch aubo_mobile_follower line_follow.launch start_robot:=false \
  start_moveit:=false start_scan_merger:=false
```

在颜色跟随 RViz 中选择顶部 `Interact` 工具，可以拖动 `Color Target Control`
的 X/Y 拉杆改变 Gazebo 红色目标位置。两个 RViz 配置都会显示机器人模型、合并雷达
`/scan`、行驶轨迹 `/aubo_mobile_follower/path` 和相机调试图像。循线到达绿色终点、
黑线离开视野后会发布 `line_lost` 并停车。

## 算法说明

### 激光跟随

只搜索车体前方 ±60°。相邻距离差小于阈值的扫描点组成点簇，至少四点才会成为
候选目标；随后根据上一帧位置保持目标关联。控制器维持默认 1.0 m 距离。找不到
目标或雷达超过 0.5 秒未更新时立即停车。

### 颜色目标跟随

对手眼相机图像做 HSV 阈值、形态学去噪和最大轮廓筛选。轮廓横向偏差控制角速度，
轮廓面积占比近似表示距离并控制线速度。默认识别红色目标；这是一种单目近似，
目标实物尺寸应保持一致。

### 循线

仅处理画面下方区域，提取引导线掩码及重心，使用 PD 控制角速度。弯道偏差越大，
线速度会自动降低。默认跟随黑线。

## 参数调整

所有速度、PID、目标距离、HSV 和丢失超时参数都在
`config/follower.yaml`。调试图像话题：

- `/aubo_mobile_follower/color_debug`
- `/aubo_mobile_follower/line_debug`

状态话题为 `/aubo_mobile_follower/state`。首次实机测试应架空驱动轮或使用急停，
先把最大线速度降到较低值，再标定相机 HSV、目标面积和雷达安全距离。

## YOLO语义循线

`semantic_line_follow.launch` 在普通循线控制器与激光安全过滤器之间加入
`semantic_drive_supervisor.py`。循线节点只向
`/aubo_mobile_follower/line_cmd` 发布基础速度；管理器根据YOLO类别执行停车、减速、
恢复以及基于 `/odom` 的闭环左右转，随后向 `/cmd_vel_raw` 发布。它不会使用固定
时间估算转角，也不会向差速底盘发送横向速度。

默认订阅 `/darknet_ros/bounding_boxes`，期望类别名为 `stop`、`slow_down`、
`resume`、`turn_left` 和 `turn_right`。类别映射、置信度、连续确认帧数、冷却时间和
转向参数均位于 `config/semantic_actions.yaml`。检测器及其模型不由该启动文件启动；
可以接入现有 `darknet_ros`，或者把类别映射改成自己模型的标签。

没有YOLO环境时，可先用字符串后端验证完整控制链：

```bash
roslaunch aubo_mobile_follower semantic_line_follow.launch \
  detection_backend:=string gui:=false start_rviz:=false

# 默认连续4帧确认，因此测试时连续发布同一标签
rostopic pub -r 10 /aubo_mobile_follower/semantic_detection \
  std_msgs/String "data: 'stop'"
```

状态输出为 `/aubo_mobile_follower/semantic_state`。常见状态包括 `FOLLOW_LINE`、
`SLOW_FOLLOW`、`STOPPED`、`TURN_LEFT`、`TURN_RIGHT`、`WAITING_FOR_ARM`、
`NOMINAL_CMD_TIMEOUT`、`ODOM_TIMEOUT` 和 `BASE_LOCKED`。该模式与普通跟随模式一样，
不应和 `move_base` 同时运行；后续如需导航途中响应标志，应先增加速度仲裁器。

激光 RViz 诊断话题为 `/aubo_mobile_follower/laser_debug`：青色球表示通过点数和
连续性检查的候选点簇，绿色球表示当前控制目标，黄色弧线表示期望跟随距离。
`laser_follow_debug.launch` 默认还会在车前生成名为 `follower_target` 的红色测试
立柱，可以通过 `/gazebo/set_model_state` 服务移动；不需要测试目标时传入
`spawn_test_target:=false`。
RViz 中的 `Target Control` 使用标准 Interactive Markers 插件显示 X/Y 拉杆；选择
顶部的 Interact 工具后拖动拉杆，即可实时移动 Gazebo 中的测试立柱。
