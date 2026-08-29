# 眼在手上视觉位置伺服

本实现只保留一条容易检查的控制链：

```text
相机系目标位姿
  -> 眼在手上 PBVS 误差
  -> 相机雅可比阻尼伪逆
  -> 关节速度
  -> 速度/加速度/角度受限的前向积分
  -> 有界位置缓冲队列
  -> Gazebo 六轴位置控制器 或 AUBO SDK
```

Gazebo 与真机共用视觉误差、雅可比逆解、限制器和目标丢失状态机，只有队列消费端不同。这能让仿真参数较直接地迁移到真机，也避免两套算法逐渐不一致。

## 1. 输入约定

目标话题默认为：

```text
/visual_servo/target_pose   geometry_msgs/PoseStamped
```

推荐检测器直接发布到 `camera_color_optical_frame`。光学坐标约定为 `+Z` 向前、`+X` 向右、`+Y` 向下。若消息使用其他 TF 坐标系，节点会先变换到相机光学坐标系。没有 `frame_id` 时按相机光学坐标系处理。真机必须把相机安装关节更新为实际手眼标定外参；Gazebo 模型中的安装尺寸不能直接当作真机标定结果。

`desired_target_position: [0, 0, 0.35]` 表示期望目标保持在图像中心前方 35 cm。目标必须来自实时观测，不能把旧位姿持续加新时间戳重新发布，否则节点无法判断目标已经丢失。

状态话题为 `/visual_servo/state`（同时保留 `~state` 兼容输出），可能值：

- `DISABLED`：闭环未启动，机械臂保持当前位置。
- `WAITING`：尚未见过目标，保持当前位置。
- `TRACKING`：目标新鲜，进行视觉伺服。
- `COAST`：目标刚丢失，短时保持原运动方向并指数衰减。
- `SEARCH_OPEN`：向相机视野较开阔的关节姿态移动，途中仍可随时重捕获。
- `HOLD`：搜索超时或选择停止策略，受加速度限制地减速并保持。

## 2. Gazebo

```bash
roslaunch aubo_ros_control visual_servo_gazebo.launch
```

该命令会同时启动 RGB-D 颜色识别和带“AUBO 眼在手上视觉伺服”控制面板的
RViz。为避免启动界面时机械臂立即运动，闭环默认处于 `DISABLED`：先在面板
选择红、绿、蓝或任意目标，再点击“启动闭环跟踪”。“停止并保持”会先停控制器
再停识别输出；“清除目标 / 重新搜索”会清除滤波历史和目标丢失状态机。
无人值守测试可显式使用 `auto_start:=true rviz:=false`。

流程控制接口为：

```text
/visual_servo/set_enabled              std_srvs/SetBool
/visual_servo/reset                    std_srvs/Trigger
/visual_servo/perception/set_enabled   std_srvs/SetBool
/visual_servo/perception/reset         std_srvs/Trigger
/visual_servo/target_selection         std_msgs/String (red/green/blue/any)
/visual_servo/perception_state         std_msgs/String
```

该启动文件加载六个 `position_controllers/JointPositionController`。视觉节点以 200 Hz 从缓冲队列取点，并分别向六个控制器持续发送位置指令。不要同时启动原有 `aubo_i5_controller`，因为它会争用相同的六个位置接口。

最小测试目标：

```bash
rostopic pub -r 20 /visual_servo/target_pose geometry_msgs/PoseStamped \
  "header: {frame_id: 'camera_color_optical_frame'}
   pose: {position: {x: 0.08, y: 0.0, z: 0.55},
          orientation: {w: 1.0}}"
```

## 3. 真机（直接 SDK）

真机模式不启动 `aubo_hw_node`、`controller_manager` 或任何 `ros_control` 控制器：

```bash
roslaunch aubo_ros_control visual_servo_real.launch robot_ip:=192.168.1.2
```

默认会启动 RealSense 的彩色流、深度流和深度到彩色对齐，并运行
`color_visual_target_node.py`。仿真和真机使用相同话题：

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/aligned_depth_to_color/image_raw
```

OpenCV 前端在颜色轮廓内部取有效深度中值并发布
`/visual_servo/target_pose (geometry_msgs/PoseStamped)`。这是感知与控制之间的稳定接口；未来 Torch/YOLO 节点只需发布相同的相机系 `PoseStamped`，无需修改视觉伺服控制器。检测或深度无效时不得重复发布旧位姿。

节点建立三条独立 SDK 连接：控制模式、状态读取、MAC 缓冲下发。输出线程从本地有界队列取点，对每个点再次做有限值、速度和加速度检查，再通过 `robotServiceSetRobotPosData2Canbus()` 批量送入控制柜。控制柜目标缓冲默认约为 10 个六轴点（约 50 ms），在抗调度抖动与目标丢失响应之间折中。SDK 模式由本节点独占；运行时不要再启动其他会接管机械臂或进入 TCP2CANBUS 的节点。

首次真机运行建议：

1. 降低 `joint_velocity_limits`、`joint_acceleration_limits` 和 `max_camera_linear_velocity`。
2. 将 `loss_strategy` 暂设为 `stop` 验证坐标方向。
3. 用手持目标做小范围运动，确认目标偏右时相机会向右修正、目标偏远时相机会向前修正。
4. 再启用 `coast_then_open`，先把 `coast_duration` 设为 0.1–0.2 s。
5. 实测并替换 `open_posture`；它必须是当前工作单元内无碰撞、相机视野开阔的姿态。

## 4. 三层运动限制

每个输出周期依次执行：

1. 将雅可比逆解得到的关节速度限制到 `joint_velocity_limits`。
2. 将相邻速度变化限制到 `joint_acceleration_limits * dt`。
3. 前向积分后，将关节角限制到上下限内侧 `joint_limit_margin`。

真机 SDK 消费端还会独立复核相邻队列点，防止异常生产者或调度跳变绕过限制。参数默认值是保守的视觉伺服值，不是机械臂硬件极限。

## 5. 目标丢失策略

`loss_strategy` 支持：

- `stop`：目标超过 `target_timeout` 后立即请求零速度，实际按加速度限制减速。
- `coast`：在 `coast_duration` 内保持上次跟踪速度并指数衰减，之后保持。
- `coast_then_open`：先衰减滑行，再用受限关节速度移向 `open_posture`；`search_timeout` 后保持。

任何状态切换都会清空尚未执行的旧队列点，防止缓冲延迟导致丢失后继续执行旧视觉命令。重新观测到目标后会直接切回 `TRACKING`，但速度仍受加速度限制，不产生跳变。

## 6. 调参顺序

建议依次调整：

1. `desired_target_position` 与相机手眼外参。
2. `linear_gain` 和 `max_camera_linear_velocity`。
3. `dls_lambda`（越大越稳，但跟随误差也会增大）。
4. 关节速度、加速度限制。
5. 丢失时间参数和现场验证后的 `open_posture`。

姿态伺服默认关闭。只有当检测器的目标姿态稳定且方向定义明确时，才启用 `use_orientation_control`。

## 7. 眼在手外控制

`eye_to_hand_moveit_servo_node.py` 是与高速眼在手上 SDK 控制器隔离的低频 MoveIt
PBVS实现。它把固定 RGB-D 相机给出的目标变换到机械臂基座坐标系，以限制步长反复
规划 TCP 位置。配置默认 `plan_only: true`、`start_enabled: false`。

眼在手上与眼在手外节点共用 `/visual_servo/target_pose`、启停服务和状态话题，因此
同一时刻只能启动一个控制节点。YOLO适配器也只负责发布目标，不直接下发关节命令。
