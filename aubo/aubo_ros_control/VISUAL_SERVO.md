# AUBO 统一视觉位置伺服

视觉伺服能力属于机械臂，因此实现位于 `aubo`。固定机械臂和移动机械臂都复用同一
控制节点 `aubo_visual_servo_node`，移动端只负责把导航、底盘锁定和抓取任务组合起来。

## 架构

```text
aubo_perception: RGB-D/YOLO -> /visual_servo/target_pose
                               |
aubo_ros_control: 坐标误差 -> 雅可比逆解 -> 限速/限加速度 -> 有界队列
                               |
                         Gazebo 控制器 或 AUBO SDK
```

`servo_mode` 只改变坐标误差的计算：

- `eye_in_hand`：目标由运动相机光学坐标系变换到 `tcp_link`，控制夹爪 TCP 与目标对齐；
- `eye_to_hand`：目标变换到 `base_link`，控制 `tcp_link` 到目标加偏移的位置。

感知接口、状态机、关节限制、Gazebo/真机执行链完全共用。不要同时启动两个视觉
伺服实例，它们使用相同服务和机械臂命令通道。

## 启动入口

| 安装方式 | Gazebo | 真机 |
| --- | --- | --- |
| 眼在手上 | `eye_in_hand_visual_servo_gazebo.launch` | `eye_in_hand_visual_servo_real.launch` |
| 眼在手外 | `eye_to_hand_visual_servo_gazebo.launch` | `eye_to_hand_visual_servo_real.launch` |

示例：

```bash
roslaunch aubo_ros_control eye_in_hand_visual_servo_gazebo.launch
roslaunch aubo_ros_control eye_to_hand_visual_servo_gazebo.launch
roslaunch aubo_ros_control eye_to_hand_visual_servo_gazebo.launch target_label:=blue
roslaunch aubo_ros_control eye_to_hand_visual_servo_real.launch \
  robot_ip:=192.168.1.2 camera_serial_no:=<serial>
```

旧名称 `visual_servo_gazebo.launch`、`visual_servo_real.launch` 仅作为兼容包装保留，
新代码和文档应使用带 `eye_in_hand` 的完整名称。

真机默认 `auto_start:=false`。先核对 TF、目标方向、工作空间和避碰，再调用
`/visual_servo/set_enabled`。SDK 模式会独占机械臂，不能同时启动 `aubo_hw_node` 或
其他会进入 TCP2CANBUS 的控制节点。

## 相机与感知

两种安装方式都使用 `aubo_perception/rgbd_visual_target_node.py` 和稳定接口：

```text
/visual_servo/target_pose              geometry_msgs/PoseStamped
/visual_servo/set_enabled              std_srvs/SetBool
/visual_servo/reset                    std_srvs/Trigger
/visual_servo/perception/set_enabled   std_srvs/SetBool
/visual_servo/perception/reset         std_srvs/Trigger
/visual_servo/state                    std_msgs/String
```

两种 Gazebo 模式默认共用 `aubo_color_sorting/worlds/sorting.world`。该场景同时包含
红、绿、蓝分拣块以及 `workspace_camera`：眼在手上使用腕部相机，眼在手外使用固定
相机，并用共同的 `target_label:=red|green|blue` 参数选择跟踪颜色。分拣用的彩色
放置区域不在该 world 内，只由 `sorting_gazebo.launch` 按需生成，因此不会参与伺服
识别。眼在手外仍使用
不含手部相机的 `aubo_i5.xacro`，避免无用的手部图像和深度话题。真机入口也只启动外部 RealSense；
外参由 `aubo_description/launch/eye_to_hand_camera_tf.launch` 统一发布。默认外参仅用于
仿真，真机必须替换为标定结果。

启动后也可以在共用 RViz“视觉伺服控制”面板的“跟踪目标”下拉框中切换红、绿、蓝；
该选择通过 `/visual_servo/target_selection` 同时适用于眼在手上和眼在手外。
眼在手上使用完整画面，`ignored_regions` 为空；眼在手外保留固定相机画面
下方的机械臂屏蔽区，并在调试图中标记为 `ROBOT MASK`。
Gazebo 腕部相机使用 `1280×720` 分辨率和 80° 水平视场角；真实相机的视场角由镜头和采集模式决定，
不使用该仿真参数。
眼在手上的 `maximum_contour_area` 已按该分辨率放宽为 `250000 px²`，近距离目标主要由
与分辨率无关的 `maximum_projected_contour_area` 继续限制，避免目标接近时因像素面积增大而断检。

## 控制状态与到位标志

控制核心只保留四个状态：

| 状态 | 行为 |
| --- | --- |
| `DISABLED` | 未使能，按加速度限制减速并保持 |
| `HOLD` | 已使能但没有有效目标，减速保持，等待新目标 |
| `TRACKING` | 使用有效目标持续闭环修正 |
| `FAULT` | 安全深度触发或运行中的关节反馈超过 0.5 秒未更新，锁存故障 |

`/visual_servo/aligned`（`std_msgs/Bool`）是独立的到位报告。误差在阈值内持续
`alignment_hold_time` 后置位；误差超过阈值乘 `alignment_release_multiplier` 后清除。
迟滞只影响报告，不影响速度求解。因此眼在手上默认 6 mm 死区、2 倍报告迟滞下，
到位后误差增长到 10 mm 仍会产生修正速度。目标丢失、禁用、复位和动态调参都会
清除到位报告。任务层如需冻结姿态，应调用 `set_enabled(false)`，不应依赖到位标志停机。

核心不再执行初始搜索、丢失续行或返回观察位。眼在手上的上层任务应先用运动规划
到达观察姿态，再启动视觉伺服；需要恢复搜索时，先停用伺服并按系统的命令通道所有权
规则切换到任务规划器，避免两个控制器同时输出。此包不自动新增任务规划节点。

旧 `loss_strategy` launch 参数仅保留调用兼容性，所有取值均按目标丢失保持处理。
旧搜索/续行参数不再被读取，也不再出现在动态参数面板中。

普通禁用和目标丢失通过现有限加速度链路减速，不是瞬时机械制动。
反馈故障会清除软件队列；Gazebo 停止发布新命令，既有位置控制器保留最后设定值。
SDK 故障会请求停止新批次下发，节点退出时离开流模式，需排除原因后重新启动节点。
软件停止无法撤回控制柜已接收或正在发送的批次，不等同于硬件急停。
`set_enabled(true)` 不能清除故障。`reset` 要求故障时有新鲜关节反馈且后端健康，
复位后保持禁用，需重新使能并等待新的目标。

眼在手上目标期望位置仍为 TCP 系 `[0, 0, 0.115] m`，眼在手外仍使用
`target_offset`。本次未改增益、死区阈值、DLS 阻尼、速度/加速度限制、队列容量或 SDK 缓冲量。
姿态控制也使用连续软死区，避免取消到位停机后持续追逐阈值内的姿态噪声。
感知器在目标或深度无效时不发布旧位姿，控制器根据测量时间判断目标超时。

## 诊断与回归验证

新增默认话题（节点私有话题随节点名变化）：

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/visual_servo/aligned` | `std_msgs/Bool` | 到位标志，控制仍持续修正 |
| `/aubo_visual_servo/position_error` | `geometry_msgs/Vector3Stamped` | TRACKING 时软死区前的误差；frame 为 TCP 或 base |
| `/aubo_visual_servo/target_age` | `std_msgs/Float64` | 目标年龄（秒），尚无目标时为 `1e9` |
| `/aubo_visual_servo/queue_size` | `std_msgs/UInt32` | 本周期生产前的软件队列长度，不含控制柜缓冲 |

在 ROS 工作空间运行 `catkin_make servo_policy_test`，然后执行
`devel/lib/aubo_ros_control/servo_policy_test`。该测试也注册为 CTest 测试。
已有 gtest 可用 `catkin_make run_tests_aubo_ros_control` 和 `catkin_test_results` 检查。
新增的 `servo_policy_test` 检查故障优先级、丢失保持、重获恢复、到位计时及迟滞区内
仍有非零修正的行为。它可脱离 ROS 独立编译，不替代整节点或真机测试。

Gazebo 验收顺序：先将机械臂移到可观测位置；使能后确认 TRACKING；目标稳定后确认
aligned=true；小幅移动目标使误差超出死区但仍在报告迟滞内，确认关节指令继续变化；
遮挡超过 target_timeout 后确认 HOLD、到位清除、无回观察位动作；恢复目标后确认
TRACKING；中断关节反馈超过 0.5 秒确认 FAULT，重新使能不能绕过故障。

记录相同目标阶跃、缓慢移动和遮挡过程的误差、目标年龄、队列长度及关节位置，比较
收敛时间、稳态误差和超调。双层轨迹限制仍保留，后续根据数据单独优化输出链路。

配置分为三份：

- `visual_servo_common.yaml`：关节限制、队列、频率和 SDK 参数；
- `visual_servo_eye_in_hand.yaml`：夹爪 TCP 对齐误差和搜索策略；
- `visual_servo_eye_to_hand.yaml`：固定相机 TCP 偏移和遮挡策略。

控制增益、笛卡尔速度上限、期望位姿、安全深度以及目标超时参数支持通过
`rqt_reconfigure` 在线调整。启动节点后运行
`rosrun rqt_reconfigure rqt_reconfigure`，选择 `/aubo_visual_servo`。动态修改仅在
当前进程中生效，不会回写上述 YAML；确认参数后需手动保存。后端、坐标系、话题、
频率、关节硬限制和 SDK 连接参数仍需修改 YAML/launch 并重启节点。
位置误差采用三轴独立的连续软死区：死区内不输出速度，超出后从零连续增长，
避免图像和深度小噪声驱动夹爪在目标附近来回震荡。

## 安全调试顺序

1. 先保持 `auto_start:=false`，检查相机话题和 TF 连通性。
2. 在 RViz 中确认目标位姿在正确的物体上，外部相机目标可变换到 `base_link`。
3. 眼在手外先把 `target_offset` 的高度设大，确认横向避遮挡方向正确。
4. 真机首次运行降低速度与加速度限制，测量命令到执行的延迟。
5. 确认丢失保持和故障锁存通过，再由上层任务接入观察位和恢复动作。
