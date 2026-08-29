# wheeltec_multi 多机器人编队

本包提供主车状态广播、从车槽位跟踪、麦轮二维控制和本地激光避障。本轮优化重点解决四类常见问题：槽位坐标定义不清、无线状态延迟不可见、小误差下无法归位，以及避障后突然冲向槽位。

## 本轮已经实现的改进

### 1. 明确槽位坐标系

`slave_x` 表示前后偏置，`slave_y` 表示左右偏置，单位均为米。新增参数 `formation_frame`：

- `leader`（默认）：偏置随主车朝向旋转。`slave_x > 0` 始终表示主车前方，`slave_y > 0` 始终表示主车左侧。
- `map`：偏置固定在地图坐标系，不随主车朝向旋转。

旧参数 `multi_mode` 暂时保留，便于兼容原启动方式，但不再用于解释槽位坐标系。新项目应显式设置 `formation_frame`。

### 2. 可检测延迟和乱序的主车状态

主车 UDP 包升级为 V2，包含：

- 固定协议标识 `WFM2`；
- 单调递增序号；
- 主车发送时间；
- 主车二维位姿和底盘坐标系速度。

从机把数据发布为 `wheeltec_multi/LeaderState`，麦轮控制器会丢弃重复或乱序数据，并使用真实数据年龄做短时预测。接收节点仍发布旧 `/multfodom`，旧差速控制器可以继续使用。

主从机时间差在 `max_clock_skew`（启动文件默认 0.5 s）以内时使用发送时间；否则自动回退到接收时间并报警。推荐所有机器人启用 chrony 或 NTP。只有时间同步后，`leader_age` 才能代表无线传输与处理延迟。时间戳有效且数据年龄超过 `mecanum_leader_timeout` 时，从机会立即停车。

### 3. 归位控制和底盘死区补偿

麦轮控制器现在包含：

- 位置 P 控制和受限 I 控制；
- 积分抗饱和；
- 超出容差时的最小有效修正速度；
- 大误差归队状态及增益倍率；
- 避障期间清空并冻结位置积分；
- 速度、加速度和减速度限制。

积分用于抵消麦轮侧滑、轮径误差和静摩擦造成的固定偏差，不应设置得过大。机器人持续来回摆动时，首先减小 `k_i_x/k_i_y`，然后再检查 AMCL 位姿是否跳变。

### 4. 定位质量门控

控制器可同时要求：

1. 收到一次真实的 `/initialpose`；
2. `/amcl_pose` 的位置和航向标准差低于阈值。

未满足条件时从机保持停止。`initial_x/initial_y/initial_yaw` 必须是从机在共享地图中的真实初始位姿，不能填写期望编队偏置。

### 5. 避障状态和数据超时

避障节点新增 `obstacle_timeout`。激光目标消息过期后，不再永久使用旧障碍物距离。节点发布 `avoidance_active`，编队控制器据此冻结积分。危险区内仍然以碰撞安全为最高优先级，因此暂时脱离槽位是预期行为。

## 编译与升级

V2 新增了 ROS 消息，主车和所有从车必须使用同一版本代码并重新编译工作空间：

```bash
cd ~/wheeltec_robot
catkin_make
source devel/setup.bash
```

建议先升级从车。新版接收节点可接受旧的 24 字节 UDP 包，但旧包没有时间戳和序号；升级主车后才能获得完整延迟诊断。

## 启动示例

主车：

```bash
roslaunch wheeltec_multi navigation.launch
```

从车位于主车后方 0.8 m、左侧 0.8 m，并使用真实地图初始位姿：

```bash
roslaunch wheeltec_multi wheeltec_slave.launch \
  formation_frame:=leader slave_x:=-0.8 slave_y:=0.8 \
  set_initial_pose:=true initial_x:=1.25 initial_y:=-0.40 initial_yaw:=0.0
```

如果初始位姿无法准确测量，保持 `set_initial_pose:=false`，在 RViz 中对每台从车分别执行 `2D Pose Estimate`。AMCL 未收敛时控制器不会运动。

临时调试定位门控时可使用：

```bash
roslaunch wheeltec_multi wheeltec_slave.launch \
  require_initial_pose:=false require_localization_convergence:=false
```

这只适合架空轮胎或低速空旷场地，不建议作为日常设置。

## 关键话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/leader_state` | `wheeltec_multi/LeaderState` | 带发送时间和序号的主车状态 |
| `/multfodom` | `std_msgs/Float32MultiArray` | 旧控制器兼容话题 |
| `/formation_status` | `wheeltec_multi/FormationStatus` | 目标、实际位姿、误差、状态和控制输出 |
| `/avoidance_active` | `std_msgs/Bool` | 从机本地避障是否正在改写速度 |
| `/cmd_vel_ori` | `geometry_msgs/Twist` | 编队控制器原始输出 |
| `/cmd_vel` | `geometry_msgs/Twist` | 避障处理后的底盘输出 |

`FormationStatus.state`：

- `0 WAITING`：等待通信、初始位姿或 AMCL 收敛；
- `1 TRACKING`：正常跟踪；
- `2 RECOVERING`：位置误差超过 `recovery_error`，正在归队；
- `3 AVOIDING`：避障正在修改最终速度。

建议现场录制以下数据：

```bash
rosbag record /leader_state /formation_status /amcl_pose /odom \
  /cmd_vel_ori /cmd_vel /avoidance_active /scan
```

## 参数说明和初始建议

| 参数 | 默认值 | 作用与调节建议 |
| --- | ---: | --- |
| `formation_frame` | `leader` | 槽位相对主车或地图 |
| `mecanum_k_x/y` | `1.0` | 位置比例增益；响应慢时逐步增加 |
| `mecanum_k_i_x/y` | `0.08` | 消除固定偏差；振荡时减小 |
| `mecanum_integral_limit` | `0.35` | 积分上限，防止长时间积累 |
| `mecanum_position_tolerance` | `0.03 m` | 允许的位置误差 |
| `mecanum_recovery_error` | `0.25 m` | 超过后进入归队状态 |
| `mecanum_recovery_gain_scale` | `1.4` | 归队时比例增益倍率 |
| `mecanum_acc_lim` | `0.5 m/s²` | 从机加速能力；应通过实车测量确定 |
| `mecanum_prediction_horizon` | `0.15 s` | 最大外推时间；不宜掩盖严重网络延迟 |
| `mecanum_leader_timeout` | `0.4 s` | 超时立即停车 |
| `max_position_stddev` | `0.35 m` | AMCL 位置标准差门限 |
| `max_yaw_stddev` | `0.35 rad` | AMCL 航向标准差门限 |
| `obstacle_timeout` | `0.5 s` | 障碍物消息失效时间 |

推荐调试顺序：

1. 关闭避障、主车静止，确认从机能从不同方向回到槽位。
2. 将积分设为零，只调 `k_x/k_y` 和速度、加速度限制，使响应快速但不振荡。
3. 逐步增加 `k_i_x/k_i_y`，消除稳定残差。
4. 主车低速直线运动，检查 `/formation_status` 中 `leader_age` 和误差。
5. 测试旋转和曲线，确认最大槽位速度没有超过从机能力。
6. 最后开启避障，检查 `AVOIDING -> RECOVERING -> TRACKING` 状态转换。

## 如何判断问题属于哪一层

- 从机静止时目标与实际误差一直固定，但实体相对位置看起来正确：主从地图或初始位姿没有对齐。
- `leader_age` 偶尔明显增大，随后误差突增：无线延迟或丢包。
- `/cmd_vel_ori` 正常而 `/cmd_vel` 长期不同：避障持续介入或障碍物检测误报。
- 控制输出已达到速度上限且误差继续增加：主车轨迹对从机不可行，应降低主车速度或角速度。
- AMCL 位姿跳变后实体突然高速归队：定位不稳定，应先解决地图、雷达和 AMCL 参数。

## 尚未解决与下一步路线

本轮没有把本地避障升级为真正的多机器人协同规划，也没有加入机器人间相对传感器。建议下一阶段按以下顺序推进：

1. 主车根据所有从机的误差和状态自动限速，任一从机持续 `RECOVERING` 时减速。
2. 广播未来 200–500 ms 的短轨迹，而不只是当前位姿和速度。
3. 对 X/Y/旋转三个方向做底盘速度标定，加入比例补偿表。
4. 使用 AprilTag、UWB 或视觉相对定位修正独立 AMCL 的相对漂移。
5. 将局部排斥避障替换为“纵队通过—恢复编队”的状态机或约束优化控制。
6. 增加自动化回放测试，用 rosbag 对最大误差、均方误差、归队时间和通信延迟做版本对比。

安全提示：首次启用新参数时请架空轮胎检查方向，再在空旷场地以低速测试，并保留急停人员。
