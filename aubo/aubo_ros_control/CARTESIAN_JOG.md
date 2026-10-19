# 末端 XYZ 连续点动（C++）

这是独立的 ROS 1 点动控制器，面向按住按钮移动、松手减速停止的操作。
不调用 `computeCartesianPath()`，不启动 MoveIt 轨迹执行器。使用 KDL 雅可比阻尼逆解，
将末端线速度变成六轴关节速度，经速度/加速度/jerk 平滑后积分成 5 ms 关节位置点。
核心、中文 Qt 窗口和平滑器测试均使用 C++14。

本实现使用 ROS 1 / MoveIt 1 的接口；完整 ROS 构建和运行需要 Linux ROS 工作空间。
本次开发环境只有 Windows/MSVC，已验证独立平滑器，尚未验证完整 ROS 编译、Gazebo 或真机。

## 编译与启动

在已有依赖的 catkin 工作空间根目录运行：

```bash
catkin_make
source devel/setup.bash

# 空场景仿真，同时打开中文点动窗口
roslaunch aubo_ros_control cartesian_jog_gazebo.launch

# 无桌面仿真（通过自定义 ROS 客户端操作）
roslaunch aubo_ros_control cartesian_jog_gazebo.launch gui:=false panel:=false
```

Gazebo 入口从非零 home 姿态开始，并加载六个独立关节位置控制器。不得在同一模型上
同时运行 MoveIt 的关节轨迹控制器、视觉伺服或其他关节指令发布者。

真机使用专用入口：

```bash
roslaunch aubo_ros_control cartesian_jog_real.launch robot_ip:=192.168.1.2
```

该入口直接登录 SDK、执行机器人 startup/握手并进入 TCP2CANBUS，可能上电和松刹车；
点动运动默认仍为禁用。启动前机械臂必须报告停止状态。不要同时运行
`aubo_real_bringup.launch` 或视觉伺服真机入口。启动文件默认使用带腕部相机的模型；
如果实际没有相机，传入 `robot_xacro:=aubo_i5.xacro robot_srdf:=aubo_i5.srdf`。

窗口中先点击“使能点动”，选择基坐标系或工具坐标系，再按住 `X± / Y± / Z±`。
默认速度 10 mm/s，默认上限 20 mm/s；界面 mm/s 自动转换为消息中的 m/s。
松手、窗口失焦/隐藏或关闭时发布零指令；控制器没有收到新指令时也会超时减速。
“停止并禁用”使用同一减速路径。GUI 服务调用阻塞时，控制器自身的超时机制仍生效。

## 接口与使能流程

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/cartesian_jog/command` | `geometry_msgs/TwistStamped` | 持续发送末端线速度；建议 30–50 Hz |
| `/cartesian_jog/set_enabled` | `std_srvs/SetBool` | 使能或减速禁用 |
| `/cartesian_jog/reset` | `std_srvs/Trigger` | 停稳、反馈正常且停止位姿安全时清除故障；不自动使能 |
| `/cartesian_jog/state` | `std_msgs/String` | 10 Hz 状态心跳，含故障原因 |

`header.frame_id` 仅接受配置中的 `base_link` 或 `tool_link`，默认分别为
`base_link`、`tcp_link`。工具坐标系方向按当前指令位姿旋转到基坐标系。
`linear` 单位 m/s；`angular` 必须全部为零，本版只开放 XYZ 平移。
控制器在开始移动时记录末端朝向，并在移动期间加入姿态误差修正。

每条有效消息都需要当前 ROS 时间戳，且时间戳递增。相同时间戳的重复消息丢弃且
不刷新超时计时，兼容 Gazebo 时钟分辨率低于 GUI 事件频率的情况。过期、未来偏差过大、乱序、
非有限值、不支持的坐标系和角速度输入会触发故障锁存。多个指令发布者不应混用。
无论输入时间戳如何，指令与反馈超时都使用本机单调时钟，防止 ROS 时间暂停绕过超时。

每次使能后，必须先发送零速度，再发送非零速度；使能前的命令不会积存。
如果一直没有发送零指令，会在 `command_timeout` 到期后进入故障。
中文窗口会自动持续发送带新时间戳的零指令，满足此要求。

```bash
rosservice call /cartesian_jog/set_enabled "data: true"
rosservice call /cartesian_jog/set_enabled "data: false"
rosservice call /cartesian_jog/reset "{}"
rostopic echo /cartesian_jog/state
```

`DISABLED` 表示已禁用并停稳；`ENABLED_IDLE` 表示已使能、无运动；
`JOGGING` 表示运动或仍在减速；`STOPPING` 表示禁用后的减速阶段；
`FAULT: 原因 | STOPPING/STOPPED` 表示锁存故障。故障不会自动重启运动。
仿真暂停/恢复可能触发调度间隙或陈旧时间戳故障，恢复后需检查状态并复位。

## 平滑、缓存与停止

`VelocitySmoother` 对每个关节采用两层状态：第一层把目标速度变成加速度受限的斜坡，
第二层滤波该斜坡。固定步长 `h=0.005 s`，系数
`alpha=min(1, J*h/(2*A))`。从静止开始且目标速度始终限制在 `[-V,V]` 时，
下发速度序列满足 `|v|<=V`、`|Δv/h|<=A`、`|Δ²v/h²|<=J`。
反向输入同样经过此过程，不会直接翻转关节速度。滤波有响应延迟，不能既要求任意小 jerk
又要求立即停止；默认等效滤波时间约为 0.13–0.15 s。

此界限针对离散指令序列，不代表电机实际速度/加速度一定满足相同界限，也不保证
控制柜内部插值具有连续 jerk。零指令下速度渐近归零，以 1e-6 的内部状态阈值判定停稳，
不通过强制清零速度制造突变。有限的姿态误差、阻尼逆解和滤波会造成末端偏离理想直线。

SDK 后端只在 MAC 缓冲有空位时请求新点，点动不使用视觉伺服的 80 点软件队列。
默认 `sdk_mac_buffer_target: 36`，按本驱动使用的六轴标量计数解释，相当于约 6 点 / 30 ms。
点动模式只接受 6–60 的六倍数（最多名义 50 ms），防止配置出过长的控制柜前瞻。
反馈仍为 50 Hz，200 Hz 是路点消费/生成节拍，不是 200 Hz 实机反馈闭环。
控制柜消费周期需要与本仓库使用的 5 ms SDK 透传协议一致。

停止命令不会撤回已经进入控制柜的点。实际停止延迟包括输入采样/网络、已有缓冲和
减速尾段。正常 SIGINT/SIGTERM 会尝试先减速、等待缓冲排空；ROS 被直接 shutdown、
断线、急停、SDK 失败或缓冲欠载时无法保证平滑退出。软件停止不能替代硬件急停。

同一 Linux 主机上的本包三个真机写入路径（ros_control、视觉伺服、点动）共享
`/tmp/aubo_motion_writer.lock` 文件锁。只读示教器模式不持锁。锁文件不应手动删除；
进程结束会由内核释放锁。该锁不能约束其他主机、示教器或仓库外的 SDK 客户端，
也不支持同一主机并行控制多台机械臂。Gazebo 输出不使用此文件锁。

## 限位、奇异位形和碰撞

节点验证关节名称与 KDL 链顺序一致，且 YAML 限制不超过模型限制。
雅可比最小奇异值接近阈值时整体缩放六轴速度；达到停止阈值时锁存故障并减速。
接近关节限位时还考虑当前平滑器的停止距离，不单独截断某个关节。
机器人处于奇异位形或关节限位之外时，本版不会自动选择逃逸方向。

碰撞检查使用独立 `PlanningSceneMonitor`，检查候选位姿及其零指令减速尾段，
而非每次规划完整路径。尾段以 5 ms 推演关节限位，以 `collision_resolution` 指定的
最大关节增量采样自碰撞/环境碰撞。默认推演最多 3 秒，不能在此时限停稳的配置拒绝运动。
候选点不安全时接续上一周期已验证的减速尾段，不清队列、不把指令位置重置为反馈位置。
过长计算、反馈超时、跟随误差过大或缓冲断流均会触发停止/退出。

启动场景默认只含机器人，**不会自动把 Gazebo 地面、桌子或其他模型加入 MoveIt 场景**。
外部可向 `/collision_object`、`/planning_scene_world`、`/planning_scene` 发布障碍物，
向 `/attached_collision_object` 发布附着物。节点不自行运行点云建图。
如果已有纯规划 MoveIt 场景，使用自定义配置设置：

```yaml
planning_scene_topic: /move_group/monitored_planning_scene
initial_scene_service: /get_planning_scene
```

此时完整场景快照获取失败会拒绝启动，后续更新由指定话题同步。已有 MoveIt 必须关闭
轨迹执行，且没有其他硬件控制进程占用 SDK。默认独立入口不需要启动 move_group。
场景访问按 MoveIt 的
[PlanningSceneMonitor 锁接口](https://github.com/moveit/moveit/blob/noetic-devel/moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h)
保护，避免读场景时与更新线程并发修改。

碰撞检测是基于已知几何的离散预测，不覆盖采样点间薄障碍、突然进入的动态障碍、
错误 TCP/模型、未发布的环境物体或反馈跟随误差。模型检查不能构成真机安全保证。
复杂网格可能使预测耗时超出预算；需要测量实际计算耗时，再决定是否简化碰撞模型。

## 配置与代码入口

参数位于 `config/cartesian_jog.yaml`；两套完整入口均支持 `config:=自定义YAML路径`。
启动时读取参数，本版不提供运动期间动态改参。

| 文件 | 职责 |
| --- | --- |
| `src/cartesian_jog.cpp` | 输入、状态、KDL 逆解、限位/碰撞预测、两种输出后端 |
| `include/aubo_ros_control/cartesian_jog_common.h` | 可独立测试的平滑器和停止距离上界 |
| `src/cartesian_jog_panel.cpp` | 中文 XYZ 按钮、速度/坐标系、失焦释放和状态心跳 |
| `src/direct_sdk_backend.cpp` | SDK 连接、控制柜按需补点、下发边界校验 |
| `include/aubo_ros_control/motion_lease.h` | 同机运动写入互斥 |

## 验证与调试

```bash
catkin_make tests
rosrun aubo_ros_control cartesian_jog_common_test
rostest aubo_ros_control cartesian_jog.test
```

独立数值测试覆盖 30 万步输入、启停/反向/随机目标/NaN、速度/加速度/jerk 上界、
停止距离上界、停稳和参数拒绝。ROS 集成测试使用满秩 XYZ+RPY 合成模型及模拟反馈，
覆盖禁用不运动、使能前零指令、基/工具坐标系、输入/反馈超时、过期时间戳、
非法坐标系/角速度/NaN 和故障复位，无需连接机械臂。

当前只运行通过了独立数值测试。ROS 集成测试、Gazebo/SDK 完整编译运行以及真机
按下/松手延迟、低速匀速性、反向平滑性、TCP 偏差和停止距离仍需在目标环境验证。
不要把 200 Hz 配置值当作硬实时或真机平滑性的证明。
