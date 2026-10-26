# AUBO 笛卡尔轨迹跟踪

本包在 `aubo_ros_control/launch/cartesian_jog_real.launch` 上增加圆形、正弦曲线跟踪和可视化。它是 ROS1 Noetic catkin 功能包，不建立第二个 SDK 连接、不改写当前点动参数。启动后不自动使能、不自动 HOME、不自动执行曲线。

执行链路：

```text
解析曲线 + 五次时间缩放 + 实际 TCP 位置回授
  → /cartesian_jog/command (TwistStamped，基坐标系线速度，角速度为零)
  → 现有点动节点：姿态保持 / 雅可比逆解 / 平滑 / 限位 / 制动碰撞检查
  → 现有 SDK 按需供点 → 控制柜
```

## 启动

在工作空间根目录编译并加载环境：

```bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
catkin_make --pkg aubo_trajectory_tracking -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source devel/setup.bash
```

`CMAKE_POLICY_VERSION_MINIMUM` 用于兼容本机较新 CMake 与系统旧版 googletest。

如果工作空间设置过 `CATKIN_WHITELIST_PACKAGES`，必须先在原白名单中加入 `aubo_trajectory_tracking`。`--pkg` 只选择 make 的构建目标，不会自动把新包加入 CMake 白名单。可查看 `build/CMakeCache.txt` 中的 `CATKIN_WHITELIST_PACKAGES:STRING`；重新配置时保留原列表并追加新包。编译完成后，节点应位于 `devel/lib/aubo_trajectory_tracking/trajectory_tracker_node`。

实机（会复用原入口的 SDK 登录、上电与透传初始化）：

```bash
roslaunch aubo_trajectory_tracking tracking_real.launch robot_ip:=192.168.1.2
```

Gazebo：

```bash
roslaunch aubo_trajectory_tracking tracking_gazebo.launch
```

如果已单独启动 `cartesian_jog_real.launch panel:=false`，只挂接上层：

```bash
roslaunch aubo_trajectory_tracking tracking.launch
```

同一套机械臂只能运行一套点动后端；使用本包时关闭点动面板、串口遥控和其他速度指令发布源。开始时检查一个指令订阅者，但这不是多个发布者之间的仲裁协议。所有入口均支持 `panel:=false rviz:=false`。自定义上层配置使用 `config:=/绝对路径/tracking.yaml`，实机底层配置使用 `jog_config:=...`。

## 操作

1. 等待 SDK、关节反馈和点动节点就绪，底层状态为 `DISABLED`。机械臂当前位置应有足够工作空间且避开奇异位形；本包不会自动调整起始关节姿态。
2. 在面板设置 `circle` 或 `sine`、平面、尺寸和总时间，点击“更新预览”。预览捕获当前实际 TCP 作为起点，冻结本次参数，并显示完整参考曲线。
3. 检查 RViz 的曲线与实际场景。点击“开始跟踪”：显式使能、发送至少约 0.3 s 零指令、等待新的使能状态确认，再开始推进轨迹。
4. 完成后先等待实际 TCP 收敛，再请求禁用。途中可点击“停止”；关闭面板也会先请求停止。停止或故障后必须重新预览并显式开始。

预览后如果 TCP 位置移动超过到位容差，开始请求会被拒绝。面板修改参数后须重新“更新预览”，否则开始仍采用上一次预览冻结的参数。

普通停止会经过点动程序和控制柜已有前瞻点及减速尾段，不等于硬件急停。`COMPLETED` 表示末点位置与估计速度满足上层判据且禁用请求已被接受，不证明柜内队列完全排空或机械臂已经物理停稳。上层进程突然退出时，停止仍依赖底层输入超时保护。

## 曲线与控制

所有曲线相对预览时的 TCP 位置生成；`xy/xz/yz` 都是基坐标系平面。圆的起点在圆周上，圆心位于第一条平面轴负方向一个半径处。

令 `u = clamp(t / duration, 0, 1)`，`s = 10u³ − 15u⁴ + 6u⁵`：

- 圆：`a = radius × (cos(2π × cycles × s) − 1)`，`b = radius × sin(2π × cycles × s)`。整数圈数执行一次后返回起点。
- 正弦：`a = length × s`，`b = amplitude × sin(2π × waves × s)`。执行一次后停在前进距离 `length` 处，不自动往返。

起终点的参考速度、参考加速度为零。上层以 50 Hz 更新：`v = derivative(p_ref) + kp × (p_ref − p_actual)`，再按合速度限幅。实际 TCP 使用原始关节反馈及与点动相同的 `robot_description`、`base_link → tool_link` KDL 链计算；不使用参考位置充当实际反馈。姿态保持由现有点动节点完成。

默认圆半径 30 mm，总时间 40 s，最大指令速度 15 mm/s，`kp=0.5/s`。这是带时间参数的轨迹跟踪，上层误差是同一时刻参考位置与实际 TCP 的欧氏距离，不是到整条曲线的最近距离。SDK 前瞻、平滑和通信延迟会造成相位滞后；当前回授未补偿这些延迟，实机跟踪精度需现场测量，不能由模拟测试推断。

预览检查解析速度上界，为回授预留 20% 速度预算。五次时间缩放最大 `ds/dt=1.875/duration`，圆的速度界为 `2π × cycles × radius × 1.875/duration`；正弦使用 `hypot(length, 2π × waves × amplitude) × 1.875/duration`。超限时增加运行时间或减小尺寸。

预览仅显示几何轨迹并校验数值、尺寸、速度，**不承诺整条路径可达、无碰撞或远离奇异点**。运动中的底层检查继续生效，遇到不能执行的局部路径会停止，不做全局绕障。现实障碍物仍需按原点动流程发布到规划场景。

## 参数

详见 [config/tracking.yaml](config/tracking.yaml)，加载到 `/trajectory_tracking`。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `shape / plane` | `circle / xy` | `circle/sine`；`xy/xz/yz` |
| `radius / amplitude / length` | `0.03 / 0.02 / 0.10` | m |
| `cycles / waves` | `1 / 1` | 圆圈数 / 正弦波数，整数 1–20 |
| `duration` | `40.0` | 一次完整轨迹时间 s，含起止时间缩放 |
| `kp` | `0.5` | 位置回授增益 1/s |
| `max_speed` | `0.015` | 线速度合量上限 m/s，另受底层限速 |
| `max_error` | `0.04` | 超过此 TCP 误差则中止，m |
| `max_extent` | `0.15` | 相对起点最大几何位移界，m |
| `feedback_timeout / state_timeout` | `0.15 / 0.35` | 原始关节反馈 / 底层状态新鲜度，s |
| `settle_tolerance / settle_speed` | `0.002 / 0.003` | 末点收敛位置 m / 估计 TCP 速度 m/s |
| `settle_timeout` | `15.0` | 轨迹时间结束后最大收敛等待 s |
| `base_link / tool_link` | `base_link / tcp_link` | 必须与底层点动配置一致 |
| `joint_states_topic` | `/cartesian_jog/sdk_joint_states` | 原始六轴反馈，Gazebo 入口覆盖为 `/aubo_i5/joint_states` |

关节反馈检查时间戳、顺序、关节完整性、有限数值和接收新鲜度；重复/倒序或无效反馈不会刷新有效期。运行循环超过 100 ms、反馈/底层状态过期、底层退出使能状态、位置误差越界或末点不收敛，均请求零速度和禁用。失败的禁用请求会持续重试；不自动复位底层故障。

## 可视化与接口

RViz 默认配置包含机械臂模型、网格、青色参考路径、橙色实际 TCP 路径、黄色目标点、红色误差线和状态文字。面板显示运行时间、即时/峰值/RMS 位置误差及估计实际 TCP 速度。实际路径最多保留最近 10000 个采样点，参考路径为 1001 个点；统计覆盖本次运行及末点收敛阶段。

| 接口 | 类型 / 内容 |
| --- | --- |
| `/trajectory_tracking/preview` | `std_srvs/Trigger`，更新预览、捕获起点与参数 |
| `/trajectory_tracking/start` | `std_srvs/Trigger`，开始已预览的轨迹 |
| `/trajectory_tracking/stop` | `std_srvs/Trigger`，请求停止 |
| `/trajectory_tracking/state` | `std_msgs/String`，状态及原因 |
| `/trajectory_tracking/reference_path` | `nav_msgs/Path`，锁存的完整参考曲线 |
| `/trajectory_tracking/actual_path` | `nav_msgs/Path`，实际 TCP 路径 |
| `/trajectory_tracking/markers` | `visualization_msgs/MarkerArray` |
| `/trajectory_tracking/metrics` | `Float64MultiArray`，`[elapsed_s, error_m, peak_m, rms_m, actual_speed_m_s]` |

无界面示例（先启动节点）：

```bash
rosparam set /trajectory_tracking/shape sine
rosparam set /trajectory_tracking/plane xz
rosservice call /trajectory_tracking/preview
rosservice call /trajectory_tracking/start
# 随时停止：
rosservice call /trajectory_tracking/stop
```

保存一次跟踪记录：

```bash
rosbag record -O tracking.bag /trajectory_tracking/state /trajectory_tracking/metrics /trajectory_tracking/reference_path /trajectory_tracking/actual_path /cartesian_jog/command /cartesian_jog/state /cartesian_jog/sdk_joint_states
```

## 验证边界

已独立编译，使用三轴直线运动学模型和模拟速度执行端完成 ROS 集成测试：三种平面、圆形/正弦末点、零指令握手、速度界、参数拒绝、反馈/状态丢失、非有限反馈、重复时间戳、超差及主动停止。测试不登录 SDK，不连接或移动实机。

```bash
rostest aubo_trajectory_tracking tracking.test
```

模拟执行端没有复现 SDK 前瞻与真实动力学，实机效果仍需验证。现有实机点动约束参见 [调试复盘](../../docs/AUBO实机点动问题与调试复盘.md)。

2026-10-25 补充验证：已在实际工作空间编译，并使用独立 Gazebo 实例完成 5 mm 半径、8 s 圆形轨迹，状态到达 `COMPLETED`，实际 TCP 路径记录 441 个采样点。该记录验证仿真控制链路，不代表实机精度。

## 启动问题排查

- `Cannot locate node ... trajectory_tracker_node`：C++ 可执行文件尚未编译到当前加载的工作空间。确认编译白名单已包含新包，编译后重新 `source devel/setup.bash`。给 `.cpp` 文件增加执行权限不能解决该问题。
- 配置阶段提示旧 `moveit_ws` 中找不到 `PlanningScene.msg`：检查 `moveit_msgs_DIR` 缓存是否指向缺少消息源文件的目录。本机已改为完整的 `/opt/ros/noetic/share/moveit_msgs/cmake`，不需要修改消息定义。
- `QXcbIntegration: Cannot create platform OpenGL context, neither GLX nor EGL are enabled`：本机复现原因为 `QT_XCB_GL_INTEGRATION=none`。本包启动入口对 RViz/Gazebo 显式设置 `xcb_glx`，只影响启动的进程，不修改用户 shell；检查已成功创建 OpenGL 4.6 上下文。需要 EGL 的环境可传 `qt_gl_integration:=xcb_egl`。
- `No p gain specified ... /gazebo_ros_control/pid_gains/...`：这是 Gazebo 硬件仿真层的 PID 参数命名空间，与各位置控制器的 `pid` 参数不同。当前模型仍成功加载和运行位置控制器，并完成上述圆形跟踪；这组日志不是此次上层节点缺失的原因。不要为了消除提示直接复制控制器增益到硬件层，以免改变仿真驱动方式。

轨迹跟踪节点设置为 `required`，意外退出时整套 launch 会关闭，避免界面仍在运行却没有跟踪控制节点。
