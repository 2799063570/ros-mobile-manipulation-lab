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
catkin_make --pkg aubo_ros_control aubo_trajectory_tracking -DCMAKE_POLICY_VERSION_MINIMUM=3.5
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

1. 查看面板“准备状态”，等待关节反馈和点动节点就绪、底层状态为 `DISABLED`。未就绪时会显示具体原因（反馈无效/过期、时钟异常、底层状态超时或未禁用），预览和开始按钮按条件开放。机械臂当前位置应有足够工作空间且避开奇异位形；本包不会自动调整起始关节姿态，移动起点必须显式点击。
2. 设置 `circle` 或 `sine`、平面、尺寸、总时间和速度上限。允许最大位移 `max_extent` 也可在面板修改，圆需要至少为直径；不代表机械臂可达范围。
3. 点击“搜索推荐起点”：从当前姿态、配置 HOME 及其单关节扰动形成 22 个候选，使用同一曲线定义进行连续逆解、关节余量、已知场景碰撞、奇异性和关节速度/加速度预算检查。推荐有限候选中的较优姿态，不声称全局最优。RViz 紫色曲线显示推荐起点对应的完整路径，面板显示起点 XYZ 和检查结果。搜索不移动机械臂。
4. 检查场景后，点击“移动到推荐起点”。移动前重新检查已知场景与到起点的关节直线路径，沿原点动后端的关节控制流程限速、平滑移动，可用“停止”取消。参数或当前姿态改变后，旧推荐会被拒绝。遇到障碍不做全局绕障，需调整当前姿态后重新搜索。
5. 到位后点击“更新预览”：基于实际 TCP 和当前实际关节姿态，重新检查整条曲线，记录起点并冻结本次参数。也可跳过搜索和移动，直接检查并使用当前位置。预览仅在成功后清空上一轮跟踪数据。
6. 检查 RViz 曲线，点击“开始跟踪”：显式使能、发送至少约 0.3 s 零指令、等待新的使能状态确认，再推进轨迹。点击“跟踪曲线 / 导出”查看实时目标/实际路径、XYZ 随时间变化、位置误差与实际速度；支持保存 PNG、导出 SI 单位 CSV。切换“六轴关节 / 链路诊断”可查看六轴位置、误差、速度及控制链路，一键导出四张图。关闭绘图窗口后仍继续接收数据。
7. 完成后等待实际 TCP 收敛，再请求禁用。途中“停止”和关闭主面板都会同时请求取消起点准备并停止跟踪。完成、停止或故障后，须重新预览并显式开始。

Gazebo 入口已设置非零初始关节姿态以避开零位伸直奇异位形。请保持 Gazebo 物理运行；暂停会导致反馈超时。本包不自动回零或复位故障。新版本扩展了 `MoveHome.action` 的可选 `target_joint_positions` 字段，空数组仍使用原 HOME；升级时需重新编译底层节点与相关动作客户端并重启，避免旧动作消息的 MD5 不匹配。

预览后如果 TCP 移动超过到位容差，开始会显示偏移量和允许值。面板修改参数后会禁用开始和移动起点按钮，直到对应的预览或搜索重新成功；直接调用跟踪服务仍采用上一次成功预览冻结的参数。

普通停止会经过点动程序和控制柜已有前瞻点及减速尾段，不等于硬件急停。`COMPLETED` 表示末点位置与估计速度满足上层判据且禁用请求已被接受，不证明柜内队列完全排空或机械臂已经物理停稳。上层进程突然退出时，停止仍依赖底层输入超时保护。

控制器结构、PID/前馈公式、参数和逐图讲解见 [CONTROLLER_GUIDE.md](CONTROLLER_GUIDE.md)；误差对照与数据接口细节见 [CONTROL_ANALYSIS.md](CONTROL_ANALYSIS.md)。

## 曲线与控制

所有曲线相对预览时的 TCP 位置生成；`xy/xz/yz` 都是基坐标系平面。圆的起点在圆周上，圆心位于第一条平面轴负方向一个半径处。

令 `u = clamp(t / duration, 0, 1)`，`s = 10u³ − 15u⁴ + 6u⁵`：

- 圆：`a = radius × (cos(2π × cycles × s) − 1)`，`b = radius × sin(2π × cycles × s)`。整数圈数执行一次后返回起点。
- 正弦：`a = length × s`，`b = amplitude × sin(2π × waves × s)`。执行一次后停在前进距离 `length` 处，不自动往返。

起终点的参考速度、参考加速度为零。上层以 50 Hz 更新：`v = bounded_feedforward(v_ref + tau_ff × a_ref) + bounded(kp × (p_ref − p_actual) + integral + kd × velocity_error)`，纠偏速度限于总速度预算的 20%，再按合速度限幅。可选解析加速度前馈用于补偿平滑滞后，Gazebo 入口默认 0.10 s，实机默认 0。积分具有限幅和条件抗饱和，默认关闭；快速曲线上的积分可能增加相位误差，需根据实测选择。实际 TCP 使用原始关节反馈及与点动相同的 `robot_description`、`base_link → tool_link` KDL 链计算；不使用参考位置充当实际反馈。姿态保持由现有点动节点完成。

默认圆半径 30 mm，总时间 40 s，最大指令速度 15 mm/s，`kp=1.0/s`、`ki=0`。这是带时间参数的轨迹跟踪，上层误差是同一时刻参考位置与实际 TCP 的欧氏距离，不是到整条曲线的最近距离。SDK 前瞻、平滑和通信延迟会造成相位滞后；加速度前馈只对部分平滑滞后作有界补偿，尚未辨识并精确补偿 SDK 队列、通信和真实执行器延迟；实机跟踪精度需现场测量，不能由模拟测试推断。

轨迹相位按 ROS 时间推进，Gazebo 降速时与 `/clock` 一致；回调、反馈、状态、使能确认和收敛超时仍使用真实时间看门狗。统计/绘图将目标位置计算到反馈本身的时间戳，每帧有效反馈最多计入一次，避免用较新的目标与旧反馈比较产生速度相关偏差。控制命令仍使用当前目标与新鲜的实际 TCP 误差；不对实测位置做平滑或以预测值替代。时钟倒退或异常跳变会停止跟踪。

原始速度仍是相邻反馈位置差分，完整保留。另提供默认 80 ms 带时间戳的三维位置线性拟合趋势速度，便于分辨持续运动与高频抖动。停稳判断同时要求原始速度和趋势速度满足阈值，不能依赖平滑掩盖真实尖峰。末点误差小于一半位置容差时进入零指令保持，超过完整容差才恢复修正，以减少抖动驱动的反复启停。

绘图提供参考/指令/原始/趋势速度对比，并显示进入末点收敛阶段的虚线；位置默认显示相对首个参考点的位移，可切回绝对坐标。误差分为总误差、沿参考切向的有符号分量、其余法向分量；法向误差不是整条曲线的严格最近距离。参考速度为零时切向分量记为零，法向分量等于总误差。

预览检查解析速度上界，为回授预留 20% 速度预算。五次时间缩放最大 `ds/dt=1.875/duration`，圆的速度界为 `2π × cycles × radius × 1.875/duration`；正弦使用 `hypot(length, 2π × waves × amplitude) × 1.875/duration`。超限时增加运行时间或减小尺寸。

预览默认启用 `check_feasibility`，由准备节点对固定末端姿态的整条曲线进行离散检查：至少 300 段、按路径长度约 1 mm 加密（最多 5000 段），以前一点逆解为下一点种子，拒绝无解、残差过大或关节跳变。关节路径以最大 0.01 rad 增量采样碰撞，并检查关节限位余量及与底层一致的雅可比最小奇异值阈值。按时间采样的关节速度/加速度估计保留 20% 预算，速度估计考虑底层奇异性缩放，超限时提示建议时长。

这些检查是离散模型预测，不是连续碰撞证明，不模拟 jerk 平滑、DLS 误差、控制柜前瞻或真实动力学，也不保证跟踪误差。执行时保留底层实时制动碰撞和限位保护。准备节点订阅与底层相同的规划场景话题/完整场景服务；Gazebo 模型不会自动转为 MoveIt 碰撞物体，实际障碍物必须发布到规划场景。合成运动学单元测试可显式关闭可行性检查；正常运行保持启用。

## 参数

详见 [config/tracking.yaml](config/tracking.yaml)，加载到 `/trajectory_tracking`。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `shape / plane` | `circle / xy` | `circle/sine`；`xy/xz/yz` |
| `radius / amplitude / length` | `0.03 / 0.02 / 0.10` | m |
| `cycles / waves` | `1 / 1` | 圆圈数 / 正弦波数，整数 1–20 |
| `duration` | `40.0` | 一次完整轨迹时间 s，含起止时间缩放 |
| `kp` | `1.0` | 位置回授增益 1/s，面板可调 |
| `kd` | `0.0` | 短窗速度误差 D 增益，默认关闭 |
| `acceleration_ff_time` | `0.0` | 解析加速度前馈时间 s，Gazebo 入口覆盖为 0.10，限于 0..0.25 |
| `ki` | `0.0` | 有界积分增益 1/s²，默认关闭，面板可调 |
| `correction_fraction` | `0.2` | 纠偏速度预算比例，上限 20% |
| `velocity_window` | `0.08` | 趋势速度拟合窗口 s；原始速度仍保留 |
| `max_speed` | `0.015` | 线速度合量上限 m/s，另受底层限速 |
| `max_error` | `0.04` | 超过此 TCP 误差则中止，m |
| `max_extent` | `0.25` | 相对起点最大几何位移界，m |
| `feedback_timeout / state_timeout` | `0.15 / 0.35` | 原始关节反馈 / 底层状态新鲜度，s |
| `settle_tolerance / settle_speed` | `0.002 / 0.003` | 末点收敛位置 m / 估计 TCP 速度 m/s |
| `settle_timeout` | `15.0` | 轨迹时间结束后最大收敛等待 s |
| `check_feasibility` | `true` | 预览时要求准备节点完成整条曲线离散检查 |
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
| `/trajectory_tracking/readiness` | `std_msgs/String`，`WAITING / PREVIEW_READY / START_READY` 及准备条件说明，持续更新，不使能或移动机械臂 |
| `/trajectory_tracking/state` | `std_msgs/String`，状态及原因 |
| `/trajectory_tracking/reference_path` | `nav_msgs/Path`，锁存的完整参考曲线 |
| `/trajectory_tracking/actual_path` | `nav_msgs/Path`，实际 TCP 路径 |
| `/trajectory_tracking/markers` | `visualization_msgs/MarkerArray` |
| `/trajectory_tracking/samples` | `Float64MultiArray`，`[反馈时间相对运行起点 s, 同时刻目标 XYZ m, 实际 XYZ m, 位置误差 m, 原始实际速度 m/s, 参考速度 m/s, 本次指令速度 m/s, 趋势速度 m/s, 反馈年龄 s, 收敛阶段标志 0/1, 切向误差 m, 法向误差 m]`，共 16 项；前 9 项保持原顺序；预览成功发送空数组作为新轮次标记 |
| `/trajectory_preparation/analyze` | `std_srvs/Trigger`，搜索起点，不移动 |
| `/trajectory_preparation/move` | `std_srvs/Trigger`，复查后显式移动到推荐姿态，异步报告到位 |
| `/trajectory_preparation/check_current` | `std_srvs/Trigger`，检查当前起点对应的整条曲线 |
| `/trajectory_preparation/stop` | `std_srvs/Trigger`，取消搜索、取消起点移动并请求底层禁用 |
| `/trajectory_preparation/state` | `std_msgs/String`，搜索/推荐/移动/到位及失败原因 |
| `/trajectory_preparation/candidate_path` | `nav_msgs/Path`，最近推荐起点的完整曲线，RViz 紫色 |
| `/trajectory_tracking/joint_samples` | 34 项，时间对齐的六轴生成目标、实际位置、误差及速度，详见控制分析 |
| `/trajectory_tracking/servo_samples` | 14 项，逆解/限速/平滑链路诊断，详见控制分析 |
| `/trajectory_tracking/run_start` | `std_msgs/Header`，本轮运行起点时间；预览清零 |
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
rosbag record -O tracking.bag /trajectory_tracking/state /trajectory_tracking/metrics /trajectory_tracking/reference_path /trajectory_tracking/actual_path /cartesian_jog/command /cartesian_jog/state /cartesian_jog/sdk_joint_states /aubo_i5/joint_states /trajectory_tracking/joint_samples /trajectory_tracking/servo_samples /cartesian_jog/joint_target /cartesian_jog/servo_diagnostics
```

## 验证边界

已独立编译，使用三轴直线运动学模型和模拟速度执行端完成 ROS 集成测试：三种平面、圆形/正弦末点、零指令握手、速度界、参数拒绝、反馈/状态丢失、非有限反馈、重复时间戳、超差及主动停止。测试不登录 SDK，不连接或移动实机。

```bash
rostest aubo_trajectory_tracking tracking.test
rostest aubo_trajectory_tracking preparation.test
rostest aubo_trajectory_tracking tracking_clock.test
```

模拟执行端没有复现 SDK 前瞻与真实动力学，实机效果仍需验证。现有实机点动约束参见 [调试复盘](../../docs/AUBO实机点动问题与调试复盘.md)。

2026-10-25 补充验证：已在实际工作空间编译，并使用独立 Gazebo 实例完成 5 mm 半径、8 s 圆形轨迹，状态到达 `COMPLETED`，实际 TCP 路径记录 441 个采样点。该记录验证仿真控制链路，不代表实机精度。

## 启动问题排查

- `Cannot locate node ... trajectory_tracker_node`：C++ 可执行文件尚未编译到当前加载的工作空间。确认编译白名单已包含新包，编译后重新 `source devel/setup.bash`。给 `.cpp` 文件增加执行权限不能解决该问题。
- 配置阶段提示旧 `moveit_ws` 中找不到 `PlanningScene.msg`：检查 `moveit_msgs_DIR` 缓存是否指向缺少消息源文件的目录。本机已改为完整的 `/opt/ros/noetic/share/moveit_msgs/cmake`，不需要修改消息定义。
- `QXcbIntegration: Cannot create platform OpenGL context, neither GLX nor EGL are enabled`：本机复现原因为 `QT_XCB_GL_INTEGRATION=none`。本包启动入口对 RViz/Gazebo 显式设置 `xcb_glx`，只影响启动的进程，不修改用户 shell；检查已成功创建 OpenGL 4.6 上下文。需要 EGL 的环境可传 `qt_gl_integration:=xcb_egl`。
- `No p gain specified ... /gazebo_ros_control/pid_gains/...`：这是 Gazebo 硬件仿真层的 PID 参数命名空间，与各位置控制器的 `pid` 参数不同。当前模型仍成功加载和运行位置控制器，并完成上述圆形跟踪；这组日志不是此次上层节点缺失的原因。不要为了消除提示直接复制控制器增益到硬件层，以免改变仿真驱动方式。

轨迹跟踪节点设置为 `required`，意外退出时整套 launch 会关闭，避免界面仍在运行却没有跟踪控制节点。

跟踪优化验证新增：时间戳对齐、慢仿真时钟与暂停保护、积分抗饱和、末点保持滞回、速度趋势估计，以及存在增益损失/一阶滞后的模拟执行端新旧参数对照。该对照不代表当前 Gazebo 场景或实机的精度承诺。CSV 中位置和误差保持原始测量，速度趋势作为额外诊断列，启用 D 项时也参与速度误差估计；旧 9 列样本仍可绘制，新增诊断缺失时记为 NaN。
