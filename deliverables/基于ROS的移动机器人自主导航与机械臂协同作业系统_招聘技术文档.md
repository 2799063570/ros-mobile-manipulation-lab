# 基于 ROS 的移动机器人自主导航与机械臂协同作业系统

> 招聘展示技术文档｜底盘通信 · 自主探索 · 定位导航 · 主从编队 · 移动操作

| 项目信息 | 内容 |
| --- | --- |
| 开发环境 | ROS 1、C++、Python、Gazebo、RViz、MoveIt；仓库中有 Melodic 相关依赖 |
| 平台 | WheelTec 底盘/树莓派与下位机通信实践；差速及 AUBO 复合机器人仿真；多机器人编队功能包 |
| 建议展示方式 | 先展示自主探索建图和导航，再展示到站后视觉识别与机械臂分拣，最后说明主从编队 |
| 项目时间 | **待核对：原始简历写作“2026.02 – 2025.06”，起止顺序相反** |
| 个人职责 | **投递前按实际参与模块填写，不应将仓库中所有模块视为个人独立开发** |

## 一、项目概览

项目面向未知室内环境中的建图、移动导航、多机器人编队，以及机器人到达工位后的识别和抓取。工程由几条可以独立演示的链路组成：WheelTec 实机底盘通信与位姿估计、自主差速机器人及复合机器人仿真、RRT 前沿探索、多机器人主从跟随、AUBO 移动底盘与机械臂的顺序协同。它们共用 ROS 话题、TF 和导航接口，但**不同场景的硬件、传感器及局部规划器配置不能混为一次实验**。

原始简历给出的结果为：在约 **20 m²** 室内环境中，地图覆盖率 **96.3%**、导航成功率 **96.7%**、位置误差 **≤0.1 m**、航向误差 **≤0.1 rad**。仓库中没有找到与这组数字直接对应的原始 rosbag、目标点清单、试验次数或统计脚本，因此本文件将其列为**待补证的简历口径**，不作为代码本身已经证明的性能结论。正式投递前应补齐第八节所列记录。

### 技术栈

| 类别 | 技术与工具 | 项目中的用途 |
| --- | --- | --- |
| 开发语言与平台 | C++、Python、ROS 1、Linux、树莓派 | ROS 节点开发、任务编排与底盘上位机运行 |
| 通信与机器人基础 | 串口通信、ROS Topic/Service/Action、TF、URDF/Xacro | 下位机数据收发、模块解耦、坐标变换与机器人建模 |
| 状态估计与建图 | 编码器反馈里程计、IMU、`robot_pose_ekf`、GMapping | 底盘位姿估计与未知环境二维建图 |
| 自主探索与导航 | RRT-Exploration、AMCL、`move_base`、代价地图、DWA/TEB | 前沿目标生成、地图定位、路径规划和局部避障 |
| 多机器人协同 | 主从状态广播、UDP、槽位跟踪、定位质量门控 | 多车编队及通信超时保护 |
| 视觉与机械臂 | OpenCV、RGB/RGB-D 相机、MoveIt、`ros_control` | 目标识别定位、抓取规划与分类放置 |
| 仿真与调试 | Gazebo、RViz、rosbag、catkin | 场景验证、数据可视化、记录与构建 |

其中 DWA 与 TEB、RGB 与 RGB-D 分别对应不同的场景配置；对外介绍时应以实际演示场景的启动参数为准。

### 系统能力与仓库落点

| 能力 | 具体链路 | 主要目录 |
| --- | --- | --- |
| 实机底盘接入 | 串口收发、速度反馈、IMU 发布、里程计积分、超时停车 | `turn_on_wheeltec_robot/` |
| 建图与导航 | GMapping、AMCL、`move_base`、代价地图、DWA/TEB 场景配置 | `simple_diff_robot_gazebo/`、`aubo_mobile_robot/aubo_mobile_navigation/` |
| 自主探索 | 全局/局部 RRT 前沿、聚类过滤、信息增益和距离效用、目标分配 | `rrt_exploration/` |
| 主从编队 | 主车状态广播、从车槽位跟踪、定位门控和本地避障 | `wheeltec_multi/` |
| 移动操作 | 到站、精停、观察、识别、抓放与任务状态管理 | `aubo_mobile_robot/aubo_mobile_nav_sorting/`、`aubo/aubo_sorting_core/`、`aubo/aubo_perception/` |

### 端到端数据流

```text
底盘编码器/IMU → 下位机 → 串口协议 → /odom、/imu
                                      └→ robot_pose_ekf → odom_combined → base_footprint
激光 /scan ──→ GMapping → /map → RRT 前沿检测 → 聚类/过滤 → 目标效用评价
         └──→ AMCL（已有地图时）              └→ move_base → /cmd_vel → 底盘

导航至工位 → 低速停靠及安全校验 → 机械臂观察位 → 相机识别/目标定位
                                               └→ MoveIt 抓取/放置 → 任务结果

主车位姿/速度 → 主车状态广播 → 从车定位与槽位控制 → 本地避障 → 从车底盘
```

建图模式由 GMapping 维护 `map → odom`；已有地图的定位模式由 AMCL 维护该变换。启动文件应保证两者不同时竞争同一 TF。实机底盘配置另有 `odom_combined` 和 `robot_pose_ekf`，下文分别说明。

## 二、底盘通信与位姿估计

WheelTec 底盘节点订阅速度指令，将线速度、角速度编码为串口帧发给下位机；接收帧经帧头、帧尾和校验检查后，解析底盘速度、板载 IMU 数据及电源信息。底盘节点按采样周期对平面速度积分，发布 `/odom`，同时发布 `/imu`。这一实现的关键是**上位机串口桥接、数据解析与速度积分**：源码并未展示上位机直接读取编码器原始脉冲，因此简历中“融合编码器与 IMU 构建里程计”应表述为“使用底盘反馈里程计与 IMU，经 `robot_pose_ekf` 融合”，避免暗示自行实现编码器计数或完整融合算法。

实机启动配置默认串口别名 `/dev/wheeltec_controller`、波特率 `115200`。它配置串口超时、重连，以及 `/cmd_vel` 指令超时；重连后先发送停止帧，防止旧速度命令恢复执行。`robot_pose_ekf` 将里程计和 IMU 作为输入，输出 `odom_combined → base_footprint`。这条链路应结合实际车型确认坐标系、轮向和速度符号，不能直接搬用仿真机器人的 `odom → base_footprint`。

| 接口 | 含义 | 核查点 |
| --- | --- | --- |
| `/cmd_vel` | 底盘速度输入 | 停止时是否归零、串口断连后是否保持停车 |
| `/odom` | 底盘反馈积分位姿 | 时间戳、坐标系、运动方向与实际一致 |
| `/imu` | 下位机 IMU 数据 | 角速度、线加速度单位及安装方向 |
| `odom_combined → base_footprint` | 实机融合位姿 TF | 是否只有一个节点发布此变换 |

对应源码：`turn_on_wheeltec_robot/src/wheeltec_robot.cpp`、`turn_on_wheeltec_robot/launch/include/base_serial.launch` 和 `turn_on_wheeltec_robot/launch/include/robot_pose_ekf.launch`。

## 三、自主建图、定位与导航

### 建图与定位如何衔接

未知环境探索时，激光扫描、底盘里程计和 GMapping 共同生成二维占据栅格；地图中 `-1` 表示未知、`0` 附近表示自由、较高值表示障碍。探索完成后保存地图文件；复用地图导航时，由 `map_server` 加载地图，AMCL 根据激光与运动估计机器人在 `map` 下的位姿，`move_base` 使用全局和局部代价地图规划到目标点。

仓库有两套值得区分的场景配置：`simple_diff_robot_gazebo` 的 RRT 演示在建图过程中启动 `move_base`，默认使用 DWA；AUBO 复合机器人导航场景先融合前后激光，再加载静态地图和 AMCL，默认也可使用 DWA，四工位场景可按配置切换 TEB。前后雷达融合属于复合机器人仿真配置，不应描述成所有 WheelTec 实机车型的必备硬件。

### 路径执行与避障

`move_base` 的 action 接口承接目标位姿，导航栈使用代价地图避障并输出速度。复合机器人导航入口把规划输出先送到 `/cmd_vel_raw`，再经过激光安全过滤节点输出 `/cmd_vel`。任务节点近场直行停靠也走这条安全过滤链路。导航到达工位预停靠点后，任务层会结合桌边间距、机械臂可达性和相机视野选择精停位姿；这样导航终点不仅是底盘可达点，也要服务于后续抓取。

### 常见故障与定位顺序

1. 先检查 `/scan`、`/odom`、`/tf` 时间戳和坐标系；激光与地图错位时不要先调规划器增益。
2. 检查 `map → odom → base_footprint → base_link` 的发布者是否唯一，确认建图与 AMCL 模式没有并行抢占 TF。
3. 在 RViz 查看全局/局部代价地图、机器人轮廓和目标是否可达；机械臂伸展期间的实体轮廓不能按收臂轮廓继续移动。
4. 如果靠桌时局部规划反复转圈，先检查预停靠目标、桌边净距和激光安全层，再评估近场精停参数。

## 四、RRT 前沿自主探索

仓库的 `rrt_exploration` 采用全局和局部两类 RRT 检测器寻找自由区域与未知区域的边界。全局树面向较大区域采样，局部树从机器人周边重建，提高近场前沿响应。检测器向 `/detected_points` 发布候选；过滤节点使用 MeanShift 聚类，将候选转到地图坐标后，结合全局代价地图、障碍邻域和信息增益剔除不可用目标；分配节点再为候选计算效用并发送给导航 action。

效用计算可以概括为：

```text
候选效用 = 信息增益 × 权重 − 机器人到候选点的距离代价
```

代码还对附近目标施加滞回增益，以减少频繁切换区域；多机器人模式会考虑已分配目标并折减重复信息增益。信息增益由候选点周围的未知栅格面积估计。这里的“RRT”用于**发现前沿**，路径执行仍交给 `move_base`，不是用这两个检测节点替代全局导航规划器。

| 阶段 | 输入 | 输出与意义 |
| --- | --- | --- |
| 全局/局部检测 | `/map`、机器人 TF | 原始前沿候选 `/detected_points` |
| 聚类与过滤 | 候选、地图、全局代价地图 | `/filtered_points`，降低重复、障碍附近和低收益目标 |
| 目标分配 | 过滤后的候选、各机器人位姿 | 信息收益与移动代价折中后的导航目标 |
| 底盘执行 | `move_base` action | 到达前沿并触发地图更新 |

仓库保留了 RRT-Exploration 相关参考实现。招聘描述宜说“集成并配置探索框架、完成过滤/分配联调及场景验证”，具体原创改动需结合提交记录和个人工作确认。

## 五、主从编队

`wheeltec_multi` 是与单机探索并列的多机器人能力。主车通过状态广播提供二维位姿和底盘速度；从车按照相对主车或地图坐标系定义的槽位偏置生成跟随目标，结合自身定位输出底盘速度。最新协议携带序号和发送时间，接收端可识别重复、乱序及数据过期；状态超时则停车。编队执行前还可要求真实初始位姿和 AMCL 收敛，避免从车在定位不确定时运动。

从车控制包含位置反馈、受限积分、速度与加减速限制。避障节点会修改最终速度，并向控制器发布避障状态，以便冻结位置积分；解除避障后逐步回到槽位。编队偏置 `slave_x/slave_y` 分别表示前后、左右距离，`formation_frame=leader` 时随主车朝向旋转，`formation_frame=map` 时固定在地图方向。该模块需要共享地图、可靠定位与时钟同步，不能把多车跟随效果归因于 RRT 探索算法。

演示时建议显示 `/leader_state`、`/formation_status`、`/amcl_pose`、`/cmd_vel` 与 `/avoidance_active`，分别解释通信新鲜度、槽位误差、定位状态和避障后控制输出。

## 六、移动底盘与机械臂协同作业

### 任务主线

复合机器人以顺序状态管理组织底盘和机械臂。场景启动后，机械臂先收至运输姿态，底盘调用 `/move_base` 前往工位预停靠点；任务层再执行近场对正与低速精停。完成停靠后，分拣模块建立桌面碰撞场景、切换工作准备姿态，机械臂到相机观察位识别目标，随后使用 MoveIt 规划抓取和分类放置。最后机械臂收回、底盘后退或返回出发点。四工位模式复用这套流程逐桌执行。

```text
IDLE → STOWING_ARM → NAVIGATING → ALIGNING_BASE / DIRECT_DOCKING
     → PREPARING_ARM → VALIDATING_DOCK → AT_WORKSTATION
     → SORTING → WORKSTATION_COMPLETE → [下一工位 / 返回出发点] → SUCCEEDED
```

实际状态还包括初始化、调整底盘、停止、停止未确认和失败；以上为便于面试讲述的主路径。精停位姿不仅检查底盘与桌边净距，也检查目标落在相机检测范围内，并尝试通过真实 MoveIt 规划验证机械臂可达性。候选失败时先收臂再尝试下一位姿。

### 感知与抓取链路

基础颜色识别按红、绿、蓝 HSV 阈值分割，结合轮廓面积、长宽比等条件过滤背景。复合机器人当前颜色配置还启用 RGB-D 顶面深度，用相机内参及 TF 得到 `base_link` 下的物体位置。旧的纯 RGB 固定平面求交说明仍出现在部分早期文档中，但当前配置 `use_depth: true` 且 `require_depth: true`，招聘展示应以实际启动配置为准。机械臂依次执行观察、张爪、预抓取、下降、闭爪、抬升、移动到分类位置、下降释放和回退。

### 协同约束与异常处理

底盘运动前机械臂回到运输姿态；机械臂观察和抓放期间底盘保持静止。导航、停靠和退出动作均由任务层管理，近场速度仍经过激光安全过滤。任务节点提供启动、停止和状态接口；停止请求需要确认机械臂动作终态与底盘锁状态，不能只凭服务请求返回就认为硬件已停稳。定位 TF 超龄、停靠无进展、分拣超时或停止未确认时会进入相应失败/保护状态。

| 接口 | 用途 |
| --- | --- |
| `/nav_sorting/start`、`/nav_sorting/stop` | 启动与停止整体任务 |
| `/nav_sorting/state` | 总任务阶段及结果 |
| `/move_base` | 工位导航 action |
| `/cmd_vel_raw` → 激光安全过滤 → `/cmd_vel` | 导航及近场底盘速度链路 |
| `/sorting/detections`、`/sorting/state` | 识别目标和分拣子任务状态 |
| `/sorting/base_locked` | 机械臂作业期间底盘锁状态 |

该系统实现的是**顺序协同与工位精停**，不等同于底盘和机械臂同时运动的全身联合规划。仿真中使用的相机、碰撞场景和抓取辅助能力也应与实机效果分开介绍。

## 七、工程实现要点与面试讲法

| 面试主题 | 可以说清楚的技术选择 | 可展示的证据 |
| --- | --- | --- |
| 为什么要融合 IMU 与轮式里程计 | 轮速积分会积累漂移，IMU 有助于约束短时姿态变化；还需校准坐标与协方差 | 串口节点、`robot_pose_ekf` 配置、TF/RViz |
| 为什么探索目标需要过滤 | 原始前沿可能重复、贴近障碍或信息收益低，直接导航会反复失败 | RRT 检测器、`filter.py`、`assigner.py` |
| 为什么工位需要两段式到达 | 全局导航目标只保证底盘接近；抓取还要求桌边净距、相机视野和机械臂可达 | `scenario.yaml`、任务状态与精停实现 |
| 为什么要有底盘/机械臂互锁 | 机械臂伸出会改变碰撞范围，底盘运动也会使感知坐标过期 | `/sorting/base_locked`、状态机与停止处理 |
| 为什么编队要处理网络新鲜度 | 延迟或乱序状态会把从车导向过时槽位 | `LeaderState.msg`、`FormationStatus.msg`、超时参数 |

**建议的两分钟讲述**：先介绍任务目标和 ROS 模块划分；再说明底盘反馈、IMU、激光和 TF 如何支撑建图与定位；接着解释 RRT 如何寻找“下一处值得去的未知边界”，由导航栈负责实际路径；最后讲工位前收臂、预停靠、精停、视觉识别和 MoveIt 抓放的状态管理，并指出停止确认及激光安全链路。若岗位偏多机器人，再补充主车状态传输、槽位控制和从车避障。

## 八、验证记录与指标口径

原始简历中的四个数值需要形成可复核记录。建议将环境平面图、试验日期、地图分辨率、有效区域掩膜、目标点清单、每次导航终态和真值测量方法放在同一份记录中，再生成汇总表。**不要把导航参数中的目标容差直接当作实际位置误差**，也不要把 Gazebo 单次成功等同于实机成功率。

| 指标 | 推荐计算口径 | 当前文档状态 |
| --- | --- | --- |
| 20 m² 环境 | 给出可通行区域边界与面积计算方式 | 仅有用户提供的描述 |
| 地图覆盖率 96.3% | 已探明的有效区域栅格数 ÷ 有效区域总栅格数；明确障碍和不可达区如何处理 | 缺原始地图与掩膜统计记录 |
| 导航成功率 96.7% | 成功到达次数 ÷ 发起任务次数；列出目标点、重复次数、超时与失败判定 | 缺逐次 action 结果清单 |
| 位置误差 ≤0.1 m | 最终位姿与外部真值/测量基准的平面欧氏距离 | 缺真值测量方式和分布 |
| 航向误差 ≤0.1 rad | 最终航向与真值的归一化角差绝对值 | 缺真值测量方式和分布 |

展示材料至少包括一段带 RViz 地图与路径的探索/导航录像、一段分拣任务从导航到抓放的连续录像、一个失败或急停场景，以及对应运行日志。若只完成仿真验证，应在视频和简历中注明“Gazebo 仿真”；若实机与仿真均有结果，应分别列出，不能合并成一个成功率。

## 九、源码与配置索引

| 阅读目的 | 文件 |
| --- | --- |
| 串口协议、IMU、里程计和异常停车 | `turn_on_wheeltec_robot/src/wheeltec_robot.cpp` |
| 底盘串口和融合入口 | `turn_on_wheeltec_robot/launch/include/base_serial.launch`、`turn_on_wheeltec_robot/launch/include/robot_pose_ekf.launch` |
| RRT 建图探索演示 | `simple_diff_robot_gazebo/launch/rrt_exploration.launch` |
| 前沿检测、过滤和目标分配 | `rrt_exploration/src/global_rrt_detector.cpp`、`rrt_exploration/src/local_rrt_detector.cpp`、`rrt_exploration/scripts/filter.py`、`rrt_exploration/scripts/assigner.py` |
| 复合机器人导航与激光安全 | `aubo_mobile_robot/aubo_mobile_navigation/launch/navigation.launch`、`aubo_mobile_robot/aubo_mobile_navigation/scripts/laser_safety_filter.py` |
| 导航到分拣工位的启动入口 | `aubo_mobile_robot/aubo_mobile_nav_sorting/launch/mission_gazebo.launch` |
| 任务状态及 C++ 实现 | `aubo_mobile_robot/aubo_mobile_nav_sorting/include/aubo_mobile_nav_sorting/mission_state.h`、`aubo_mobile_robot/aubo_mobile_nav_sorting/src/navigation_sorting_mission.cpp` |
| 工位几何、超时和安全参数 | `aubo_mobile_robot/aubo_mobile_nav_sorting/config/scenario.yaml` |
| 目标检测和机械臂抓放 | `aubo/aubo_perception/scripts/color_object_detector.py`、`aubo/aubo_sorting_core/src/color_sorting_task.cpp` |
| 主从编队 | `wheeltec_multi/README.md`、`wheeltec_multi/msg/LeaderState.msg`、`wheeltec_multi/src/slave_tf_listener_mecanum.cpp` |

## 十、投递前填写清单

1. 更正项目时间，并按实际参与周期填写；原文“2026.02 – 2025.06”不应直接投递。
2. 按个人真实工作明确“负责开发、二次开发、集成配置、联调验证”四类贡献。WheelTec、RRT-Exploration、ROS Navigation、MoveIt 等开源组件应写明集成或改造关系。
3. 为 96.3%、96.7%、≤0.1 m、≤0.1 rad 补齐原始记录；若无法补证，删去具体数字或改成有证据的描述。
4. 标明对应成果来自实机、Gazebo 仿真还是两者均有；分开提供录像及日志链接。
5. 如用于对外分享，补充本人姓名/职责、仓库链接、演示视频和允许公开的现场照片。
