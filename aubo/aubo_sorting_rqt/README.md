# AUBO 分拣 rqt 工作台

同包新增 **AUBO 视觉伺服调试台**，用于独立 PBVS 的画面、误差曲线、启停、在线调参和相机视线/姿态草稿。
运行 `rosrun aubo_sorting_rqt visual_servo_panel`，使用方法和姿态依据见
[视觉伺服调试台与避遮挡姿态](docs/visual_servo_workbench.md)。
实机 YOLO 伺服测试入口默认 `rqt:=true`；打开界面不自动使能或写参数。

界面连接现有分拣任务。打开和关闭界面均不会启动、停止机械臂或改写任务参数。

```bash
source /home/zlab/aubo/ros_mobile_manipulation_lab/devel/setup.bash
rosrun aubo_sorting_rqt sorting_panel
# 或只启动界面的 launch
roslaunch aubo_sorting_rqt sorting_panel.launch
# 与当前实机工位一起启动
roslaunch aubo_sorting eye_to_hand_can_sorting_real.launch rqt:=true
```

也可从 rqt 的 `Plugins → Robot Tools → AUBO 分拣工作台` 打开。
初次新增插件后，若 rqt 缓存尚未更新，可执行 `rqt --force-discover`。

界面包括：

- 状态与流程：观察、识别入队、抓取、放置、返位；另显示下降、夹紧、抬升、释放及退离等实际子步骤。
- 工位与队列：识别图、桌面 XY 投影、按入队 ID 排列的实例状态和坐标，以及旋转角度（°）、夹取宽度（mm）。确认帧数及观测年龄用于判断目标是否就绪。
- 操作：前往观察位、开始队列分拣、停止任务、前往工作位、返回结束位和夹爪全开。
- 在线调参：速度/加速度比例、抓取 XY 修正、夹爪 TCP 最低离桌高度、检测超时、队列确认帧数、待领取目标有效期，以及重复目标合并距离/面积重叠比例。
- 运动规划算法：在“在线调参与算法”页选择规划管线与具体算法，支持已加载的 NSG、OMPL / RRTConnect 等，点击“应用在线参数”生效，并显示服务器实际选择。
- 过程记录：状态迁移、子步骤、服务接受/拒绝及参数返回结果，最多保留 400 行。

操作顺序：点击“前往观察位”，等待“已就绪”，再点击“开始队列分拣”。
“请求已接受”表示后台操作已启动，不代表动作完成；完成结果以任务状态和失败原因显示为准。
停止任务始终保留独立调用通道，不被其他阻塞服务请求占用；它是软件停止，不能替代现场硬件急停。
控制节点断开、初始化或运动中禁用新的运动按钮。检测到反馈/流故障锁停时也禁用新运动。

参数通过 `/color_sorting_task/set_parameters` 的 dynamic_reconfigure 接口更新，并回读服务器实际结果。
后台在执行中拒绝修改；界面保留未提交编辑，不会被周期性参数消息覆盖。
队列参数改变后要求现有可领取目标重新积累稳定观测，不沿用旧确认。
参数仅在当前节点生效；“导出参数记录 YAML”保存服务器实际值，需自行合并到原任务 YAML 后才会在重启时保留。
新增队列在线参数和子步骤消息需要重启本次编译后的分拣节点；旧节点仍可显示状态、队列并控制已有服务。

算法列表从 MoveIt 的 `/query_planner_interface` 只读获取，按任务的 `planning_group` 过滤；
不会因打开界面而加载管线或改变当前算法。“MoveIt 默认”和“管线默认算法”对应空字符串配置。
后台通过 dynamic_reconfigure 的 `planning_pipeline`、`planner_id` 更新后续关节空间规划，
任务执行中、规划器不存在或查询失败时拒绝切换并回传原值；笛卡尔直线段继续使用原有插补流程。
要在 NSG 与 OMPL 间切换，MoveIt 必须已同时加载这两条管线；仅启动 OMPL 时不会出现 NSG 选项。
使用 `planning_service` 外部规划服务的实验任务暂不支持从此处切换，以原启动入口配置为准。
需编译并重启新版分拣任务节点及 rqt，旧节点会显示升级提示。算法选择也包含在导出 YAML 中。
自定义 MoveIt 查询服务可用插件参数 `--planner-service /your_namespace/query_planner_interface` 指定。

“夹爪 TCP 最低离桌 / m”默认实机值为 `0.06`，作用于抓取和放置下降终点；
必须小于预抓取、最低抬升及预放置高度。全开操作等待 Inspire 最大开度回读确认，
启动界面、停止任务或失败不会自动释放手中物体。

实例表的“检测置信度”来自模型，和“确认帧数”分开显示；合并时保留高置信度完整检测数据。
“旋转角度”是缓存物体短边在队列坐标系（默认 `base_link`）XY 平面内相对 X 轴的角度，
界面将 rad 转为 °；实际 TCP 偏航还会叠加 `grasp_rpy[2]`。
“夹取宽度”是缓存检测的夹取方向尺寸，界面将 m 转为 mm；它不表示夹爪实时开度，
当前 Inspire 实机仍按配置的力度闭合。预留目标的两项数据随执行快照冻结。
几何无效、数值无效或旧节点未发布字段时显示“—”，合法的 0° 会正常显示。
增加这两个字段后需要重启新版分拣任务节点和 rqt；无需因这次显示更新重启硬件驱动。
队列模式旁显示累计合并次数及感知状态：“观测目标”、“预抓取遮挡保护”或“抓放阶段屏蔽”。
前者表示接近中仍跟踪目标；只有采集时刻的机器人 TF 包络挡住相机到目标的视线时，
才进入预抓取遮挡保护。下降、夹紧、抬升、放置及退离阶段按任务流程屏蔽队列更新。
阶段切换会丢弃先前采集但推理迟到的画面，恢复采集后才允许领取下一个目标。

默认连接：

| 数据 | 接口 |
| --- | --- |
| 状态 / 失败 / 检测汇总 | `/sorting/state`、`/sorting/failure`、`/sorting/detection_summary` |
| 实例队列 | `/sorting/target_cache`，schema 2 JSON |
| 过程子步骤 | `/sorting/progress`，包含 `phase/description/wall_stamp` 的锁存 JSON |
| 识别图像 | `/sorting/debug_image` |
| 机械臂诊断 | `/aubo_i5/aubo_hw_node/flow_diagnostics` |

独立运行时可通过插件参数切换接口：

```bash
rosrun aubo_sorting_rqt sorting_panel --task-node /color_sorting_task \
  --sorting-ns /sorting --image-topic /sorting/debug_image
```

界面网络请求在后台线程中执行，断连不会阻塞 Qt 窗口。图像最多转换 5 Hz，队列按最新消息刷新，
不积压图像，也不改变分拣 FIFO 规则。`done` 表示控制器抓放流程成功，不能独立证明实物已放入容器。

离线验证：

```bash
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m unittest discover \
  -s $(rospack find aubo_sorting_rqt)/test -v
```
