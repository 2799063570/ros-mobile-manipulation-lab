# 毕业论文补充实验工具

本包将已有感知、MoveIt分拣与 `motion_planning` 的真实Python NSG服务连接起来，并记录可复算的数据。新增代码不代表实验已经完成，没有自动生成实测结果。普通分拣入口默认不开启日志、不切换规划器；本包启动入口也不会自动启动分拣或移动到观察位。

## 对照仓库后的结论

| 论文所需证据 | 原有实现 | 本次补充 | 仍须完成 |
|---|---|---|---|
| RGB-D抓取参数 | 已有按RGB时间匹配深度/TF、平面投影角度和米制宽度 | 独立真值CSV与误差统计 | 实际标定、真值测量、角度阈值预注册 |
| 四组合系统消融 | 有YOLO权重切换；NSG在另一仓库 | S1–S4入口、真实NSG适配、随机顺序矩阵 | 每组合重新布置同一场景并完成试验 |
| 完整碰撞复核 | NSG快速模型的`check_self`未实际启用 | 固定完整PlanningScene、整机自碰撞/环境/附着体/约束/限位复核 | 所有历史轨迹配对应场景复核；确认URDF网格和ACM正确 |
| 在线障碍输入 | 分拣有OctoMap；原NSG ROS转换未处理OctoMap | 从占据叶节点、旋转几何、网格转换保守世界AABB | 真实点云噪声、速度、遮挡与规划开销试验 |
| 运行时间和失败归因 | 有控制状态与失败提示 | 嵌套阶段、对象尝试、规划路径、关节反馈、相机内参、版本哈希日志 | 核对反馈话题；人工或独立传感器标注实际成功 |
| 携带物体几何 | Gazebo吸附与释放已实现 | 复核会检查PlanningScene已有的附着体 | Gazebo吸附不等于MoveIt附着体；仍须按实测物体形状同步场景 |

## 部署

在Linux ROS1工作空间中同时放置机器人仓库和 `motion_planning` ROS包，使用其训练环境安装PyTorch及原规划仓库依赖。保持两仓库各自的Git历史，不复制227MB权重进机器人仓库。

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
python3 -m unittest discover -s src/wheeltec_robot/aubo/aubo_thesis_experiments/test -v
```

`motion_planning/scripts/ros_planner_node.py` 的配套修改默认启用严格运行：权重不兼容/缺失、碰撞引擎初始化失败或实际机器人限位缺失时停止启动；通过 `/compute_neural_path/metadata` 公布实际权重SHA256、限位、完整配置JSON、PyTorch/NumPy版本及关键源码/碰撞资源哈希。不要用实验性C++插件替代Python NSG：原插件存在退回均匀采样/普通RRT-Connect的路径，尚不能直接当作论文NSG。

ROS/C++部分已在Linux ROS Noetic环境独立编译，并通过最小模型几何测试；完整服务联调和真实NSG仍待验证。工具按Python 3.8+使用。

已完成检查与部署机验收步骤见 [VALIDATION.md](VALIDATION.md)。

## S1–S4配对试验

| 条件 | 视觉权重 | 关节空间规划 |
|---|---|---|
| S1 | 同一CGD划分训练的YOLO11n-OBB | MoveIt RRT-Connect |
| S2 | 同一划分的GC-YOLO | MoveIt RRT-Connect |
| S3 | 同一划分的YOLO11n-OBB | 真实Python NSG |
| S4 | 同一划分的GC-YOLO | 真实Python NSG |

先启动机器人/仿真、MoveIt、相机和标定TF，关闭旧的分拣与检测节点，避免重复发布。再启动本入口。例如S4（路径按Linux部署位置替换）：

```bash
roslaunch aubo_thesis_experiments run_experiment.launch \
  condition:=S4 run_id:=batch01_hard01_seed42_r1_S4 scene_id:=hard01 \
  model_path:=/home/zlab/models/GC-yolo.pt \
  checkpoint_path:=/home/zlab/catkin_ws/src/motion_planning/motion_planning/weight/best_cvae_model.pth \
  planner_python:=/home/zlab/anaconda3/envs/planning/bin/python \
  output_dir:=/home/zlab/thesis_runs/batch01_hard01_seed42_r1_S4
```

仿真需要 `use_grasp_attachment:=true`；实物Inspire夹爪需 `gripper_backend:=inspire`。用 `camera_mount`、`table_z`、`object_height`、`task_config` 指定实际配置，桌高/物高的launch参数会覆盖任务YAML；不要直接沿用默认尺寸。`use_detected_angle`、`use_detected_width` 控制几何输出的使用，四组比较时保持一致。上面命令不自动启动动作，按原面板观察/开始。日志目录必须不存在，防止混入旧批次。

S1/S2不启动神经节点。S3/S4缺少真实NSG、权重元数据或完整复核服务时不能获得可执行候选。NSG候选先在请求时固定场景复核；任务侧在执行前再次获取场景并复核，然后才交给原MoveIt执行。

```bash
rosrun aubo_thesis_experiments make_experiment_matrix.py \
  --scenes easy01,hard01 --seeds 42,43,44,45,46 --repetitions 3 \
  --prefix batch01 --output matrix.csv
```

矩阵只是实验顺序与配对编号，不自动复位物体或启动动作。每一行前恢复物体/障碍布局，并保存场景照片或仿真状态。`run_seed`是批次标签；NSG通过launch的`seed`设定Python/NumPy/PyTorch RNG，矩阵不会自动设置OMPL进程的随机种子。未控制OMPL RNG时不要写成严格的同随机流比较。两类规划器的接近/抬升/放置Cartesian段继续共用原代码；对比的是关节空间规划阶段替换。

## 输出与统计

- `manifest.json`：任务参数、视觉权重SHA256和元数据；支持配置`~repositories`、`~hash_files`记录更多版本。机器人URDF、SRDF和MoveIt规划配置独立保存。
- `events.jsonl`：对象尝试、嵌套阶段、失败类别、检测和关节误差。阶段耗时用单调时钟计算，ROS时间用于与传感器对齐。
- `trajectories.jsonl`：实际准备执行的RobotTrajectory与起始RobotState。
- `audits.jsonl`：S1–S4执行前每次复核的完整场景、实际起点和候选路径，连同尝试编号及通过/拒绝结果一起记录。每一行都是独立的JSON，可提取后交给复核CLI；不依赖不同话题的接收顺序配对。
- 相邻的 `*_nsg/`：每次NSG的请求、完整固定场景、复核和响应，可追溯该候选使用的权重。

`controller_success`只表示控制流程返回成功。用 `config/attempt_annotations.csv` 对照录像或独立传感器填写类别正确、夹持成功、正确放置三个布尔标签，按 `attempt_id` 匹配；不标注时只输出控制完成率，最终分拣成功率保持空值。

```bash
rosrun aubo_thesis_experiments evaluate_experiment.py events events.jsonl \
  --annotations attempt_annotations.csv --output system_metrics.json
```

统计保留未完成尝试，不把失败样本丢掉；报告独立标注覆盖率。嵌套阶段同时给出inclusive/exclusive耗时，不能把父、子阶段直接相加。`operation`涵盖等待和恢复；`pick_place`从目标选定后开始，不包含此前检测等待。检测消息的采集到接收延迟也不等于纯网络推理耗时。

目前反馈误差按全部反馈采样合并统计，区别于论文按轨迹先统计再平均。默认话题与本仓库控制器一致，可用`~feedback_topic`覆盖；没有反馈就输出空值，不能填零。

## 三维定位、角度和宽度误差

复制 `config/localization_samples.csv`，在至少覆盖不同位置/角度/距离/类别的独立样本中记录预测与真值。坐标单位m、角度rad，预测与真值使用同一明确坐标系。真值不能直接取生成预测的同一深度值或同一标定拟合点；标定验证应使用留出观测。失败样本也保留，`valid=0`。

```bash
rosrun aubo_thesis_experiments evaluate_experiment.py localization localization_samples.csv \
  --angle-threshold-deg 5 --output localization_metrics.json
```

这里的5°是命令示例，须在实验前按任务要求确定，不能倒推已有97.4%的阈值。短边方向按π周期计算，避免把0°和180°错误视作相反抓取。输出各轴偏差/RMSE、三维距离误差、角度误差、宽度误差和几何有效覆盖率，同时报告全部样本与有效样本条件指标。

## 历史轨迹碰撞复核

```bash
rosrun aubo_thesis_experiments audit_trajectory.py saved_path.json \
  --group aubo_i5 --step 0.02 --output collision_audit.json
```

JSON至少包含完整`start_state`和`trajectory`；建议包含该次试验的完整`scene`、路径和目标约束。缺场景时复核的是当前场景，不能拿它证明历史轨迹当时有效。附着体以完整场景为准，单独写在start_state中的附着体不能代替场景同步。服务返回首次无效段、检查样本数和理由。采样间距按六维关节L2距离控制，包含实际起点到首路点；自碰撞遵守SRDF/场景ACM。它仍是离散几何检查，不是连续碰撞、制动或动态障碍安全证明。

完整OctoMap场景日志可能较大，应预留磁盘空间，并保持四组使用相同记录配置；该记录开销属于本实验入口的实际运行开销。

NSG输入采用保守AABB，旋转物体和网格可能明显膨胀，影响成功率/耗时；超过障碍上限会明确失败，不静默丢弃OctoMap叶节点。物体接触、携带物体、机器人自过滤、障碍物新鲜度、执行中更新/停止策略仍须在真实平台验证。

## 论文中仍不能直接填结果的部分

相机内外参与留出标定残差、定位/开度真值、四组合实测结果、真实障碍闭环测试、全量历史场景对应、训练划分泄漏审计和原始CSV版本差异，都需要真实材料或实际运行。新增工具只补采集与验证能力，不把旧的270次视觉试验或预规划回放改称已完成四组合在线实验。
