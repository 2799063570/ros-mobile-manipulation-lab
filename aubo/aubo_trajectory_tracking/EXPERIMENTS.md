# 跟踪实验：自动采集与参数对比

`tracking_experiment.py` 是独立的只读采集器。原面板负责预览、开始和停止；采集器不发布运动指令，
不调用运动服务，不设置参数，不登录 SDK。停止采集器不会停止机械臂。
指令数据订阅独立的 `/trajectory_tracking/command_telemetry` 副本，不增加运动指令话题的订阅者，
保留跟踪节点“只有一个执行订阅者”的原有检查。

## 首次使用

先在工作空间编译并加载环境：

```bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
catkin_make --pkg aubo_trajectory_tracking -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source devel/setup.bash
```

新版跟踪节点增加 `/trajectory_tracking/run_metadata`，在每轮 RUNNING 开始时发布本轮冻结的
曲线参数、起点、起始姿态和运行时间戳。需要正常结束旧节点后重新启动才能使用新接口。
该接口读取预览时冻结的曲线，不会把运行后改过的面板值当成历史实验参数。

当前整定值单独保存在 `config/tracking_experiment_baseline.yaml`，不会改变原默认配置。
需要复现 120 mm / 40 s 的已试验任务，可在正常结束旧节点后显式加载它：

```bash
roslaunch aubo_trajectory_tracking tracking_real.launch \
  config:="$(rospack find aubo_trajectory_tracking)/config/tracking_experiment_baseline.yaml"
```

此命令是原实机入口，会执行原 SDK 初始化；启动后仍需操作者检查、预览并显式开始。

旧节点也能采集，但记录会标记 `parameters_not_frozen`，只作诊断，不进入严格的重复实验汇总。
采集器中途加入的轮次会标记 `collector_joined_late`，不能恢复之前未订阅的六轴历史。

## 采集

机械臂按原入口启动；在另一个终端先启动采集器，再用面板进行实验：

```bash
source /home/zlab/aubo/ros_mobile_manipulation_lab/devel/setup.bash
rosrun aubo_trajectory_tracking tracking_experiment.py record \
  --output /home/zlab/aubo/ros_mobile_manipulation_lab/records/tracking_experiments \
  --label P2_FF010
```

也可使用只启动采集器的 launch：

```bash
roslaunch aubo_trajectory_tracking collect_experiment.launch \
  output:=/home/zlab/aubo/ros_mobile_manipulation_lab/records/tracking_experiments \
  label:=P2_FF010
```

`label` 只是说明，不用于推断增益，真实数值来自冻结参数。
不用每轮导出或重启采集器：修改面板、重新预览并显式开始后，程序自动新建一个 run 目录。
轮次由进入 RUNNING 时的 `run_start` 识别；未成功进入 RUNNING 的使能尝试不会创建实验轮次。
每轮终态后默认再收集 2.5 s，保存现有关节记录器的 2 s 减速尾段；原始事件也保留到尾段结束。
更新预览不会删除已经写入磁盘的上一轮数据；若提前开始下一轮导致尾段不完整，会明确标记。

需要自动结束采集时，可加 `--runs 3`，表示收完三轮有 TCP 数据的记录后退出，包含故障轮次；
不是自动运行三次机械臂。`--duration 60` 限制采集的真实时间。默认持续采集，按 Ctrl+C 结束。
退出时自动生成已记录轮次的对比 CSV、JSON 和图。`--no-plots` 可关闭画图。

当前已试验的任务基线是 **P=2、I=D=0、加速度前馈=0.100 s、120 mm 半径、40 s**。
保持相同起点和底层配置，先用该组重复三次，再每次只改变一个量。
该基线仅对应已试验的任务，不自动写入默认 YAML，也不自动应用到机械臂。

## I/D 扩展统计与离线筛查

已有完整记录可直接批量分析，无需重新启动或修改机械臂参数：

```bash
rosrun aubo_trajectory_tracking tracking_experiment.py pid-study \
  /home/zlab/aubo/ros_mobile_manipulation_lab/records/tracking_experiments \
  --output /home/zlab/aubo/ros_mobile_manipulation_lab/records/tracking_statistics_20261102/id_study
```

默认回放 6 个 Ki（0、0.005、0.01、0.02、0.05、0.1）与 5 个 Kd
（0、0.01、0.02、0.05、0.1）的 30 种组合，P 与前馈保持本轮冻结值。
可以通过 `--ki-grid`、`--kd-grid` 显式提供网格，范围与原控制器一致。
输出平均有符号 XYZ 偏差、误差积分指标 IAE/ISE/ITAE、阈值采样占比、分阶段误差、
参考减趋势速度误差、频谱描述及每个候选的 I/D 修正量、限幅占用、抗饱和和窗口敏感度。

输出目录内有 `I_D批量分析.md`，每轮分别保存 `id_report.md`、`id_statistics.json`、
`id_candidates.csv`、`id_diagnostics.png`、审计用的 `replay_inputs.csv` 和若干候选 YAML。
候选文件保留 P、前馈、轨迹与限额，只分别改变 I 或 D；程序不会应用它们。

**这是在同一条已观测轨迹上的控制信号回放，不是机器人模型仿真，也不是 I/D 闭环实测。**
输入轨迹没有随候选参数重新演化，因此不输出候选的新轨迹峰值/RMS，不排序“最优 PID”，
不以不触发限幅证明稳定。误差频谱包含变化的参考运动，不直接认定为机械振动。
三维趋势速度由接收到的 TCP 样本重建，遗漏的控制周期、运行前窗口历史以及窗口边界误差会影响 D；
报告给出与原记录速度模长的重建差异。新 I/D 的实际效果需要新的同工况记录。

数据与候选信号算法测试位于 `test/tracking_pid_study_test.py`。
本次还编译 `test/pid_replay_reference.cpp`，直接调用生产 `TrackingCorrection`，
验证 Python 回放在限幅、抗积分饱和、积分释放和末点保持情况下与生产实现一致；不调用 ROS/SDK。

## 自动闭环实机矩阵（会请求运动）

`run_tracking_experiments.py` 与只读采集器不同：只有显式传入 `--execute` 才会设置外环参数，
并调用现有 preview/start/stop 服务。缺省只输出计划，不接入 ROS 或运动。
用于已明确授权的实验，不绕过原检查，不自动调用 reset、HOME、SDK 登录或节点重启。

实机模式固定检查已试验的 120 mm / 40 s / xy 单圈任务、P=2、前馈=0.100 s、
速度上限 0.07 m/s 和启用的完整曲线检查。轨迹、速度及底层限制不随 I/D 改变。
六组为基线、I=0.02、I=0.05、D=0.01、D=0.02、I/D=0.02/0.02；默认两轮正/反序排列，
共 12 次运动。每轮前等待 DISABLED 与关节停稳，检查起点关节/TCP 漂移，调用预览复核，
运行后核对冻结参数，等待终态、停稳和采集尾段保存。

运行中检测到故障、反馈/状态过期、参数被外部改变、TCP 误差超过 6 mm、采集失败或超时，
会终止余下矩阵；若本轮运动已请求，则发送停止/禁用请求，不自动复位重试。
停止请求和 SDK 返回不证明机械臂已物理停稳；异常时保留日志供现场核查。
结束后恢复启动前的四个外环参数，输出 `experiment.json`、服务日志、独立完整采集和对比报告。

```bash
# 只看计划，不接入 ROS
rosrun aubo_trajectory_tracking run_tracking_experiments.py --output /tmp/pid_plan
# 以下命令实际控制已启动的真机；需已明确授权并保持实验空间条件。
rosrun aubo_trajectory_tracking run_tracking_experiments.py --execute --backend sdk \
  --repeats 2 --output /home/zlab/aubo/ros_mobile_manipulation_lab/records/tracking_real_pid_experiments
```

自动矩阵使用自己的只读采集进程，不停止用户原有采集器，不占用运动指令话题的订阅者。
矩阵运行时保持面板参数和操作不变；其他操作会作为干预导致本批中止。
发生中断后，只有在现场确认物理停稳、监督恢复原起点和服务后，才可显式使用
`--resume 上次experiment.json` 补测未完成项；运行器本身不会重启节点或回位。
补测仍检查原始起点 2 mm / 0.02 rad 容差。完成后的 `completion.json` 会阻止对旧中断清单重复补测。
`experiment_runner.test` 只连接私有 ROS 假执行端，覆盖六组运行、超差中止和恢复参数；
`experiment_runner_test.py` 验证任务限制与矩阵设计。它们不连接真实 SDK。

2026-11-02 已实际完成六组各两次、共 12 次有效真机实验；另一次 SDK 发送 112.72 ms
超过 100 ms 预算中断，经操作者确认物理停稳后监督重启、按记录的剩余轨迹分段回位并补测。
整定比较报告位于
`/home/zlab/aubo/ros_mobile_manipulation_lab/records/tracking_real_pid_experiments/real_pid_20261102_171936_3fa153/completed_with_recovery/真机I_D实验报告.md`。
本次 I 项未见 RMS 改善，小 D 项约 1% 的差异小于首尾基线漂移，不能声称显著收益。
最后已恢复 I=D=0 并读回确认 DISABLED，原始故障与补测均保留。

## 文件与统计

每个会话都创建唯一目录，避免覆盖旧数据：

```text
session_日期_时间_唯一后缀/
  session.json                  采集器配置、内部队列丢弃计数
  run_运行起点纳秒/
    metadata.json               冻结参数、运行状态、模型哈希、底层配置快照、质量标记
    frozen_curve.yaml           可复用的冻结曲线参数（JSON 兼容 YAML）
    tcp.csv                     16 列，字段与面板 TCP 导出一致
    joints.csv                  34 列，字段与面板六轴导出一致
    servo.csv                   14 列，逆解、平滑、奇异值、限速缩放
    events.jsonl                原始关节反馈、外环指令、状态、记录器状态、相关 rosout
    summary.json                本轮统计、峰值时刻及附近 ±0.25 s 的日志
  comparison/
    comparison.csv              逐轮结果，包含故障、中断、缺数据轮次
    groups.csv                  可汇总轮次按同一参数组合的重复统计
    comparison.json             完整统计与质量标记
    comparison_*.png            每个可比较工况的误差和反馈年龄曲线
    report.md                   对比说明和图表入口
```

所有 CSV 使用 SI 单位（s、m、rad）；报告中的 TCP 误差转为 mm、反馈年龄转为 ms。
TCP 分别统计运行段、末点收敛段及两者合并。`all_peak_mm / all_rms_mm` 与面板范围一致；
`running_peak_mm / running_rms_mm` 用于参数对照，避免不同收敛时长改变对比结果。
RMS 按收到的样本计算，不是按时间加权。报告还包含 XYZ、切向、法向、圆轨迹径向误差、
末点误差、反馈年龄、采样间隔、六轴误差、最小奇异值及限速缩放。

比较时按轨迹尺寸/时长/速度预算、模型、后端、关节约束等分组，再检查起点位置在 2 mm、
姿态在 0.02 rad 容差内；不同组不混合计算重复统计。重复汇总包含每轮 RMS 的均值/标准差、
峰值均值、峰值最坏值和标准差，不把不同轮次的全部采样点混成一条曲线。
同一 label 但实际参数不同仍会分开；缺失或空数据不会填成零误差，也不会自动选择“最优参数”。

只有已 COMPLETED、非中途加入、有冻结参数、运行段覆盖齐全且三个采样流齐全、无已检测
丢弃/无效数据的轮次进入重复汇总。其余轮次仍保留在逐轮 CSV/报告中。
内部队列满时记录丢弃计数，控制线程不会等待采集器；收到重复、非有限或不属于本轮时间范围的
样本会拒绝并计数。采样间隔超过 100 ms 会标记，但这些检查不能证明所有传输层样本均未丢失。

SDK 六轴“生成目标减实测”包含控制柜队列超前，不能直接称为电机伺服误差。
TCP 来自反馈关节角与 URDF 正解，不是外部测量的绝对定位精度。峰值附近日志只用于定位，不能单凭
时间接近就认定因果。本程序不替代底层完整 SDK RPC 时序记录。

## 离线合并比较

可在采集结束后重新生成，或者合并多个 session；纯离线命令不连接 ROS：

```bash
rosrun aubo_trajectory_tracking tracking_experiment.py compare \
  /home/zlab/aubo/ros_mobile_manipulation_lab/records/tracking_experiments \
  --output /home/zlab/aubo/ros_mobile_manipulation_lab/records/tracking_comparison
```

正在采集、状态仍为 OPEN 的目录不会参与比较。采集器因退出而关闭的未完成轮次会显示为中断，
不会被当成成功。比较图和数值保留已检测出的数据质量问题。

如果同时需要排查 SDK 发送/诊断耗时，可继续通过 `tracking_real.launch timing_trace_path:=新的绝对路径`
启用原时序记录；该文件要在 SDK 节点正常退出后才能读取，并需检查容量外丢弃数。
建议每个完整 SDK 时序会话只采所需的一轮，不把一次长时间运行的有限容量记录当成所有轮次的历史。

## 验证

```bash
/usr/bin/python3 src/aubo/aubo_trajectory_tracking/test/tracking_experiment_test.py
rostest aubo_trajectory_tracking experiment_capture.test
rostest aubo_trajectory_tracking tracking.test
```

第一项为无 ROS 的数据层测试，第二项仅发布合成遥测验证采集、分轮、故障保留和文件输出；
第三项使用原有假执行端验证冻结参数接口及跟踪行为。均不登录 SDK或运行实机动作。

本次验证通过：12 项数据层测试、双轮 ROS 遥测采集测试（含故障轮次）、原跟踪集成测试；
另将采集器与完整 AUBO 模型的原准备/跟踪假执行端一起运行，成功保存 TCP、六轴、链路诊断及
独立指令遥测副本，未增加运动话题订阅者。包内生产节点编译和离线 rosrun 对比入口已验证。
