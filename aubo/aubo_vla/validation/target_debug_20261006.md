# 2026-10-06 目标环境调试

首次在 Ubuntu / ROS Noetic 目标环境执行，未发现初轮验证范围内需要修复的代码问题。
启动独立 Gazebo 场景，禁用任务、感知、自动观察位与自动开始；结束后关闭本次启动的节点和模型服务。

## 环境与结果

| 检查 | 结果 |
| --- | --- |
| `catkin_make --only-pkg-with-deps aubo_vla` | 通过，Python 模块及节点包装器生成 |
| 系统 Python 单元测试 | 37 项全部通过 |
| GPU | RTX 5060 Ti，驱动 580.82.09，16 GiB |
| 模型运行时 | torch 2.9.1+cu128，CUDA 12.8，bitsandbytes 0.48.2 |
| `check_gpu.py` | BF16 矩阵乘法及 NF4 均通过 |
| Gazebo 图像 → 真实模型 → ROS 诊断 | `observed`，`mock=false`，七维有限动作；一次诊断推理 345.8 ms |
| `check_ros_lifecycle.py` | 时钟回退清空帧、暂停停止请求、恢复继续观察均通过 |
| `check_inflight_pause.py` | 跨暂停/恢复的在途响应被丢弃 |
| 静止 episode 采集 | 12 帧，零拒收，包含原始关节反馈、图像时间 TF 与相机标定 |
| `inspect_episode.py` | `passed=true`，11 对有效差分，周期 0.5 s |
| 失败样本导出保护 | 默认拒绝导出；显式 `--include-failure` 后导出 11 行 |
| 记录器暂停/恢复 | 暂停自动 aborted；暂停期间 start 拒绝；恢复后可新建 episode |
| 默认影子预览 | `rejected`，原因 `Model has not declared the AUBO action schema` |
| HTTP benchmark，预热后 5 次 | 中位数 341.6 ms，最大 347.5 ms |
| 独立 `validate_model.py`，预热后 5 次 | `stage=complete`，`passed=true`，中位数 338.6 ms |

独立模型验收使用本机缓存和 `HF_HUB_OFFLINE=1`，固定 revision 为
`47a0ec7fc4ec123775a391911046cf33cf9ed83f`，统计键为 `bridge_orig`。
PyTorch allocated 峰值 4510.99 MiB，约 4.41 GiB。
详细结果保存在 [模型验收](target_model_20261006.json) 与
[HTTP benchmark](target_benchmark_20261006.jsonl)，历史验收文件未覆盖。

## 临时数据

- Episode：`/tmp/aubo-vla-debug-20261006/episode_20261006_105954_74aeb7c0`
- 差分导出：`/tmp/aubo-vla-debug-actions-20261006.jsonl`
- 运行日志：`/tmp/aubo-vla-{model,observe,record,shadow}-20261006.log`
- 独立模型验收日志：`/tmp/aubo-vla-validate-model-20261006.log`

静止 episode 的 outcome 为 failure，用于验证采集链路，不是成功抓放示范。
另有暂停回归产生的 aborted episode；这些临时数据未纳入训练集。

## 发现与范围

沙箱内的 socket 测试与 GPU 访问失败，沙箱外复测正常，无需修改模型或 ROS 代码。
Gazebo 启动报告 `/aubo_i5/gazebo_ros_control/pid_gains/*` 缺失警告，
控制器随后完成加载且反馈正常；本次未验证这些警告对运动控制的影响。

初轮没有执行抓放，也没有真实相机/机械臂验收。未提供 AUBO 微调 checkpoint，
因此只验证影子预览对未声明动作规范模型的拒绝行为；有效候选绘制由单元测试覆盖。
采集运动中的位姿、旋转方向及夹爪开合语义仍需用短抓放示范核对。

## 后续修改与运动采集回归

继续排查时复现 PNG 校验漏洞：只有 PNG 文件头的损坏内容，在校验和与文件
一致时仍被写入并通过离线检查。已在 `dataset.py` 的写入及检查路径共用
完整图像校验，验证 PNG 结构/CRC、像素解码、RGB 模式和 CameraInfo 尺寸。
拒收发生在图片与样本写入前。

新增回归覆盖损坏文件头、缺失尾块、错误尺寸及灰度 PNG，以及离线图片被
替换并重算 SHA256 的场景；修改前这些用例失败，修改后完整 39 项测试通过。

专用 Gazebo 内执行肩关节增量 0.08 rad，并将夹爪从 0 移到 0.20 rad 再返回。
使用原始 `/aubo_i5/joint_states` 反馈与图像时间 TF 采集，结果为 22 帧、零拒收、
21 对有效差分；检查和显式 failure 导出均通过。
相邻 TCP 最大平移约 7.37 mm，最大旋转约 0.02196 rad；夹爪反馈范围
0 到 0.71429，与 0.20/0.28 的标定一致。
详情见 [运动回归](target_motion_20261006.json)，导出保存在
`/tmp/aubo-vla-motion-actions-20261006.jsonl`。

本次运动属于数据链路回归，没有执行抓放或接触物体；保持 failure 调试标签。
PID 缺失警告仍存在，当前 PositionJointInterface 的控制方式未修改。
本轮结束后关闭专用 Gazebo 和记录器。

## 完整抓放采集回归

新增 `launch/grasp_recording_gazebo.launch`，将固定平台三色分拣场景和采集节点组合，
默认关闭自动开始；`tests/check_grasp_recording.py` 负责观察位、指令冻结、采集、
任务启动、世界状态验收和动作导出。该入口仅用于专用 Gazebo；不在节点默认
测试中自动执行运动。

首轮脚本未正确解析任务使用的 ` | ` 状态分隔符，在观察阶段超时；修正后通过，
失败报告保留在 `target_grasp_attempt_20261006.json`。新增状态解析和真实抬升/放置
要求的单元回归，完整 41 项测试通过，包构建与组合 launch 解析通过。

成功报告见 [完整抓放验收](target_grasp_20261006.json)：

| 项目 | 结果 |
| --- | --- |
| 任务 | green、red、blue 依次抓放，回到 down，最终 READY |
| 方块峰值高度 | 约 0.370 m，初始高度约 0.120 m |
| 放置中心 XY 偏差 | 红约 4.47 mm，绿约 2.56 mm，蓝约 2.75 mm |
| 采集 | 169 帧，零拒收，指令固定为三个颜色方块分别归位 |
| 离线检查 | passed=true，168 对有效动作，周期 0.5 s |
| 任务标签 | success，由任务结束、实际抬升和最终世界位置共同验证 |

成功示范已复制到持久目录
`/home/zlab/aubo_vla_data/episode_20261006_112916_24e5c77d`，再次检查通过；
其 `actions.jsonl` 包含 168 行，episode 路径已更新为持久目录。
原始采集目录仍在 `/tmp/aubo-vla-grasp-20261006/`。

Gazebo 接触辅助插件参与此次抓放，不代表实机抓取能力。PID 缺失日志仍存在，
本轮既有位置控制路径完成抓放，不据此宣称其他控制模式或负载已验收。
有效 AUBO 微调模型预览和真实机器人验证仍待进行。

## 一键自动化程序

新增 `scripts/run_grasp_demo.py`，通过 `rosrun` 自动启动独立 ROS/Gazebo 实例、
调用抓放采集工作流、验收导出并关闭自身进程；`--keep-open` 成功后保留窗口直到
Ctrl+C。工作流已从测试入口迁移到 `aubo_vla.grasp_workflow`，原测试入口继续兼容。

实测发现连续分拣在遮挡阶段后只检查源心跳便预留下一个目标，可能立即触发
“valid observation stream unavailable before descent”。已在
`aubo_sorting_core/src/instance_tracking.cpp` 预留入口同时要求新鲜有效观测，
等待新帧或走既有观察位恢复逻辑，保留后续下降前检查。
曾尝试逐色模式，该模式在本轮环境未获取到 red 的新目标；最终配置保留连续分拣。

完整自动化验收见 [一键自动化结果](target_automation_20261006.json)：
176 帧、零拒收、175 对有效动作，三色方块抬升及放置均通过；程序退出码 0。
成功后验证本次 ROS/Gazebo master 端口均已关闭。测试数据在
`/tmp/aubo-vla-automation-validation/episode_20261006_114608_efddd842`。
失败尝试报告和 aborted episode 保留在同一临时根目录，不标为成功。
完整 44 项 Python 测试、15 个实例队列场景与相关包构建均通过。

## 完整 launch 入口

新增 `auto_grasp_recording.launch`，包含专用场景、记录器与
`execute_grasp_workflow.py` 自动控制节点，无需再发 start 服务。
工作流支持 ROS 名称/日志重映射参数，并自动创建唯一验收报告。
任务完成后 launch 保留场景，直到操作员 Ctrl+C。

独立端口实测通过：184 帧、零拒收、183 对有效动作；三色方块抬升、放置及
离线导出均通过。详情见 [完整 launch 验收](target_full_launch_20261006.json)。
44 项 Python 回归、launch 节点解析和包构建通过；验证场景随后关闭。

## 旧场景并存时的 launch 隔离修复

用户日志出现 `entity already exists`、gzserver 退出 255 和控制器服务无响应。
原完整 launch 在当前 master 上重复启动同名场景，导致与保留场景冲突。
现改为从完整 launch 启动隔离编排程序，每次使用独立 ROS/Gazebo master 端口，
外层节点使用匿名名称；GUI/RViz 参数仍可分别配置。
子进程 PATH 优先系统 Python，避免 env-python ROS 工具选中 Conda 环境。
同时限制子场景关闭等待时间，使清理可以在外层 roslaunch 的关闭期限内完成。

在已有 `/gazebo` 节点保持运行时实测，新实例完成三色抓放：185 帧、零拒收，
184 对有效动作，报告 passed=true，原 `/gazebo` 注册地址保持不变。
见 [隔离回归](target_launch_isolation_20261006.json)。45 项 Python 测试通过。
