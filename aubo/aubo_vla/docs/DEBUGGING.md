# Ubuntu / ROS Noetic 调试入口

本次推进提供观察、示范采集、离线动作差分检查与 RViz 影子预览。
VLA 节点没有运动发布器、执行 action client 或抓放服务调用。
已有分拣仍由操作员单独启动。没有 AUBO 微调权重时，先完成前两阶段。

## 1. 构建和基础验证

新增 Python 包需要 catkin 构建一次。ROS 终端用系统 Python，不激活模型 venv。
下面从实际 catkin 工作空间根目录运行（本机 Linux 路径以实际部署为准）：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps aubo_vla
source devel/setup.bash
/usr/bin/python3 -m unittest discover -s src/aubo/aubo_vla/tests -v
```

如旧环境没有依赖，用 rosdep 检查 package.xml 中的 rospy、cv_bridge、message_filters、
tf2_ros、visualization_msgs、std_srvs 和系统 PIL。源码离线测试需要 Pillow；不需要
torch、GPU、ROS 或模型下载。ROS/Gazebo、真实 GPU 回归仍要在目标环境执行。

## 2. 观察模式与运行时指令

场景、模型独立环境的原启动方法见包 README。观察-only 场景关闭任务与感知：

```bash
roslaunch aubo_sorting sorting_gazebo.launch camera_mount:=eye_to_hand \
  start_task:=false start_perception:=false auto_move_to_observation:=false \
  auto_start:=false gui:=false rviz:=false
roslaunch aubo_vla observe.launch interval:=0.5 max_age:=2.0 timeout:=5.0
rostopic echo /openvla_observe/diagnostics
```

模型服务需在另一个终端按 README 启动。单独验证通信可以加 `--mock`，诊断为 mock，
不作为模型验收或影子预览依据。

```bash
rostopic pub -1 /openvla_observe/instruction std_msgs/String "data: 'pick up the blue block'"
```

新指令清空待处理帧并使旧指令的在途结果失效。记录器的 `/vla_recorder/instruction`
只改变**下一条 episode**的指令；当前 manifest 固定，不混合多个指令。

真实相机使用已有硬件 bringup，再启动观察节点并传入真实图像话题。
`/use_sim_time=false` 时不等待 `/clock`；`true` 时继续检测仿真暂停/重置。
相机的源时间戳必须正常递增且与 ROS 时间一致，不能重写旧图像时间戳规避检查。

## 3. 用现有分拣采集示范

启动固定平台分拣，默认保留人工启停：

```bash
roslaunch aubo_sorting sorting_gazebo.launch camera_mount:=eye_to_hand \
  auto_move_to_observation:=false auto_start:=false
```

另一个已 source 的 ROS 终端：

```bash
rostopic hz /workspace_camera/color/image_raw
rostopic hz /aubo_i5/joint_states
rostopic echo -n 1 /workspace_camera/color/camera_info
rosrun tf tf_echo base_link tcp_link
roslaunch aubo_vla record.launch output_root:=/home/zlab/aubo_vla_data
rostopic echo /vla_recorder/status
```

默认使用原始 `/aubo_i5/joint_states`，避免 `/joint_states` 转发节点重新给缓存反馈打时间戳。
核对原始消息包含六个臂关节与 `joint1`。相机信息与图像必须有相同 optical frame
和尺寸，TF 必须在**图像时间戳**存在；缺失数据会拒收，绝不使用最新 TF 顶替。
TF 尚未到达时会用单调墙钟最多等待 0.25 s，再跳过样本；持续拒收要检查 robot_state_publisher 的频率、
时间戳和相机静态 TF。记录器只保留最新同步样本，繁忙时不积压。

先进入观察位并确认检测，再开始录制：

```bash
rosservice call /sorting/move_to_observation '{}'
rostopic echo /sorting/state
rosservice call /vla_recorder/start '{}'
rosservice call /sorting/start '{}'
```

观察 `/sorting/state` 和真实画面，任务结束后选择其中一个结果：

```bash
rosservice call /vla_recorder/finish_success '{}'
# 或：rosservice call /vla_recorder/finish_failure '{}'
# 中止：rosservice call /vla_recorder/abort '{}'
```

start 返回创建的 episode 绝对路径。finish_success 不会停止分拣，也不会检查物体是否
真的被抓起；只能在确认任务停止且实际成功后标记。暂停或重置 Gazebo 会自动中止
当前 episode；恢复后新建 episode。若 status 长期停在 samples=0 且没有 rejected，
先查两个输入话题是否到达，源时间差是否在 sync_slop=0.05 s 内。

眼在手上使用对应图像、CameraInfo 话题覆盖参数；实机反馈话题也可覆盖：

```bash
roslaunch aubo_vla record.launch image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info joint_states_topic:=/actual_joint_feedback \
  config:=/absolute/path/to/real_record.yaml output_root:=/absolute/path/to/dataset
```

相机话题与关节话题 launch 参数优先于 YAML。Inspire 夹爪尚未提供反馈转接节点，
需先提供真实、同步的反馈，不能直接套用 Gazebo 的 `joint1` 标定。

## 4. 离线检查与导出

### Gazebo 完整抓放采集回归

完整 launch 入口（启动后自动执行任务）：

```bash
roslaunch aubo_vla auto_grasp_recording.launch
# 无界面运行或指定输出目录：
roslaunch aubo_vla auto_grasp_recording.launch gui:=false rviz:=false \
  output_root:=/home/zlab/aubo_vla_data timeout:=300
```

此 launch 包含仿真、控制器、MoveIt、颜色感知、分拣任务、记录器及自动工作流。
无需再手动调用 start 服务。每次创建独立的 ROS/Gazebo master 和全新场景，
避免旧场景的 `entity already exists`、Gazebo 端口占用和控制器重名冲突。
每次生成唯一 `run_*/`，包含 `run.json`、`report.json` 和 `launch.log`；
episode 内生成 `actions.jsonl`。成功后保留场景供检查，按 Ctrl+C 关闭本次实例。
失败时保留数据和报告并关闭本次场景，不停止旧实例。
手动诊断子场景时，从 `run.json` 读取对应 ROS_MASTER_URI 和 GAZEBO_MASTER_URI，
设置到诊断终端后再使用 rostopic/rosservice。

推荐使用一键自动化程序。在已构建的工作空间根目录执行：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun aubo_vla run_grasp_demo.py --output-root /home/zlab/aubo_vla_data --keep-open
```

默认打开 Gazebo 和 RViz。程序自动启动独立 ROS/Gazebo 实例、等待控制器、
进入观察位、冻结语言标签、录制三色抓放、检查抬升及放置结果、结束录制并导出。
它调用的是既有分拣控制器，语言文本仍用于示范标签。
`--keep-open` 在成功后保留窗口，按 Ctrl+C 关闭本次实例；不加则运行后自动关闭。
无界面验证使用 `--headless`。`--timeout 300` 是每个工作流等待阶段的墙钟上限。

每次生成唯一的 `run_<时间>_<编号>/` 目录，包含 `launch.log`、`report.json` 和
`run.json`；episode 单独保存在 output-root 下，`actions.jsonl` 位于 episode 内。
报告 `passed=true`、`stage=complete` 且程序退出码 0 表示通过。
失败或 Ctrl+C 时先尝试停止分拣、中止录制，再关闭本次启动的进程；已有数据保留。
程序使用独立端口，不停止其他终端启动的 ROS/Gazebo 实例。

若希望分别启动场景和工作流，也可使用以下入口：

以下命令会通过现有分拣服务执行运动，只用于新启动的固定平台测试仿真。
不要与其他 ROS 场景同时运行。在工作空间根目录启动：

```bash
roslaunch aubo_vla grasp_recording_gazebo.launch output_root:=/home/zlab/aubo_vla_data
```

另一个已 source 的终端执行：

```bash
/usr/bin/python3 src/aubo/aubo_vla/tests/check_grasp_recording.py \
  --output /tmp/grasp_recording_report.json --timeout 300
```

报告文件必须不存在。脚本等待场景就绪、进入观察位，冻结三个颜色方块的指令，
开始录制并启动完整分拣。仅在任务返回 READY、每个方块至少抬升 5 cm、最终
落在对应放置中心 5 cm 内且高度为 0.12±0.03 m 后标记 success。
这是 Gazebo 世界状态验收，不能替代真实机器人上的人工确认。
随后检查 episode 并生成其目录内的 `actions.jsonl`；报告 `passed=true` 且退出码 0
才表示抓放和数据检查都通过。失败时尝试停止任务并中止录制，保留报告和样本。
若任务成功但数据检查失败，episode 保留实际任务 success 标签，回归仍不通过。
重新测试需要重启新场景；脚本拒绝方块已经位于放置区的场景。

### 手动离线检查

不启动 ROS 或机器人也可以运行：

```bash
/usr/bin/python3 src/aubo/aubo_vla/scripts/inspect_episode.py /absolute/path/to/episode
/usr/bin/python3 src/aubo/aubo_vla/scripts/inspect_episode.py /absolute/path/to/episode \
  --export /absolute/path/to/new_actions.jsonl
```

检查结果 passed=true 且退出码 0 才通过。除 SHA256 外还验证 PNG 完整解码、RGB 模式
及标定尺寸，损坏或尺寸不符的图片拒收。检查失败不导出，输出文件已存在也不覆盖。
failure episode 默认禁止导出；检查调试或失败样本时，需显式加 `--include-failure`。
末帧不产生动作；每行 image 指向 t 时刻图像，action 对应 t→t+dt 的实测差分和
t+dt 的夹爪位置。图像在 episode 的 images/ 下；动作中 episode 字段提供绝对路径。
详见 [动作规范](ACTION_SCHEMA.md)。这一步是数据回放核对基础，不会执行轨迹，
也没有输出原版 OpenVLA 的 RLDS 训练数据。首次先录少量短 episode，检查图像
颜色、位姿变化、旋转方向和夹爪开合再扩大采集。

## 5. 影子预览

```bash
roslaunch aubo_vla shadow.launch
rostopic echo /vla_shadow/candidates
```

原始预训练服务会返回 rejected，提示未声明 AUBO schema，这是正常现象。
在有经过动作回放验证的、合并好的 AUBO checkpoint 后：

1. checkpoint 包含 processor、模型、自己的 norm_stats，固定可追溯 revision。
2. 模型服务设置 `--model`、`--revision`、`--unnorm-key`、
   `--action-schema aubo_delta_pose_v1 --action-dt 0.5`；周期应取实际训练周期。
3. 自建 shadow.yaml，填相同 expected_revision、expected_unnorm_key、
   expected_action_dt，核对参考系、TCP 和工作空间，再用 `shadow.launch config:=...`。
4. RViz Fixed Frame 设为配置的 base_frame，添加 MarkerArray，话题 `/vla_shadow/markers`。

黄色线段是图像时刻 TCP 到候选 TCP 的平移，箭头沿候选 TCP 的 +X 轴显示姿态。
夹爪只在 candidates JSON 中显示。默认超过 1 cm 平移、0.05 rad 旋转、工作空间
或 1 s 图像龄拒绝；允许值只是示例预览配置，不能作为执行验收。结果过期、暂停、
回退或拒绝时删除本节点的 marker，不影响其他 RViz marker。

## 6. 目标环境回归

```bash
/usr/bin/python3 src/aubo/aubo_vla/tests/check_ros_lifecycle.py
/usr/bin/python3 src/aubo/aubo_vla/tests/check_inflight_pause.py
```

这两个脚本会暂停/恢复 Gazebo，仅用于测试仿真。真实模型环境再运行原有 check_gpu、
validate_model 和 benchmark，旧 validation JSON 是之前部署的证据，不代表本次新节点
已完成目标环境联调。

2026-10-06 的目标环境回归已通过，具体范围、数据与限制见
[本次调试报告](../validation/target_debug_20261006.md)。

本阶段未实现：语言任务编排、训练/微调管线、完整离线轨迹执行回放、控制权仲裁、
碰撞/IK 安全监督和 VLA 闭环执行。下一阶段应先根据采集结果确认动作监督方式，
补训练导出与标定模型，再扩展受限执行。
