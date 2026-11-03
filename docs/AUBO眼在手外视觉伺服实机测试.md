# AUBO 眼在手外 YOLO 视觉伺服实机测试

日期：2026-11-03。统一入口位于 `aubo_ros_control/launch/eye_to_hand_visual_servo_real.launch`，
同时启动 rqt 视觉伺服调试台。原 `aubo_sorting/eye_to_hand_yolo_servo_test_real.launch`
作为参数兼容包装保留，内部转发到统一入口。
它复用当前工位的 YOLO、手眼标定、桌面、相机支架和容器，执行独立按需 PBVS。
已读取实机当前关节反馈并保存为固定观测位，完成模拟反馈与假 SDK 回归；
尚未运行这一入口的实机运动。

## 默认测试行为

| 项目 | 默认值 |
| --- | --- |
| 跟踪类别 | YOLO `can`，可用 `target_class:=bottle` 或 `box` 覆盖 |
| 相机与标定帧 | `/camera`、`camera_color_optical_frame` |
| 控制后端 | 独占 SDK；50 Hz 目标更新、固定 5 ms 平滑、按需补点 |
| 线速度上限 | 0.01 m/s，即 10 mm/s |
| 目标点 | 深度观测表面点正上方 15 cm，XY 不额外偏移 |
| 观测位 | 2026-11-03 19:31:58 读取的六关节姿态；使能后先到此位 |
| 姿态控制 | 开启，跟踪时保持此次记录的 TCP 朝向 |
| 目标丢失 | 速度指数衰减、停稳后返回同一观测位，再等待新目标 |
| 启动授权 | 禁用，必须通过调试台按钮或使能服务显式授权 |
| 碰撞场景 | MoveIt 仅提供场景服务；伺服订阅监控场景并检查制动尾段 |
| 数据 | 默认录制目标、状态、SDK 反馈及 TF；CSV 路径可指定 |
| 界面 | 默认 `rqt=true`、`rviz=false`，界面与三维显示可分别开关 |

工位几何沿用 `eye_to_hand_can_sorting_real.launch`。本入口不运行分拣循环、
夹爪控制或 `aubo_hw_node`。切换前退出原分拣、点动或其它机械臂驱动入口，
避免多个节点占用同一控制通道和重复发布模型、相机及标定 TF。

观测位配置、采集依据和完整状态流程见[固定观测位与丢失回位](眼在手外观测姿态与丢失回位.md)。
`observation_config` 默认加载已记录的 YAML；启动、使能和丢失时都不会重新采集覆盖。

## 1. 启动并保留记录

在工作空间终端执行：

```bash
cd ~/aubo/ros_mobile_manipulation_lab
source devel/setup.bash
servo_run=$(mktemp -d "$HOME/aubo_servo_test_XXXXXX")
echo "$servo_run"
roslaunch aubo_ros_control eye_to_hand_visual_servo_real.launch \
  target_class:=can \
  timing_trace_path:="$servo_run/timing.csv" \
  bag_prefix:="$servo_run/servo"
```

启动包含真实 SDK 登录、上电及透传初始化，但不会自动授权运动。
需要只检查相机/识别而完全不连接机械臂时，先使用：

```bash
roslaunch aubo_ros_control eye_to_hand_visual_servo_real.launch \
  connect_robot:=false rviz:=false record_data:=false
```

结束只识别阶段后，再启动上面的完整入口。`start_camera:=false` 可复用外部相机，
此时外部必须提供相同 `/camera` 话题和唯一的已标定 `base_link → camera_color_optical_frame`。

## 2. 查看目标与状态

另一个已 source 工作空间的终端：

```bash
rostopic echo -n 1 /visual_servo/target_pose
rostopic hz /visual_servo/target_pose
rostopic delay /visual_servo/target_pose
rostopic echo /visual_servo/state
```

`hz`、`delay` 和持续 `echo` 各自占用当前终端，读完可 Ctrl+C 再执行下一条。
默认 `target_timeout=0.20 s`；如果源图像到目标发布的延迟经常达到 200 ms，
先定位推理、深度匹配和通信延迟，不通过重写时间戳或直接放宽超时隐藏问题。

等待 `WAITING_FOR_SDK` 结束、SDK 原始关节反馈持续更新，再进行使能。
确认场景包含 `sorting_table`、`camera_stand` 和各容器：

```bash
rosservice call /get_planning_scene "components: {components: 16}"
```

16 为 `WORLD_OBJECT_GEOMETRY`。目标只放一个同类物体，并从 TCP 附近的小误差开始；
现有感知按置信度选择同类目标，没有同类实例身份锁定。先确认目标 TF/方向、
当前姿态及物体上方 15 cm 的期望 TCP 与实际工位一致。

测试入口默认同时打开 rqt“视觉伺服调试台”，支持 can/bottle/box 类别切换、伺服启停、
在线增益/速度调节和姿态/视线预览，详见[界面与姿态说明](../aubo/aubo_sorting_rqt/docs/visual_servo_workbench.md)。
已运行控制器时也可执行 `rosrun aubo_sorting_rqt visual_servo_panel` 单独打开。
`rqt:=false` 可关闭自动打开界面，`rviz:=false` 可只保留 rqt。
RViz 继续显示三维目标和轨迹，旧控制面板现支持从感知状态读回 YOLO 类别。
以下终端服务也可用于控制。

## 3. 显式使能与观察

```bash
rosservice call /visual_servo/set_enabled "data: true"
```

只有 `success: true` 才表示请求被接受。控制器先进入 `MOVE_TO_OBSERVATION`，
到达已记录的六关节位且停稳后，等待此后采集的目标，再进行跟踪。
因此相机暂时没有目标时，也可以显式使能前往观测位。
静止目标应看到 `TRACKING → ALIGNED`，位置误差逐渐进入当前各轴 4 mm 死区；
ALIGNED 还要求稳定保持及输出停稳条件。它不是独立测量的绝对定位精度。

先观察静止收敛和稳定性，再小幅移动目标观察滞后、反向以及是否振荡，
最后遮挡或移走目标，检查 `COAST → HOLD → RETURN_TO_OBSERVATION → WAITING`。
回位途中重新看到目标不会打断回位；到位后等待新目标持续稳定，再恢复跟踪。
回位是受碰撞保护的关节速度控制，路径检查不负责寻找绕障路径。

## 4. 停止、复位与导出

```bash
rosservice call /visual_servo/set_enabled "data: false"
```

等待减速和真实机械臂停稳。碰撞、跟随误差或目标快照保护锁存后，排除原因再调用：

```bash
rosservice call /visual_servo/reset "{}"
```

该复位服务只清除允许复位的锁存状态，不执行回位，也不覆盖保存的观测位，且不自动使能。
碰撞、反馈或通信故障不会触发自动回位。SDK 传输层锁存故障需要重启节点。松手、禁用和退出都不能撤回
已经进入控制柜的轨迹，当前仍有约 250 ms 名义前瞻及减速尾段。

结束本轮时，在 launch 终端 Ctrl+C 并等待退出，CSV 在生成线程退出后导出，
rosbag 也需要正常关闭。该终端的 `servo_run` 仍指向本次目录，可执行：

```bash
python3 src/aubo/aubo_ros_control/scripts/analyze_motion_timing.py \
  "$servo_run/timing.csv" --output "$servo_run/analysis" --plot
```

本轮数据用于核对 SDK 延迟、MAC 水位、目标反向和停止表现。`/visual_servo/status`
提供显示用几何误差与有效性标志；要测实际停稳时间和位移，应同时检查 bag 中
SDK 原始反馈，不能只依赖 ALIGNED/DISABLED 状态或命令导数。

## 离线验证

`aubo_sorting/test/test_real_servo_launch.py` 仅解析启动图，确认默认禁用、单一 SDK
控制入口、MoveIt 禁用轨迹执行、相机标定帧一致、低速参数覆盖、场景及记录路径。
它还验证 `connect_robot=false` 不加载任何机械臂控制后端。

## 相机启动超时的排查（2026-11-03）

17:14 的实机启动出现 `UVCIOC_CTRL_QUERY` 超时、`Protocol error` 和持续不出帧。
核对实际驱动构建/运行均为 librealsense 2.56.5，设备原日志固件为 5.17.0.10。
USB 协商速率 5000M，但内核同时报告 UVC 控制失败；后续独立相机启动还出现
`xhci` 传输错误、`AMD-Vi IO_PAGE_FAULT`。这些证据指向设备/USB 主机通信路径，
仅凭当前日志不能判定具体是线缆、供电、Hub、控制器或其它系统因素。

单独启动相机并请求 `initial_reset=true` 时，SDK 在 `rs2_create_device` 阶段已超时，
没有取得可执行硬件复位的设备对象；不能把传入复位参数视为已经成功复位。
先退出占用相机的进程、重新给相机断电并更换接口，优先验证另一 USB 主机控制器。
恢复前保持机械臂禁用，不通过改变伺服增益或延长目标超时处理这类设备故障。

相机入口已透传 `initial_reset`、`fps`；工位测试入口对应参数为
`camera_initial_reset`、`camera_fps`，默认仍为 false、30。
这台相机 ROS/RealSense SDK 日志中的序列号为 `335222076553`；USB 描述符中的编号
不同，不能直接拿来替代 RealSense `serial_no`。

只运行相机，验证出帧：

```bash
roslaunch aubo_perception realsense_d435i.launch \
  camera_name:=camera serial_no:=335222076553 initial_reset:=true \
  enable_pointcloud:=false enable_imu:=false publish_tf:=false
```

另一个终端执行以下有界只读检查，要求 JSON 中 `ok: true`：

```bash
rosrun aubo_perception check_rgbd_camera.py --camera /camera --duration 10
```

检查通过后先退出单相机入口，再运行工位测试，可增加：

```text
camera_serial_no:=335222076553 camera_initial_reset:=true
```

每次完整测试继续使用新的记录目录。单相机阶段没有发布安装外参，不能直接将其
当作已完成标定 TF 的完整视觉伺服环境。

本次后续恢复结果：相机换到 Bus 006、`6-1` 直连路径（5000M），旧 Hub 枚举重试
结束后设备查询恢复，确认 SDK 序列号 `335222076553`、固件 05.17.00.10。
单独启动并软件复位后，640×480、30 Hz 配置下采样 10 秒：

| 流 | 帧数 | 实测频率 |
| --- | --- | --- |
| 彩色图 | 300 | 29.98 Hz |
| RGB 对齐深度 | 300 | 29.98 Hz |
| 彩色内参 | 300 | 29.98 Hz |

RGB/深度最近时间戳差 P95 为 0 ms，有效深度像素 98.28%，检查器返回 `ok=true`。
启动阶段仍出现部分 UVC GET_CUR -32 内核记录；以上证明该 10 秒窗口正常出帧，
不代表长时间运行已验证，也未据此定位原链路故障的唯一根因。
临时检查在独立 ROS master 11319 上进行，没有启动机械臂连接或使能服务。

```bash
python3 -m unittest discover -s src/aubo/aubo_sorting/test \
  -p test_real_servo_launch.py -v
```
