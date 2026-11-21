# AUBO 单写入者混合控制

在同一台 AUBO i5 上保留 MoveIt 和原分拣的 `ros_control` 轨迹链路。近距离伺服由 `aubo_visual_servo` 计算六轴位置点，经 ROS topic 交给 `aubo_hw_node`。两路使用同一驱动队列和 SDK 发送线程。

## 启动

```bash
roslaunch aubo_hybrid_control hybrid_control_real.launch \
  robot_ip:=192.168.1.2 camera_mount:=eye_in_hand
```

`camera_mount:=eye_to_hand` 时传入工位实际使用的 `calibration_file`。已有视觉目标发布者时，可以关闭 `start_servo_perception`，通过 `target_topic` 指定目标源。默认伺服保持禁用，且分拣任务节点关闭。需要同时使用原分拣任务节点时可设 `start_sorting_task:=true`。

## 模式切换

模式值为 `1=ROS_CONTROL`、`2=VISUAL_SERVO`、`3=HOLD`。启动后驱动进入 `ROS_CONTROL`。

```bash
rostopic echo /aubo_i5/control_mode_state
rosservice call /aubo_i5/set_control_mode "mode: 2"
# 等同一个 switch_id 进入 active_mode=2、phase=0，再使能伺服
rosservice call /visual_servo/set_enabled "data: true"
# 切回时伺服收到切换状态后自动停止跟踪并减速
rosservice call /aubo_i5/set_control_mode "mode: 1"
# 等同一个 switch_id 进入 active_mode=1、phase=0，再发新的 MoveIt 轨迹
```

`accepted` 表示已登记请求。切换时先等待旧来源停稳，再关闭两路运动入口并持续发送保持点。驱动检查 SDK 已接受的保持点、MAC 诊断和实测关节反馈，然后切换轨迹控制器。伺服点必须匹配当前会话、连续序号、时间戳和 URDF 关节限位。交接超时或伺服流异常会锁停，并由驱动关闭流程请求柜端停止。故障恢复要求排除原因后重新启动。`HOLD` 是有序交接至保持状态，急停仍使用机械臂的硬件急停。

现有分拣任务仍执行其原有抓放序列。上层如需替换预抓取后的下降阶段，可在原轨迹成功结束后请求伺服模式，使用已绑定到当前分拣实例且经过标定的视觉目标，再在抬升前请求切回规划模式。

首次联调先在静止位运行 `ROS_CONTROL → VISUAL_SERVO → ROS_CONTROL`，伺服保持禁用。在线交接依赖控制柜 MAC FIFO 的容量和诊断语义，需在实际控制柜上确认。伺服的默认目标偏移为预留距离的对准配置，需要按工位重新标定后才能用于抓取下降。
