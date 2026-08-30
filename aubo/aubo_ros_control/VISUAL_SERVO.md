# AUBO 统一视觉位置伺服

视觉伺服能力属于机械臂，因此实现位于 `aubo`。固定机械臂和移动机械臂都复用同一
控制节点 `aubo_visual_servo_node`，移动端只负责把导航、底盘锁定和抓取任务组合起来。

## 架构

```text
aubo_perception: RGB-D/YOLO -> /visual_servo/target_pose
                               |
aubo_ros_control: 坐标误差 -> 雅可比逆解 -> 限速/限加速度 -> 有界队列
                               |
                         Gazebo 控制器 或 AUBO SDK
```

`servo_mode` 只改变坐标误差的计算：

- `eye_in_hand`：目标变换到运动相机光学坐标系，控制相机到期望观测位姿；
- `eye_to_hand`：目标变换到 `base_link`，控制 `tcp_link` 到目标加偏移的位置。

感知接口、状态机、关节限制、Gazebo/真机执行链完全共用。不要同时启动两个视觉
伺服实例，它们使用相同服务和机械臂命令通道。

## 启动入口

| 安装方式 | Gazebo | 真机 |
| --- | --- | --- |
| 眼在手上 | `eye_in_hand_visual_servo_gazebo.launch` | `eye_in_hand_visual_servo_real.launch` |
| 眼在手外 | `eye_to_hand_visual_servo_gazebo.launch` | `eye_to_hand_visual_servo_real.launch` |

示例：

```bash
roslaunch aubo_ros_control eye_in_hand_visual_servo_gazebo.launch
roslaunch aubo_ros_control eye_to_hand_visual_servo_gazebo.launch
roslaunch aubo_ros_control eye_to_hand_visual_servo_gazebo.launch target_label:=blue
roslaunch aubo_ros_control eye_to_hand_visual_servo_real.launch \
  robot_ip:=192.168.1.2 camera_serial_no:=<serial>
```

旧名称 `visual_servo_gazebo.launch`、`visual_servo_real.launch` 仅作为兼容包装保留，
新代码和文档应使用带 `eye_in_hand` 的完整名称。

真机默认 `auto_start:=false`。先核对 TF、目标方向、工作空间和避碰，再调用
`/visual_servo/set_enabled`。SDK 模式会独占机械臂，不能同时启动 `aubo_hw_node` 或
其他会进入 TCP2CANBUS 的控制节点。

## 相机与感知

两种安装方式都使用 `aubo_perception/rgbd_visual_target_node.py` 和稳定接口：

```text
/visual_servo/target_pose              geometry_msgs/PoseStamped
/visual_servo/set_enabled              std_srvs/SetBool
/visual_servo/reset                    std_srvs/Trigger
/visual_servo/perception/set_enabled   std_srvs/SetBool
/visual_servo/perception/reset         std_srvs/Trigger
/visual_servo/state                    std_msgs/String
```

两种 Gazebo 模式默认共用 `aubo_color_sorting/worlds/sorting.world`。该场景同时包含
红、绿、蓝分拣块以及 `workspace_camera`：眼在手上使用腕部相机，眼在手外使用固定
相机，并用共同的 `target_label:=red|green|blue` 参数选择跟踪颜色。分拣用的彩色
放置区域不在该 world 内，只由 `sorting_gazebo.launch` 按需生成，因此不会参与伺服
识别。眼在手外仍使用
不含手部相机的 `aubo_i5.xacro`，避免无用的手部图像和深度话题。真机入口也只启动外部 RealSense；
外参由 `aubo_description/launch/eye_to_hand_camera_tf.launch` 统一发布。默认外参仅用于
仿真，真机必须替换为标定结果。

启动后也可以在共用 RViz“视觉伺服控制”面板的“跟踪目标”下拉框中切换红、绿、蓝；
该选择通过 `/visual_servo/target_selection` 同时适用于眼在手上和眼在手外。

## 初始化、遮挡与目标丢失

- 眼在手上在第一次看到目标前进入 `SEARCH_INITIAL`，缓慢移动到经过相机光轴验证的
  `initial_search_posture`；目标出现后立即进入 `TRACKING`。丢失后的可选恢复动作使用
  独立的 `recovery_posture`，两者不再复用通用 `open_posture`。默认丢失策略为
  `coast_then_open` 且 `coast_duration: 0.0`：目标超时后直接向观测姿态移动；移动中
  任意一帧重新获得有效目标都会立即中断恢复动作并回到 `TRACKING`。
- 眼在手上的期望观测深度默认为 `0.15 m`。腕部相机光心沿接近方向比 TCP 后缩约
  `0.035 m`，因此最终 TCP 到目标约为 `0.115 m`，这是预抓取对准距离而不是接触距离。
  `TRACKING` 仅表示持续收到有效目标，不代表已经收敛；是否停止由三维位置误差和
  `position_deadband` 决定。
- 眼在手外不需要机械臂搜索。`target_offset` 默认包含横向避遮挡分量和抓取上方的
  高度分量，使机械臂尽量不挡住相机到目标的视线。实际工位必须标定方向和大小。
- 两种模式都拒绝超时目标。状态依次可能为 `TRACKING`、`COAST`、
  `SEARCH_RECOVERY`、`HOLD`。眼在手外默认保持；眼在手上会在丢失后回到观测姿态
  重新搜索。
- 检测器在目标或深度无效时不发布旧位姿；否则看门狗无法识别目标丢失。
- 检测器先用 `minimum_contour_area` / `maximum_contour_area` 过滤像素轮廓，再结合
  深度和相机内参计算投影物理面积。默认
  `maximum_projected_contour_area: 0.006`（平方米），用于排除大块同色区域，同时
  避免目标靠近相机后仅因像素面积变大而丢失。

配置分为三份：

- `visual_servo_common.yaml`：关节限制、队列、频率和 SDK 参数；
- `visual_servo_eye_in_hand.yaml`：运动相机坐标误差和搜索策略；
- `visual_servo_eye_to_hand.yaml`：固定相机 TCP 偏移和遮挡策略。

## 安全调试顺序

1. 先保持 `auto_start:=false`，检查相机话题和 TF 连通性。
2. 在 RViz 中确认目标位姿在正确的物体上，外部相机目标可变换到 `base_link`。
3. 眼在手外先把 `target_offset` 的高度设大，确认横向避遮挡方向正确。
4. 真机首次运行将 `loss_strategy` 临时设为 `stop`，并降低速度与加速度限制。
5. 确认目标丢失后机械臂按预期减速，再启用恢复搜索。
