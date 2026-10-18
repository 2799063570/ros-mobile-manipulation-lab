# 眼在手外标定验证

如需使用 YOLO 目标让夹爪低速朝下移动并悬停，见
[`YOLO 低速运动验证`](yolo_motion_validation.md)。该入口用于实际对齐检查，
与本文 ArUco 留出姿态一致性验证分别运行。

`validate_handeye.py` 读取已保存的 easy_handeye YAML，验证固定相机与机器人基座之间的
外参。程序只订阅 TF、提供采集服务并写入结果，不发布 TF、轨迹或运动指令。

默认输出是**留出姿态的一致性残差**，包括视觉检测、运动学与时间同步误差。
它不等同于外参相对真实安装位置的绝对误差。只有外参 YAML、没有新的视觉观测时，
无法推算实际精度。单姿态或重复同一姿态可能得到接近零的残差，不能用来验收。

## 数据准备

1. 相机相对基座固定，ArUco 标记相对 `gripper_link` 刚性固定。
   标记尺寸、相机内参/畸变参数必须正确，检测节点应使用图像采集时间戳。
2. 启动真实机器人反馈和 `robot_state_publisher`，使 `base_link -> gripper_link`
   能在图像时刻查询。使用真实关节反馈，不能用规划目标或 fake joint states。
3. 启动相机和原先做标定时的 ArUco 检测节点。此仓库的启动文件**没有启动 ArUco 检测器**；
   需使用你采集 easy_handeye 数据时的节点，配置到相同的光学坐标系和标记 ID。
4. 检测器必须在 `/tf` 发布直接的动态边
   `camera_color_optical_frame -> aruco_marker_frame`（具体名称以 YAML 为准）。
   程序直接订阅这条边，不通过 TF 树查询相机到标记，避免循环验证。
   不接受 `/tf_static` 中的固定标记或经过其他参考系中转的观测。
   这仍要求操作者确认该边确实来自视觉检测，程序无法鉴别伪造的动态 TF。
5. 留出 15–20 个未参与拟合的姿态，覆盖实际工作区的位置、高度和不同旋转轴。
   每个姿态停稳后采集；两次采集之间移动机械臂。不要把同一姿态的多帧当成多个姿态。

当前 YAML 的 `tracking_base_frame` 为 `camera_color_optical_frame`。
现有 `eye_to_hand_camera_real.launch` 默认使用 `workspace_camera_color_optical_frame`，
二者不一致，发布脚本会拒绝加载。请核实物理相机与光学坐标系的含义后统一配置，
不要用新增的零变换连接两个不明坐标系来绕过检查。
验证器直接读取 YAML 中的外参做计算，因此不要求额外发布 `base_link -> camera`。

标记必须是实际视觉观测对应的那个刚体。标定后若移动相机，需要重新标定；
重新装夹标记后，应重新开始一次验证，不能把不同安装位姿混合统计。

## 编译与采集（ROS1 / Python3）

在 Linux ROS 工作空间编译并加载环境：

```bash
catkin_make --pkg aubo_handeye_validation
source devel/setup.bash
```

启动前先检查标定 YAML 的数值字段。编辑期间若出现 `0.9094763530288582neng` 这类
字符串，必须核对并改回有效数值；程序会拒绝加载，不会自动猜测或修改原文件。

机器人反馈、相机及标记检测已运行后，启动验证器：

```bash
roslaunch aubo_handeye_validation validate_handeye.launch \
  output_dir:=$HOME/handeye_check_01
```

输出目录必须尚不存在，避免覆盖历史实验。要验证另一份结果，可增加
`calibration_file:=/absolute/path/calibration.yaml`。程序使用 YAML 中的四个坐标系名称，
不提供静默重命名参数。

每到一个新姿态，停稳后在另一个终端调用：

```bash
rosservice call /handeye_validation/capture "{}"
```

默认采集 2 秒，要求至少 10 个不同时间戳的新观测。只接受调用之后的观测，
观测延迟不得超过 0.5 秒，并查询该观测时间戳的机器人 TF，不回退到最新 TF。
检测到机器人在采样窗口内的位置跨度超过 2 mm 或旋转跨度超过 1°，整组拒收。
机器人运动检测是辅助检查，操作者仍应在停稳后调用。

可通过 launch 参数调整 `window`、`min_samples`、`max_age`、`stationary_mm`、
`stationary_deg` 和 `tf_topic`。这些是数据采集条件，**不是精度验收阈值**。
无观测、陈旧时间戳、缺失同步 TF 或运动导致拒收时，服务返回 `success: false` 和原因。
此时先解决数据问题，再在同一姿态重试。

采满后读取汇总（每次成功采集也会自动更新）：

```bash
rosservice call /handeye_validation/report "{}"
```

验证器不会自动移动机械臂。应由现场既有的示教/运动流程完成姿态切换。

## 输出和指标

每组采集计算：

```text
E_T_M[i] = inverse(B_T_E[i]) * B_T_C_saved * C_T_M_measured[i]
```

`A_T_B` 表示把 B 系坐标变换到 A 系；B/E/C/M 分别代表基座/末端/相机/标记。
各组先对组内帧求位置均值及四元数均值，再对各组等权统计，避免高帧率姿态占据更大权重。
旋转误差用 SO(3) 的最小旋转夹角，不对欧拉角直接相减。

| 文件 | 内容 |
| --- | --- |
| `session.json` | 外参快照、采集条件、逐帧时间戳、原始机器人/标记位姿、成功及失败尝试 |
| `report.json` | 组间位置/旋转残差的均值、RMSE、P95、最大值；组内残差及姿态覆盖提示 |
| `poses.csv` | 每组标记在末端系下的估计位置、位置/旋转残差和组内 RMSE |

位置指标为 **mm**，旋转指标为 **°**；原始位姿的位置为 **m**，四元数顺序为 **xyzw**。
`between_pose` 表示不同姿态下标记安装位姿相对总体均值的偏差；
`within_pose` 表示停稳时的逐帧抖动。`robot_pose_span` 和旋转激励奇异值用于提示
样本覆盖不足，不能证明所有自由度都已被充分验证。

程序不自动判定“合格”。误差要求应由夹爪容差、目标尺寸及任务精度预算确定。
不要用位置残差直接当作外参平移误差，也不要用此处的旋转残差替代抓取 yaw 误差。

## 可选：与独立测得的标记安装位姿比较

如果能独立测量 `gripper_link -> aruco_marker_frame` 的完整六维位姿，创建文件：

```yaml
parent_frame: gripper_link
child_frame: aruco_marker_frame
transformation:
  x: 0.0     # 示例格式，必须替换为独立测量的实际值，单位 m
  y: 0.0
  z: 0.15
  qx: 0.0
  qy: 0.0
  qz: 0.0
  qw: 1.0
```

通过命令行启动（不要同时运行 launch 中的验证器）：

```bash
rosrun aubo_handeye_validation validate_handeye.py collect \
  --calibration "$(rospack find aubo_ros_control)/config/aubo_realsense_handeye_eye_on_base.yaml" \
  --reference /absolute/path/independent_marker_mount.yaml \
  --output "$HOME/handeye_check_reference_01"
```

报告增加 `independent_reference_error`。参考位姿不能由正在验证的外参推算，
也不能取本次观测的均值；参考测量的不确定度、机器人运动学误差仍应独立说明。
因此报告的 `absolute_accuracy_established` 始终为 false，程序不会自动宣称已经证明绝对精度。

## 离线复算与测试

离线分析只需要 Python3、NumPy、PyYAML，无需 ROS：

```bash
python3 /absolute/path/aubo_handeye_validation/scripts/validate_handeye.py analyze \
  "$HOME/handeye_check_01/session.json" --output "$HOME/handeye_check_recomputed_01"
```

`session.json` 保存采集时的外参快照；以后修改工程中的 YAML 不会改变历史报告。
离线分析同样要求新的输出目录。停止节点前不必额外保存，每次采集都会写入数据。

开发检查：

```bash
python3 -m unittest discover -s /absolute/path/aubo_handeye_validation/test -p test_validate_handeye.py -v
```

测试使用已知几何关系生成独立观测，检查变换方向、错误外参的可检测性、四元数符号与
旋转跨界、逐姿态等权统计、无效输入和完整离线导出流程。真机 TF、时间同步和
实测误差需要在 ROS 现场验证，离线测试不能替代这些检查。
