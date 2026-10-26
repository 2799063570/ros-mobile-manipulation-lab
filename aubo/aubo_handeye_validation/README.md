# AUBO 手眼标定验证

独立 ROS1 包 `aubo_handeye_validation`，用于验证已保存的**眼在手外**标定。

| 验证方式 | 用途 | 说明 |
| --- | --- | --- |
| ArUco 留出姿态一致性 | 标记固定在末端，采集新姿态，统计安装位姿的一致性残差 | [采集与离线复算](docs/aruco_validation.md) |
| YOLO 低速运动 | 用三维目标定位，让 TCP 朝下移动至目标上方悬停，检查实际对齐 | [规划、移动与停止](docs/yolo_motion_validation.md) |

```text
aubo_handeye_validation/
├── scripts/       # 两个验证节点，均不在启动时自动移动
├── launch/        # ArUco 只读采集、YOLO 真机验证入口
├── config/        # YOLO 稳定性、工作区、悬停高度和低速参数
├── docs/          # 两种验证方式的完整说明
└── test/          # 几何/误差、模拟 ROS/MoveIt、启动图与资源检查
```

验证包直接复用 `aubo_ros_control` 的硬件启动、标定 TF 和桌面场景辅助，以及
`aubo_perception` 的 YOLO/RGB-D 定位；不依赖分拣场景或分拣状态机。
共享标定结果仍以
`aubo_ros_control/config/aubo_realsense_handeye_eye_on_base.yaml` 为准，不复制一份。
固定相机配置及 180° 朝向问题的检查记录见
[2026-10-26 诊断](docs/diagnosis_20261026.md)。

## 编译

在 Linux ROS 工作空间中，首次构建或依赖尚未构建时执行：

```bash
catkin_make
source devel/setup.bash
```

已有依赖构建完成后，可只重新构建此包：

```bash
catkin_make --pkg aubo_handeye_validation
source devel/setup.bash
```

## ArUco 精度验证

机器人真实反馈、TF、相机和 ArUco 检测器已运行后启动采集程序：

```bash
roslaunch aubo_handeye_validation validate_handeye.launch \
  output_dir:=$HOME/handeye_check_01
```

标记固定在末端，每换一个未参与标定的姿态、停稳后采集：

```bash
rosservice call /handeye_validation/capture "{}"
rosservice call /handeye_validation/report "{}"
```

只读采集，不发送运动指令。默认统计一致性残差；测绝对误差需要独立参考。

## YOLO 低速移动验证

该入口启动机器人驱动、MoveIt、相机、标定 TF、YOLO 和验证节点。
使用实际类别、桌高和物高，避免与已有机械臂控制入口重复启动：

```bash
roslaunch aubo_handeye_validation handeye_yolo_verify_real.launch \
  target_class:=can table_z:=0.10 object_height:=0.05
```

```bash
rosservice call /handeye_yolo_verify/plan "{}"
rosservice call /handeye_yolo_verify/move "{}"
rosservice call /handeye_yolo_verify/stop "{}"
```

默认速度缩放 3%、加速度缩放 2%。动作保持“抬升 → 高处朝下对齐 → 横移 → 悬停”，
停在目标顶面上方 10 cm，不闭合夹爪。完整路径不成立时拒绝执行。
程序的跟踪误差不能直接当作手眼误差，实际对齐偏差应独立观察或测量。

## 迁移说明

之前位于 `aubo_ros_control`、`aubo_sorting_core` 和 `aubo_sorting` 中的验证脚本、
启动入口、配置、文档和专属测试已经统一迁入本包。
调用 `roslaunch` / `rosrun` 时改用 `aubo_handeye_validation`；节点名、服务名、
结果文件格式和低速动作参数保持原有定义。旧包中不再提供重复脚本或入口。

## 检查

无需 ROS 即可运行离线及模拟接口测试：

```bash
python3 -m unittest discover -s /absolute/path/aubo_handeye_validation/test -v
```

需要 NumPy 和 PyYAML。ROS 不可用时启动图测试明确跳过；在完整 ROS 环境中运行
`catkin_make run_tests_aubo_handeye_validation` 可检查解析后的启动图。
这些检查不执行真机运动，不能替代现场的模型、时间同步和实际误差验证。
