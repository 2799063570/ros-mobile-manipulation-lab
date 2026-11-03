# 分拣启动入口索引

日常使用先按下面的表选择；颜色 / YOLO 和相机安装方式由参数选择，
无需为新组合再复制一份完整 launch。

| 要做什么 | 入口 | 默认行为 |
| --- | --- | --- |
| MoveIt、控制器、相机和 TF 已运行，只启动分拣算法 | [sorting.launch](sorting.launch) | 颜色、眼在手上；自动前往观察位，等待开始服务 |
| 启动完整 Gazebo 分拣环境 | [sorting_gazebo.launch](sorting_gazebo.launch) | 颜色方块场景、眼在手上；自动前往观察位，等待开始服务 |
| 启动通用实机分拣环境 | [sorting_real.launch](sorting_real.launch) | 颜色、眼在手上；等待显式运动指令 |
| 使用当前实测的眼在手外三容器工位，连续分拣 | [eye_to_hand_yolo_sorting_real.launch](scenarios/eye_to_hand_yolo_sorting_real.launch) | YOLO、深度定位、实例队列；等待显式运动指令 |
| 同一实测工位，按缓冲队列分拣 | [eye_to_hand_can_sorting_real.launch](scenarios/eye_to_hand_can_sorting_real.launch) | 按入队顺序领取就绪实例，允许 bottle / can / box；等待显式运动指令 |
| 同一工位，测试 YOLO 实机局部视觉伺服 | [eye_to_hand_yolo_servo_test_real.launch](scenarios/eye_to_hand_yolo_servo_test_real.launch) | 10 mm/s、表面上方 15 cm、默认禁用；[测试步骤](../../../docs/AUBO眼在手外视觉伺服实机测试.md) |
| MoveIt 接近 + 局部视觉伺服的 hybrid 控制 | [hybrid/](hybrid/) | 选择眼在手上 / 眼在手外、Gazebo / 实机入口 |

## 目录分工

```text
launch/
├── sorting.launch          # 感知 + 几何定位 + 分拣任务
├── sorting_gazebo.launch   # Gazebo + MoveIt + sorting.launch
├── sorting_real.launch     # 实机硬件 + MoveIt + 相机 + 夹爪 + sorting.launch
├── presets/                # 14 个通用快捷入口，仅选择检测器 / 相机安装方式
├── scenarios/              # 实测工位分拣及局部视觉伺服测试入口
└── hybrid/                 # 4 个混合控制入口，复用 aubo_ros_control
```

流程图放在 [docs/](../docs/感知、维护目标、依次抓取流程图.png)。

## 常用命令

```bash
# 仿真：眼在手外 + YOLO；会按 detector 选择对应 world 和任务配置。
roslaunch aubo_sorting sorting_gazebo.launch camera_mount:=eye_to_hand detector:=yolo

# 通用实机：眼在手外 + 颜色；任务配置应使用该工位的实测值。
roslaunch aubo_sorting sorting_real.launch camera_mount:=eye_to_hand detector:=color \
  task_config:=/实际路径/task.yaml calibration_file:=/实际路径/calibration.yaml

# 当前实测三容器工位：默认连续分拣，加载桌面、相机支架和容器。
roslaunch aubo_sorting eye_to_hand_yolo_sorting_real.launch
```

`sorting_real.launch detector:=yolo camera_mount:=eye_to_hand` 是通用组合，
不会自动加载 `scenarios/` 的实测桌面、支架、容器和连续队列配置。
`eye_to_hand_yolo_sorting_real.launch` 是具体工位入口，两者不能视为等价命令。
工位尺寸和运动流程见 [包说明](../README.md#原实机工位三类容器分拣)。

实机默认 `auto_start:=false`、`auto_move_to_observation:=false`，
先检查识别与场景，再按包说明调用观察位和开始服务。
眼在手外实机默认使用 `real_sort_ready` 工作观察位，原 `real_observe` 保留作结束收臂；
位置来源和离线检查范围见[工作观察位说明](../docs/实机工作观察位.md)。
场景入口可通过 `observation_named_target:=real_observe` 显式选择原观察位。
两个实测工位入口默认 `continuous_sorting:=true`；`sort_classes` 只筛选允许类别，
队列按入队 ID 领取就绪物体，不等待某个特定类别出现。

需要可视化控制与在线调参时，实测入口可加 `rqt:=true`，或单独运行
`rosrun aubo_sorting_rqt sorting_panel`。界面提供过程、画面、实例队列、控制按钮及服务器回读的在线参数。
使用说明见 [rqt 工作台](../../aubo_sorting_rqt/README.md)。

## 原快捷命令继续使用

ROS 会递归查找包内的 launch 文件；文件名保持唯一，所以移动目录后仍可使用
`roslaunch aubo_sorting 文件名.launch`，不用在命令中增加子目录。

| 组合 | 仅算法：`presets/` | 仿真：`presets/` | 实机 |
| --- | --- | --- | --- |
| 眼在手上 + 颜色 | `eye_in_hand_color_sorting.launch` | `eye_in_hand_color_sorting_gazebo.launch` | `presets/eye_in_hand_color_sorting_real.launch` |
| 眼在手上 + YOLO | `eye_in_hand_yolo_sorting.launch` | `eye_in_hand_yolo_sorting_gazebo.launch` | `presets/eye_in_hand_yolo_sorting_real.launch` |
| 眼在手外 + 颜色 | `eye_to_hand_color_sorting.launch` | `eye_to_hand_color_sorting_gazebo.launch` | `presets/eye_to_hand_color_sorting_real.launch` |
| 眼在手外 + YOLO | `eye_to_hand_yolo_sorting.launch` | `eye_to_hand_yolo_sorting_gazebo.launch` | `scenarios/eye_to_hand_yolo_sorting_real.launch`（实测工位） |

`color_sorting.launch`、`yolo_sorting.launch` 和 `yolo_sorting_gazebo.launch`
也在 `presets/`，继续转发调用者的参数覆盖。

在其他 XML 中使用 `<include file="...">` 时必须写实际路径，例如：

```xml
<include file="$(find aubo_sorting)/launch/scenarios/eye_to_hand_yolo_sorting_real.launch"
         pass_all_args="true"/>
```

## 维护规则

- 通用算法和参数默认值在 `sorting.launch` 维护；环境差异在两个总入口维护。
- `presets/` 只固定模式并转发参数，共用参数随总入口更新。
- `scenarios/` 保存具体工位组合，尺寸和任务参数继续放在 YAML 中。
- `hybrid/` 保存混合控制应用入口，底层控制仍由 `aubo_ros_control` 维护。
- 保持所有 launch 文件名唯一，避免 ROS 按文件名查找时报多个匹配。
