# YOLO 眼在手外低速运动验证

使用现有 YOLO 三维目标和手眼外参，让 TCP 朝下移动到物体上方，检查夹爪中心与物体
中心的对齐情况。启动后等待服务命令，不自动运动、不闭合夹爪、不执行抓放循环。

程序：`aubo_handeye_validation/scripts/handeye_yolo_verify.py`。
参数：[`handeye_yolo_verify.yaml`](../config/handeye_yolo_verify.yaml)。
另见 [ArUco 一致性验证](aruco_validation.md)。

## 真机启动

在 Linux ROS1 工作空间编译并加载环境：

```bash
catkin_make --pkg aubo_handeye_validation
source devel/setup.bash
```

该入口启动真实硬件接口、MoveIt、机器人 TF、固定相机、标定 TF、YOLO 和验证节点。
先结束已有分拣、视觉伺服及其他运动节点，避免重复启动控制通道。

```bash
roslaunch aubo_handeye_validation handeye_yolo_verify_real.launch \
  robot_ip:=192.168.1.2 \
  target_class:=can \
  table_z:=0.10 object_height:=0.05
```

`target_class` 要填模型实际输出的类别。Python/权重路径沿用工程现有 YOLO 环境，
可通过 `python:=... model_path:=...` 覆盖。
`table_z` 是 **base_link 下的实际桌面高度**；`object_height` 是物体竖直高度，单位 m。
默认 `height_mode:=depth`，要求深度通过现有几何节点的桌高/物高校验。
显式使用 `height_mode:=table` 时，高度来自已知桌高和物高，不能用来评价深度精度。
感知工作区及各类别物高覆盖由 `perception_config:=...` 设置。

桌面碰撞体的默认中心 XY 为 `[0.70, 0.0]`，尺寸为 `[0.70, 1.00, 0.20]`。
实际工位不同须通过 `table_center_x`、`table_center_y`、`table_size_x`、`table_size_y`、
`table_thickness` 修改；碰撞体顶面随 `table_z` 设置。其他障碍物仍应加入 MoveIt 场景。
规划依赖实际机器人、TCP、夹爪状态与碰撞模型准确。

相机命名默认 `/camera`，匹配当前标定 YAML 中的 `camera_color_optical_frame`。
更换标定文件时通过 `calibration_file:=...` 指定，并核对 `camera_namespace:=...`。
发布脚本继续严格检查坐标系，不应通过重命名把不同物理坐标系的外参混用。
若 YAML 仍有 `z: 0.9094763530288582neng`，先核对并修正数值，程序不会自动改动标定文件。

## 一次验证

工作区中只放置一个待测类别的物体，保持不动。TCP 起始高度应大于桌面高度加 5 cm，
夹爪无需已经朝下。程序读取原始 `/aubo_i5/joint_states`，要求真实反馈新鲜且静止。
默认不启动夹爪控制节点；需要读取现有夹爪反馈时可设 `start_gripper:=true`，
验证程序仍不发送夹爪动作。

可先预览，不执行运动：

```bash
rosservice call /handeye_yolo_verify/plan "{}"
```

预览话题为 `/handeye_yolo_verify/display_trajectory`，可在 RViz 的 Trajectory 显示中选择。
最终目标位姿发布到 `/handeye_yolo_verify/goal`。

执行一次低速移动：

```bash
rosservice call /handeye_yolo_verify/move "{}"
```

`move` 重新采集目标、重新规划，不执行先前缓存的预览轨迹。动作顺序：

1. 等待请求之后至少 5 帧新观测，跨越至少 0.25 秒，位置抖动不超过 5 mm。
2. 在当前 XY 位置抬升到通行高度，保留起始朝向。
3. 在高处调整为 TCP 局部 +Z 指向基座 -Z，固定 yaw=π/2。
4. 保持朝下，在高处平移到目标上方，再竖直下降到悬停点。
5. 在目标**顶面上方 10 cm**停止，保持朝向，不闭合夹爪。

通行高度默认至少高于目标顶面 20 cm，不会为了横移而先降低当前高度。
更大距离可设置 `hover_clearance:=0.15 travel_clearance:=0.25`。
悬停距离不得小于 5 cm；该工具不用于接触物体。

停止当前验证：

```bash
rosservice call /handeye_yolo_verify/stop "{}"
```

`stop` 请求取消本节点发送的 MoveIt 轨迹，不替代控制柜急停。
规划期间收到停止命令也会阻止随后下发轨迹。程序不会自动开始下一次运动。

```bash
rostopic echo /handeye_yolo_verify/status
```

## 低速设置与拒收条件

默认速度缩放 **3%**、加速度缩放 **2%**。笛卡尔轨迹生成后显式重新时间参数化，
并在需要时进一步拉长时间，保留路径且同步缩放时间戳、速度和加速度。

| 设置 | 默认值 |
| --- | --- |
| 关节速度检查上限 | 0.05 rad/s |
| 关节加速度检查上限 | 0.08 rad/s² |
| TCP 采样点段速度限制 | 0.02 m/s |
| 笛卡尔插值步长 | 0.005 m |
| 相邻轨迹点关节跳变上限 | 0.12 rad |

关节限速检查包含轨迹点间五次多项式的速度/加速度极值；TCP 速度由 FK 采样段估算
并留出余量，不是控制器级连续 TCP 限速保证。实际速度还受硬件轨迹控制器影响。
配置对可设置速度有上限，避免误用普通分拣的高速参数。

必须获得完整、避碰的笛卡尔路径。路径不完整、IK 关节跳变、碰撞、横移时姿态不朝下
或高度不足、桌面碰撞体未就绪时拒绝执行，不回退到改变动作顺序的关节空间规划。
如果起始位置无法完成抬升/高处转向，应先用已有示教流程调整到开阔观察位。

执行前重新检查目标是否仍新鲜稳定，以及机器人是否仍在规划起点。
多目标、目标丢失、反馈过期和目标在规划期间移动都会阻止执行。
执行时反馈丢失会请求取消轨迹。执行过程中不持续重选目标，避免遮挡导致目标跳变，
因此待测物体应保持静止。

## 怎样评价标定结果

观察或独立测量**夹爪/TCP 的竖直投影与选定物体中心点的偏差**。
在中心、四角及不同高度分别测试，记录实际 Δx、Δy 和重复性。
验证期间不要额外添加抓取 XY 补偿来抹掉偏差。

默认 `~/handeye_yolo_verify/` 保存规划、完成或失败的 JSON，包含感知坐标、悬停目标、
参数、标定 YAML 快照、轨迹时长和最终反馈 FK。程序还检查观测相机坐标系与标定一致。
显示的 `tracking error` 仅为机器人跟踪规划目标的误差，
**不是手眼标定误差**。YOLO 框中心、深度、物体几何、TCP 和机器人运动学都会影响对齐。
需要更纯粹的标定验证时，仍使用 ArUco 留出姿态一致性程序。

## 连接已经启动的环境

已有机器人控制、MoveIt、相机、标定 TF 和 YOLO 三维目标时，不要重复启动完整入口。
先停止原分拣任务，准备同名桌面碰撞体 `hybrid_sorting_table` 与就绪话题
`/hybrid/planning_scene_ready`（完整入口使用 `hybrid_planning_scene.py` 发布），再启动：

```bash
rosparam load "$(rospack find aubo_handeye_validation)/config/handeye_yolo_verify.yaml" /handeye_yolo_verify
rosparam set /handeye_yolo_verify/calibration_file "$(rospack find aubo_ros_control)/config/aubo_realsense_handeye_eye_on_base.yaml"
rosrun aubo_handeye_validation handeye_yolo_verify.py
```

参数也可以放在已有 launch 的节点内加载：

```xml
<node pkg="aubo_handeye_validation" type="handeye_yolo_verify.py"
      name="handeye_yolo_verify" output="screen">
  <rosparam command="load" file="$(find aubo_handeye_validation)/config/handeye_yolo_verify.yaml"/>
  <param name="calibration_file" value="$(find aubo_ros_control)/config/aubo_realsense_handeye_eye_on_base.yaml"/>
</node>
```

## 已验证范围

离线检查覆盖目标稳定性、已知几何和朝下方向、悬停高度、连续关节插值限速，
以及模拟 MoveIt/消息下的两种笛卡尔 API、不完整路径/碰撞拒收、目标筛选、
反馈过期、预览不执行与停止取消。未执行真机动作，仍需在实际控制器和工位验证。
