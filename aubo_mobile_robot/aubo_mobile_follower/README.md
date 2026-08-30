# AUBO 移动机器人跟随与循迹

本功能包参考 `simple_follower` 的激光跟随、颜色目标跟随和循线思路，针对 AUBO
复合移动机器人重新实现。三个模式都先处理机械臂姿态，再允许底盘运动：

| 模式 | 传感器 | 机械臂命名姿态 | 作用 |
| --- | --- | --- | --- |
| 激光跟随 | 合并后的 `/scan` | `transport` | 收拢机械臂后跟随前方目标点簇 |
| 颜色跟随 | `/hand_camera/image_raw` | `follow_forward` | 机械臂低位折叠，相机保持水平朝前，跟随指定 HSV 色块 |
| 循线 | `/hand_camera/image_raw` | `observe` | 相机向下，沿地面引导线行驶 |

`prepare_arm_pose.py` 会先检查命名姿态是否存在，再要求 MoveIt 从当前状态规划出
非空、无碰撞轨迹。只有执行成功（相机模式还必须收到有效图像）才发布
`/aubo_mobile_follower/arm_ready=true`。因此机械臂不可达、规划失败或相机未启动时，
底盘始终保持停止。

`follow_forward` 保留 `transport` 的低位 A 形折叠构型，调整腕部使
相机光轴水平对准车体正前方，同时用 `wrist3_joint` 校正相机横滚，
保持图像地平线水平。这会将相机从高位观测
收至靠近底盘的低位观测位置。六个关节值均在 URDF 限位内，并且不进行
未经验证的笛卡尔目标直达。

## 使用方法

新增功能包后先在 catkin 工作空间根目录重新编译并加载环境：

```bash
catkin_make
source devel/setup.bash
```

机器人或 Gazebo 已经运行时，不要同时运行 `move_base`，直接选择一种模式：

```bash
# 激光目标跟随
roslaunch aubo_mobile_follower laser_follow.launch

# 激光跟随 + RViz 诊断（默认同时启动 Gazebo）
roslaunch aubo_mobile_follower laser_follow_debug.launch

# 红色色块跟随
roslaunch aubo_mobile_follower color_follow.launch

# 黑线循迹
roslaunch aubo_mobile_follower line_follow.launch
```

需要由跟随启动文件一并启动 Gazebo 时：

```bash
roslaunch aubo_mobile_follower color_follow.launch start_robot:=true
```

默认会启动 MoveIt、双雷达合并和激光安全层。如果这些节点已经由其他入口启动，
应避免重复节点：

```bash
roslaunch aubo_mobile_follower line_follow.launch \
  start_moveit:=false start_scan_merger:=false
```

## 算法说明

### 激光跟随

只搜索车体前方 ±60°。相邻距离差小于阈值的扫描点组成点簇，至少四点才会成为
候选目标；随后根据上一帧位置保持目标关联。控制器维持默认 1.0 m 距离。找不到
目标或雷达超过 0.5 秒未更新时立即停车。

### 颜色目标跟随

对手眼相机图像做 HSV 阈值、形态学去噪和最大轮廓筛选。轮廓横向偏差控制角速度，
轮廓面积占比近似表示距离并控制线速度。默认识别红色目标；这是一种单目近似，
目标实物尺寸应保持一致。

### 循线

仅处理画面下方区域，提取引导线掩码及重心，使用 PD 控制角速度。弯道偏差越大，
线速度会自动降低。默认跟随黑线。

## 参数调整

所有速度、PID、目标距离、HSV 和丢失超时参数都在
`config/follower.yaml`。调试图像话题：

- `/aubo_mobile_follower/color_debug`
- `/aubo_mobile_follower/line_debug`

状态话题为 `/aubo_mobile_follower/state`。首次实机测试应架空驱动轮或使用急停，
先把最大线速度降到较低值，再标定相机 HSV、目标面积和雷达安全距离。

激光 RViz 诊断话题为 `/aubo_mobile_follower/laser_debug`：青色球表示通过点数和
连续性检查的候选点簇，绿色球表示当前控制目标，黄色弧线表示期望跟随距离。
`laser_follow_debug.launch` 默认还会在车前生成名为 `follower_target` 的红色测试
立柱，可以通过 `/gazebo/set_model_state` 服务移动；不需要测试目标时传入
`spawn_test_target:=false`。
RViz 中的 `Target Control` 使用标准 Interactive Markers 插件显示 X/Y 拉杆；选择
顶部的 Interact 工具后拖动拉杆，即可实时移动 Gazebo 中的测试立柱。
