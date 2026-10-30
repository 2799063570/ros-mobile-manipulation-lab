# AUBO 眼在手外单罐实机分拣测试

日期：2026-10-30。固定底座 AUBO i5、D435i（335222076553）、Inspire 串口夹爪。

本文的单罐抓放结果保留为历史实测。当前入口已更新为三类容器配置，
新位置与最低放置高度见文末；原临时放置点不再是当前启动默认值。
`eye_to_hand_can_sorting_real.launch` 默认各类处理一个，
`eye_to_hand_yolo_sorting_real.launch` 共享该工位并默认连续处理多个实例。

## 观察位置

从 `/aubo_i5/joint_states` 读取 15 帧静止实机反馈，最大单轴跨度
`3.34e-6 rad`，保存为 `real_observe`，未使用仿真 `observe` 覆盖。

| 关节 | 弧度 |
| --- | ---: |
| shoulder_joint | 1.067721366882 |
| upperArm_joint | -0.941510260105 |
| foreArm_joint | -2.496562957764 |
| wrist1_joint | 0.026582198218 |
| wrist2_joint | -1.634069800377 |
| wrist3_joint | -2.071427106857 |

该姿态下 TCP 在 `base_link` 下约为 `[0.024065,-0.180215,0.153386] m`，
本轮图像中机械臂未遮挡桌面 can。关节值已加入
`aubo_i5.srdf` 和 `aubo_i5_with_camera.srdf`；原始采样信息保存在
[`real_observe_capture.json`](../aubo/aubo_sorting/config/real_observe_capture.json)。
分拣观察、工作准备及结束姿态均选择 `real_observe`。

## 工位与规划场景

- 用户确认沿用原桌面：顶高 `0.00285 m`，XY 中心 `[-0.0442,-0.6707]`，
  尺寸 `0.74 × 0.93 × 0.20 m`。碰撞体中心 Z 为 `-0.09715 m`，
  XY 各边加 `0.02 m` 余量，ID 为 `sorting_table`。
- 相机支架采用仓库 `camera_stand_reference.yaml` 的四段原工位参考包络，
  ID 为 `camera_stand`，包括下支撑、立柱、横杆、相机本体。
  本轮没有重新实测支架尺寸，换安装位置需更新该文件或传入实测副本。
- 两个对象通过 `/apply_planning_scene` 写入，并从 `/get_planning_scene`
  读回核对；当前实机姿态通过 `/check_state_validity`。
- 用户要求先验证抓取、允许任选放置点，因此选取桌面空闲点
  `[0.18,-0.65] m`，未向右上方纸箱内投放。

## 固定相机 TF

本轮初始运行的 `aubo_real_bringup.launch` 使用腕部相机模型，包含
`camera_color_optical_frame`。启动固定相机标定后，同一帧出现两个父变换来源，
导致几何节点误用腕部变换并拒绝深度目标。

在实机静止时，仅重启 MoveIt 与 robot_state_publisher，加载
`aubo_i5.xacro` / `aubo_i5.srdf`，保持已有硬件接口和控制器运行。
固定标定唯一发布 `base_link → camera_color_optical_frame`。
真实分拣共用入口的默认相机命名空间已修正为 `/camera`，与保存的标定帧一致。

相机配置采用彩色、对齐深度和固定标定。当前平躺 can 的物高（直径）为
`0.065 m`，深度模式检测中心约为 `[-0.036,-0.708,0.036] m`，
置信度约 `0.94`，抓取角约 `1.50 rad`，检测短边约 `0.070 m`。
桌面空白区域实测 Z 中值约 `0.003–0.004 m`，接近沿用的桌高。

## 当前入口启动

完整新会话使用以下命令，它加载当前三容器配置，不是重放历史单罐实验：

```bash
source /home/zlab/aubo/ros_mobile_manipulation_lab/devel/setup.bash
roslaunch aubo_sorting eye_to_hand_can_sorting_real.launch \
  containers_config:=$(rospack find aubo_planning)/config/sorting_containers_real.yaml \
  camera_stand_config:=$(rospack find aubo_planning)/config/camera_stand_reference.yaml
```

硬件、MoveIt、相机或夹爪已启动时，按实际启动情况追加对应的
`start_robot:=false start_camera:=false start_gripper:=false`；已经运行的 MoveIt 和
TF 必须使用眼在手外模型，不能仅通过跳过启动来纠正已有的腕部模型。
夹爪使用 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，本轮映射至 `/dev/ttyUSB0`。
默认速度、加速度比例均为 `0.10`，默认不自动运动。

```bash
rosservice call /sorting/move_to_observation '{}'
# 等待 /sorting/state = READY | waiting for panel command
rosservice call /sorting/start '{}'
# 需要停止时
rosservice call /sorting/stop '{}'
```

上述单罐实测时，任务配置为只抓放一个 can，然后返回 `real_observe`。
当前配置为 `sort_classes: [bottle, can, box]`、`continuous_sorting: false`，
即按类别顺序各处理一个；某类缺失会等待检测并在超时后失败。
完整路径预检通过：预抓取、下降、抬升、预放置、放置、退离、返回观察位，
全部直线段完整率 `100%`。下降、抬升、放置、退离均启用碰撞检查。
本轮未修改硬件控制算法、控制器参数或运动限位。

## 验证记录

抓取后 TCP 到达约 `[-0.035718,-0.707605,0.252863] m`；夹爪返回
`error_code=0, motion_state=6, curopen=962`，相机图像确认罐体随夹爪悬空。

放置及退离直线段同样达到 `100%`，执行时长分别约 `10.29 s` 和 `10.34 s`。
释放后相机重新检测到 can，中心约为 `[0.161,-0.646,0.032] m`，原位置已空出。
本轮验证了实际抓起、搬运和释放；最终 XY 是释放后视觉估计，
与指令放置点约有 `19 mm` 差异，尚未分离手眼误差、夹持偏心及释放滚动的影响。

预抓取、横移和返回采用 MoveIt 关节空间规划。本轮横移、返回出现较高绕行，
因此 `lift_height=0.25 m` 仅描述抓取后直线抬升终点，不是整条搬运路径的高度上限。
本轮以验证抓放为目的，尚未优化路径长度和节拍。

09:14:43 完成最终核对（北京时间）：任务为 `READY | waiting for panel command`，
已回到 `real_observe`；最大关节回位误差 `3.30e-5 rad`，15 帧静止跨度
`6.20e-6 rad`。夹爪已张开，状态 `1`，错误码 `0`。
返回后的相机视野无遮挡，连续 10 帧均检测到放置区域的 can。
桌面与四段支架碰撞体仍在，最终姿态碰撞检查有效。
ROS bag 中 7 段运动对应的 14 条控制器/MoveIt 结果消息全部成功。

本轮证据保存在工作区 `records/eye_to_hand_can_20261030_090117/`：
[最终核对](../../records/eye_to_hand_can_20261030_090117/final_check.json)、
[抓取抬升图像](../../records/eye_to_hand_can_20261030_090117/lifted.jpg)、
[返回观察位图像](../../records/eye_to_hand_can_20261030_090117/final_observe.jpg)，
以及 ROS bag、执行结果、运行时参数、规划场景和过程帧。

分拣包 launch 配置测试共 11 项通过，包括复用硬件、固定相机话题、
单次任务、观察/结束命名姿态、桌面碰撞尺寸与任务参数一致性。

## 三容器与最低放置高度更新

配置文件为
[`sorting_containers_real.yaml`](../aubo/aubo_planning/config/sorting_containers_real.yaml)。
用户输入按厘米、容器中心 XY、外部尺寸理解，换算如下（米）：

| 类别 | 中心 X | 中心 Y | X 长度 | Y 长度 | 容器高度 |
| --- | ---: | ---: | ---: | ---: | ---: |
| bottle | -0.40 | -0.18 | 0.27 | 0.15 | 0.18 |
| can | -0.42 | -0.3375 | 0.20 | 0.165 | 0.16 |
| box | -0.42 | -0.5425 | 0.20 | 0.245 | 0.16 |

三者均以 `base_link` 表示，外侧底面 Z 为原桌高 `0.00285 m`；
壁厚和底厚未实测，暂用 `5 mm`，通过 YAML 可修改。
每个容器发布五个碰撞长方体，保持上方及内部开放。
`place_targets` 同时用于任务目的地与碰撞容器中心，避免坐标重复维护。

`minimum_place_z: 0.18` 表示 TCP 在任务目标坐标系中的最低绝对 Z。
当前目标坐标系为 `base_link`，实际放置高度为：

```text
place_z = max(table_z + object_height/2 + grasp_height_offset + place_clearance,
              minimum_place_z)
```

抓取高度不受影响；预放置/退离高度须高于 `place_z`，否则在抓取前报配置错误。
未设置该参数的既有仿真和移动平台保留原高度算法。
壁厚、物体尺寸及夹爪实际包络仍影响箱内路径可行性。
本次配置更新在独立 MoveIt 环境中验证三容器、桌面、相机支架共存，
未进行新的三容器实机抓放；原实机 ROS 主控当前已退出。
两份 C++ 任务入口已编译，31 项 Python 测试和 4 项 C++ 高度测试通过。
场景验证报告见工作区
[`records/sorting_containers_20261030/scene_validation.json`](../../records/sorting_containers_20261030/scene_validation.json)。

## 连续多物体入口

运行 `roslaunch aubo_sorting eye_to_hand_yolo_sorting_real.launch` 可使用同一工位的连续模式。
该入口通过已有实机入口加载桌面、相机支架和三容器，观察及结束姿态均为 `real_observe`，
最低放置 TCP Z 仍为 `0.18 m`。运动仍由观察服务与开始服务触发。
类别和抓放参数复用 `eye_to_hand_can_real.yaml`；连续实例参数单独保存在
`aubo_sorting/config/eye_to_hand_continuous_real.yaml`。

稳定目标独立编号，同类多个目标分别领取；缺少某类不阻塞其他就绪类别。
完成一个抓放后继续处理有效实例，队列不可用时回观察位。
观察位持续 2 秒、至少 5 帧有效空检测后结束本轮；感知失联或抓放失败会停止并报错。
放置区排除半径为 `0.18 m`，覆盖当前容器轮廓；容器内已放置目标不作为新抓取实例。
每类仍共用容器中心放置点，未实现堆叠高度更新或自动排列。
连续入口已验证 launch 参数、驱动参数透传和现有实例队列测试，尚未进行多物体实机抓放。
