# AUBO Planning

该包提供 AUBO i5 + 双指夹爪的 MoveIt 1 C++ 接口示例：

- `gripper_control_demo`：按 MoveIt 命名状态执行夹爪打开、闭合或循环动作。
- `pick_place_demo`：向 PlanningScene 加入工作台和物体，使用 MoveIt `pick()` / `place()` 完成抓取与放置。

## 编译

在包含本仓库源码的 catkin 工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

## 快速验证（MoveIt fake controller）

```bash
roslaunch aubo_planning gripper_control.launch command:=cycle
roslaunch aubo_planning pick_place.launch
```

单独控制夹爪时，`command` 可设为 `open`、`close` 或 `cycle`。

## Gazebo 或真机已有 MoveIt 时

先启动机器人、控制器和 `move_group`，再关闭示例 launch 对 MoveIt demo 的重复启动：

```bash
roslaunch aubo_planning gripper_control.launch start_demo:=false command:=open
roslaunch aubo_planning pick_place.launch start_demo:=false
```

抓取点和放置点可通过 launch 参数调整：

```bash
roslaunch aubo_planning pick_place.launch \
  object_x:=0.45 object_y:=-0.15 object_z:=0.15 \
  place_x:=0.45 place_y:=0.15 place_z:=0.15
```

## 机构参数

- `joint1`、`joint2` 的零位为张开，`0.45 rad` 为默认闭合状态。
- URDF 安全范围暂设为 `0.0–0.55 rad`，速度上限为 `1.0 rad/s`。
- 两个关节使用同一正方向命令；两侧关节坐标系的镜像朝向使手指对称闭合。
- 真机运行前应低速标定零位、闭合角和最大无干涉角，并同步修改
  `jiazhua.urdf`、`aubo_i5.srdf` 以及抓取程序的 `gripper_closed` 参数。
- `grasp_offset_z` 默认为 `0.12 m`，表示俯抓时物体中心到
  `gripper_base_link` 的竖直距离；若更换指尖或 TCP，需要同步标定该值。
- 当前仓库已配置 ros_control transmission、Gazebo 控制器和 MoveIt 控制器映射。
  真机驱动仍须把 `joint1`、`joint2` 注册为 PositionJointInterface，并把轨迹命令
  转换为实际夹爪电机协议；仅有 YAML 配置不会自动驱动物理电机。

## 运行前检查

```bash
rostopic echo /aubo_i5/joint_states
rostopic list | grep gripper_controller
rosservice call /aubo_i5/controller_manager/list_controllers
```

控制器正常时，应看到 `aubo_i5_controller` 和 `gripper_controller` 均为 `running`。
