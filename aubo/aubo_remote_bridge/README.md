# AUBO ROS Noetic：串口遥控与命名位置验证

## 笛卡尔点动遥控（新增）

`cartesian_serial_bridge` 对接 `aubo_ros_control`，提供 XYZ 点动、基/工具坐标系、
0.1–100 mm/s速度设置、显式使能/停止、home action及取消、故障复位和六轴状态回传。
支持“摇杆XY + Z按键”或“XYZ六方向键”，要求按住独立允许运动键才点动。
串口丢帧或输入失效会停止、禁用并取消回home；重新连接不会自动恢复运动。

```bash
roslaunch aubo_remote_bridge cartesian_remote_gazebo.launch port:=/dev/ttyUSB0
# 实机入口会启动SDK，可能上电/松刹车，运动需显式请求
roslaunch aubo_remote_bridge cartesian_remote_real.launch port:=/dev/ttyUSB0 robot_ip:=192.168.1.2
```

STM32应依据[笛卡尔串口协议](CARTESIAN_SERIAL_PROTOCOL.md)实现 `0x11/0x12/0x13` 报文；
其中包含逐字节格式、状态表、开机/使能/回home/取消流程及可复用C代码。
此入口使用独立报文类型；下述原HOME/ZERO验证入口及其固件协议继续单独使用。

ROS节点已改为 **C++14 / roscpp**。Python仅保留无ROS串口monitor和离线协议对照工具。新增代码以中文注释说明协议、线程/回调归属、请求去重、取消和到位核对。

## 实现与版本

`serial_bridge_node` 负责串口与ROS数据转换，`named_pose_server` 负责Gazebo命名位置请求；两者为独立C++进程，规划期间串口仍正常收发。默认 `serial_bridge.launch` 不转发运动命令；`named_pose_gazebo.launch` 才启动专用Gazebo、MoveIt和请求节点。

以下为原HOME/ZERO验证入口说明。**必须同时更新第九项v2固件和电脑程序。** v1固件/旧Python ROS桥不兼容；原serial_bridge_node.py已删除。此命名位置验证入口只运行Gazebo，不连接AUBO SDK。

## 构建

整个aubo_remote_bridge放入Noetic Catkin工作空间src下，同一环境只保留一份同名包。

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws  # 改成实际工作空间
rosdep install --from-paths src --ignore-src -r -y
catkin_make --pkg aubo_remote_bridge
source devel/setup.bash
```

ROS串口使用Linux termios。仅运行Python串口monitor时需要pyserial。

若提示找不到 `named_pose_server`，请检查 `devel/lib/aubo_remote_bridge/named_pose_server`
是否已生成，并重新 `source devel/setup.bash`。这是C++编译产物，无需给源码执行 `chmod +x`。
若工作空间设置了 `CATKIN_WHITELIST_PACKAGES`，须在原白名单中加入 `aubo_remote_bridge`
后再编译；单独使用 `catkin_make --pkg aubo_remote_bridge` 不会将该包加入配置白名单。

## 无硬件，先验证Gazebo

```bash
roslaunch aubo_remote_bridge named_pose_gazebo.launch serial:=false
```

若出现 `Could not find parameter robot_description`，随后机械臂/夹爪控制器加载失败，
请确认 `/remote_validation/aubo_i5/robot_description` 存在；轨迹控制器直接读取该命名空间，
不会使用父级 `/remote_validation/robot_description`。本launch已在两个命名空间加载相同URDF。
修改launch后重启即可，无需重新编译。`No p gain specified for pid` 在当前PositionJointInterface
仿真配置下不阻止控制器启动；判断启动结果应检查controller_manager中三个控制器均为`running`。

场景包含固定AUBO i5、地面、工作区外两侧围挡和方向参考色块；初始为零关节角。home/zero直接取现有aubo_i5.srdf，不另写目标角度。色块不代表末端目标点。RViz显示实际机器人，Gazebo显示物理运动。

```bash
rostopic echo /remote/motion_state
rostopic echo /remote/command_result
```

等motion_state.state=0（IDLE）后请求home；**accepted只表示接收，state=4才表示实际到位**。

```bash
rosservice call /remote/request_named_pose "{target: home, session: 100, request_id: 1}"
# 等待state=4，再用新编号请求zero
rosservice call /remote/request_named_pose "{target: zero, session: 100, request_id: 2}"
```

相同session/request_id重试回放缓存应答，不再发goal；同编号更换目标会拒绝。一次只运行一个任务，执行中第二个HOME/ZERO返回BUSY。

幂等缓存保留最近64条请求；不要复用已完成编号，超出缓存窗口的历史服务请求不保证重试去重。串口端另有最新编号/会话校验。

```bash
rosservice call /remote/request_named_pose "{target: home, session: 100, request_id: 3}"
# 执行中取消
rosservice call /remote/request_named_pose "{target: cancel, session: 100, request_id: 4}"
```

只取消本节点当前goal，不使用cancelAllGoals。2秒仍未终止会显示FAILED并保持任务锁，不能据此认定已停止。若取消与成功同时发生，可能最终DONE，原因见中文message。

## 接入遥控器

PA9(TX)->USB-TTL RXD，PA10(RX)<-TXD，GND共地；3.3V逻辑，115200 8N1。独立供电时转接器只接TX/RX/GND。摇杆和OLED接线与第九项说明相同。

```bash
roslaunch aubo_remote_bridge named_pose_gazebo.launch serial:=true port:=/dev/ttyUSB0
```

可以用稳定的/dev/serial/by-id路径。串口只能由一个程序使用；不要同时启动第二套同名验证场景。

- 上电松手校准；菜单页上下选择HOME/ZERO，回中后短按并松开确认。
- 左右切换五页：菜单、输入、J1/J2、J3/J4、J5/J6；回中后才能再次导航。
- 任意页长按约0.8秒发送一次CANCEL；松开不追加短按。反馈页短按不执行动作。
- 恢复连接/ADC重新校准的第一份输入仅建立基线，旧请求显示REJECT，需要重新确认。
- 串口请求运行期间需保持有效输入和链路；拔线、ADC失效、桥节点无新输入时请求取消。

电脑服务测试不需要串口心跳，但仍需专用Gazebo场景。ROS详细message用中文，OLED使用ASCII短状态，未加入中文字库。

## 状态与隔离

| 编号 | OLED | 含义 |
|---:|---|---|
| 0 | IDLE | 空闲且仿真就绪 |
| 1 | ACCEPT | 请求已接收 |
| 2 | PLAN | MoveIt规划中 |
| 3 | RUN | 执行监控中 |
| 4 | DONE | Action成功，实际六关节误差≤0.03rad连续保持200ms |
| 5 | FAILED | 规划/执行/核对失败，或取消未确认；以message为准 |
| 6 | STOPPING | 等待取消确认 |
| 7 | CANCELED | 请求已取消 |
| 8 | REJECT | BUSY、未就绪、编号冲突或禁用命令 |
| 9 | VERIFY | Action成功后核对实际位置 |
| 10 | UNKNOWN | 状态缺失或尚未就绪 |

OLED第3行是最新请求应答，第4行是当前动作；新请求REJECT时，原任务仍可能RUN。

运动模型、MoveIt和控制器均在/remote_validation下，控制器绝对路径为/remote_validation/aubo_i5/aubo_i5_controller。节点同时检查/use_sim_time、特定Gazebo模型、前进的/clock、近期完整关节反馈及专用MoveGroup Action。建议独立ROS master运行，避免与其他机器人同名TF混用。

仿真速度/加速度缩放默认0.6（原为0.15），相对于MoveIt关节限制生效。可在启动时调整，范围均为(0, 1]；例如：

```bash
roslaunch aubo_remote_bridge named_pose_gazebo.launch serial:=false velocity_scaling:=0.8 acceleration_scaling:=0.8
```

规划5秒、3次尝试，单任务墙上时间上限60秒。保留MoveIt碰撞检查；失败不退回直接发关节角。规划地面顶部在固定基座下5cm，避免固定安装接触误报；围挡与SDF一致。

## 自动验证

```bash
catkin_make tests
rostest aubo_remote_bridge named_pose_gazebo.test
catkin_test_results
```

rostest无需串口，自动检查home实际到位、同编号不增加Action goal、zero实际到位、非法目标拒绝、执行中取消。headless Gazebo，总期限240秒；不要与已运行的同名场景同时启动。

离线回归：仓库tools/test_remote.ps1执行真实MCU C、C++核心及Python协议。ROS运行代码阅读顺序：serial_bridge_node.cpp -> posix_serial.hpp -> remote_core.cpp -> named_pose_server.cpp -> launch/worlds -> test/named_pose_gazebo_test.cpp。

## 验证边界

此前已运行C/C++14/Python协议回归和Keil构建。2026-10-20在Linux/ROS Noetic环境完成两个ROS节点编译，
并以默认速度/加速度缩放0.6通过Gazebo动态rostest：home/zero实际到位、重复请求去重、非法目标拒绝及执行中取消。
测试无需串口；未执行真机动作。
