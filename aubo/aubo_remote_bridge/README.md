# AUBO ROS Noetic：C++串口桥与Gazebo命名位置验证

ROS节点已改为 **C++14 / roscpp**。Python仅保留无ROS串口monitor和离线协议对照工具。新增代码以中文注释说明协议、线程/回调归属、请求去重、取消和到位核对。

## 实现与版本

`serial_bridge_node` 负责串口与ROS数据转换，`named_pose_server` 负责Gazebo命名位置请求；两者为独立C++进程，规划期间串口仍正常收发。默认 `serial_bridge.launch` 不转发运动命令；`named_pose_gazebo.launch` 才启动专用Gazebo、MoveIt和请求节点。

**必须同时更新第九项v2固件和电脑程序。** v1固件/旧Python ROS桥不兼容；原serial_bridge_node.py已删除。没有真机遥控入口或AUBO SDK节点。

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

## 无硬件，先验证Gazebo

```bash
roslaunch aubo_remote_bridge named_pose_gazebo.launch serial:=false
```

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

速度/加速度缩放0.15，规划5秒、3次尝试，单任务墙上时间上限60秒。保留MoveIt碰撞检查；失败不退回直接发关节角。规划地面顶部在固定基座下5cm，避免固定安装接触误报；围挡与SDF一致。

## 自动验证

```bash
catkin_make tests
rostest aubo_remote_bridge named_pose_gazebo.test
catkin_test_results
```

rostest无需串口，自动检查home实际到位、同编号不增加Action goal、zero实际到位、非法目标拒绝、执行中取消。headless Gazebo，总期限240秒；不要与已运行的同名场景同时启动。

离线回归：仓库tools/test_remote.ps1执行真实MCU C、C++核心及Python协议。ROS运行代码阅读顺序：serial_bridge_node.cpp -> posix_serial.hpp -> remote_core.cpp -> named_pose_server.cpp -> launch/worlds -> test/named_pose_gazebo_test.cpp。

## 验证边界

本机已运行C/C++14/Python协议回归和Keil构建。Windows仅有停止的docker-desktop WSL，没有可用Noetic/Gazebo，**ROS节点目标编译及Gazebo动态rostest尚未实跑**。XML/配置检查和离线核心测试不替代运行时联调；未执行真机动作。
