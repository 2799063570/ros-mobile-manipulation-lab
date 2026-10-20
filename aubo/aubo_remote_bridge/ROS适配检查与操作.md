# 第11项与 cartesian_remote_gazebo.launch 适配检查

检查日期：2026-10-20。

检查对象：

- STM32：`D:\files\code\stm32-demo\11-1 AUBO笛卡尔串口遥控`
- ROS：`D:\files\code\wheeltec_robot\wheeltec_robot\aubo\aubo_remote_bridge`
- 点动控制器：同一工作空间的 `aubo_ros_control`

## 结论

两边使用配套协议，可以连接；第11项的双轴摇杆加自带SW方案不需要增加Z按键或独立许可按键。但发现一个使能应答与控制器状态不同步的竞态，可能出现“PC已使能，STM32仍LOCKED，摇杆不动”。因此当前结论是协议与接口适配，仍有使能时序缺陷，尚未完成实板/Gazebo联调。

本次未修改固件、ROS源码或HEX，只新增本说明及 `tests/build/jog/check_enable_feedback_order.c` 离线复现程序。

## 已核对的适配项

| 项目 | 检查结果 |
|---|---|
| 串口 | 两边均为115200、8N1、无流控；STM32 USART1，PA9 TX、PA10 RX |
| 外层协议 | v2，A5 5A帧头，小端字段，CRC-16/CCITT-FALSE |
| 类型 | POLL=0x11、INPUT=0x12、FEEDBACK=0x13 |
| 载荷长度 | POLL 4字节、INPUT 18字节、FEEDBACK 28字节；均不含帧头和CRC |
| 编解码源码 | 两个头文件相同；两个C文件去除include路径前缀后相同 |
| 命令 | ENABLE=1、STOP=2、HOME=3、CANCEL=4、RESET=5 |
| 输入 | 三个方向分量−1000～1000；PC仅解释向量和deadman，不要求某种实体按键布局 |
| XY/Z | STM32在XY模式输出X/Y，在Z模式用摇杆Y输出Z；两种模式分开操作 |
| 速度 | 串口单位0.1 mm/s，STM32默认100即10 mm/s；PC换算为m/s并限制合成速度 |
| 坐标 | 0=BASE、1=TOOL；桥与控制器读取同一base_link/tcp_link配置 |
| ROS接口 | /cartesian_jog/command、state、set_enabled、reset、move_home一致 |
| 关节反馈 | 总入口给桥和控制器均传入/aubo_i5/joint_states；六轴顺序相同，mrad回传后显示为度 |
| 轮询/断线 | PC约20ms轮询；双方输入/反馈窗口100ms；超时或序号回卷更换会话，恢复后需要重新使能 |

该launch会同时启动Gazebo点动场景与串口桥，并设置panel=false。没有电脑点动窗口是预期行为。它不启动实机SDK。

旧的 `serial_bridge.launch`、`named_pose_gazebo.launch` 及第9项固件使用另一套命令语义，不适用于本第11项点动入口。

## 发现：使能完成可能早于ENABLED_IDLE状态

代码位置：

- `aubo_ros_control/src/cartesian_jog.cpp:81`：控制器状态每100ms发布一次。
- 同文件 `:405`：使能服务设置enabled后直接返回，不等待状态话题发布。
- `aubo_remote_bridge/src/cartesian_serial_bridge.cpp:248`：服务成功后设置armed并将请求记为DONE，没有等待状态变为ENABLED_IDLE。
- 同文件 `:309`：FEEDBACK中controller_state取最近收到的状态话题，因此仍可能为DISABLED。
- `System/JogRemote.c:307`：MCU收到对应ENABLE的DONE，只有同帧为IDLE且armed=1才授权；其他终态组合会撤销EnablePending。

可能的顺序：

1. 控制器最近一次状态为DISABLED。
2. ENABLE服务成功，桥报告DONE、armed=1。
3. 下一次串口反馈仍携带缓存的DISABLED。
4. MCU清除使能等待；后来即使收到ENABLED_IDLE也不再授权。

本次用真实MCU业务源码，注入上述反馈顺序，得到：

```text
DONE + DISABLED: authorized=0 pending=0
Later ENABLED_IDLE: authorized=0 pending=0
Hold SW and deflect: axis_x=1000 deadman=0
```

这是离线复现该反馈组合的处理结果；未在实际ROS进程或实板上测量出现频率。原有ROS串口测试用Python模拟MCU，未执行MCU的EnablePending授权逻辑，因此不能覆盖此问题。

识别方式：OLED第3行LOCKED、第4行交替显示DONE/ENABLED；ROS日志armed=1，但按住SW时deadman仍为0。

临时恢复：松开并回中，短按进菜单，选择STOP / DISABLE；等请求DONE、控制器DISABLED后再选ENABLE。重试仍可能遇到同一竞态。应在后续修复中让电脑端等待控制器确认ENABLED_IDLE后再回ENABLE/DONE，并保留等待期间的超时和输入变化撤销逻辑；不能只增加固定延时或无条件放宽MCU授权。

## 接线和烧录

| 连接 | 引脚 |
|---|---|
| 摇杆X/Y | PA0/PA1，3.3V供电 |
| 摇杆SW | PA2，按下接GND |
| STM32 TX → USB-TTL RXD | PA9 |
| STM32 RX ← USB-TTL TXD | PA10 |
| 共地 | STM32、摇杆、OLED、USB-TTL的GND |
| OLED CLK/DIN | PB14/PB15 |
| OLED CS/DC/RES | PB0/PB1/PB12 |

USB-TTL使用3.3V逻辑。STM32独立供电时，转接器接TX、RX、GND即可。

打开本目录 `Project.uvprojx` 编译并烧录 `Objects/Project.hex`；BOOT0=0。上电松开SW、让摇杆保持中位，等待约160ms完成32个稳定样本校准。

## Ubuntu / ROS端操作

以下命令在运行ROS Noetic/Gazebo的Linux终端中执行，不能直接在Windows PowerShell执行。USB-TTL需由该Linux环境实际识别；如果在虚拟机中运行ROS，要将USB串口分配给虚拟机。

1. 在实际Catkin工作空间根目录编译并加载环境：

```bash
source /opt/ros/noetic/setup.bash
# 先cd到实际工作空间根目录，即包含src和devel的目录
catkin_make
source devel/setup.bash
rospack find aubo_remote_bridge
rospack find aubo_ros_control
ls -l /dev/serial/by-id/
```

`rospack find`应指向本次检查版本所在的工作空间，避免加载另一份同名旧包。若有Catkin白名单，应确保两个包都被构建。

2. 关闭串口助手/monitor；从零启动时执行：

```bash
roslaunch aubo_remote_bridge cartesian_remote_gazebo.launch \
  port:=/dev/ttyUSB0 max_speed_mm_s:=10.0
```

将端口替换为实际ttyUSB/ttyACM或by-id路径。这里把遥控点动速度上限限制为10 mm/s便于首次联调；HOME仍按控制器的关节速度配置执行，不受此线速度参数限制。

如果已经运行 `cartesian_jog_gazebo.launch`，只追加桥节点，不能再启动第二套完整场景；同时关闭已有电脑点动面板，避免多个操作端：

```bash
roslaunch aubo_remote_bridge cartesian_serial_bridge.launch \
  port:=/dev/ttyUSB0 joint_states_topic:=/aubo_i5/joint_states \
  max_speed_mm_s:=10.0
```

若提示Permission denied，检查 `ls -l /dev/ttyUSB0` 和 `id`；典型Ubuntu设备属dialout组，需将当前用户加入实际串口所属组并重新登录。

3. 在另外的终端加载同一工作空间环境后查看状态；每条echo持续占用一个终端：

```bash
rostopic echo /remote/cartesian_status
```

```bash
rostopic echo /cartesian_jog/state
```

需要时再检查：

```bash
rostopic hz /aubo_i5/joint_states
```

准备条件为：OLED PC:OK、校准提示消失、控制器DISABLED、关节页有有效角度。初始未使能时armed=0、deadman=0是正常状态。

## 摇杆操作顺序

1. 默认菜单选中KEEP POSITION；上下拨动选择菜单，每次先回中，再短按SW并松开确认。左右可在菜单、三组关节页和状态页之间切换。
2. 要点动可直接在菜单选择ENABLE，无需先选KEEP POSITION；ENABLE成功后自动进入点动页。保持回中、松键，直到请求DONE、控制器ENABLED，且点动页第3行显示HOLD SW。仅看到DONE不足以判定已获运动授权。
3. 默认BASE/XY、10 mm/s。按住摇杆自带SW，再左右拨动控制X、上下拨动控制Y。实际轴正负取决于摇杆安装方向与JogConfig.h反向设置。松开SW停止点动并减速，保留本次使能。
4. 切Z：松键回中，单独做一次全程中位的短按进入菜单，选XY / Z MODE，确认后选BACK TO JOG；在Z页按住SW，上下控制Z，左右不输出方向。XY和Z不能同时输出。
5. 只查看或保持当前位置时选KEEP POSITION。菜单和反馈页不发点动许可；点动时没有按住SW也不会运动。
6. 停用：松键回中，短按进入菜单，选STOP / DISABLE，等待DONE和DISABLED。修改BASE/TOOL或速度前先执行STOP。
7. 回HOME：禁用、停稳、回中松键后选HOME；等待DONE与DISABLED。HOME是固定关节目标，默认[0,0,1.6292,0,1.6628,0] rad，完成后不会自动使能。仿真初始姿态已接近此目标，刚启动就HOME可能看不到明显运动。
8. 取消HOME：在菜单/反馈页长按SW约0.8秒，或选择CANCEL HOME。等待最终CANCELED与DISABLED；取消与完成竞争时也可能最终DONE。STOPPING不是停止确认。HOME不要求全程按住SW，但要求有效输入和串口持续在线。

点动中持续按住SW超过0.8秒仍属于点动，不触发长按取消；点动过程中偏转过，回中再松键也不会进入菜单，需另做一次中位短按。软件停止不替代硬件急停。

## 不动时如何判断

| 表现 | 检查方向 |
|---|---|
| OLED OFFLINE/PC:--；serial_rx显示NO_RX | Linux端口、串口占用、TX/RX交叉、共地、是否烧录第11项HEX |
| 有字节但NO_FRAME | 115200 8N1、二进制帧、CRC；不能发送ASCII字母控制 |
| INPUT_REJECTED | 是否混入第9项旧协议、session/sequence/100ms窗口是否匹配 |
| INPUT_OK但input_valid=0 | 摇杆ADC、回中校准、输入抑制状态；先松键回中并等待禁用 |
| controller=UNKNOWN/WAITING_FOR_FEEDBACK | 点动控制器和/aubo_i5/joint_states是否正常，Gazebo是否运行 |
| armed=0 | 尚未成功请求使能，或断线/故障后授权已撤销 |
| armed=1、OLED LOCKED、按住SW仍deadman=0 | 可能是本文复现的使能竞态；STOP后重试，需修复时序 |
| OLED HOLD SW但不动 | 确认处于点动页、按住SW且有方向；继续查看deadman和控制器故障信息 |
| J1～J6为-- | 关节消息缺失/过期、joint_mask无效，不应视为实时角度 |

## 本次执行的验证

- `tools/test_jog_remote.ps1`：真实MCU C逻辑回归通过，Python串口助手6项测试通过。
- `tools/check_projects.py`：11个工程检查通过；第11项现有编译产物Flash 19640 B、RAM 3448 B。
- 协议两个头文件与两个C文件内容比对通过（C文件仅去除include前缀）。
- 补充真实MCU使能反馈乱序复现通过，确认存在授权丢失问题。
- 未连接STM32/USB-TTL，未运行Linux ROS/Gazebo或实际机械臂；没有把历史构建记录当成本次端到端验证。
