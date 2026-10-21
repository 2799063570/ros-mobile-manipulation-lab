# STM32 笛卡尔遥控协议与电脑端控制

本入口通过串口控制 `aubo_ros_control` 的笛卡尔点动与可取消 home action。
PC 节点 `cartesian_serial_bridge` 使用 C++14 / termios，STM32 只负责采集输入、菜单确认和显示。
输入采用 XYZ 方向向量：摇杆 XY 加 Z± 按键，或六个 XYZ 方向键，都使用同一格式。
不在 STM32 中计算逆解，不在串口节点中创建第二个 SDK 连接。

## 启动

首次需重新编译；本工作空间已完成对应节点编译。

```bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
catkin_make
source devel/setup.bash

# 仿真 + 串口遥控；默认不启动电脑点动窗口
roslaunch aubo_remote_bridge cartesian_remote_gazebo.launch port:=/dev/ttyUSB0

# 已经启动cartesian_jog时，仅附加串口节点
roslaunch aubo_remote_bridge cartesian_serial_bridge.launch port:=/dev/ttyUSB0 joint_states_topic:=/aubo_i5/joint_states

# 实机 + 串口遥控
roslaunch aubo_remote_bridge cartesian_remote_real.launch port:=/dev/ttyUSB0 robot_ip:=192.168.1.2
```

实机入口与 `cartesian_jog_real.launch` 一样，启动 SDK 时可能上电、松刹车；初始不执行运动。
使用实际的相机模型参数 `robot_xacro` / `robot_srdf`；无腕部相机时传入
`robot_xacro:=aubo_i5.xacro robot_srdf:=aubo_i5.srdf`。
已有点动进程时使用“仅附加”入口，不再运行第二套完整入口。
同一控制器只使用一个操作端：本串口入口关闭 Qt panel，不要同时向 `/cartesian_jog/command` 发布 GUI/其他指令。
原 `serial_bridge.launch` / `named_pose_gazebo.launch` 的 HOME/ZERO 协议入口继续独立使用，不能占用同一个串口。

串口参数固定 **115200、8 数据位、无校验、1 停止位、无流控、3.3 V TTL**。
STM32 TX 接转接器 RX，RX 接 TX，GND 共地。`port` 可使用稳定的 `/dev/serial/by-id/...`。

`max_speed_mm_s` 为遥控端速度上限，默认 100，允许 `(0,100]`；点动节点还会执行自身限速。
速度范围、home 目标及 home 关节速度由 `aubo_ros_control/config/cartesian_jog.yaml` 定义，
线速度与 home 的关节速度分别设置。home 默认关节速度上限 0.10 rad/s。

## 帧格式

复用 `RemoteProtocol.h/.c` 的 v2 帧头与 CRC，新增独立的 `0x11/0x12/0x13` 类型。
旧固件的 `0x01/0x02/0x03` 不能作为本入口的点动报文；MCU 必须实现下列扩展。
所有多字节整数均为 **小端**，负数采用二进制补码；不要直接发送 C 结构体内存。

| 字节偏移 | 长度 | 内容 |
|---:|---:|---|
| 0 | 2 | 帧头 `A5 5A` |
| 2 | 1 | 版本 `02` |
| 3 | 1 | 类型：POLL=`11`，INPUT=`12`，FEEDBACK=`13`（十六进制） |
| 4 | 2 | sequence，INPUT 必须原样回显本次 POLL 编号 |
| 6 | 1 | payload 长度，最多 32 字节 |
| 7 | N | payload |
| 7+N | 2 | CRC-16/CCITT-FALSE，低字节在前 |

CRC 覆盖 `[版本 ... payload末字节]`，不含帧头和 CRC 本身；
多项式 `0x1021`，初值 `0xFFFF`，不反射，结果不异或。ASCII `123456789` 校验值为 `0x29B1`。
解析器支持分包、粘包、噪声和 CRC 错误后重新寻找帧头。

抓包对照示例（session=`0x12345678`，sequence=1）：

```text
POLL:
A5 5A 02 11 01 00 04 78 56 34 12 EF 45
INPUT，request_id=1，ENABLE，有效/已校准/松开允许运动键，基坐标，10mm/s，零方向:
A5 5A 02 12 01 00 12 78 56 34 12 01 00 01 03 00 00 64 00 00 00 00 00 00 00 3F 4E
```

实际session来自PC，不可把此固定示例当实时命令发送；新会话首帧中的ENABLE也只会建立基线。

PC 每约 20 ms 发一个 POLL，同一时间只等待一个 INPUT，最长等待 100 ms。
STM32 收到有效 POLL 后应立即读取当前输入并回复一个 INPUT，不定时主动重发旧帧。
PC 只接收当前 session、当前 sequence、100 ms 内的有效响应；重复帧不会刷新看门狗。
丢失响应超时后换新 session；sequence 到达 65535 后同样换新 session，不能把旧响应用于新会话。
输入从有效退化为未校准/无效时，PC也会在下一次POLL换新session，以覆盖MCU快速复位但仍持续回复的情况。
校准期间不会每帧反复换session；恢复后首帧仅重建请求基线，旧请求不自动执行。
PC 收到有效 INPUT 后发送相同 sequence 的 FEEDBACK。

### POLL：类型 0x11，payload 4 字节

| payload偏移 | 类型 | 内容 |
|---:|---|---|
| 0 | uint32 | session，PC生成，STM32原样回显 |

STM32 不自行生成 session。每次 session 改变时保留当前请求编号作为旧请求基线，
不要自动增加编号或重新确认旧操作；等待用户松开方向和允许运动键，再进行新的显式操作。

### INPUT：类型 0x12，payload 18 字节

| payload偏移 | 类型 | 内容 |
|---:|---|---|
| 0 | uint32 | session，回显 POLL |
| 4 | uint16 | request_id，离散命令编号，0 表示无命令 |
| 6 | uint8 | command，见下表；0 表示无命令 |
| 7 | uint8 | flags：bit0 ADC/按键采样有效，bit1 校准完成，bit2 允许运动键正被按住 |
| 8 | uint8 | frame：0 基坐标系，1 工具坐标系 |
| 9 | uint8 | 保留，必须为 0 |
| 10 | uint16 | speed，单位 0.1 mm/s；1..1000，即0.1..100 mm/s；零向量时允许0 |
| 12 | int16 | X方向分量，-1000..1000 |
| 14 | int16 | Y方向分量，-1000..1000 |
| 16 | int16 | Z方向分量，-1000..1000 |

flags 其他位必须为0；frame、speed、方向或命令字段越界的帧会被丢弃，不刷新看门狗。
flags bit0/bit1 任意为0时，PC立即发零指令并请求停止、禁用、取消home。
纯按键遥控器也须设置 bit0/bit1 表示输入系统有效且已完成开机检查。

方向向量按 `v = speed_mm_s / 1000 × axis / max(1000, norm(axis))` 转为 m/s。
例如 speed=100，X=1000 表示 X+ 10 mm/s；speed=1000，X=Y=1000 表示
XY对角线合速度100 mm/s，每轴约70.71 mm/s。输入分量小于1000可用于摇杆比例调速。
STM32应自行做摇杆死区和零点校准，零位发送精确0。
只有 flags bit2=1 且已经显式使能，才会发送非零速度；释放允许运动键立即发零速度并平滑停下。
保持链路有效时，释放该键不会自动撤销使能。切换坐标系、操作菜单前应回中并释放该键。

| command | 建议显示 | 功能 |
|---:|---|---|
| 0 | NONE | 无新的离散请求，request_id必须为0；仍上报实时方向与键状态 |
| 1 | ENABLE | 请求使能点动，需要回中并松开允许运动键 |
| 2 | STOP | 发零、请求禁用并取消回home；确认停稳后反馈DONE |
| 3 | HOME | 请求home action，必须已禁用、停稳、回中、松开允许运动键 |
| 4 | CANCEL | 请求减速停止、取消home并禁用；确认停稳后反馈CANCELED |
| 5 | RESET | 复位故障；要求未使能、回中、松开允许运动键，不自动使能 |

每次用户确认离散操作时递增 request_id：1..65535，65535后跳到1。
command非0与request_id非0必须成对出现。请求发出后可以持续保持同一个 command/request_id，
后续轮询只更新方向、速度等实时字段；相同请求不会再次调用服务或发送home目标。
也可以随后发送0/0作为纯心跳；最新应答仍保留。
同编号更换command会被拒绝并停止，编号倒退会拒绝；前进距离不得超过32767。
每次新session的第一份INPUT只建立编号基线，不执行其中的命令。

ENABLE/HOME/RESET需要新的请求编号，并等待上一操作完成；BUSY或失败的请求不会自动重试。
应在用户再次确认后分配新编号。未发生变化的请求编号重发只回放原结果。
ENABLE成功前必须保持回中和松开允许运动键，否则该使能结果作废并请求禁用。
ENABLE服务成功后PC先保持ACCEPTED和flags bit3=1，继续发送零速度；
等到服务返回后新收到的控制器ENABLED_IDLE状态，才同时反馈DONE和flags bit1=1，清除bit3。
确认最多等待1秒；输入改变、断链、控制器状态失效/故障、STOP/CANCEL或确认超时都会撤销使能并请求禁用。

### FEEDBACK：类型 0x13，payload 28 字节

| payload偏移 | 类型 | 内容 |
|---:|---|---|
| 0 | uint32 | session |
| 4 | uint16 | 最新请求request_id；0表示尚无请求 |
| 6 | uint8 | 最新请求command |
| 7 | uint8 | 最新请求ack_state，见下表 |
| 8 | uint8 | controller_state，见下表；与最新请求应答分别显示 |
| 9 | uint8 | 本节点home action状态，使用ROS GoalStatus编号；255表示尚无home目标 |
| 10 | uint8 | flags：bit0输入有效且新鲜，bit1本端已使能，bit2控制器状态新鲜，bit3服务/禁用请求/停止确认处理中 |
| 11 | uint8 | joint_mask，低6位分别表示J1..J6有效；清零位的角度不能当实时反馈 |
| 12 | int16[6] | J1..J6当前关节位置，单位mrad，按肩/上臂/前臂/腕1/腕2/腕3排序 |
| 24 | uint16 | 最近home反馈的最大关节误差，mrad；65535=尚无反馈，最多65534 |
| 26 | uint16 | fault，见下表；详细文本见ROS话题 |

关节反馈超过500 ms、消息时间戳无效或关节缺失时清除对应有效位。
控制器状态超过300 ms时反馈UNKNOWN且撤销本端使能。home误差是最近一次action反馈，
不是持续计算的当前位置到home距离；新home请求时重置为65535。

| ack_state | 名称 | 含义 |
|---:|---|---|
| 0 | IDLE | 尚无请求 |
| 1 | ACCEPT | 收到/已发起操作，尚未确认完成 |
| 2 | PLAN | 本入口保留，不使用 |
| 3 | RUN | home目标已激活 |
| 4 | DONE | 使能/复位成功、STOP确认完成，或HOME成功到位；按command区分 |
| 5 | FAILED | 服务/动作失败，或使能输入条件已变化 |
| 6 | STOPPING | 等待减速停止，可包括home正常到位前的减速 |
| 7 | CANCELED | CANCEL已确认停稳，或home已取消 |
| 8 | REJECT | 未就绪、BUSY、旧请求、编号冲突等 |
| 9 | VERIFY | 保留 |
| 10 | UNKNOWN | 保留 |

| controller_state | 名称 |
|---:|---|
| 0 | UNKNOWN，控制器离线或状态过期 |
| 1 | WAITING_FOR_FEEDBACK |
| 2 | DISABLED |
| 3 | ENABLED_IDLE |
| 4 | JOGGING |
| 5 | STOPPING |
| 6 | HOME_MOVING |
| 7 | HOME_STOPPING |
| 8 | FAULT |

home_state：0=PENDING，1=ACTIVE，2=PREEMPTED，3=SUCCEEDED，4=ABORTED，
5=REJECTED，6=PREEMPTING，8=RECALLED，9=LOST，255=无目标。
取消与完成同时发生时可能成功到位，必须以最终action结果与控制器状态为准。

fault：0=无故障；1=未知/控制器状态不可用；2=feedback_timeout；3=command_timeout；
4=invalid_or_stale_command；5=singularity_stop；6=collision_or_stopping_limit；
7=following_error；8=stream_schedule_gap；9=control_computation_overrun；10=home_timeout；
11=hard_joint_limit；12=sdk_feedback_failed；13=invalid_feedback；14=kinematics_failed；
15=invalid_joint_velocity。新增未映射故障使用1，ROS状态仍保留原始文本。

## STM32 操作流程

1. 开机完成采样检查、摇杆校准，等待POLL；未完成时flags bit0/bit1不得假报有效。
2. 收到POLL后回显session/sequence，发送当前真实输入。首帧缓存命令不会执行。
3. 用户回中、松开允许运动键。可先比较有效的六轴位置与home
   `[0, 0, 1629, 0, 1663, 0]` mrad，菜单提供“回home”和“保持当前位置”。
   home目标若在ROS配置中修改，MCU显示用参考值也要同步；运动目标始终以ROS配置为准。
4. 选择HOME时发送新编号command=3，保持持续响应POLL。到位/取消后仍禁用。
   选择保持当前位置则不发HOME，直接等待用户显式ENABLE。
5. 等待控制器DISABLED且flags bit3=0，再发新编号ENABLE；等待对应DONE和flags bit1=1，
   然后才允许用户按住允许运动键点动。
6. 回home途中长按/独立取消键发送新编号CANCEL。方向、速度字段归零，允许运动键释放。
   持续轮询，显示STOPPING；最终CANCELED且控制器DISABLED才显示“已停止”。
7. 断链、输入无效或PC状态超时后，必须重新回中并显式ENABLE；恢复连接不会自动恢复旧运动。

串口节点每约20 ms向 `/cartesian_jog/command` 发布带当前时间戳的速度，
调用 `/cartesian_jog/set_enabled`、`/cartesian_jog/reset`，使用 `/cartesian_jog/move_home` action。
服务调用位于独立异步任务，串口接收、取消和链路检查继续运行；迟到的使能服务结果不能重新授权运动。

HOME采用底层点动节点已有的关节空间运动及碰撞/制动余量检查，不做全局绕障规划。
串口输入停止超过100 ms，会发零、取消home并请求禁用。软件停止保留已有平滑减速与控制柜缓冲延迟。
若反馈丢失，底层无法确认停稳时不会伪造成功/取消完成；控制器退出、严重故障可能需要人工处理。
MCU在PC反馈超时后也应立即清除显示中的“已使能”，停止保持旧运动请求，等待重新操作。
这些软件操作不代替硬件急停。

## 可复用 C 代码

STM32项目加入以下四个文件，保留 `aubo_remote_bridge/` 头文件搜索路径：

- `include/aubo_remote_bridge/RemoteProtocol.h`
- `src/RemoteProtocol.c`
- `include/aubo_remote_bridge/RemoteJogProtocol.h`
- `src/RemoteJogProtocol.c`

均不依赖ROS、C++或动态内存。安装后也位于本包share目录的 `mcu/` 子目录。
UART接收中断/DMA只负责缓存数据，在主循环用 `Remote_ParseByte` 取出帧，再执行响应。
`RemoteJog_EncodeInput` 返回0表示本地字段无效，不能继续发送该帧。

```c
Remote_FrameTypeDef rx, tx;
RemoteJog_Input input = {0};
uint8_t wire[REMOTE_MAX_FRAME];
/* parser用Remote_ParserInit初始化；每收到一个字节调用Remote_ParseByte。 */
if (Remote_ParseByte(&parser, byte, &rx)) {
    if (rx.Type == REMOTE_JOG_POLL && rx.Length == 4) {
        input.session = Remote_GetU32(rx.Payload);
        /* 更新flags、frame、speed、axis和用户显式确认的command/request_id。 */
        if (RemoteJog_EncodeInput(&input, rx.Sequence, &tx)) {
            uint8_t size = Remote_Encode(&tx, wire);
            /* 将wire[0..size-1]加入UART发送队列；避免阻塞接收。 */
        }
    } else if (rx.Type == REMOTE_JOG_FEEDBACK) {
        RemoteJog_Feedback status;
        if (RemoteJog_DecodeFeedback(&rx, &status)) {
            /* 核对session/sequence，只更新当前会话的OLED状态。 */
        }
    }
}
```

## 调试与自动验证

```bash
rostopic echo /remote/cartesian_status
rostopic echo /cartesian_jog/state
rosrun aubo_remote_bridge cartesian_remote_core_test
rostest aubo_remote_bridge cartesian_serial.test
```

协议测试覆盖CRC标准向量、错误CRC后恢复、字段范围、反馈编解码、重复响应、超时与旧session拒绝。
PTY端到端测试使用真实串口节点、真实点动/action节点与模拟关节反馈，不需要USB串口或STM32。
覆盖首帧缓存HOME忽略、显式使能、XYZ合速度限制、释放允许运动键、断链停用、重连不自启动、
STOP、HOME、CANCEL、再次回home成功、请求重发去重、编号冲突拒绝和关节有效位。
另覆盖home途中断链取消及ADC/校准失效后不自动恢复使能。
本次验证不连接实机；STM32实体UART及真实机械臂运动需在后续对接时验证。
