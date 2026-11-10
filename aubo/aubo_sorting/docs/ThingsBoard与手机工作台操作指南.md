# ThingsBoard 与手机工作台操作指南

## 1. 两种访问方式

```text
本机分拣任务 /sorting 状态、目标队列、成功完成统计
  ├─ aubo_cloud_bridge → MQTT/TLS → ThingsBoard 仪表盘 → 电脑 / 手机浏览器
  └─ 本机网页服务 → cpolar HTTPS → 电脑 / 手机浏览器（可用 4G/5G）

相机画面 → 本地 rqt / RViz；远程通道只上传状态、坐标、统计，不上传相机图片。
```

两个访问方式可以同时使用。手机无需安装 App 或申请小程序 AppID。
ThingsBoard 使用平台账号登录；cpolar 网页使用本机工作台密码，两者不是同一个账号。
本机网页和隧道脚本目前复用 `aubo_huawei_bridge` 中的实现，读取本地 ROS 数据，
统一入口不启动华为云 MQTT 桥，也不需要华为云设备凭据。

## 2. 首次准备

已经配置并验证过的本机无需重复创建账号或更换凭据。

| 项目 | 配置位置 / 操作 |
| --- | --- |
| ThingsBoard 设备 Token | `~/.config/aubo/thingsboard.token`，内容为设备 Access Token |
| ThingsBoard MQTT 主机 | 默认 `mqtt.thingsboard.cloud`；其他区域按设备 Check connectivity 页面填写 |
| 本机网页密码 | `~/.config/aubo/huawei/dashboard.json` 中的 `password`；首次启动缺失时自动生成 |
| cpolar 账号配置 | 优先已有 `~/.config/aubo/huawei/cpolar.yml`，否则使用 `~/.cpolar/cpolar.yml` |
| 配置文件权限 | 网页密码文件与 cpolar 配置必须为 `600`；Token 文件也按 `600` 保存 |

凭据保留在本机私有文件中，不写入 launch、Git、截图或聊天。
cpolar 未配置时，在自己的终端运行 `rosrun aubo_huawei_bridge huawei_tunnel --configure`，按提示隐藏输入。
安装与 Token 配置详见 [ThingsBoard 包说明](../../aubo_cloud_bridge/README.md)；
本机网页配置见 [本地工作台说明](../../aubo_huawei_bridge/dashboard/LOCAL_ROS.md)。

新增 launch / 文档直接从源码工作空间使用，无需重新编译；新电脑先按包说明安装并编译依赖。
安装部署时，`aubo_sorting` 已声明两个桥接包的运行依赖。

## 3. 日常推荐：统一入口选择仿真或实机

先正常结束上一轮的仿真、ThingsBoard 桥、本机网页和 cpolar 启动终端，再执行统一入口。
同名 ROS 节点再次启动会替换旧节点，重复的 Gazebo、8099 端口或隧道也会发生冲突。
如果旧华为云 launch 还承载 ROS master，应在整套系统正常停机后退出它，再开始新一轮。

```bash
source /opt/ros/noetic/setup.bash
source ~/aubo/ros_mobile_manipulation_lab/devel/setup.bash

roslaunch aubo_sorting sorting_workstation.launch mode:=gazebo \
  start_thingsboard:=true start_web:=true start_cpolar:=true \
  enable_commands:=true
```

该命令同时启动颜色分拣 Gazebo、MoveIt、感知、任务节点、ThingsBoard 云桥、本机网页和 cpolar。
默认 `auto_move_to_observation:=false`、`auto_start:=false`，等待用户在工作台发出任务指令。
终端保持运行，`Ctrl+C` 结束这一入口管理的进程；退出不会替代任务中的“停止任务”按钮。
分拣任务节点重启后，本次运行的累计完成数量清零。

**网络可选项失败不会使分拣仿真因 `required` 退出**：云桥、网页和隧道均为非必需节点。
但对应访问通道会不可用，需根据终端错误修正配置后再启动该通道。

### 实机：眼在手外 bottle / can / box 三容器工位

将模式改为 `real`：

```bash
roslaunch aubo_sorting sorting_workstation.launch mode:=real \
  start_thingsboard:=true start_cpolar:=true start_rqt:=true \
  enable_commands:=true
```

该模式直接复用
[`scenarios/eye_to_hand_can_sorting_real.launch`](../launch/scenarios/eye_to_hand_can_sorting_real.launch)，
启动对应实机硬件、MoveIt、相机、夹爪、YOLO 感知、分拣任务和实测工位场景。
保留原入口的标定文件、三类容器位置、工作观察位 `real_sort_ready` 和连续队列配置。
默认仍不自动前往观察位、不自动开始分拣；`enable_commands` 只允许接收任务按钮请求，
不表示启动后立即执行动作。

`start_rqt` 统一决定是否打开本地面板；原实机入口内部的 rqt 在组合中关闭，避免重复启动。
修改硬件 IP、串口或模型等参数时直接追加到同一条命令，例如：

```bash
roslaunch aubo_sorting sorting_workstation.launch mode:=real \
  robot_ip:=192.168.1.2 gripper_port:=/实际串口路径 \
  velocity_scaling:=0.12 acceleration_scaling:=0.08 \
  start_thingsboard:=true start_cpolar:=true enable_commands:=true
```

示例串口路径需要替换为实际设备。速度、标定、权重及容器参数以已验证的实机配置为准。
`real` 模式固定使用该工位的 YOLO 和眼在手外相机，`detector` / `camera_mount` 只用于选择仿真组合。
如果实机整套分拣已启动，使用第 4 节的 `mode:=existing`，不要再启动一套硬件或任务。

### 开关与参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `mode` | 当前预设 `real` | `gazebo` 完整仿真；`real` 实机三容器工位；`existing` 连接已有分拣 |
| `start_sorting` | 通常 `true`，existing 为 `false` | 是否启动所选分拣环境；existing 始终复用已有环境，也兼容原来的 `start_sorting:=false` |
| `start_thingsboard` | 当前预设 `true` | 上传到 ThingsBoard 并订阅其任务 RPC |
| `start_web` | 当前预设 `true` | 启动本机投影网页 |
| `start_cpolar` | 当前预设 `true` | 启动手机公网 HTTPS 隧道；会同时启动本机网页 |
| `start_rqt` | 当前预设 `true` | 统一控制仿真 / 实机的本地相机 rqt 工作台，只启动一份 |
| `enable_commands` | 当前预设 `true` | 开放 ThingsBoard 与本机网页任务按钮；为 false 时只读 |
| `auto_move_to_observation` | `false` | 分拣节点启动后是否自动前往观察位 |
| `auto_start` | `false` | 是否自动开始分拣 |
| `detector` | 仿真 `color`，实机 `yolo` | 仿真可选 `color` / `yolo`；real 模式由原工位固定为 yolo |
| `camera_mount` | 仿真 `eye_in_hand`，实机 `eye_to_hand` | 仿真可选两种安装方式；real 模式固定眼在手外 |
| `thingsboard_host` | `mqtt.thingsboard.cloud` | ThingsBoard MQTT 主机，不带 `https://` |
| `thingsboard_token_file` | `~/.config/aubo/thingsboard.token` | 设备凭据文件 |
| `web_port` | `8099` | 本机网页监听端口；隧道自动跟随该端口 |
| `web_auth_file` | `~/.config/aubo/huawei/dashboard.json` | 本机网页密码文件 |
| `cpolar_region` | `cn` | 隧道区域，支持 `cn` / `cn_top` |
| `sorting_ns` | `/sorting` | 云桥、网页、rqt 连接的 ROS 接口命名空间 |
| `task_node` | `/color_sorting_task` | 云桥、网页、rqt 读取的任务参数节点 |

`start_web:=false start_cpolar:=true` 仍会启动一份网页服务，因为隧道需要它。
`sorting_ns` / `task_node` 用于连接已有接口，不会修改本入口分拣核心的节点名或话题名。
所选底层入口的其他参数继续转发：仿真如 `rviz`、`gui`、`task_config`、`world`、`model_path`；
实机如 `robot_ip`、`start_robot`、`start_camera`、`start_gripper`、`calibration_file`、
`gripper_port`、`containers_config`、`queue_config`、`velocity_scaling` 等。
不用复制实机工位配置；本统一入口的 `start_rqt` 替代底层 `rqt` 开关。
launch 文件中的注释和 `doc` 参数说明为中文，参数名与 true / false 保留 ROS 使用的英文格式。
以上记录的是当前本机设置：实机模式、全部通道和任务控制开启；如再次修改 `default`，以 launch 文件为准。
仿真命令显式写 `mode:=gazebo`；需要关闭某个通道或只读监控时，显式传入对应的 `false`。
实机入口显式设置 `/use_sim_time=false`，防止复用旧 Gazebo master 时被遗留仿真时钟阻塞。
existing 模式保持已有系统的时钟设置。

### 常见组合

```bash
# 仅本地分拣仿真：不启动联网服务
roslaunch aubo_sorting sorting_workstation.launch mode:=gazebo \
  start_thingsboard:=false start_web:=false start_cpolar:=false start_rqt:=false

# 分拣仿真 + 本机网页控制：不用 cpolar 或 ThingsBoard
roslaunch aubo_sorting sorting_workstation.launch mode:=gazebo \
  start_thingsboard:=false start_web:=true start_cpolar:=false enable_commands:=true

# 分拣仿真 + ThingsBoard，只读监控
roslaunch aubo_sorting sorting_workstation.launch mode:=gazebo \
  start_thingsboard:=true start_web:=false start_cpolar:=false enable_commands:=false

# 分拣仿真 + 手机公网网页控制：自动带起本机网页
roslaunch aubo_sorting sorting_workstation.launch mode:=gazebo \
  start_thingsboard:=false start_cpolar:=true enable_commands:=true

# 全部远程功能 + 本地 rqt
roslaunch aubo_sorting sorting_workstation.launch mode:=gazebo \
  start_thingsboard:=true start_cpolar:=true start_rqt:=true enable_commands:=true

# YOLO + 眼在手外；按实际权重路径覆盖 model_path
roslaunch aubo_sorting sorting_workstation.launch mode:=gazebo \
  detector:=yolo camera_mount:=eye_to_hand model_path:=/实际路径/模型.pt \
  start_thingsboard:=true start_cpolar:=true enable_commands:=true
```

## 4. 已有仿真或实机分拣在运行时

保留现有分拣系统，仅启动远程服务：

```bash
roslaunch aubo_sorting sorting_workstation.launch mode:=existing \
  start_thingsboard:=true start_cpolar:=true enable_commands:=true
```

该模式不启动 Gazebo、MoveIt、感知或任务节点，也不清零正在运行的任务统计。
也可先单独使用已经验证的实机启动入口，再以 `mode:=existing` 连接。
该模式即使误传 `start_sorting:=true`，仍不重复启动分拣环境；原来的 `start_sorting:=false` 用法继续支持。
若当前云桥 / 网页 / cpolar 已运行，先在其原终端 `Ctrl+C` 结束相应进程，避免重复启动。
不要为切换入口而直接结束正在执行抓放的任务；先用工作台停止并确认任务退出动作。

## 5. 电脑和手机怎样访问

### 本机网页

在机器人电脑浏览器打开 `http://127.0.0.1:8099`，使用 `web_auth_file` 中的 `password` 登录。
修改 `web_port` 后，将地址中的端口相应更换。服务只监听本机回环地址，外网通过 cpolar 访问。

### cpolar 手机网页

终端中找到 `Tunnel established at https://…`，用手机浏览器打开其中 **HTTPS** 地址。
手机可以关闭 Wi-Fi、使用 4G/5G，登录密码与本机网页相同。
随机隧道重启后地址可能变化，以当次启动输出为准，不将旧地址写死在文档中。
隧道检查工作台端口后才启动；如果 15 秒内网页尚未就绪，会退出并提示检查端口。

### ThingsBoard 仪表盘

电脑或手机登录原 ThingsBoard 区域和账号，打开绑定设备的 AUBO 仪表盘。
它通过云平台连接，与 cpolar 地址无关；关闭 cpolar 不会关闭 ThingsBoard 上报。

首次导入或升级：

1. **Resources → Widgets → Widgets bundles → ＋ → Import widgets bundle**，
   选择 [aubo_widgets_bundle.json](../../aubo_cloud_bridge/dashboards/aubo_widgets_bundle.json)。
   必须选择组件包入口，单组件 `Import widget` 会报数据结构错误。
2. 同名组件冲突时，编辑已有 `AUBO Sorting Console`，分别用
   [console.html](../../aubo_cloud_bridge/dashboards/console.html)、
   [console.css](../../aubo_cloud_bridge/dashboards/console.css)、
   [console.js](../../aubo_cloud_bridge/dashboards/console.js) 的全文替换后保存。
3. **Dashboards → ＋ → Import dashboard**，选择
   [aubo_sorting_dashboard.json](../../aubo_cloud_bridge/dashboards/aubo_sorting_dashboard.json)。
4. 打开仪表盘并进入编辑模式，顶部 **Aliases** 中将 **AUBO 分拣设备** 设置为
   **Single entity → Device → 已配置 Token 对应的设备**；保存别名，再保存仪表盘。
5. 如果继续复用旧仪表盘，在组件数据源补上 `completed_counts`、`completed_total`、
   `statistics_available`、`workcell` 四个遥测键。新生成的仪表盘已包含这些键。

## 6. 验证操作与统计含义

1. 等待分拣系统状态为 `IDLE`；ThingsBoard 终端应出现 `ThingsBoard connected`。
2. 用网页点击 **前往观察位**；等待进入 `READY`，查看投影和目标队列。
3. 点击 **开始分拣**；状态、队列与任务步骤持续更新。
4. 每次成功完成抓取、放置和退离，对应类别的累计完成数增加。
5. 手机切换至 4G/5G，打开当前 HTTPS 地址，确认数据仍更新。

“分拣完成”是**本次任务节点运行累计**，例如 `bottle:1 / can:2 / box:3`；
颜色场景对应 `red / green / blue` 等实际类别。清理队列不清零，重启任务节点清零。
`queue_done` 只是当前队列中的完成数量，不能当作累计产量；历史累计快照也不能相加。
没有统计消息时显示等待数据；不会填入演示数字。投影仅显示有效的新鲜坐标，过期时不保留当前目标点。
“停止任务”是软件停止，现场硬件急停独立使用。

## 7. 快速排查

| 现象 | 检查方向 |
| --- | --- |
| 没有 cpolar 地址 | 确认 `start_cpolar:=true`；查看账号配置权限、cpolar 安装及本机网页是否正常启动 |
| 本机网页打不开 | 检查 `web_port`、8099 是否被旧工作台占用，以及密码文件是否为 600 |
| 旧手机地址打不开 | 隧道可能已经重启，改用本次输出的新 HTTPS 地址 |
| ThingsBoard 未连接 | 检查 Token 文件、MQTT 主机、TCP 8883、系统时间与 CA |
| 页面接口未就绪 | 检查分拣节点是否运行，状态发布者和四个任务服务是否已注册 |
| 投影没有目标 | 先前往观察位，检查目标坐标和工位参数；无目标或过期时不绘点 |
| 完成统计显示等待 | 检查 `/sorting/completed_counts` 发布者及仪表盘四个新增遥测键 |
| 切换统一入口后节点退出 | 检查是否还运行着上一份同名桥接、网页或仿真节点 |
| 网页报告命令日志被占用 | 新节点最多等待旧节点退出 3 秒；仍被占用时先结束旧网页，不删除去重数据库 |
| 实机场景等待 `/apply_planning_scene` 超时 | 检查 MoveIt 是否启动完成；`rosparam get /use_sim_time` 应为 false，切换实机前先结束旧 Gazebo |

只读检查：

```bash
rosnode list
rostopic info /sorting/state
rostopic info /sorting/completed_counts
rostopic echo -n 1 /sorting/completed_counts
```

统一入口只组合已有功能，不自动购买云服务；ThingsBoard 账号权益与 cpolar 可用额度沿用现有账号。
