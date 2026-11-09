# AUBO → ThingsBoard Cloud：电脑 / 手机网页工作台

新增独立 ROS1 Python3 节点，不改变分拣核心，不启动机械臂或 SDK。
本机通过 MQTT/TLS 主动连云；手机通过 ThingsBoard 的 HTTPS 页面查看和操作。
首版覆盖状态、步骤、识别汇总、故障、实例列表，以及观察 / 开始 / 停止 / 回结束位。
不需要开发或安装手机 App。

## 1. 注册云端，创建设备

1. 在 [ThingsBoard Cloud](https://thingsboard.cloud/) 注册并登录。
   选择离部署地点合适的区域，确认当前试用、套餐和消息配额；本项目不包含云服务付费开通。
2. 在 **Entities → Devices** 创建设备，名称例如 `aubo-sorter-01`；
   设备 Profile / Type 使用 `aubo_sorting`。若界面使用默认类型，后面手动绑定设备即可。
3. 在设备详情的 **Manage credentials** 使用 **Access Token**，复制设备 Token。
   这是机器人凭据，不是网站登录密码。只保存在机器人电脑，勿提交 Git 或粘贴到聊天。
4. 打开 **Check connectivity**，查看 MQTT/TLS 的主机和端口。
   默认北美端点为 `mqtt.thingsboard.cloud:8883`，其他区域按页面提供的端点覆盖 `host`。
   网址 `https://thingsboard.cloud` 是登录地址，不能当作 MQTT 主机参数。

参考：[Cloud MQTT / TLS / 凭据](https://thingsboard.io/docs/paas/reference/mqtt-api/getting-connected/)。

## 2. 机器人 Ubuntu / ROS Noetic 端安装

以下命令在运行分拣系统的 **Linux ROS 电脑** 执行，工作空间路径按实际替换。
只要求 Python 3.8+；系统 apt 的 Paho 1.x 和 Paho 2.x 均有适配分支。

```bash
sudo apt update
sudo apt install python3-paho-mqtt ca-certificates
source /opt/ros/noetic/setup.bash
cd ~/aubo/ros_mobile_manipulation_lab
chmod +x src/aubo/aubo_cloud_bridge/scripts/thingsboard_bridge
catkin_make --pkg aubo_cloud_bridge -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

如果工作空间用了 `CATKIN_WHITELIST_PACKAGES`，将 `aubo_cloud_bridge` 加入现有白名单。
编译会生成 `devel/lib/aubo_cloud_bridge/thingsboard_bridge` 和 Python 模块入口。
本机默认 `python3` 指向 Conda；ROS Noetic 使用系统 `/usr/bin/python3`。
检查 apt 安装的依赖时也使用 `/usr/bin/python3`，避免误判 Paho 缺失。

将 Token 写入本机文件，下面使用 Bash 隐藏输入，不把 Token 写入命令历史：

```bash
install -d -m 700 ~/.config/aubo
read -rsp 'ThingsBoard device token: ' aubo_tb_token
printf '\n'
(umask 077; printf '%s' "$aubo_tb_token" > ~/.config/aubo/thingsboard.token)
unset aubo_tb_token
```

先按项目原来的入口启动 Gazebo 或分拣系统，再在新终端启动云桥。
初次连接使用只读模式，检查上传字段和设备选择正确：

```bash
source ~/aubo/ros_mobile_manipulation_lab/devel/setup.bash
roslaunch aubo_cloud_bridge thingsboard.launch
# 其他区域示例：用 Check connectivity 显示的实际 MQTT 主机替换
# roslaunch aubo_cloud_bridge thingsboard.launch host:=YOUR_MQTT_HOST
```

出现 `ThingsBoard connected` 后，打开设备的 **Latest telemetry**，应每两秒收到
`bridge_wall_ms`、`bridge_session`、`state` 等值。`commands_enabled=false` 为正常只读模式。

若设备页面要求指定 CA 文件，将其保存到本机，在 `config/thingsboard.yaml` 设置 `ca_file`。
默认使用系统 CA，始终校验服务端证书；不提供关闭 TLS 校验的模式。

## 3. 导入可视化工作台

仓库已经生成两个 JSON，**先导入组件包，再导入仪表盘**：

1. **Resources → Widgets → Widgets bundles 标签 → + Add widgets bundle → Import widgets bundle**：
   选择 `dashboards/aubo_widgets_bundle.json`，会创建 `AUBO Sorting` 组件包。
   部分版本入口名为 **Widgets library**。务必选择组件包导入，不能在单个 **Widgets**
   列表中使用 **Import widget**；将组件包 JSON 交给单组件入口会报
   `Unable to import widget: Invalid widget data structure`。
2. **Dashboards → Import dashboard**：选择 `dashboards/aubo_sorting_dashboard.json`。
3. 编辑仪表盘的 **Entity aliases**，将 `AUBO 分拣设备` 设置为 **Single entity → Device**，
   选择刚创建的 `aubo-sorter-01`，保存。
   模板也支持自动匹配类型 `aubo_sorting`；存在多台设备时必须显式绑定一台。
4. 保存并打开仪表盘。应看到状态、步骤、队列计数、目标表格及故障信息。
5. 用手机浏览器打开同一个 Cloud 区域的网址，登录有该设备和仪表盘访问权限的账号，
   打开工作台即可。页面自动适配窄屏，可使用手机浏览器“添加到主屏幕”。

控制页面使用当前登录用户的 ThingsBoard `deviceService`，不向浏览器提供设备 Token。
不要将控制仪表盘设为公开访问。若给其他人员使用，分配相应设备、仪表盘和 RPC 权限。

JSON 按官方 ThingsBoard 导入结构生成，组件源码是 `console.html/css/js`。
若平台导入时为组件分配了不同 FQN，请在仪表盘中重新从 `AUBO Sorting` 选择
`AUBO Sorting Console` 并配置同一设备数据源；不要修改设备凭据。
界面标签随 Cloud 版本可能略有差异。

若旧版组件首次打开时报 `Cannot read properties of undefined (reading 'fresh')`，
在 **Resources → Widgets** 找到 `AUBO Sorting Console`，打开组件编辑器的 HTML，
将第一行改为 `<main class="aubo-console" *ngIf="vm">`，保存后刷新仪表盘。
这个保护让模板等待 `onInit` 初始化视图数据后再显示内容。
仓库的 `console.html` 和生成的组件包已包含修复；修改本地文件不会自动更新云端已导入的组件。

## 4. 开启任务指令

建议先在 Gazebo 上验证。结束只读桥节点，再运行：

```bash
roslaunch aubo_cloud_bridge thingsboard.launch enable_commands:=true
```

网页启用“前往观察位 / 开始分拣 / 返回结束位 / 停止任务”。
先前往观察位，等待分拣核心确认目标并进入 READY，再开始分拣。
默认只读开关是初次接入检查选项；启用后不需要为每条指令重新启动节点。

| RPC 方法 | 本地 std_srvs/Trigger 服务 | 语义 |
|---|---|---|
| `move_to_observation` | `/sorting/move_to_observation` | 观察并确认目标 |
| `start` | `/sorting/start` | 按本地状态机条件开始分拣 |
| `stop` | `/sorting/stop` | 请求软件停止；不等于硬件急停 |
| `home` | `/sorting/home` | 返回配置的结束姿态 |

不提供任意 ROS 服务调用、关节目标、夹爪释放或故障复位 RPC。
网页会对运动请求显示确认；停止可在另一个请求等待期间独立发送。
本地已有工作台仍可操作，最终忙闲和动作条件由原 ROS 核心裁决；首版不实现多端控制权租约。

### 指令与反馈协议

采用 ThingsBoard 标准设备 MQTT API：

- 上传：`v1/devices/me/telemetry`；最新快照，QoS 0，不积压离线遥测。
- 下行：`v1/devices/me/rpc/request/+`；订阅 QoS 1，拒绝 retained 消息。
- 应答：`v1/devices/me/rpc/response/<request_id>`；QoS 0，不积压离线应答。

RPC 请求示例（网页自动生成字段，无需手动填时间）：

```json
{
  "method": "start",
  "params": {
    "command_id": "e2811a75-8422-4dbb-8c98-e20a67d08779",
    "session": "当前遥测中的 bridge_session",
    "issued_at_ms": 1794196800000,
    "expires_at_ms": 1794196810000
  }
}
```

`command_id` 为 16–80 字符字母、数字、下划线或连字符；同编号相同请求只回放结果。
相同编号不同内容拒绝。使用 SQLite 在调用 ROS **之前**提交编号；重启后仍去重。
若进程在调用前后崩溃，该编号保持 `unknown`，不会为了自动恢复而重执行。
数据库不自动清理，按使用量监控磁盘；关闭桥后才能做维护，不要删库后重放历史指令。
同一设备仅运行一个桥进程，使用稳定的本地 `journal_path`。

有效期最长 30 秒，网页默认 10 秒；未来签发时间容差 5 秒。
手机、云端和机器人需校准系统时间。每次 MQTT 重连更新 `bridge_session`，
断线期间或旧会话的指令不能在重连后启动动作。网页发非持久 RPC，不配置离线指令重放。

`accepted` 只表示本地 Trigger 接收请求，动作结果继续查看 `state/progress/failure`。
服务连接异常显示 `unknown`，必须核对本地状态，不要换编号盲目重发运动。
一个运动服务等待线程和一个独立停止线程：ROS 服务若一直不返回，不继续创建运动线程，
MQTT 和遥测仍可运行。停止会撤销尚未调用 ROS 的运动请求；已经发送的请求由本地核心处理。
软件不能保证互联网停止的时延或送达，现场硬件急停继续独立工作。

网络中断不会自动改变已在本地执行的分拣任务；没有云端时本地系统仍按原逻辑运行。
机械臂运动控制、碰撞检查和感知始终留在本机。

## 5. 遥测字段与边界

| 字段 | 内容 |
|---|---|
| `bridge_wall_ms` / `bridge_session` | 本次云连接的心跳 / 会话编号 |
| `sorting_available` | ROS master 上状态发布者及四个服务均已注册；不是硬件就绪认证 |
| `commands_enabled` | 本机是否启用远程任务 |
| `state` / `state_detail` | 分拣状态及说明 |
| `phase` / `description` | 当前子步骤及说明 |
| `failure` / `detection_summary` | 故障与识别汇总 |
| `target_cache` | schema 2 目标队列的精简 JSON 字符串，最多 200 条 |
| `queue_*` | 当前队列的 candidate / ready / reserved / done / review 数量 |
| `*_age_s` | 桥接收到该 ROS 消息后过去的单调时间，-1 表示从未收到 |
| `last_command` | 最近完成服务应答的指令编号、方法、反馈和时间 |

状态话题是锁存消息：桥接启动时收到的值可能早于启动时间。
`*_age_s` 表示接收年龄，不保证原数据采集时间；`sorting_available` 检查注册信息，
不能发现所有节点假死。网页超过 10 秒无有效心跳会显示离线并禁用按钮，但保留最后快照。
故障字段是原核心最后一次报告，不代表每次新任务都会自动清空。

`queue_done` 是当前队列中的抓放流程完成数，会随观察、重建队列、过期清理而变化，
不是当天累计产量，也不能证明实物已入筐。首版云端存储这些遥测，历史记录可在
ThingsBoard 中加时序图查看；可靠累计产量 / 成功率需要另建唯一抓放事件及实物确认。
不上传相机视频、点云或图像；后续视频需要单独设计链路。

默认每 2 秒上传一个快照（每天约 43,200 次上传），含 20 多个遥测键；
评估 Cloud 套餐的消息数、数据点、存储和流量配额。`publish_interval` 可调 0.5–60 秒，
网页的 10 秒在线阈值适用于默认 2 秒周期；调大周期时同步调整 `console.js` 的阈值。

## 6. 验证与排查

无 ROS、无账号也可验证协议和网页控制器：

```bash
cd ~/aubo/ros_mobile_manipulation_lab
/usr/bin/python3 -m unittest discover -s src/aubo/aubo_cloud_bridge/test -v
node src/aubo/aubo_cloud_bridge/test/test_widget.js
/usr/bin/python3 src/aubo/aubo_cloud_bridge/dashboards/build_dashboard.py
```

Catkin 中也配置了 `catkin_add_nosetests(test)`。

### 找不到 `thingsboard_bridge` 节点

若出现 `Cannot locate node of type [thingsboard_bridge]`，先执行第 2 节的
`chmod +x`、编译和 `source devel/setup.bash`，再检查：

```bash
rospack find aubo_cloud_bridge
ls -l ~/aubo/ros_mobile_manipulation_lab/devel/lib/aubo_cloud_bridge/thingsboard_bridge
/usr/bin/python3 -c 'import aubo_cloud_bridge, paho.mqtt.client; print("Python dependencies OK")'
```

`rospack find` 应指向当前工作空间的 `src/aubo/aubo_cloud_bridge`。
只执行 `chmod +x` 而没有编译时，包内的 `aubo_cloud_bridge` Python 模块仍可能无法导入。
如果节点启动后报告 `FileNotFoundError`，先检查 Token 文件是否已按第 2 节创建，
或通过 `token_file:=/实际路径` 指定；也需检查自定义 CA 路径。

本地联调检查：

1. 云设备 Latest telemetry 持续变化、只读时按钮禁用。
2. 仿真启用远程任务：观察 → READY → 开始 → 状态和队列更新。
3. 停止动作后核对本地 STOPPED 和机械臂实际状态，不能仅看 RPC 接收成功。
4. 断网后页面显示过期、恢复后 `bridge_session` 更新；不自动恢复旧运动请求。
5. 检查同编号重试不会产生第二次 ROS 调用、旧会话与过期请求被拒绝。

连接失败先检查出网 TCP 8883、MQTT 主机、Token 文件、系统时间与 CA；
不要开放 ROS master 的 11311 端口到公网。
`sorting_available=false` 时检查本地分拣节点和 `sorting_ns`；本包没有启动它们。
有遥测但按钮不亮时检查数据源设备、启用开关、当前状态与用户 RPC 权限。

2026-11-09 在本机 Ubuntu / ROS Noetic 环境完成包编译，通过 11 项离线 Python 测试
（含模拟 Paho 1.x、2.x 适配器）及网页控制器 Node 测试；使用系统 Python / Paho、
临时 Token 和独立 ROS master 验证了正式 launch 能启动只读桥并注册五个分拣话题订阅。
默认 MQTT 端点的 TCP 8883、TLS 握手和系统 CA 校验通过。
本机默认 Token 文件尚未配置，**尚未验证真实设备认证、云端遥测、仪表盘导入、Gazebo 或真机联调**。
配置设备凭据后，按本说明继续验证上述步骤。

官方接口依据：[MQTT RPC](https://thingsboard.io/docs/reference/mqtt-api/rpc/)、
[仪表盘](https://thingsboard.io/docs/user-guide/dashboards/)、
[导入结构实现](https://github.com/thingsboard/thingsboard/blob/master/ui-ngx/src/app/shared/import-export/import-export.service.ts)。
