# AUBO → 国内华为云 IoTDA：免费单元演示

独立功能包 `aubo_huawei_bridge`，连接已有 ROS 分拣系统和华为云 IoTDA。
本阶段使用华为云控制台管理设备、查看状态、调试命令，使用平台分配的接入地址。
无需购买域名、自建云服务器或部署小程序后台。

```text
华为云 IoTDA 控制台 ↔ IoTDA 设备接入 ↔ MQTT/TLS ↔ aubo_huawei_bridge ↔ /sorting
```

本包包含只读 ROS 云桥、限量上报、产品模型导入包、API 请求体和命令参数生成器。
纯 Python 状态整理、指令有效期、持久化去重和独立停止线程复用 `aubo_cloud_bridge` 依赖。
使用独立节点、配置和日志路径；腾讯云包另行保留。
本阶段尚未接入微信小程序，也不启动机械臂、SDK、MoveIt 或分拣系统。

## 1. 免费范围和本包的流量设置

2026-11-09 核对华为云[产品规格](https://support.huaweicloud.com/productdesc-iothub/iot_04_0014.html)：
标准版免费单元 **S0** 支持 1000 台同时在线设备、每天 10000 条消息、10 TPS。
免费单元限测试调试，不承诺商用 SLA；超出消息数会被丢弃。
请在当前账号、地域的控制台确认免费单元可开通，再继续配置。

为适配单台机器人演示，本包默认：

- 每 **30 秒**上报一个完整快照，连续一天约 **2880 次**。
- 一次属性上报正文最多 **3800 UTF-8 字节**，控制在单个 4KB 计量单位内。
- 本机持久化记录滚动 24 小时的上行预算，默认 4000 单位，其中 100 单位预留给命令应答。
  节点重启不会清零；达到报告预算后暂停上报，旧记录超出 24 小时后恢复。
- 在分配本机应答预算前，不会将云命令交给 ROS。预算不足时不执行任务。
- 本地 ROS 健康检查和诊断话题仍每秒更新，不受云端 30 秒上报周期限制。

这里的 4000 仅限制本桥的上行及预扣的应答，**不是整个实例的完整计量**：
命令下行、应用下发、其他设备和规则转发也可能消耗平台额度。
MQTT 协议层连接、心跳、订阅和 Ack 的计费规则见[计费项](https://support.huaweicloud.com/price-iothub/iot_price_00020.html)。
本包不会开通付费单元、数据存储、OBS、日志转发或其他增值服务。

## 2. 你现在先做什么：开通 S0

1. 注册并登录[华为云](https://www.huaweicloud.com/)，完成平台要求的账号认证。
2. 在服务中搜索 **设备接入 IoTDA**。
3. 建议首轮选择官方快速入门使用的 **华北-北京四**，进入 **IoTDA 实例**。
4. 找到 **开通免费单元**，确认是标准版 **S0**、价格为免费/0 元，再按页面提示开通。
5. 进入实例，记录“接入信息”中的 **设备侧 MQTTS 接入地址**，端口为 **8883**。

如果当前账号页面只出现付费实例，先核对地域和免费资格；无需为这轮演示购买其他实例。
入口依据：[开通免费单元](https://support.huaweicloud.com/qs-iothub/iot_05_0008.html)。

## 3. 创建产品及模型

产品协议选择 **MQTT**，数据格式选择 **JSON**；认证使用设备密钥。
服务 ID 为 **`Sorting`**，所有遥测属性只读，唯一命令为 **`task_command`**。

推荐直接在控制台导入完整模型：

1. 创建产品时选择自定义设备类型 **`AUBOSorter`**，产品名称可填 `AUBO_Sorting`。
2. 打开该产品详情的 **基本信息 → 模型定义 → 上传模型文件**。
3. 选择 `config/AUBOSorter_AuboLab.zip`，保持 ZIP 格式直接上传并确认。
4. 成功后应出现 **`Sorting`** 服务，包含 20 个只读属性，以及 **`task_command`** 命令。
   命令输入包含 `method`、`command_id`、`session`、`issued_at_ms`、`expires_at_ms`。
5. 确认后继续注册设备。

压缩包按官方格式包含 `profile/devicetype-capability.json` 和
`service/Sorting/profile/servicetype-capability.json`；设备类型为 `AUBOSorter`，
厂商标识为 `AuboLab`。打包格式依据：[离线开发产品模型](https://support.huaweicloud.com/devg-iothub/iot_02_4012.html)。

同时保留与官方 API 对应的请求体，便于自动化配置：

| 文件 | 用途 |
|---|---|
| `config/AUBOSorter_AuboLab.zip` | 产品详情页面“上传模型文件”的导入包 |
| `config/create_product.json` | `CreateProduct` 接口的请求体，创建完整产品和模型 |
| `config/update_product.json` | `UpdateProduct` 接口的请求体，用于已有空产品 |
| `config/generate_model.py` | 重新生成 ZIP 和两个 API JSON 文件 |

如需通过[华为云 API Explorer](https://apiexplorer.developer.huaweicloud.com/)创建产品：

1. 登录后选择 **设备接入 IoTDA → 创建产品 CreateProduct**。
2. 地域和项目选择与刚才的 S0 实例一致，接入 Endpoint 选择该实例对应的应用侧地址。
   调试页面若提供 `Instance-Id` 字段，按控制台实例信息填写。
3. 请求体 `body` 使用 `config/create_product.json` 的内容；存在多个资源空间时另填目标 `app_id`。
4. 检查后执行创建请求，回到该 S0 实例的产品列表，确认出现 `AUBO_Sorting`。
5. 在模型定义中检查 `Sorting` 服务和状态属性，以及 `task_command` 命令。

`create_product.json` 和 `update_product.json` 是 **API 请求体 JSON**，与控制台“上传模型文件”使用的 ZIP 格式不同。
若希望完全通过控制台逐项添加，可按 `create_product.json` 中 `service_capabilities` 列表填写，
属性类型、长度和只读方式均在文件中定义。
`UpdateProduct` 会更新模型，请只对本次新建的空产品使用该文件。

API 结构依据：[创建产品](https://support.huaweicloud.com/api-iothub/iot_06_v5_0050.html)。

## 4. 注册真实设备并保存密钥

1. 在该实例的 **设备 → 所有设备 → 注册设备** 中选择 `AUBO_Sorting` 产品。
2. 设备标识码可填写 `aubo-sorter-01`，认证类型选择密钥认证。
3. 保存平台提供的完整 **设备 ID（device_id）** 和 **设备密钥（secret）**。
   设备 ID 不是设备显示名称；复制平台返回的完整值。
4. 在 ROS 电脑终端运行：

   ```bash
   cd /home/zlab/aubo/ros_mobile_manipulation_lab
   install -d -m 700 ~/.config/aubo/huawei
   cp -n src/aubo/aubo_huawei_bridge/config/device.example.json ~/.config/aubo/huawei/device.json
   chmod 600 ~/.config/aubo/huawei/device.json
   ```

5. 编辑 `~/.config/aubo/huawei/device.json`：

   ```json
   {
     "host": "设备侧MQTTS接入主机，不带https前缀或端口",
     "device_id": "平台返回的完整设备ID",
     "device_secret": "平台返回的设备密钥",
     "ca_file": ""
   }
   ```

`ca_file` 留空时使用系统 CA。若接入页面指定 CA，下载华为云提供的 CA 证书并填本机路径。
始终验证服务端证书和主机名，TLS 最低版本为 1.2。
设备密钥只保存在本机配置文件，不写入 ROS 参数、仓库或网页。

连接认证按[官方 MQTT 鉴权](https://support.huaweicloud.com/api-iothub/iot_06_v5_3009.html)：
设备 ID 作为用户名，UTC 小时时间戳作为 HMAC-SHA256 的 key，设备密钥作为待签名内容。
采用官方支持的签名类型 `0`，不校验连接时间戳，以支持跨小时的自动重连；
命令仍单独校验本机当前时间、短时有效期和连接会话。

## 5. ROS 端编译和只读接入

```bash
source /opt/ros/noetic/setup.bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
catkin_make --pkg aubo_cloud_bridge aubo_huawei_bridge -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
roslaunch aubo_huawei_bridge huawei.launch
```

运行依赖为 ROS Noetic、系统 Python 3、`python3-paho-mqtt` 和 CA 证书。
新环境可用 `sudo apt install python3-paho-mqtt ca-certificates` 安装 MQTT 和 CA 依赖。
本机默认 `python3` 可能是 Conda，Catkin 入口使用系统 `/usr/bin/python3`。
若工作空间使用白名单，将本包和 `aubo_cloud_bridge` 加入已有白名单。

看到以下日志后，保持终端运行：

```text
Huawei IoTDA connected; task commands disabled; report interval 30.0s
```

在控制台打开设备详情：

- **设备状态**：应显示在线。
- **设备影子 / 上报值**：`Sorting` 下应出现 `bridge_wall_ms`、`bridge_session` 等字段。
  心跳约 30 秒更新，页面不自动刷新时手动刷新。
- **在线调试 / 消息跟踪**：检查属性上报 Topic 和报文。

即使没有启动分拣系统，云桥仍上报心跳，此时 `sorting_available=0`、`state=UNKNOWN` 正常。
本地诊断命令：

```bash
rostopic echo /aubo_huawei_bridge/status
```

该话题包含连接会话、接口可用性、上报周期及本机滚动上行计数，不包含设备密钥。
云端使用[设备影子](https://support.huaweicloud.com/usermanual-iothub/iot_01_0049.html)查看最新状态，
不自动创建额外的历史存储服务。

## 6. 从控制台验证命令与反馈

可以先在只读模式验证下行链路：收到请求后会明确返回 `rejected`，不会调用运动服务。
为避免旧命令在重连后被执行，命令含会话和最长 30 秒有效期。参数由本地工具生成：

1. 先在控制台准备好设备的命令下发窗口，服务选择 `Sorting`，命令选择 `task_command`。
2. 新开 ROS 终端，生成参数（默认 25 秒有效）：

   ```bash
   source /home/zlab/aubo/ros_mobile_manipulation_lab/devel/setup.bash
   rosrun aubo_huawei_bridge huawei_make_command start --paras-only
   ```

3. 立即将生成的参数填入命令参数 JSON 框，或逐项填写对应字段，然后下发。
   `issued_at_ms` 和 `expires_at_ms` 在模型中是字符串，保留双引号。
   结果中 `result_code=1`、`status=rejected` 在只读测试中属于预期反馈。
4. 若超过有效期才发送，生成新的请求并再次确认；不要重放历史运动请求。

完整 API 命令请求体可用不加 `--paras-only` 的方式生成；`--output /tmp/huawei-command.json`
可保存到文件。工具只生成参数，不向云端或 ROS 分拣服务发送任务。

控制测试先在 Gazebo 完成本地验证，确认 `/sorting/state` 及四个 Trigger 服务均已注册。
建议先单独运行 `roscore`，再分别启动分拣系统和云桥，便于重启桥时保留 master。
结束只读桥后运行：

```bash
roslaunch aubo_huawei_bridge huawei.launch enable_commands:=true
```

支持的方法仅为：

| `method` | 本地 Trigger 服务 |
|---|---|
| `move_to_observation` | `/sorting/move_to_observation` |
| `start` | `/sorting/start` |
| `stop` | `/sorting/stop` |
| `home` | `/sorting/home` |

先观察并等待 READY，再开始分拣。跨重启去重、会话检查、有效期检查和独立停止线程均在本机执行。
`accepted` 只说明本地 Trigger 接收请求，后续任务结果继续看状态；超时或通信异常可能为 `unknown`，
此时需核对本地状态。软件停止不等于硬件急停。
本机应答预算不足时命令不执行，云端可能显示超时。
同一机器人联调时只启用一条可控云桥。

协议依据：[属性 Topic](https://support.huaweicloud.com/api-iothub/iot_06_v5_3004.html)、
[平台命令下发](https://support.huaweicloud.com/api-iothub/iot_06_v5_3014.html)。

## 7. 数据边界、排查和验证

- `queue_*` 保留最多 200 个目标的完整队列计数，不表示累计产量。
- `target_cache` 为最多 1200 UTF-8 字节的有效 JSON，`target_cache_truncated=1` 表示只显示部分目标。
- 故障及描述文本做长度限制，完整诊断仍以本地 ROS 日志为准。
- 云端心跳周期是 30 秒；设备影子保留旧值，断线后不要把旧数据当作当前状态。
- SQLite 指令日志及预算日志保存在 `~/.local/state/aubo_huawei_bridge/`，需监控磁盘用量。
- 设备 ID、密钥、接入地址或 CA 错误会导致连接失败；先检查地域和实例是否一致。
- 出现节点找不到时先编译、检查可执行权限并加载 `devel/setup.bash`。
- 云桥在线但没有分拣状态时检查 `sorting_ns` 和本地分拣入口。

无账号测试：

```bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
/usr/bin/python3 -m unittest discover -s src/aubo/aubo_huawei_bridge/test -v
```

真实 ROS + 系统 Paho + 本地 TLS MQTT 测试：

```bash
source devel/setup.bash
/usr/bin/python3 src/aubo/aubo_huawei_bridge/test/smoke_ros_tls.py
```

本地测试覆盖官方签名示例、产品字段、报文大小、预算持久化、只读、去重、旧会话和 MQTT 应答。
真实华为云账号接入、API 创建产品和控制台联调需要开通 S0 并配置设备后验证。
微信小程序后续可复用现有界面思路，但需另做华为应用侧接口及用户授权，不能直接使用腾讯云函数。

## 8. 电脑仪表盘与手机 4G/5G

当前采用 **本机 ROS 工作台 + cpolar HTTPS 手机入口**，一次启动电脑网页和手机访问。
请先看 [完整启动步骤](dashboard/LOCAL_ROS.md)。本方案不需要 APIG、FunctionGraph 或 IAM 应用 AK/SK。
华为云 IoTDA 保留状态上报；网页直接调用本机的四个任务服务。

已新增参照 `aubo_sorting_rqt` 的响应式工作台，手机浏览器无需安装 App。
界面、应用侧后台、FunctionGraph 上传包及逐步配置见 [dashboard/README.md](dashboard/README.md)。
当前已有可连接真实 ROS 的工作台及手机隧道启动命令；公网手机访问还需在本机填写 cpolar Token 并验证网络。
保持 IoTDA S0 和现有设备不变，先验证只读监控，再单独开放任务控制。
