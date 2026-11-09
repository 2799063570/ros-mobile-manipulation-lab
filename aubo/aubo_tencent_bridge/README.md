# AUBO 腾讯云 IoT Explorer / 微信小程序

独立 ROS1 功能包：`aubo_tencent_bridge`。
提供腾讯云设备节点、产品物模型、微信原生小程序和微信云开发云函数。
首版支持单台或配置列表中的设备：状态、步骤、目标队列、故障，以及观察 / 开始 / 停止 / 回结束位。

```text
微信小程序 → 微信云开发 auboApi → 腾讯云 IoT Explorer 云 API
                                         ↕ MQTT / 双向 TLS
                                  aubo_tencent_bridge
                                         ↕ ROS
                                  现有 /sorting 接口
```

账号、产品、设备和小程序尚未创建时，可以先使用演示模式预览页面。
真实链路需要完成本文的云端配置。本仓库不自动创建、购买、部署或发布云资源。

## 域名与免费使用说明

本方案使用腾讯云提供的 MQTT 接入地址，以及微信云开发的 `wx.cloud.callFunction`，
无需购买自有域名或自建网站。IoT Explorer 设备接入和 CloudBase 环境分别计费。

2026-11-09 核对腾讯云官方[购买与计费 FAQ](https://cloud.tencent.com/document/faq/1081/122398)：
新用户消费实例暂不提供免费激活码，普通设备激活码最小购买数量为 1000 个；
企业实例按订阅计费。因此不能将本包的真实腾讯云接入视为免费服务。
CloudBase 的体验额度也不会抵扣 IoT Explorer 的设备接入费用。
单台机器人零预算实验应先确认可用的免费平台及额度，再决定是否迁移。
本地小程序演示和本地测试可以在未开通这些云资源时运行。

## 1. 目录与现有包的关系

| 路径 | 用途 |
|---|---|
| `scripts/tencent_bridge` | 腾讯云 MQTT / ROS 节点 |
| `launch/tencent.launch` | 默认只读的启动入口 |
| `config/thing_model.json` | 腾讯云物模型模板 |
| `config/device.example.json` | 本机证书及设备身份配置示例 |
| `wechat/miniprogram/` | 微信原生 WXML / WXSS / JS 页面 |
| `wechat/cloudfunctions/auboApi/` | 可信身份、设备授权、查询及行为调用 |
| `test/` | 无账号测试与本地 TLS / ROS 联调脚本 |

本包依赖 `aubo_cloud_bridge` 的纯 Python `Telemetry`、`Dispatcher` 和指令日志实现，
用于复用已有 ROS 数据格式、有效期检查、持久化去重和独立停止线程。
启动本包只连接腾讯云；依赖包中的 ThingsBoard 节点由其自己的 launch 启动。
本包使用独立节点名和指令日志目录，也不启动机械臂、SDK、MoveIt 或分拣任务。

## 2. 没有账号时先看小程序

1. 安装[微信开发者工具](https://developers.weixin.qq.com/miniprogram/dev/devtools/download.html)。
2. 将下面整个 `wechat` 目录作为小程序项目导入：

   ```text
   /home/zlab/aubo/ros_mobile_manipulation_lab/src/aubo/aubo_tencent_bridge/wechat
   ```

3. 使用开发者工具提供的测试/游客模式预览。项目初始 AppID 为 `touristappid`，
   `miniprogram/config.js` 的 `demoMode` 默认为 `true`。
4. 编译后看到“演示模式 · 当前显示样例数据，未连接机器人”。四个任务按钮保持禁用。

演示模式不调用云函数。电脑开发者工具中可以先检查布局；真实设备扫码预览、云开发及正式发布
需要在开发者工具中填写自己申请的小程序 AppID，并配置相应开发者/体验者权限。

## 3. 创建腾讯云产品和设备

本版采用 **IoT Explorer 物模型 + 设备证书认证，MQTT/TLS 8883**。
设备密钥认证的 TLS-PSK 接入不在首版支持范围内。

1. 注册腾讯云账号，完成平台要求的账号认证，打开[物联网开发平台控制台](https://console.cloud.tencent.com/iotexplorer)。
2. 选择地域，建议首轮使用广州 `ap-guangzhou`；后面的云函数地域参数必须与设备一致。
   按控制台选择可用实例，开通前确认实例及消息计费。
3. 创建产品，选择直连设备、适合电脑联网的通信方式，数据协议选择 **物模型**，
   认证方式选择 **证书认证**。记录产品 `ProductId`。
4. 在工作空间运行下面命令，用真实产品 ID 生成物模型：

   ```bash
   cd /home/zlab/aubo/ros_mobile_manipulation_lab
   /usr/bin/python3 src/aubo/aubo_tencent_bridge/config/generate_thing_model.py \
     --product-id YOUR_PRODUCT_ID \
     --output src/aubo/aubo_tencent_bridge/config/thing_model.local.json
   ```

5. 在该产品的“功能定义 / 导入物模型”中导入生成的 JSON 内容。模板包含只读属性及一个
   `task_command` 行为，行为入参 `request`、出参 `result` 均为 JSON 字符串。
6. 在产品中创建一台设备，例如 `aubo-sorter-01`，下载该设备的证书、私钥，以及平台指定的 CA。
   MQTT 主机复制设备详情中的实际接入地址，不填控制台网址。

创建和认证步骤依据：[创建产品](https://cloud.tencent.com/document/product/1081/103586)、
[设备证书认证](https://cloud.tencent.com/document/product/1081/138028)、
[物模型配置](https://cloud.tencent.com/document/product/1081/103588)。

## 4. 机器人电脑编译与连接

系统依赖：ROS Noetic、系统 Python 3.8+、`python3-paho-mqtt`、`ca-certificates`。
新环境可用 `sudo apt install python3-paho-mqtt ca-certificates` 安装 MQTT 及 CA 依赖。

```bash
source /opt/ros/noetic/setup.bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
catkin_make --pkg aubo_cloud_bridge aubo_tencent_bridge -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
install -d -m 700 ~/.config/aubo/tencent
cp src/aubo/aubo_tencent_bridge/config/device.example.json ~/.config/aubo/tencent/device.json
chmod 600 ~/.config/aubo/tencent/device.json
```

把下载的 CA、设备证书、私钥分别保存为目录中的 `ca.crt`、`device.crt`、`device.key`，
私钥权限设为 `600`。编辑 `device.json`，填写实际 `product_id`、`device_name`、`host`；
证书文件路径按实际位置填写。证书和私钥保存在本机，不写入 ROS 参数或小程序代码。

```bash
roslaunch aubo_tencent_bridge tencent.launch
```

出现 `Tencent IoT Explorer connected; task commands disabled` 后，检查腾讯云设备在线，
属性中的 `bridge_wall_ms` 每两秒更新。即使尚未启动分拣系统，也会有心跳；
此时 `sorting_available=0`、`state=UNKNOWN` 属于正常情况。

若启用了 Catkin 包白名单，将本包和依赖包加入已有白名单。
本机默认 `python3` 可能指向 Conda；ROS 节点通过 Catkin 入口使用 `/usr/bin/python3`。

## 5. 注册小程序并配置微信云开发

1. 在[微信公众平台](https://mp.weixin.qq.com/)注册自己的小程序，获取 AppID。
2. 在微信开发者工具中，把项目 AppID 改成自己的 AppID。
3. 开通该小程序的微信云开发环境，记录环境 ID。配置计费方案按平台当时提供的选项确认。
4. 在云开发数据库创建集合 **`aubo_acl`**，设置自定义安全规则：

   ```json
   {"read": false, "write": false}
   ```

   该集合由管理员和服务端维护；小程序通过云函数查询自己的权限。
5. 在开发者工具选择云环境，右键 `cloudfunctions/auboApi`，选择“上传并部署：云端安装依赖”。
   使用平台支持的 Node.js 20 或更高运行时，函数超时配置为至少 20 秒。
6. 为云函数配置以下环境变量。`config.example.json` 是配置清单，不会自动被函数读取。

   | 环境变量 | 示例 / 用途 |
   |---|---|
   | `WX_APP_ID` | 自己的小程序 AppID |
   | `IOT_REGION` | `ap-guangzhou` |
   | `IOT_DEVICES` | `{"sorter-01":{"ProductId":"实际产品ID","DeviceName":"aubo-sorter-01"}}` |
   | `ENABLE_COMMANDS` | 初次使用填 `false` |

7. 为函数配置调用 IoT Explorer 的凭据。优先使用函数运行角色的临时凭据，授权
   `DescribeDeviceData` 和 `CallDeviceActionSync`，资源范围按控制台支持收敛到目标产品/设备。
   代码会读取运行时的 `TENCENTCLOUD_SECRETID`、`TENCENTCLOUD_SECRETKEY`、`TENCENTCLOUD_SESSIONTOKEN`。
   若环境无法提供相应角色权限，可在**云函数服务端**配置专用受限子账号的 `IOT_SECRET_ID`、
   `IOT_SECRET_KEY`；使用临时凭据时另填 `IOT_SESSION_TOKEN`。
8. 修改 `miniprogram/config.js`：

   ```javascript
   module.exports = {
       demoMode: false,
       cloudEnvId: '你的微信云开发环境ID',
       functionName: 'auboApi'
   };
   ```

本项目采用“自有云函数 + 腾讯云云 API”的对接方式，不需要在客户端保存腾讯云账户密钥。
此路径不依赖 IoT Explorer 应用端 AppKey / AppSecret；设备 ID 由云函数配置，客户端只提交设备别名。

官方说明：[云云对接](https://cloud.tencent.com/document/product/1081/117768)、
[获取设备属性](https://cloud.tencent.com/document/api/1081/34979)、
[调用设备行为](https://cloud.tencent.com/document/api/1081/38940)、
[数据库安全规则](https://cloud.tencent.com/document/product/876/41802)。

## 6. 给自己的微信账号授权

1. 重新编译小程序。第一次可能显示“尚无设备权限”，页面底部会显示当前微信用户 OpenID。
2. 点击 OpenID 复制，在云开发控制台的 `aubo_acl` 集合新增文档：

   ```json
   {
     "_id": "刚才复制的OpenID",
     "devices": {"sorter-01": "viewer"}
   }
   ```

3. `sorter-01` 必须与 `IOT_DEVICES` 中的别名一致。重新进入小程序页面，设备选择器中应出现该设备，
   并显示真实心跳、状态和队列。
4. 在开发者工具点击预览，授权的开发者/体验者使用微信扫码，即可在手机测试。
   扫码预览权限与这里的设备授权分别设置；正式发布继续按微信平台流程操作。

## 7. 接入本地分拣与开启任务控制

先通过原项目入口启动 Gazebo 或已有分拣系统，确认以下服务及状态话题存在：

| 方法 | 本地接口 |
|---|---|
| `move_to_observation` | `/sorting/move_to_observation` |
| `start` | `/sorting/start` |
| `stop` | `/sorting/stop` |
| `home` | `/sorting/home` |
| 状态 | `/sorting/state`，`std_msgs/String` |

如果需要分阶段停止、重启云桥，建议先在独立终端运行 `roscore`，再分别启动分拣和云桥，
便于保持 ROS master 常驻。

在仿真中完成只读状态核对后，开启控制需要同时满足三项：

1. 将自己在 `aubo_acl` 中的角色改为 `operator`。
2. 云函数环境变量 `ENABLE_COMMANDS=true`。
3. 重启腾讯云桥并开启本机开关：

   ```bash
   source /home/zlab/aubo/ros_mobile_manipulation_lab/devel/setup.bash
   roslaunch aubo_tencent_bridge tencent.launch enable_commands:=true
   ```

小程序会根据心跳、账号权限、本地接口和任务状态启用按钮。
先“前往观察位”，等待 READY，再“开始分拣”。运动请求有确认，停止可以在运动请求等待期间独立发送。
同一台机器人进行远程控制测试时只启用一条可控云桥，避免多个控制入口同时发起任务。

## 8. 指令和数据边界

- 属性上行：`$thing/up/property/{ProductId}/{DeviceName}`，`method=report`，QoS 0。
- 行为下行：`$thing/down/action/{ProductId}/{DeviceName}`，订阅 QoS 1。
- 行为应答：`$thing/up/action/{ProductId}/{DeviceName}`，`method=action_reply`，QoS 0。
- 所有遥测属性只读；四个任务方法封装在 `task_command` 行为的 `request` 字符串中。
- 云函数 `prepare` 签发 10 秒有效请求；`command` 保留原请求编号、会话和有效期，不自动重发。
- 本机在 ROS 调用前持久化编号，重复请求不会再次执行；重连更换会话，旧请求失效。
- `accepted` 表示 Trigger 接收了请求，任务结果继续查看状态。
  腾讯云行为接口有应答时限，ROS 服务耗时或网络超时会得到 `unknown`，此时需核对本地状态。
- 云桥使用墙上时间上报，Gazebo 暂停不影响心跳。小程序离线、切到后台或网络异常时禁用控制；
  软件停止不能替代现场硬件急停。
- `bridge_context` 将会话、心跳、状态、接口可用性和开关放在单一属性，供云函数校验控制条件。
- 腾讯云字符串属性有长度限制，`target_cache` 保持有效 JSON 且最多 2048 UTF-8 字节。
  小程序会提示列表截断；`queue_*` 保留完整队列计数，不是累计产量或实物成功率。
- 首版不传输相机视频、点云，也不提供任意 ROS 服务、关节目标或夹爪释放指令。
- 单机指令 SQLite 日志长期保留，请监控其磁盘使用；设备证书和 CA 到期前需更新。

协议依据：[物模型 Topic 通信](https://cloud.tencent.com/document/product/1081/138037)。

## 9. 本地验证与排查

从工作空间根目录执行，不需要腾讯云账号：

```bash
/usr/bin/python3 -m unittest discover -s src/aubo/aubo_tencent_bridge/test -v
node src/aubo/aubo_tencent_bridge/test/test_cloud.js
node src/aubo/aubo_tencent_bridge/test/test_miniprogram.js
```

编译并加载工作空间后，还可运行独立 ROS master + 临时双向 TLS MQTT 测试端点：

```bash
source devel/setup.bash
/usr/bin/python3 src/aubo/aubo_tencent_bridge/test/smoke_ros_mtls.py
```

| 现象 | 检查 |
|---|---|
| 找不到 `tencent_bridge` | 编译包、脚本执行权限、`source devel/setup.bash` |
| 节点报告 FileNotFoundError | `device.json`、CA、设备证书和私钥路径 |
| TLS / MQTT 失败 | 选择证书认证产品、主机、证书配对、系统时间、8883 出网 |
| 已连接但属性不更新 | 节点日志是否提示 report 被拒绝；检查物模型是否完整导入 |
| 小程序显示演示数据 | 在 `miniprogram/config.js` 设置 `demoMode:false`，填写环境 ID |
| `login_required` | 检查 AppID、`WX_APP_ID` 和云环境绑定 |
| 没有设备权限 | 使用当前微信 OpenID 在 `aubo_acl` 中授权，别名需一致 |
| 只读、按钮禁用 | 检查角色、云端开关、本机开关、心跳和 `/sorting` 服务 |
| `server_not_configured` | 检查云函数环境变量、依赖、运行角色或服务端凭据 |
| `cloud_service_unavailable` | 检查 IoT API 权限、地域、产品设备及数据库集合 |

本地测试覆盖传输适配、字符串边界、授权、有效期、会话、超时和小程序控制逻辑。
演示页面仍需在微信开发者工具编译预览；真实腾讯云证书接入、云函数部署和微信真机联调
需要配置上述账号资源后验证，不能以本地测试替代。
