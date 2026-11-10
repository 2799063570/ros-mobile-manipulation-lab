# AUBO 华为云工作台：电脑 + 手机浏览器

**当前方案：无相机投影工作台 + cpolar 手机入口，同时保留 IoTDA 上报。启动步骤见 [LOCAL_ROS.md](LOCAL_ROS.md)，华为云模型与大屏配置见 [cloud_dashboard/README.md](../cloud_dashboard/README.md)。**
下面保留已有演示和 FunctionGraph 备选部署说明；当前无需继续配置 APIG 或 IAM 应用密钥。

按照 `aubo_sorting_rqt` 的浅色工作台布局实现：状态、五步流程、识别画面区域、XY 平面图、
11 列目标队列、队列统计、过程记录、四个任务入口。手机竖屏自动重排。
无需安装 App；部署公网 HTTPS 入口后，手机用 4G/5G 打开同一链接，可从浏览器添加到主屏幕。

**代码 ZIP 已上传到北京四的 `aubo-dashboard` 函数（Python 3.10、256 MB、40 秒），
应用权限及公网入口尚未配置，FunctionGraph 路线没有公网入口；当前采用的 cpolar 路线已验证公网 HTTPS 工作台，见 LOCAL_ROS.md。**
本机 `127.0.0.1` 仅用于检查页面效果；手机输入该地址不会访问到机器人电脑。

## 1. 先在电脑看效果（演示数据）

```bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
/usr/bin/python3 src/aubo/aubo_huawei_bridge/dashboard/app.py --demo
```

电脑浏览器打开 `http://127.0.0.1:8099`，访问口令为 `demo`。
此模式始终标注“演示数据 · 只读”，所有任务入口禁用，即使设置控制环境变量也不能执行。
`Ctrl+C` 结束预览。预览不需要填写设备密钥或 IAM 凭据，也不需要 ROS 环境。

## 2. 手机 4G/5G 的候选云端链路（入口待配置）

```text
手机 / 电脑浏览器
       │ HTTPS + 工作台访问口令
       ▼
华为云 FunctionGraph：页面 + 应用接口
       │ IAM 应用身份，查询设备影子 / 同步命令
       ▼
华为云 IoTDA 免费 S0
       │ 现有 MQTT/TLS 出站连接
       ▼
aubo_huawei_bridge → /sorting
```

机器人电脑保持上网、分拣系统及云桥运行；不用对机器人电脑做端口映射。
设备侧 `device.json` 中的密钥用于 MQTT 接入，**不能代替应用侧查询权限**。
FunctionGraph 使用独立 IAM 应用凭据，设备密钥不上传网页或函数包。

### 公网入口、域名与费用

2026-11-09 已在当前账号的北京四控制台确认：设置菜单没有“函数 URL”，
公网 API 触发器显示 **APIG 专享版**，尚未选择或购买实例。
因此当前不能把 FunctionGraph 当作已验证的免费网页托管入口；无需继续寻找该菜单。
专享版涉及实例与公网费用，见 [APIG 计费模式](https://support.huaweicloud.com/price-apig/apig_08_0003.html)。

对于其他账号，可在 **函数工作流 FunctionGraph → 函数设置** 检查入口能力。
官方已提供 [创建函数 URL API](https://support.huaweicloud.com/api-functiongraph/functiongraph_06_1706.html)，
用于获得平台分配的访问地址；**所在地域/账号是否提供可供浏览器使用的免 IAM 认证 URL、
是否支持 HTML 内联访问，需在你的控制台实际确认**。尚未完成真实公网调用验证。
本包对函数 URL 的常见 HTTP 事件结构和 APIG 事件结构均提供适配，不能据此保证所有区域均已开放。

用户已选择低成本演示方案：本机工作台直接连接 ROS，cpolar 提供手机 HTTPS 入口，
华为 IoTDA 云桥保留监控；具体启动、口令和 Token 配置见 [LOCAL_ROS.md](LOCAL_ROS.md)。
已完成本机只读启动与控制模拟测试，已复用用户的 cpolar Token 验证公网 HTTPS 登录和状态读取，手机实际 4G/5G 访问待手机端确认。

若控制台只提供 IAM 认证、浏览器下载 HTML，或没有“函数 URL”，先保留现有 IoTDA，
反馈该页面的选项，再决定入口。不要为了本演示直接购买专享 APIG、EIP、ECS 或域名。
APIG 默认调试子域名有 [每天 1000 次访问限制](https://support.huaweicloud.com/apig_faq/apig-faq-190627032.html)，
并且新账号可用网关规格和费用需要另行确认；不应当作永久免费建站渠道。

FunctionGraph 官方 [每月免费额度](https://support.huaweicloud.com/price-functiongraph/functiongraph_00_0012.html)
为 100 万次请求、400000 GB·秒执行时间，子主账号共享。
采用按需实例、不开预留实例；本页面只在可见时每 30 秒查询，单手机持续打开一小时约 120 次查询，
另有页面资源及登录请求。**免费额度不代表整个链路永久零费用**：公网流量、API 网关、日志和
其他服务可能另外计费，开通前以控制台计费提示为准。IoTDA S0 额度与 FunctionGraph 额度各自计算。

## 3. 生成和检查函数部署包

已经生成：`build/aubo-dashboard-functiongraph.zip`（约 3.2 MB，不纳入 Git）。
部署目标 **事件函数 / Python 3.10 / Linux x86_64**，执行入口 `index.handler`。
可重新打包：

```bash
/usr/bin/python3 src/aubo/aubo_huawei_bridge/dashboard/build.py
```

脚本下载官方 `huaweicloudsdkcore` 与 `huaweicloudsdkiotda` 及依赖，按 Linux CPython 3.10 打包，
不读取 `device.json`、`.env` 或用户目录。压缩包中 `dependencies.lock` 记录实际依赖版本。
不要在浏览器或函数测试事件中粘贴 AK/SK。

## 4. 华为云控制台部署（首轮只读）

1. 进入 **FunctionGraph → 函数列表 → 创建函数**，地域选择 **华北-北京四**。
2. 选择 **创建空白函数 → 事件函数 → Python 3.10**，名称 `aubo-dashboard`。
   先确认本账号提供“函数 URL”；如果创建页没有 Python 3.10，先反馈选项，不要上传到不匹配的运行时。
3. 使用按需实例，建议内存 256 MB、超时 40 秒；不创建预留实例，不接 VPC/NAT。
   若之后启用任务控制，允许至少 2 个按需实例并行，避免等待动作应答时阻塞停止请求。
4. 上传 `build/aubo-dashboard-functiongraph.zip`，保持 ZIP 根目录有 `index.py`、`app.py`、`dist/`。
   执行入口设置 `index.handler`。
5. 在 **设置 → 环境变量** 配置下表。密钥值只在云控制台填写，不放到聊天或仓库。
6. 使用 IAM 最小权限应用身份：只读阶段需要目标设备的 **查询设备影子 ShowDeviceShadow** 权限；
   控制阶段再增加 **同步下发命令 CreateCommand** 权限。不需要管理员身份。
   [影子 API](https://support.huaweicloud.com/api-iothub/iot_06_v5_0079.html) 说明了项目、设备与响应结构。
7. 在 **函数 URL** 创建公网 HTTPS 地址。如果提供“无认证/NONE”，由本包自己的工作台口令保护 API，
   CORS 保持关闭，网页和 API 使用同一入口。若只有 IAM，不可把 AK/SK 写进页面绕过它，先核对该地域入口能力。
8. 先在电脑打开平台分配的 URL，登录后应看到真实云端设备状态，右上角显示 **华为云 · 设备影子**。
   然后手机关闭 Wi-Fi，用 4G/5G 打开同一 HTTPS 链接登录。
   这一步验证成功后，才算公网手机链路打通。

| 环境变量 | 填写说明 |
|---|---|
| `AUBO_DASHBOARD_MODE` | `cloud`；云上不要使用 `demo` |
| `DASHBOARD_PASSWORD` | 独立随机访问口令，至少 20 字符，不能用设备密钥 |
| `DASHBOARD_SESSION_KEY` | 另一个独立随机字符串，至少 32 字符，所有实例保持一致 |
| `DASHBOARD_ENABLE_COMMANDS` | `false`，首轮保持只读 |
| `IOTDA_ENDPOINT` | **应用侧 HTTPS 地址**，如 `https://你的实例前缀.st1.iotda-app.cn-north-4.myhuaweicloud.com`；不是 `iotda-device` |
| `IOTDA_PROJECT_ID` | 北京四区域项目 ID，在“我的凭证 → API 凭证”查看；不是产品 ID 或资源空间 ID |
| `IOTDA_DEVICE_ID` | 已注册设备的完整 ID |
| `HUAWEICLOUD_SDK_AK` | 专用 IAM 应用访问密钥 AK，后台环境变量中保存 |
| `HUAWEICLOUD_SDK_SK` | 对应 SK，后台环境变量中保存 |

`DASHBOARD_PASSWORD` 与 `DASHBOARD_SESSION_KEY` 可以在本机用密码管理器分别生成，
也可在自己的终端执行 `python3 -c 'import secrets; print(secrets.token_urlsafe(32))'` 两次，
把两次不同结果分别保存。不要截图包含环境变量值的页面。
应用凭据后续可替换为函数委托临时身份；当前版本明确使用配置的 IAM AK/SK，不自动推断或创建授权。

### 本机读取真实云数据（可选）

安装 `requirements.txt` 到独立虚拟环境，按上表把环境变量载入当前终端后运行 `python app.py`。
默认绑定 `127.0.0.1:8099`；不要把该调试服务器直接暴露公网。
本地浏览真实数据仍需应用侧 IAM 权限，不会读取 ROS 设备密钥当作应用授权。

## 5. 与 rqt 的一致部分和数据边界

- 保留状态/故障、五步流程、动作描述、队列计数、11 列目标表、XY 图和过程记录。
- 手机表格可横向滚动查看全部列；可筛选状态、搜索 ID/类别、点击目标在地图中选中。
- 本机 ROS 模式的地图读取与 rqt 相同的桌面中心、尺寸和参考坐标系；无参数时按有效目标自动选取范围。超出已配置桌面的目标仍保留在表格并显示计数。
- 本次升级仅在华为桥的 `target_cache` JSON 字符串中补充坐标、角度、宽度、确认帧数和年龄。
  无需重建产品模型；需重启华为桥让新版采集生效。已有运行进程仍使用旧版，不会热更新。
- `target_cache` 仍最多 1200 UTF-8 字节，上报仍最多 3800 字节。几何数据增加后只显示前几个目标；
  裁剪提示明确显示，统计保留完整队列计数。只有旧版快照时，缺失几何字段显示“—”，地图不虚构点位。
- 网页、手机与 IoTDA 均不采集或传输相机图片，远程仅保留平面投影。相机画面留在本地 rqt。
- 在线调参、算法/规划器切换、硬件诊断和夹爪单独操作保留在本地 rqt。
- 过程记录只保存当前浏览会话观察到的变化，最多 200 条；30 秒采样可能错过中间步骤，不是完整机器人日志。
- 页面不缓存设备数据，退出清除页面数据与内存令牌。浏览器不保存口令；登录有效期两小时。

## 6. 之后开放任务操作

先在 Gazebo 和本地 rqt 验证任务，再按主 README 启动桥的 `enable_commands:=true`，
确认 IAM 身份允许同步下发命令，最后将后台 `DASHBOARD_ENABLE_COMMANDS` 改成 `true`。
两端必须同时启用。只有新鲜快照（不超过 90 秒）、会话有效、分拣接口可用时才开放；开始分拣还需要 `READY`。

网页先取得有效期 25 秒、绑定会话的签名票据，再提交；重复提交同一票据保留同一 command_id，
实际去重由本地持久化 `CommandJournal` 完成。请求不自动重试，不使用异步/离线命令排队。
设备会话改变、过期或状态不允许时拒绝。网络中断/SDK 超时可能结果未知，需检查设备影子里的 `last_command`
与现场状态，不应重新生成新 ID 连续点击。停止入口独立于其他待决请求；它不是硬件急停。

## 7. 验证

```bash
/usr/bin/python3 -m unittest discover -s src/aubo/aubo_huawei_bridge/dashboard/test -v
/usr/bin/python3 -m unittest discover -s src/aubo/aubo_huawei_bridge/test -v
node --check src/aubo/aubo_huawei_bridge/dashboard/dist/app.js
```

已使用 Chrome 检查桌面 1440px 与手机 390px 布局、登录、筛选、目标选中、标签页、断网、退出清空及 JS 错误。
后台以模拟云端验证鉴权、票据篡改/过期、会话变化、只读模式、旧快照、重复请求保留同一 ID、停止与超时未知结果。
没有通过测试调用真实机械臂动作。**FunctionGraph 部署、真实 IAM 影子查询与手机 4G/5G 仍待账号配置后验证。**
