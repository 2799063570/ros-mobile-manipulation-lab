# 华为云监控大屏 + cpolar 手机工作台（不传相机）

```text
/sorting 状态、队列、完成统计
    ├─ huawei_bridge → MQTT/TLS → IoTDA S0 设备影子
    │                               └─（免费权益确认后）规则 → OBS CSV → DLV 大屏
    └─ huawei_dashboard → cpolar HTTPS → 电脑 / 手机投影工作台与任务控制

相机图像 → 仅本地 rqt；两个远程通道均不采集、不传输相机图片。
```

**范围：只使用账号已确认的免费权益，不创建付费实例。**
IoTDA 的设备上报与 cpolar 工作台是两条独立链路，任一通道异常不阻止另一通道更新。
网页约 2 秒刷新；云端仍每 30 秒上报、单包不超过 3800 UTF-8 字节，保持现有滚动预算。

## 1. 先给现有 IoTDA 产品追加 5 个只读属性

进入现有产品 **AUBO_Sorting → 基本信息 → 模型定义 → Sorting → 新增属性**。
保留原有 20 个属性和 `task_command` 命令，不删除设备或重建产品。

| 属性名 | 数据类型 | 长度 / 范围 | 含义 |
|---|---|---|---|
| `completed_counts` | string | 最大 512 字节 | 本次任务节点运行的按类别累计计数 JSON，包含统计会话及完整合计 |
| `completed_total` | string | 最大 20 字符 | 合计完成数，十进制字符串 |
| `statistics_available` | int | 0～1 | 是否已收到有效的任务完成统计 |
| `completed_counts_truncated` | int | 0～1 | 按类别计数是否因云端报文预算而只上传部分类别 |
| `workcell` | string | 最大 256 字节 | `table_center`、`table_size`、`frame` 的 JSON |

访问方式均选 **可读**，必选设为否。现有设备本轮无需更换密钥。
只包含新增字段的参考：[add_dashboard_properties.json](../config/add_dashboard_properties.json)。
完整新产品模型也已重新生成，包含 **25 个属性**；现有产品优先逐项追加，不将 API JSON 当作控制台导入包。
自动化修改可参考 `config/update_product.json`，但本轮没有调用账号下的产品更新 API。

完成后，新版云桥才使用 `extended_telemetry:=true`，避免旧产品模型拒收新属性。
未设置时仍发送原有 20 字段，保持旧设备上报兼容。

示例（示例数量）：

```json
{
  "completed_counts": "{\"scope\":\"node_session\",\"session\":\"example\",\"total\":6,\"counts\":{\"bottle\":1,\"can\":2,\"box\":3}}",
  "completed_total": "6",
  "statistics_available": 1,
  "completed_counts_truncated": 0,
  "workcell": "{\"table_center\":[0.7,0],\"table_size\":[0.7,1],\"frame\":\"base_link\"}"
}
```

`target_cache` 继续携带目标坐标；`target_cache_truncated=1` 时仅显示部分目标，`queue_total` 仍是完整队列计数。
`completed_counts_truncated=1` 时保留完整合计，按类别明细不完整，大屏应显式提示。
`statistics_available=0` 表示未知，不应把该报文里的占位 0 展示为已确认产量。
计数仅在完整抓放退离成功后增加，任务节点重启后清零。**不能把历史快照的累计值相加。**

## 2. 同时启动两个通道

现有 `huawei.launch` 和 `mobile.launch` 继续可分别使用。
新增组合入口一次启动 IoTDA 只读云桥、投影工作台与 cpolar：

```bash
source /home/zlab/aubo/ros_mobile_manipulation_lab/devel/setup.bash
roslaunch aubo_huawei_bridge cloud_mobile.launch \
  enable_commands:=true extended_telemetry:=true
```

前提：现有产品已补齐上述属性、设备文件与 cpolar 账号已配置，且分拣程序正在运行。
`enable_commands` 仅开放网页任务操作；组合入口中的 IoTDA 云命令保持关闭。
两个通道都不订阅 `sensor_msgs/Image`，网页的 `/api/camera` 接口已移除。

**同一设备只运行一份云桥。组合入口不要和已有 `huawei.launch` / `mobile.launch` 重复启动。**
本机目前的旧云桥启动进程还承载 ROS master，因此本轮不单独杀掉它；它继续按旧格式上报。
新字段在补齐产品模型并计划重启 ROS 环境后启用。下次建议让 `roscore` 单独运行，再分别启动
分拣程序和上面的组合入口，使以后重启云桥不影响 ROS master。
本轮已运行的分拣任务及本次累计统计无需为网页界面修改而重启。

## 3. 华为云原生大屏的免费权益检查

官方示例是 [IoTDA → OBS → DLV](https://support.huaweicloud.com/bestpractice-iothub/iot_bp_0222.html)。
DLV 文档说明有 [30 天基础版体验试用](https://support.huaweicloud.com/usermanual-dlv/dlv_01_0002.html)，
但实际账号资格、地域、数据连接能力和 OBS 存储/请求免费额度都需要确认。
[DLV 各版本能力](https://support.huaweicloud.com/productdesc-dlv/dlv_07_001.html) 有差异，不能只看到免费试用就推断所有连接器均可用。

下一步只查看控制台，不先创建资源：

1. 在华为云搜索 **数据可视化 DLV**，确认是否显示体验试用、试用时长和到期规则。
2. 确认试用版可创建教程要求的 **CSV → OBS 文件** 数据连接。
3. 在费用中心确认 OBS 的存储、写入、读取及相关流量权益均足够覆盖演示；不能仅确认桶创建免费。
4. 任一项需要支付或没有相应权益时，停在配置说明阶段，继续使用现有 IoTDA 设备影子与 cpolar 工作台。

本轮没有开通 DLV、OBS、APIG、ECS，没有配置自动续费，也没有创建账号中的转发规则。
因此云端大屏尚未部署；下面的字段、转换器和布局说明均是已准备好的配置材料。

## 4. 确认免费权益后配置 IoTDA → OBS CSV

使用与 IoTDA、DLV 一致的区域。OBS 桶保持私有，通过云服务授权访问，不把私有凭据放进图表脚本。
在 IoTDA **规则 → 数据转发** 新建规则，数据来源选设备属性、触发设备属性上报。
规则限制到这一台 AUBO 设备，并选 `Sorting` 服务；避免把其他设备或服务混进同一文件。

转发目标为 OBS、文件类型 CSV；按 [obs_field_mapping.json](obs_field_mapping.json) 逐项填写
“转发字段 → 目标存储字段”。例如：

- `notify_data.header.device_id` → `device_id`
- `notify_data.body.services[0].properties.target_cache` → `target_cache`
- `notify_data.body.services[0].properties.completed_counts` → `completed_counts`
- `notify_data.body.services[0].properties.bridge_wall_ms` → `bridge_wall_ms`

完整路径依据上述官方示例，假定上报正文中第一个服务为 `Sorting`，与本包报文一致。
文件前缀建议 `aubo/sorting`，保留时间按免费额度设置；连接后应查看真实 CSV 表头和一条数据核对映射。
云端刷新还受规则缓冲、文件落盘与 DLV 查询周期影响，不承诺和本机网页一样实时。

## 5. DLV 手工配置的仪表盘

这些文件不是 DLV 的原生大屏导入包，不要直接上传 `obs_field_mapping.json` 或转换器目录作为大屏。
按照下表在 DLV 创建空白大屏，添加组件并使用同一个 CSV 数据连接：

| 区域 | 组件 | 转换器 | 字段 / 说明 |
|---|---|---|---|
| 顶部 | 文本 / 状态卡 | `status.js` | 显示 `value`、`detail`、`age_s` 和是否过期 |
| 中部主区域 | 数值型 XY 散点图 | `projection.js` | x、y 坐标（米），`s` 为类别系列，`name` 为目标 ID |
| 下方 | 数字翻牌器 | `completed_total.js` | `value` 为本次运行累计完成合计 |
| 下方 | 柱状图 / 类别卡 | `completed_by_class.js` | `name` 类别、`value` 完成数量 |
| 侧边 | 条形图 / 状态表格 | `queue_counts.js` | 当前队列状态计数，不是累计产量 |

每个 [converters](converters) 文件都包含完整的 DLV“数据转换器”函数体。
先将文件顶部 `AUBO_DEVICE_ID` 改为设备完整 ID，然后粘贴到对应组件的数据转换器编辑器。
它们只选择同一设备、`Sorting` 服务的最新 `bridge_wall_ms` 快照，跨任务重启也不累加旧会话数量。
默认快照超过 90 秒则状态卡显示过期，投影返回空数据，防止把旧坐标当成当前目标；
若 CSV 落盘延迟不能满足这一范围，先继续用 cpolar 查看实时投影，不为此开启付费加速资源。

散点组件具体字段槽位以编辑器为准，将 x、y 都设为**数值轴**，系列/图例映射 `s`；
本例输出保留了 `category`、`status`、`id` 供提示信息使用。
坐标范围按 `workcell` 设置：仿真示例 X `0.35～1.05`、Y `-0.5～0.5`；实机需换成实际桌面参数，
不要把机器人米制坐标映射为地理经纬度。
参考：[DLV 散点图组件](https://support.huaweicloud.com/usermanual-dlv/dlv_01_0832.html)。

数据源先设为静态样例时只能验证布局；必须接入实际 CSV 并看到时间戳更新，才算云端数据链路验证完成。
不在大屏中增加相机、图片流、视频组件。原生大屏本轮作为只读监控，任务控制沿用有鉴权的 cpolar 工作台。

## 6. 本地验证

```bash
/usr/bin/python3 -m unittest discover -s src/aubo/aubo_huawei_bridge/test -v
/usr/bin/python3 -m unittest discover -s src/aubo/aubo_huawei_bridge/dashboard/test -v
node src/aubo/aubo_huawei_bridge/cloud_dashboard/test_converters.cjs
```

已验证协议模型一致、旧模型兼容开关、计数/工位 JSON、截断提示、单包预算、无相机端点和转换器逻辑。
账号中的产品属性追加、云端新字段上报及 DLV/OBS 连接仍需配置后验证，不把本地模拟结果当作云端部署完成。
