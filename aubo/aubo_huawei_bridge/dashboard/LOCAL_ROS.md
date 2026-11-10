# 当前采用：运行分拣程序，电脑网页与手机 4G/5G 控制

```text
电脑浏览器 ────────────────┐
                          ▼
                   本机 ROS 工作台 :8099 → /sorting 四个任务服务
                          ▲
手机浏览器 → cpolar 随机 HTTPS 地址

/sorting → 现有 aubo_huawei_bridge → 华为云 IoTDA（保留云端监控）
```

本方案使用已有页面，工作台直接读取本机 ROS，约 2 秒刷新，不需要 FunctionGraph、APIG 或 IAM 应用 AK/SK。
此前创建的 `aubo-dashboard` 函数可以保留，本方案不调用它。
手机无需安装 App；电脑、分拣程序和 cpolar 隧道需要持续运行。

## 第一次准备

本机已安装官方 Linux amd64 客户端 **cpolar 3.3.12**，位置 `~/.local/bin/cpolar`，未创建系统后台服务。
已复用用户通过官方命令配置的 `~/.cpolar/cpolar.yml`，当前使用该账号验证公网地址。
其他电脑请从 [官方下载安装页](https://www.cpolar.com/download) 安装适合其 CPU 架构的客户端。

1. 在 [cpolar 注册](https://dashboard.cpolar.com/signup) 免费账号。
2. 登录官网后台，找到自己的 **认证 Token / Authtoken**。
3. 新开电脑终端运行：

   ```bash
   cd /home/zlab/aubo/ros_mobile_manipulation_lab
   source devel/setup.bash
   rosrun aubo_huawei_bridge huawei_tunnel --configure
   ```

4. 按提示粘贴 Token 后按回车。输入不显示是正常现象，不需要把 Token 发到聊天里。
   本脚本配置的 Token 保存到 `~/.config/aubo/huawei/cpolar.yml`，权限 600，不出现在命令行参数或仓库。
   若你已经运行官方 `cpolar authtoken` 命令，脚本会复用 `~/.cpolar/cpolar.yml`，无需重复配置；该文件也需权限 600。
   如果提示已有配置，默认保留；更换账号时仅在本机编辑该文件。

若新克隆工作空间还没有上述命令，先编译一次：

```bash
source /opt/ros/noetic/setup.bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
catkin_make --pkg aubo_cloud_bridge aubo_huawei_bridge -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

## 每次演示

### 1. 启动原来的分拣程序

按原来的流程启动 Gazebo / MoveIt / 分拣节点，让本地 rqt 可以正常操作。
仅启动 Gazebo 不代表分拣任务接口已就绪。工作台需要以下话题/服务：

- `/sorting/state` 及状态、进度、目标队列话题。
- `/sorting/move_to_observation`、`/sorting/start`、`/sorting/stop`、`/sorting/home` 四个 Trigger 服务。

现有华为云桥可继续运行；网页控制期间保持其他云桥的远程控制关闭，避免多个操作入口同时发起任务。

### 2. 一条命令启动电脑工作台和手机隧道

```bash
cd /home/zlab/aubo/ros_mobile_manipulation_lab
source devel/setup.bash
roslaunch aubo_huawei_bridge mobile.launch enable_commands:=true
```

这条命令只启动网页后台和隧道，不自动启动机械臂、Gazebo、MoveIt，也不自动发出运动任务。
终端会显示一个 `https://...` 公网地址。保持该终端运行。
默认仅监控时去掉 `enable_commands:=true`；首次验证也可先使用只读模式。

### 3. 电脑与手机打开页面

- 电脑：`http://127.0.0.1:8099`。
- 手机：关闭 Wi-Fi，用 4G/5G 打开终端输出的 **HTTPS 公网地址**。
- 两端使用同一个工作台口令，口令在 `~/.config/aubo/huawei/dashboard.json` 的 `password` 字段。
  首次启动自动生成该私有文件（权限 600），本机已完成一次只读启动，因此文件已经存在。
  用编辑器打开复制即可，不要把整份文件截图或提交到仓库。
- 工作台口令、cpolar Token、华为设备密钥是三种不同凭据，分别保存在自己的文件中。
- 正常显示 **ROS · 本机设备**。若显示“演示数据”，那是旧的 `app.py --demo` 预览。
- 网页和手机可以同时查看；任务操作先“前往观察位”，等待 **READY** 后再“开始队列分拣”。
  任务请求接收后，以后续状态判断执行情况。

网页和手机操作同一台机器人，共用同一持久化指令日志；同一个 command_id 不重复执行。
停止请求使用独立任务线程。关闭网页、隧道或后台不会自动发送停止命令，正在执行的任务由分拣程序管理。

## 仅电脑访问 / 分步排查

不需要公网时可以只启动工作台：

```bash
roslaunch aubo_huawei_bridge workstation.launch enable_commands:=true
```

然后另开终端启动隧道：

```bash
source /home/zlab/aubo/ros_mobile_manipulation_lab/devel/setup.bash
rosrun aubo_huawei_bridge huawei_tunnel
```

两个 launch 不要同时启动，都会占用 8099 端口与同一个指令日志。

| 现象 | 处理 |
|---|---|
| 电脑打不开 8099 | 检查工作台进程是否仍运行、端口是否被其他程序占用 |
| 显示本机分拣接口未就绪 | 启动原来的分拣任务节点；确认上述四个服务和状态发布者存在 |
| 数据正常但按钮灰色 | 确认带了 `enable_commands:=true`；“开始”还需要 READY；数据/网络过期也会禁用 |
| 任务“结果未知” | 查看过程记录、设备状态和本地 rqt，确认实际执行情况；不要连续生成新请求重试 |
| 提示先运行 `--configure` | Token 尚未填写，在自己的交互终端完成首次配置 |
| cpolar 登录/连接失败 | 检查本机配置文件中的 Token、账号套餐与网络；不要贴出 Token |
| 手机地址打不开 | 确认电脑与隧道运行，使用当前 HTTPS 地址；免费随机域名变化后需重新复制 |
| 旧手机收藏失效 | 复制当前终端新地址；固定域名属于另外的付费能力，本轮不用 |

## 流量与边界

官方 [免费套餐](https://www.cpolar.com/pricing) 列出随机域名及 1 Mbps 带宽，
[文档](https://www.cpolar.com/docs) 说明 HTTP 隧道可生成 HTTPS 地址，免费地址会变化。
本包使用国内 `cn` 区域，使用 HTTPS 地址访问，不预留固定域名。
cpolar 请求检查面板仅绑定本机 `127.0.0.1:4040`，不要将该面板另行发布到公网。
当前客户端会同时列出 HTTP 和 HTTPS 地址；手机使用 HTTPS 地址。
启动脚本仅为 cpolar 子进程清除代理环境变量，解决本机系统代理导致的控制连接 EOF；不会更改用户终端或其他程序的代理。
已使用当前账号验证公网 HTTPS 页面、口令登录与 ROS 状态读取；手机真实 4G/5G 访问仍需在手机端确认。

界面本轮显示完整本机目标队列与几何字段，不受 IoTDA 的 1200 字节字符串裁剪限制。
按当前需求，网页与手机只显示目标平面投影、队列、完成统计和状态，不采集或发送相机图片。
工作台不再订阅图像话题，`/api/camera` 已移除；本地 rqt 的相机画面保留。
地图读取 `/color_sorting_task` 的 `table_center`、`table_size` 和 `target_frame`，与 rqt 的工位范围一致；
可用 `task_node:=/你的任务节点` 指定参数来源；没有工位参数时按有效目标自动选取范围。
华为云同时上报及原生大屏配置见 [cloud_dashboard/README.md](../cloud_dashboard/README.md)。
本机工作台每两秒刷新与华为云桥每 30 秒上报各自独立，不增加 IoTDA 上报频率。

工作台只绑定 `127.0.0.1`，使用独立随机口令与短时签名票据。
仅通过 HTTPS 公网地址登录；页面不在远端明文 HTTP 下发送登录口令。
本次测试使用模拟服务验证控制链路，真实 ROS 启动检查保持只读，未执行真实机械臂动作。

## 按类别显示分拣完成数量

网页/手机新增“分拣完成 · 本次运行累计”区域，直接显示 C++ 任务节点的
`/sorting/completed_counts` 快照，例如 `bottle：1，can：2，box：3`、合计 6。
本地 rqt 使用同一份数据。颜色场景会显示 red/green/blue，类别随当前任务配置变化。
仅在任务成功完成抓取、放置与退离时计数，不从队列 done 数量推算，不因重复收消息而累加。
观察、清队列、重新开始和刷新网页不会清零；任务节点重启后清零，统计不跨重启保存。
本机显示无需修改 IoTDA 模型；要将完成统计也上报到华为云，需按云端大屏说明给 Sorting 服务追加 5 个属性。
