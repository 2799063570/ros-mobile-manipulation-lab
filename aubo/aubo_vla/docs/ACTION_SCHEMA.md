# AUBO 动作规范 v1

标识：`aubo_delta_pose_v1`。该规范用于新采集的 AUBO 数据与后续经过验证的
AUBO 模型。原版 OpenVLA 的 `bridge_orig` 不满足本规范，不能仅改统计键名称来转换。

| 项 | 定义 |
| --- | --- |
| 参考系 | episode manifest 的 `base_frame`，默认 `base_link` |
| 工具 | manifest 的 `tcp_frame`，默认 `tcp_link`；不能换成法兰而保留旧标签 |
| 动作顺序 | `[dx, dy, dz, rx, ry, rz, g]` |
| 平移 | 参考系中的 TCP 位置增量，单位 m；`p_next = p + delta_p` |
| 旋转 | 参考系中的旋转向量（轴乘旋转角），单位 rad，最短旋转弧 |
| 旋转合成 | 左乘：`q_next = quaternion(rotvec) * q`；存储四元数为 xyzw |
| 夹爪 | 下一采样时刻的归一化位置反馈，0 张开，1 为配置闭合位置，连续值 |
| 时间 | `image(t)` 对应 `pose(t) → pose(t+dt)`；dt 由 manifest 的 nominal_dt 定义 |
| 尾帧 | 没有下一帧，保留观测但不导出动作标签 |

默认采样 2 Hz，nominal_dt 为 0.5 s（仿真采用仿真时间，实机采用 ROS 时间）。
检查器默认要求每对实际间隔在 nominal_dt 的 ±20% 内，超出则整条 episode
不通过，不跨断流、暂停或时间重置拼接动作。不能把改变 period tolerance 当作
修复错误采样频率的方法。episode 内改变相机内参、关节顺序同样不允许。

夹爪默认使用 Gazebo 的 `joint1`，open=0 rad、closed=0.28 rad，与固定分拣配置
相符。接触物体后实际位置可能达不到闭合命令，因此反馈标签可能小于 1。
这代表实际位置，不代表“尚未发闭合命令”或“未抓到物体”。端点外超过 5% 的
反馈拒收；小幅越界会裁到 [0,1]，原关节位置仍保存在样本里。Inspire 实机夹爪
需要真实反馈桥接到带正确源时间戳的 JointState，并重新配置端点；不能使用
joint_state_publisher 的默认位置或上一次命令冒充反馈。

记录文件：

```text
episode_<时间>_<唯一编号>/
  episode.json        # 指令、场景、参考系、来源话题、标定、采样率、人工结果
  samples.jsonl       # 逐帧时间、关节、TCP、相机外参、CameraInfo、夹爪、PNG SHA256
  images/000000.png   # 原始全幅图像，BGR 编码 PNG 后正常解码为 RGB
```

写入和离线检查均验证 PNG 结构、校验块及完整像素解码；图片必须为 RGB，
尺寸必须与样本 CameraInfo 一致。SHA256 一致仍不能替代图像解码校验。

导出字段 `action_label=measured_next_state_delta` 明确说明这是**已执行运动的实测差分**。
它不是控制器命令，也不是原版 OpenVLA 的训练数据格式。接入训练前，需要根据
实际控制器选择监督方式、处理轨迹等待段、接触阶段与连续夹爪语义，并计算 AUBO
数据集自己的动作统计。不能重复反归一化 `predict_action` 已反归一化的输出。

数据只有在节点正常结束并得到人工 success/failure 标记时才是 complete；崩溃会
保留 recording，暂停/时钟回退/人工中止为 aborted。failure 数据可检查，但导出需
显式 `--include-failure`，不能默认混入成功示范。success 是人工任务结果标签，
不代表记录器执行过视觉抓取验证。

影子预览要求模型结果同时声明：本 schema、经过验证的 AUBO 统计键、revision
和训练动作周期 action_dt，并与 shadow 配置一致。模型服务的声明只描述部署者
验证过的语义，不会自动完成标定或微调。预览使用**图像时刻的 TCP**为锚点，
不向机械臂追赶当前姿态。超步长/超工作空间结果拒绝，保留原模型输出供诊断。
候选始终携带 `executable=false`、`collision_checked=false`、`ik_checked=false`。
单步限幅和工作空间检查不能替代碰撞、逆解或控制权仲裁。
