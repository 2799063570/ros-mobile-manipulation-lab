# OpenVLA Ubuntu 部署实测记录

日期：2026-09-27。对应《OpenVLA_Ubuntu20.04_Gazebo部署计划.md》。

## 已完成

- Ubuntu 20.04.6，内核 5.15.0-139-generic，ROS Noetic 1.17.4，Gazebo 11.15.1。
- GPU RTX 5060 Ti，16311 MiB，驱动 580.82.09；驱动报告 CUDA 13.0，实际 PyTorch 使用 CUDA 12.8。
- 系统内存约 60 GiB；开始部署时 /home 可用约 31 GiB，根分区约 5.7 GiB。
- 工作空间 `/home/zlab/aubo/ros_mobile_manipulation_lab`，ROS 使用 `/usr/bin/python3` 3.8。
- 固定相机 Gazebo 场景启动成功，关闭任务、感知、自动观察位、GUI 和 RViz。
- `/workspace_camera/color/image_raw` 为 RGB8、640×480，稳定墙钟频率约 20 Hz；
  `/joint_states` 有消息，`use_sim_time=true`。保存图像可见红绿蓝方块、放置区域及部分机械臂。
- 遵照用户提示，复用已有 `yolo` 的 Python 3.10.12、torch 2.9.1+cu128、torchvision 0.24.1。
  新增 `aubo-openvla-runtime` **venv 覆盖环境**，保留 yolo 的 NumPy 2.2.6、timm 1.0.22 等原版本。
- 推理覆盖环境中 transformers 4.40.1、timm 0.9.10、NumPy 1.26.4、bitsandbytes 0.48.2。
  `pip check` 通过；GPU BF16 矩阵乘法、NF4 Linear4bit 运算均通过。
- 官方 revision 固定为 `47a0ec7fc4ec123775a391911046cf33cf9ed83f`。
  AutoConfig/AutoProcessor 加载通过；真实图像输出 pixel_values `(1,6,224,224)`。
  空参数模型构造通过，参数数目 7,541,237,184。这不等于权重加载和真实推理通过。
- 新增 `aubo/aubo_vla`：回环 HTTP 服务、串行最新帧观察桥接、GPU 检查、单帧基准测试和启动文档。
- HTTP mock 服务与真实 Gazebo 图像桥接通过；5 项边界测试通过。
- 仿真暂停停止请求、恢复继续请求、时钟回退清理待处理帧测试通过。
- 请求在途时暂停再恢复 Gazebo，返回的旧响应被丢弃，实测通过。

## 下载及真实推理状态

权重已完整下载，真实单帧推理验收通过。`validation/real_model.json` 为 `stage=complete`、`passed=true`。
使用真实 Gazebo RGB 样例，预热 1 次后连续 5 次均返回七维有限数值，`mock=false`。

- 预热耗时约 543 ms；预热后中位数 338.4 ms，范围 337.7–348.2 ms。
- PyTorch 峰值 allocated 4511 MiB（约 4.41 GiB），reserved 4620 MiB；不代表整张 GPU 的总占用。
- NF4 + double quant、BF16 compute、eager attention，复用 yolo 的 torch 2.9.1+cu128。
- 首次验收遇到 attention mask 276/275 维度错误。固定 revision 的上游 `predict_action` 会补 token 29871，
  但不扩展输入 attention_mask。服务在调用前同步补齐 token 与 mask，避免上游重复追加。
  未修改 Hugging Face 缓存代码或 yolo 依赖。
- 修复后的验收日志：`/tmp/aubo-openvla-validation-fixed.log`；原始失败日志保留在 `/tmp/aubo-openvla-validation.log`。

实时 Gazebo → HTTP 真实模型 → ROS 诊断也已连续 5 次通过，HTTP 全流程约 354–358 ms。
证据见 `aubo/aubo_vla/validation/real_ros_bridge.json`。真实服务和仅观察桥接已启动，
诊断话题 `/openvla_observe/diagnostics`，日志分别为 `/tmp/aubo-openvla-service.log`、`/tmp/aubo-openvla-bridge.log`。

以上证明模型推理接口可用，不等于 AUBO 已能按模型动作抓放。

## 已知限制与后续工作

- 启动原有 Gazebo 时出现 `gazebo_ros_control/pid_gains` 缺失告警；相机、关节状态及 MoveIt 初始化正常。
  本次仅观察，不据此宣称控制器跟踪精度或抓放能力已验收。
- `aubo-openvla-runtime` 继承 yolo 的 PyTorch/CUDA；依赖 yolo 路径及版本保持稳定。
  首次创建的 `aubo-openvla` Conda Python 3.10 环境保留但未用于推理，重复 PyTorch 安装已中止。
- 本次没有修改系统 Python、驱动、现有运动控制代码或 yolo 包版本。
- 没有动作执行接口。`bridge_orig` 不能直接用于 AUBO 控制。
- AUBO 动作规范、示范采集与回放、微调和留出场景闭环抓放尚待开展。
  这些属于计划中的后续任务验收，不以模型下载完成为完成条件。

运行说明与验证素材见 [`aubo_vla/README.md`](../aubo/aubo_vla/README.md)。
依赖快照、GPU 算子结果、Gazebo RGB 样例在 `aubo/aubo_vla/validation/`。
