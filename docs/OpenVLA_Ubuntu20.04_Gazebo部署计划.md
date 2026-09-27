# OpenVLA 在 Ubuntu 20.04 / Gazebo 中的部署计划

编写日期：2026-09-27。本文是后续切换 Ubuntu 后执行的部署准备文档，尚未在 Ubuntu 验证模型或闭环控制。

实施进度：已于同日在 Ubuntu 开始执行；实际环境与验证结果见 [部署实测记录](OpenVLA_Ubuntu20.04_部署实测记录.md)。下文候选安装组合保留作为原始计划，实际采用复用 yolo PyTorch 的隔离覆盖环境。

## 1. 已确认的条件与阶段目标

| 项目 | 当前条件 |
| --- | --- |
| 电脑 | 同一台 Windows / Ubuntu 双系统电脑，当前运行 Windows |
| GPU | Windows 下实际检测为 NVIDIA GeForce RTX 5060 Ti，16311 MiB，标称 16GB |
| 部署系统 | 用户确认 Ubuntu 20.04；Linux 驱动、内核及软件环境待切换后检查 |
| 机器人项目 | ROS 1 Melodic / Noetic、AUBO i5、夹爪、MoveIt、Gazebo Classic |
| 第一阶段 | 固定底座机械臂 Gazebo 仿真，先验证视觉输入与模型推理，再推进策略适配 |
| 当前交付 | 仅本文档；未安装模型，未保留新增 VLA 功能包，未修改控制代码 |

Windows 与 Ubuntu 不会同时运行。本方案是在启动 Ubuntu 后，让 Gazebo 和模型服务都运行于这台电脑，通过本机回环网络通信，无需另一台服务器或 SSH。

第一阶段拆成两个验收点：

1. **推理链路验收**：Gazebo 图像能够进入 OpenVLA，得到七维有限数值动作，记录显存和延迟；不下发机械臂命令。
2. **仿真任务验收**：完成 AUBO 动作适配、示范采集与微调，闭环完成指定颜色方块抓放。这需要后续代码与数据工作，不等于下载模型后自动具备。

本阶段固定底盘。后续移动操作仍先由现有导航到达工位，再切换到机械臂策略。

## 2. 项目中可复用的内容

| 能力 | 仓库入口 | 用途 |
| --- | --- | --- |
| 固定机械臂分拣场景 | `aubo/aubo_sorting/launch/sorting_gazebo.launch` | 仿真世界、机械臂、控制器、MoveIt |
| 固定 RGB-D 相机 | `aubo/aubo_sorting/worlds/sorting.world` | `/workspace_camera/color/image_raw` |
| 视觉感知 | `aubo/aubo_perception/` | 图像、深度、TF、目标位姿 |
| 运动与分拣 | `aubo/aubo_sorting_core/` | 现有成功轨迹和示范采集基础 |
| 视觉伺服 | `aubo/aubo_ros_control/VISUAL_SERVO.md` | 坐标处理、逆解、限速及执行后端参考 |

优先使用眼在手外相机，便于观察全工作区。视角、裁剪、分辨率等处理一旦确定，应与后续训练数据保持一致。

现有 `/visual_servo/target_pose` 是目标位姿接口，不能直接接收 VLA 的七维数组。VLA 的动作通常描述末端平移、旋转及夹爪，具体尺度、坐标与语义由训练数据决定。

## 3. 软件环境设计

```text
同一台电脑，启动 Ubuntu 20.04
  ├─ ROS 终端：系统 Python 3.8 / ROS Noetic / Gazebo / MoveIt
  │     相机图像 → 待开发 ROS 桥接节点 → 动作诊断
  │                         ↕ HTTP，127.0.0.1
  └─ 模型终端：独立 Conda Python 3.10 / CUDA PyTorch / OpenVLA
                            图像 + 指令 → 模型动作
```

不要替换系统 `/usr/bin/python3`，不要把 PyTorch 安装到 ROS 的系统 Python，也不要在激活模型环境的终端中编译 catkin。两边通过 HTTP 交换数据，模型进程无需导入 `rospy` 或 `cv_bridge`。

### RTX 5060 Ti 的关键兼容性

上游 OpenVLA 的历史环境使用 PyTorch 2.2.0 等旧依赖；本机显卡属于更新的 Blackwell 架构，不能直接照搬旧环境。PyTorch 2.7 引入 Blackwell 支持与 CUDA 12.8 wheel。[PyTorch 官方说明](https://pytorch.org/blog/pytorch-2-7/)

建议以 **Python 3.10 + torch 2.7.1 + torchvision 0.22.1 + cu128** 作为首次兼容性实验起点，先检查 GPU 算子，再检查 OpenVLA。这个组合有官方 PyTorch 安装包，但**与旧版 OpenVLA 的组合尚未在本机验证**，不是已验证的环境锁定结果。[官方版本配套](https://pytorch.org/get-started/previous-versions/)

CUDA 12.8.1 的历史 Linux 支持表包括 Ubuntu 20.04；这不能替代对本机驱动、内核和 GPU 的检查。[NVIDIA 支持表](https://docs.nvidia.com/cuda/archive/12.8.1/cuda-installation-guide-linux/index.html)

首轮推理采用 eager attention，先避免 FlashAttention 编译。16GB 显存优先评估 4-bit 量化；不能仅凭显存容量承诺推理频率。Gazebo、桌面和 RViz 也会占用同一张 GPU。

## 4. 切换 Ubuntu 后先收集环境信息

在普通终端执行：

```bash
lsb_release -ds
uname -r
nvidia-smi
free -h
df -h "$HOME"
source /opt/ros/noetic/setup.bash
rosversion -d
gazebo --version
command -v python3
python3 --version
```

检查结果应记录：GPU 型号/显存、Linux 驱动、内核、可用内存/磁盘、ROS 发行版、Gazebo 版本及项目 Linux 路径。

- 如果 `nvidia-smi` 失败，先解决 Linux 驱动对 RTX 5060 Ti 的支持，再安装模型环境。Windows 驱动可用不代表 Ubuntu 驱动已经配置好。
- `nvidia-smi` 的 CUDA Version 表示驱动所支持的 CUDA 上限，不能当作 PyTorch 编译使用的 CUDA 版本。
- 不预先指定驱动安装命令；切换后依据内核、现有驱动及该 GPU 的支持情况选择对应驱动，避免直接覆盖可工作的图形环境。
- 将模型缓存和 catkin 构建目录放在 Ubuntu 原生文件系统。Windows 的 `D:\...` 不是 Ubuntu 的项目路径。
- 本文以已有 ROS Noetic 项目可运行为前提。如果 `/opt/ros/noetic` 不存在，应先确认实际 ROS 安装，不继续机械复制下方命令。

## 5. 先验证原有 Gazebo 场景

在 ROS 终端使用实际工作空间路径替换示例路径：

```bash
source /opt/ros/noetic/setup.bash
source /path/to/catkin_ws/devel/setup.bash
rospack find aubo_sorting
roslaunch aubo_sorting sorting_gazebo.launch \
  camera_mount:=eye_to_hand \
  start_task:=false \
  start_perception:=false \
  auto_move_to_observation:=false \
  auto_start:=false
```

这条命令对应仓库已有入口，关闭分拣任务、检测器及自动观察位动作，保留仿真与控制器初始化。
在另一个加载同一 ROS 环境的终端检查：

```bash
rostopic hz /workspace_camera/color/image_raw
rostopic echo -n 1 /workspace_camera/color/image_raw/header
rostopic hz /joint_states
rosparam get /use_sim_time
```

验收：图像持续更新，关节状态存在，`use_sim_time` 为 true，画面能看到抓取工作区。不要先引入 VLA 来排查原有 Gazebo 故障。
显存紧张时可在启动参数中增加 `gui:=false rviz:=false`；相机传感器仍需正常渲染和发布，必须复查图像话题。

## 6. 创建独立模型环境

以下为切换 Ubuntu 后的候选安装步骤，当前未执行。假设 Conda 已安装；若未安装，先从其官方发行渠道安装到用户目录。

```bash
conda create -n aubo-openvla python=3.10 -y
conda activate aubo-openvla
python -m pip install --upgrade pip
python -m pip install torch==2.7.1 torchvision==0.22.1 \
  --index-url https://download.pytorch.org/whl/cu128
```

安装后必须实际运行 GPU 运算，而不只是检查 `torch.cuda.is_available()`：

```bash
python - <<'PY'
import torch
print('torch:', torch.__version__, 'cuda:', torch.version.cuda)
assert torch.cuda.is_available(), 'CUDA unavailable'
print('gpu:', torch.cuda.get_device_name(0))
print('capability:', torch.cuda.get_device_capability(0))
x = torch.randn(1024, 1024, device='cuda')
y = x @ x
torch.cuda.synchronize()
assert torch.isfinite(y).all().item()
print('GPU matmul passed')
PY
```

如果出现 `no kernel image`、驱动不兼容或共享库错误，停在这一层排查。不要用更旧的 torch 2.2 覆盖以迎合 OpenVLA 依赖。

GPU 检查通过后，安装原版 OpenVLA 的推理侧依赖：

```bash
python -m pip install 'numpy<2' 'Pillow>=10.3,<12' \
  transformers==4.40.1 tokenizers==0.19.1 timm==0.9.10 \
  'accelerate>=0.25,<1' sentencepiece==0.1.99
```

这些是依据上游旧版推理接口选择的候选约束，需要与新 PyTorch 联合测试。不要直接运行上游仓库的 `pip install -e .`：其完整训练依赖固定旧版 torch，并包含当前推理阶段不需要的训练组件。[上游依赖定义](https://github.com/openvla/openvla/blob/main/pyproject.toml)

4-bit 推理还需要 bitsandbytes。部署时按其安装说明选择支持当前 CUDA/GPU 的版本，并记录实际版本；若依赖解析要求更改 torch 或 transformers，应先评估冲突，不直接升级整个环境。[bitsandbytes 安装说明](https://huggingface.co/docs/bitsandbytes/main/en/installation)

## 7. 单张图像推理的实现要求

此部分供后续编写和验证脚本使用，当前仓库没有对应一键启动脚本。

1. 使用 `openvla/openvla-7b` 的处理器和权重；首次下载需要访问 Hugging Face，磁盘预留权重与缓存空间。
2. 首次兼容性实验使用 `AutoModelForVision2Seq` 与 `AutoProcessor`；自定义模型实现需要 `trust_remote_code=True`，记录模型 revision，复现实验固定为具体 commit。
3. 使用 `BitsAndBytesConfig(load_in_4bit=True, bnb_4bit_quant_type="nf4", ...)`，明确计算 dtype 和 GPU 映射。量化加载后不要再照搬非量化示例的 `.to("cuda")`。
4. `attn_implementation="eager"`，先跑通再优化；推理使用 `model.eval()` 与 `torch.inference_mode()`。
5. 输入实际 Gazebo RGB 图像，以及简单英文任务指令，如 `pick up the red block`。图像按模型处理器要求处理，避免 RGB/BGR 混淆。
6. 首轮可用 `unnorm_key="bridge_orig"` 检查模型输出；**它只代表预训练数据的动作统计，不代表 AUBO 标定结果**。输出只能观察。
7. 检查动作长度为 7、无 NaN/Inf，记录实际 dtype、显存峰值、预热后推理延迟和模型 revision。测 GPU 延迟时在计时边界同步 CUDA。

验收标准：连续多次输出有效动作，显存稳定，无算子兼容错误。此时仍不能宣称机械臂能抓取；一张图片输出动作只是模型接口验收。

官方接口、微调及部署入口见 [OpenVLA 仓库](https://github.com/openvla/openvla)。

## 8. 待开发的 ROS 集成边界

后续建议新增 `aubo/aubo_vla/`，但本次未创建该包。

| 模块 | 责任 | 验收点 |
| --- | --- | --- |
| 独立推理服务 | 模型常驻 GPU，提供健康状态与动作预测 | 明确模型、revision、统计键和服务错误 |
| ROS 图像桥接 | 订阅图像、发 HTTP 请求、发布诊断 | 帧时间戳、图像通道、超时、单请求在途 |
| 动作适配器 | 反归一化、坐标变换、旋转合成、夹爪转换 | 与演示回放的动作语义一致 |
| 执行管理 | 接入现有运动后端，管理启停与控制权 | 过期动作拒绝、限速、碰撞检查、停止确认 |
| 数据录制与导出 | 同步保存观察、动作和任务标签 | 可回放、可训练、可追溯 |

服务绑定 `127.0.0.1`，ROS 与模型在同一 Ubuntu 系统内通信。上游已有 REST 服务，但不能将原版 BF16/FlashAttention 启动方式直接等同于本机 4-bit 服务，需要适配并验证。[官方服务示例](https://github.com/openvla/openvla/blob/main/vla-scripts/deploy.py)

接口设计要点：

- 图像携带仿真时间戳、坐标系和请求编号；HTTP 超时用单调墙钟计时。
- 请求串行，保留最新帧，避免慢推理积累旧动作。暂停或重置 Gazebo 时清空待执行结果。
- 动作结果携带模型标识、统计键、推理耗时；明确区分 mock 输出和真实模型输出。
- 先实现仅观察模式，其后再实现执行模式；执行前验证工作空间、碰撞、限位、超时与停止路径。
- VLA 执行与分拣状态机、独立视觉伺服不能同时持有机械臂控制权。
- MoveIt 可用于首轮受限动作验证，但每个微小动作都做完整规划可能增加延迟。执行器形式应在延迟测试后决定。

## 9. 从推理走向仿真抓取

推荐初始任务：“抓起红色方块并放到指定区域”。复用已有分拣流程生成示范，但训练标签必须来自明确的实际动作语义，不能直接把规划目标当成每一帧的动作。

每条 episode 至少记录：任务指令、RGB 图像、关节位置、TCP 位姿、夹爪状态、动作、时间戳、成功/失败标记、相机参数和动作规范版本。原版 OpenVLA 的观测输入不一定使用全部记录字段；状态仍用于动作计算和回放检查。

统一以下规范后再批量采集：

| 规范 | 要确定的内容 |
| --- | --- |
| 平移 | 基坐标系或工具坐标系、米、每步增量或其他定义 |
| 旋转 | 欧拉增量/旋转向量、旋转顺序、左乘或右乘 |
| 夹爪 | 数值范围、开合方向、离散或连续 |
| 时间 | 采样频率、观测与动作配对、实际执行周期 |
| 图像 | 相机视角、裁剪、颜色通道、分辨率处理 |

先录制少量示范并在 Gazebo 回放，确认任务可复现，再扩大数据量。上游建议目标域小规模示范微调，约 100 条可作为起点；质量与覆盖范围仍决定效果。原版策略的采样频率应参考其低频控制建议，并按本机实测延迟调整。[官方数据与性能排查](https://github.com/openvla/openvla#vla-performance-troubleshooting)

16GB 本机先承担采集与推理。官方原版 LoRA 示例指出小 batch 仍约需 27GB 显存，不能直接承诺本机可按该方案训练；后续可单独评估节省显存方案或外部训练资源。OFT 是动作分块与高效微调的后续候选，需要独立集成，不能当作原版推理脚本直接替换。[原版微调说明](https://github.com/openvla/openvla#fine-tuning-openvla-via-lora)、[OFT 项目](https://openvla-oft.github.io/)

最终评估使用未参与训练的物体位置与指令组合，记录试验次数、抓取成功率、放置成功率、完整任务成功率及失败原因。不能用演示回放成功或训练损失下降替代闭环成功率。

## 10. 常见问题与排查顺序

| 现象 | 优先检查 |
| --- | --- |
| Ubuntu 看不到显卡 | Linux 驱动、内核、设备识别；先不安装模型 |
| PyTorch 能导入但 GPU 运算失败 | 驱动与 cu128、Blackwell 算子支持 |
| `cv_bridge` 导入失败 | ROS 终端是否错误激活 Conda，系统 Python 是否被替换 |
| 模型加载显存不足 | 4-bit 是否实际启用、桌面/Gazebo/RViz 占用、其他 GPU 进程 |
| bitsandbytes 报 CUDA 错误 | 安装版本、GPU 支持和共享库来源；先保留完整报错 |
| 模型返回动作但方向不对 | 统计键、坐标系、旋转定义和训练数据，不直接提高控制增益 |
| 模型加载成功但抓取失败 | 目标域示范与微调是否完成，图像和动作预处理是否匹配 |
| 仿真暂停后执行旧动作 | 墙钟超时、仿真时间回退检测、队列清理和结果过期检查 |

## 11. 切换系统后的执行清单

- [x] 确认 Linux 识别 RTX 5060 Ti，记录驱动和系统信息。
- [x] 确认项目 Linux 路径与 catkin 工作空间，跑通原有固定相机 Gazebo 场景。
- [x] 创建模型依赖覆盖环境（复用 yolo PyTorch），完成 CUDA 与 NF4 实际运算检查。
- [x] 验证 OpenVLA 4-bit 单帧推理，记录显存和延迟（真实输出连续 5 次通过）。
- [x] 开发只观察的 HTTP/ROS 桥接，验证时间戳与异常处理（mock 异常测试与真实模型实时图像联调均通过）。
- [x] 固定可用依赖版本、模型 commit 与所有预处理参数（见 aubo_vla/README.md 与 validation/）。
- [ ] 设计 AUBO 动作规范，采集少量示范并验证回放。
- [ ] 根据显存预算安排微调，验证留出场景的闭环抓放。

下一次继续时，先读取本文并执行第 4 节环境检查；根据实际结果调整版本，不默认上述候选组合已经在 Ubuntu 测试通过。
