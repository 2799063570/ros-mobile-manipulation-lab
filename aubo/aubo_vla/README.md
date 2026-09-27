# AUBO OpenVLA 观察模式

本包只订阅相机并发布动作诊断，没有任何机器人运动发布器或执行服务调用。
`bridge_orig` 是预训练数据的动作统计键，不能作为 AUBO 的动作标定。

## 本机环境

ROS 使用 `/usr/bin/python3`（3.8）。模型使用
`/home/zlab/anaconda3/envs/aubo-openvla-runtime/bin/python`（3.10）。
模型环境是由 `yolo` Python 创建的 **venv**，不是独立 Conda 环境；使用绝对路径或
`source /home/zlab/anaconda3/envs/aubo-openvla-runtime/bin/activate`，不要用 `conda activate` 激活它。
它通过 `--system-site-packages` 复用 yolo 的 torch 2.9.1+cu128、torchvision 0.24.1 与 CUDA 库；
OpenVLA 的 transformers/timm/NumPy 等安装在 venv 内，不会覆盖 yolo。
因此不要删除或升级 yolo 后仍假设该推理环境完全可复现；升级后须重跑 GPU 与模型验收。
`validation/runtime-freeze.txt` 记录当前可见包的完整快照，包含继承的 YOLO 包。

创建同样的覆盖环境：

```bash
/home/zlab/anaconda3/envs/yolo/bin/python -m venv --system-site-packages /home/zlab/anaconda3/envs/aubo-openvla-runtime
/home/zlab/anaconda3/envs/aubo-openvla-runtime/bin/python -m pip install -r aubo/aubo_vla/requirements-model.txt
/home/zlab/anaconda3/envs/aubo-openvla-runtime/bin/python aubo/aubo_vla/scripts/check_gpu.py
```

## 启动

下列脚本命令在 `/home/zlab/aubo/ros_mobile_manipulation_lab/src` 执行。
ROS 终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/zlab/aubo/ros_mobile_manipulation_lab/devel/setup.bash
export PATH=/usr/bin:/bin:$PATH
roslaunch aubo_sorting sorting_gazebo.launch camera_mount:=eye_to_hand start_task:=false start_perception:=false auto_move_to_observation:=false auto_start:=false gui:=false rviz:=false
```

独立模型终端（首次加载会下载约 15GB 权重）：

```bash
/home/zlab/anaconda3/envs/aubo-openvla-runtime/bin/python aubo/aubo_vla/scripts/model_service.py
```

在另一个 ROS 终端加载相同 setup 后：

```bash
roslaunch aubo_vla observe.launch
rostopic echo /openvla_observe/diagnostics
```

本包纯 Python，源码工作空间下已验证 `roslaunch` 可发现，无需重建全部 catkin。
HTTP 只绑定 `127.0.0.1:8008`。`GET /health` 返回模型标识和 `mock`；
`POST /predict` 接收 `request_id`、`stamp`（仿真秒）、`frame_id`、`instruction`、`image_png`（base64 PNG）。
返回七维有限数值动作、原始请求元数据、模型 revision、统计键、延迟与 CUDA 显存统计。
NF4、double quant、BF16 compute、eager attention 固定在服务中。

图像使用 `/workspace_camera/color/image_raw` 原始 640×480 全幅 RGB，无额外裁剪。
ROS 经 BGR 编码 PNG，再由 PIL 解码 RGB，保持颜色；模型处理器使用固定 revision 的
双视觉骨干 224×224 bicubic resize/normalization，输出 `(1,6,224,224)`。

ROS 仅保留最新帧，串行请求。服务忙碌返回 409；错误发布 `status=error`。
超时和帧龄采用单调墙钟。暂停超过 2 秒、仿真时钟回退或响应过期时丢弃结果。
默认 HTTP 超时与帧龄上限为 30 秒，可用 launch 的 `timeout`、`max_age` 调整；
这些参数不构成执行控制的安全验收，因为本包没有执行模式。

## 验收与测试

```bash
/usr/bin/python3 -m unittest discover -s aubo/aubo_vla/tests -v
/home/zlab/anaconda3/envs/aubo-openvla-runtime/bin/python aubo/aubo_vla/scripts/check_gpu.py
/usr/bin/python3 aubo/aubo_vla/scripts/benchmark.py aubo/aubo_vla/validation/gazebo_rgb.png --count 5
```

自动续传并在本进程完成真实单帧验收（无需先启动 HTTP 服务）：

```bash
/home/zlab/anaconda3/envs/aubo-openvla-runtime/bin/python aubo/aubo_vla/scripts/validate_model.py
```

结果写入 `validation/real_model.json`；只有 `stage=complete` 且 `passed=true` 才表示真实单帧验收通过。
`stage=download/load/inference` 表示仍在进行，`stage=failed` 会保留失败阶段与错误。

`benchmark.py` 先预热一次，再统计五次真实模型输出；默认拒绝 mock。
仅测试通信时可给模型服务加 `--mock`，给 benchmark 加 `--allow-mock`；这种结果不计入真实模型验收。
`tests/check_ros_lifecycle.py` 会短暂暂停/恢复 Gazebo，只能在测试仿真上运行。

后续抓放需要另行定义 AUBO 动作坐标、旋转合成、夹爪语义、时间配对，采集回放示范并微调。
本包不提供把预训练动作直接发给 `/visual_servo/target_pose` 的路径。

## 本机真实模型验收结果

权重已下载，固定 Gazebo 样例预热后 5 次真实推理通过；中位数 338.4 ms，PyTorch allocated 峰值约 4.41 GiB。
详见 `validation/real_model.json`。当前服务在调用上游 `predict_action` 前同步补齐尾部 token 29871 和 attention_mask，
以修复固定 revision 在 eager attention 下的 276/275 维度错误；不修改模型缓存代码。
