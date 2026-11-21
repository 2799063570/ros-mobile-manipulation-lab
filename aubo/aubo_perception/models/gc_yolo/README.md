# GC-YOLO TensorRT · ROS Noetic

`GC-yolo.onnx` 来自 `weights/GC-yolo.pt`。输入为 `images`，尺寸 `1×3×640×640`，输出为 `output0`，尺寸 `1×8×8400`。类别 ID `0/1/2` 对应 `bottle/can/box`，输出末通道为弧度角。静态 FP32 ONNX 已通过 ONNX Runtime 与 PyTorch 的数值校验。

## 在机器人目标机生成 engine

首先确认 NVIDIA 驱动、CUDA 和 TensorRT 10.x C++ SDK 适配目标 GPU。进入这个目录，用目标机的 `trtexec` 构建 engine。FP16 只用于 TensorRT 内部计算，节点需要线性 FP32 输入和输出。

```bash
mkdir -p "$HOME/models"
trtexec --onnx=GC-yolo.onnx --saveEngine="$HOME/models/GC-yolo-fp32.engine" --noTF32 --skipInference
trtexec --onnx=GC-yolo.onnx --saveEngine="$HOME/models/GC-yolo-fp16.engine" --fp16 --noTF32 --skipInference
```

TensorRT 11.x 已移除 `--fp16`，构建混合精度 engine 前要先用 ModelOpt AutoCast 转换，并确保输入和输出仍为线性 FP32。 [NVIDIA TensorRT 11 迁移指南](https://docs.nvidia.com/deeplearning/tensorrt/latest/api/migration/tensorrt-10x-to-11x-trtexec.html)

## 编译和联调

在包含 WheelTec 源码的 catkin 工作空间根目录执行。若 SDK 不在系统默认路径，编译时补充 `-DTENSORRT_ROOT=/path/to/TensorRT`，并配置该 SDK 的运行库搜索路径。

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DAUBO_ENABLE_TENSORRT_YOLO=ON
source devel/setup.bash
roslaunch aubo_perception tensorrt_yolo.launch \
  engine_path:="$HOME/models/GC-yolo-fp16.engine" \
  image_topic:=/camera/color/image_raw
```

节点发布 `/yolo/detections`，消息为仓库已有的 `aubo_perception/YoloDetectionArray`，保留相机帧的时间戳和坐标系。框中心和宽高是原图像素，`angle` 与已有 Python 节点使用相同的图像弧度约定，`orientation_valid` 为 `true`。默认使用现有 `ultralytics_yolo.yaml` 的置信度、旋转 NMS 阈值、最大检测数及推理频率。更换权重时，应通过配置文件的 `class_names` 指定训练时的类别顺序。

在已启动工作相机后，可以用现有 RGB-D 投影链联调，此 launch 会负责启动唯一的 YOLO 进程。

```bash
roslaunch aubo_mobile_perception yolo_rgbd_detector.launch \
  camera_name:=workspace_camera engine_path:="$HOME/models/GC-yolo-fp16.engine"
rostopic echo -n 1 /yolo/detections
```

移动分拣入口 `sorting.launch` 和 `sorting_gazebo.launch` 支持 `perception_mode:=yolo yolo_engine:=/absolute/path/to/GC-yolo-fp16.engine`。

## 眼在手外实机分拣

```text
D435i 彩色图像 /camera/color/image_raw
  → TensorRT GC-YOLO：letterbox → GPU 推理 → 旋转框解码与 NMS
  → /perception/boxes（原图像素、角度、类别、图像采集时间戳）
  → grasp_geometry（对齐深度、相机内参、固定相机 TF）
  → /sorting/detections（base_link 下的三维实例）
  → 分拣队列和抓放任务
```

实机入口通过 `yolo_engine` 选择 TensorRT，`yolo_gpu_device` 默认为 `0`。省略 engine 参数会加载现有 PyTorch 权重。两个眼在手外 YOLO 入口使用同一工位配置。

```bash
roslaunch aubo_sorting eye_to_hand_can_sorting_real.launch \
  yolo_engine:="$HOME/models/GC-yolo-fp16.engine" \
  auto_move_to_observation:=false auto_start:=false
```

在另一终端查看二维框和 RGB-D 三维结果。侧边画面使用 `/sorting/debug_image`。首次联调可加 `start_task:=false`，先核对检测和坐标。

```bash
rostopic echo -n 1 /perception/boxes
rostopic echo -n 1 /sorting/detections
rostopic hz /sorting/detections
```

若已有固定相机和 TF，可单独运行感知流水线。

```bash
roslaunch aubo_perception grasp_perception.launch detector:=yolo \
  camera_mount:=eye_to_hand camera_namespace:=/camera height_mode:=depth \
  config:="$(rospack find aubo_sorting)/config/eye_to_hand_can_perception_real.yaml" \
  table_z:=0.00285 object_height:=0.065 \
  yolo_engine:="$HOME/models/GC-yolo-fp16.engine"
```

## 复现加速对比

在独立 ROS 会话运行 `roscore`。用同一权重及其导出的两种 engine，准备一批工位 RGB 图像。程序逐个启动 PyTorch CPU、PyTorch CUDA 和所提供的 TensorRT engine，每次只发送一张图，等带相同时间戳的检测结果返回后再发下一张。

```bash
rosrun aubo_perception benchmark_gc_yolo.py \
  --images /path/to/recorded_rgb_frames \
  --model-path /path/to/GC-yolo.pt \
  --python /home/zlab/anaconda3/envs/yolo/bin/python \
  --fp32-engine "$HOME/models/GC-yolo-fp32.engine" \
  --fp16-engine "$HOME/models/GC-yolo-fp16.engine" \
  --warmup 20 --repeats 3 \
  --output "$HOME/models/benchmark-gc-yolo/report.json"
```

脚本默认使用 batch 1、640×640 模型输入和同一份 YOLO 阈值配置。脚本关闭推理限频、标注图和 PyTorch TF32。TensorRT 使用 engine 构建时选择的精度，上文的 `--noTF32` 命令关闭 TF32。带自定义模型代码的权重可通过 `--config` 指定含 `project_path` 的 YOLO YAML。若要复现生产 PyTorch 的矩形补边，可复制配置并加上 `rect: true`，此时 TensorRT 仍用 640×640。JSON 保存每帧耗时、源图和模型 SHA-256、检测结果、P50/P95 和加速倍数。同类别的旋转框以 IoU ≥ 0.5 逐对匹配，记录未匹配框、中心偏移和置信度变化。长轴角差只统计长宽比大于 1.1 的旋转框。

ROS 计时从发布 RGB 消息到收到二维框，包含 ROS 传输、预处理、推理和后处理。串行 FPS 用 `1000 ÷ 平均耗时（ms）` 换算。生产配置默认将检测频率限制在 10 Hz；三维投影、队列确认、轨迹规划和机械臂运动应按各自阶段计时。

使用 `trtexec` 可再看 engine 的 GPU Compute Time。这里关闭主机与 GPU 的传输，以观察两种 TensorRT 精度的模型计算差异。 [NVIDIA `trtexec` 性能指标](https://docs.nvidia.com/deeplearning/tensorrt/10.x.x/performance/benchmarking.html)

```bash
for precision in fp32 fp16; do
  trtexec --loadEngine="$HOME/models/GC-yolo-${precision}.engine" \
    --noDataTransfers --warmUp=500 --duration=5 --percentile=50,95
done
```

## RTX 5060 Ti 实测（2026-11-21）

AMD Ryzen 7 9700X、RTX 5060 Ti、NVIDIA 驱动 580.82.09、CUDA 12.8、TensorRT 10.9.0 与 PyTorch 2.9.1+cu128。输入为 30 张工位录制图（640×480）和 30 张验证集图（640×640）。每个后端先预热 20 帧，再将 60 张图各处理 3 次。下表统一使用 640×640 模型输入。

| ROS 二维检测后端 | 平均延迟 / ms | P50 / ms | P95 / ms | 串行 FPS | 相对 PyTorch CPU 的平均加速 |
| --- | ---: | ---: | ---: | ---: | ---: |
| PyTorch CPU FP32 | 23.79 | 22.03 | 27.99 | 42.0 | 1.00× |
| PyTorch CUDA FP32 | 8.18 | 8.22 | 9.19 | 122.2 | 2.91× |
| TensorRT FP32 | 3.16 | 3.18 | 3.42 | 316.1 | 7.52× |
| TensorRT FP16 | 2.34 | 2.31 | 2.63 | 426.5 | 10.15× |

- **生产基线。** 用 PyTorch 的矩形补边单独复测，CPU 平均延迟 20.77 ms，TensorRT FP16 为 2.35 ms，同批回放加速 8.84×。统一输入尺寸时，TensorRT FP16 相对同 GPU 的 PyTorch CUDA 加速 3.49×，相对 TensorRT FP32 加速 1.35×。
- **GPU 计算。** `trtexec` 测得 FP32/FP16 的平均 GPU 计算时间为 1.78/0.97 ms，P95 为 1.95/1.13 ms。FP16 纯 GPU 计算加速 1.85×，包含传输和预、后处理的 ROS 路径加速 1.35×。FP32/FP16 engine 分别为 19.02/12.16 MiB。
- **检测一致性。** 统一输入尺寸时，FP16 与 PyTorch CPU 的 564 个框全部按同类别配对，平均旋转 IoU 为 0.9965，中心偏移 P95 为 0.090 px，长轴角差 P95 为 0.058°。有一张验证图中的瓶子因两个相近分数的候选框在 NMS 中选择不同，长轴角差达到 12.08°，重复 3 次均出现。
- **实机阈值和三维定位。** 矩形补边对照中，FP16 在同一张工位图中多检出一个置信度为 0.323 的罐框，3 次回放均出现。按工位的 0.60 置信度阈值过滤后，两边 420 个框均按同类别配对。使用一帧录制的叠放场景 RGB-D、内参和固定 TF 回放时，FP16 流水线在 `base_link` 输出了 7 个有效三维目标。

实际分拣的默认检测限频为 10 Hz。整机抓放节拍还取决于深度同步、队列确认、轨迹规划和运动时间，需按实机任务单独计时。本机的 engine、逐帧 JSON、RGB-D 回放和 `trtexec` 日志在工作空间根目录的 `records/gc_yolo_tensorrt_20261121/`。
