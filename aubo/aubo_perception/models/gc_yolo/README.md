# GC-YOLO TensorRT · ROS Noetic

`GC-yolo.onnx` 来自 `weights/GC-yolo.pt`。输入为 `images`，尺寸 `1×3×640×640`，输出为 `output0`，尺寸 `1×8×8400`。类别 ID `0/1/2` 对应 `bottle/can/box`，输出末通道为弧度角。静态 FP32 ONNX 已通过 ONNX Runtime 与 PyTorch 的数值校验。

## 在机器人目标机生成 engine

首先确认 NVIDIA 驱动、CUDA 和 TensorRT 10.x C++ SDK 适配目标 GPU。进入这个目录，用目标机的 `trtexec` 构建 engine。FP16 只用于 TensorRT 内部计算，节点需要线性 FP32 输入和输出。

```bash
mkdir -p "$HOME/models"
trtexec --onnx=GC-yolo.onnx --saveEngine="$HOME/models/GC-yolo-fp32.engine" --noTF32
trtexec --onnx=GC-yolo.onnx --saveEngine="$HOME/models/GC-yolo-fp16.engine" --fp16
trtexec --loadEngine="$HOME/models/GC-yolo-fp16.engine"
```

TensorRT 11.x 已移除 `--fp16`，构建混合精度 engine 前要先用 ModelOpt AutoCast 转换，并确保输入和输出仍为线性 FP32。 [NVIDIA TensorRT 11 迁移指南](https://docs.nvidia.com/deeplearning/tensorrt/latest/api/migration/tensorrt-10x-to-11x-trtexec.html)

## 编译和联调

在包含 WheelTec 源码的 catkin 工作空间根目录执行。若 SDK 不在系统默认路径，编译时补充 `-DTENSORRT_ROOT=/path/to/TensorRT`，并配置该 SDK 的运行库搜索路径。

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DAUBO_ENABLE_TENSORRT_YOLO=ON
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

移动分拣入口 `sorting.launch` 和 `sorting_gazebo.launch` 支持 `perception_mode:=yolo yolo_engine:=/absolute/path/to/GC-yolo-fp16.engine`。建议先检查检测话题，再用同一批现场图像对照 FP32/FP16 的类别、旋转框和角度。
