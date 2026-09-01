# ros_tensorflow

## 简介

该功能包演示如何把 TensorFlow 和目标检测 API 集成到 ROS，并提供 MNIST、图像分类和目标检测示例。

## 环境要求

- Ubuntu 16.04、Python 2.7
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) 与 Catkin 工作空间
- [TensorFlow](https://www.tensorflow.org/install/) 1.2.0～1.10.0
- NVIDIA TK1 如需 GPU 加速，可安装 [CUDA 6.5](https://gist.github.com/jetsonhacks/6da905e0675dcb5cba6f)
- 其他依赖：

  ```bash
  sudo apt-get install protobuf-compiler python-pil python-lxml
  sudo pip install jupyter matplotlib
  ```

## 获取源码与编译

```bash
cd <CATKIN_WS>/src
git clone https://github.com/cong/ros_tensorflow.git
cd <CATKIN_WS>
catkin_make
```

Python 脚本本身通常不需要单独编译。

## 运行示例

先启动 ROS Master 和 USB 相机（需预先安装 `usb_cam`）：

```bash
roscore
roslaunch usb_cam usb_cam-test.launch
```

MNIST 识别：

```bash
roslaunch ros_tensorflow ros_tensorflow_mnist.launch
rostopic echo /result_ripe
```

图像分类：

```bash
roslaunch ros_tensorflow ros_tensorflow_classify.launch
rostopic echo /result_ripe
```

目标检测：

```bash
roslaunch ros_tensorflow ros_tensorflow_detect.launch
rosrun image_view image_view image:=/result_ripe
```

## ROS 话题

- 输出结果：`/result_ripe`
- 输入图像：`usb_cam/image_raw`

如需使用自己的模型和标签，可替换 `ros_tensorflow/include/` 中的对应文件。更多信息见[原作者博客](http://wangcong.net)。
