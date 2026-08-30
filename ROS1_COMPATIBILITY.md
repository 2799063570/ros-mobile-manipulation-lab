# ROS 1 Melodic / Noetic 兼容说明

## 支持边界

本仓库维护同一套 ROS 1 源码，目标环境为：

- Ubuntu 18.04 + ROS Melodic + Python 3
- Ubuntu 20.04 + ROS Noetic + Python 3

顶层 `CMakeLists.txt` 使用仓库内的 `cmake/wheeltec_toplevel.cmake`，根据已加载的
`ROS_DISTRO` 或 `WHEELTEC_ROS_DISTRO` 选择发行版，不能提交指向某台机器
`/opt/ros/noetic/...` 的绝对软链接。

## 构建

在工作空间的 `src` 上一级执行：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source src/setup_ros.sh
```

切换发行版时不得复用 `build/` 和 `devel/`。推荐为 Melodic、Noetic 各建一个工作
空间，或者在切换前清理这两个生成目录。

编译前应执行 `conda deactivate`（必要时打开未加载 Conda 的新终端）。Conda 自带
protobuf CMake 配置会与 Gazebo/ignition 的系统 protobuf targets 冲突。

Ubuntu 18.04 的 ROS Melodic 二进制 `cv_bridge` 通常面向 Python 2。需要运行
`simple_follower`、KCF、RRT 图像检测或 AUBO Python 视觉节点时，应在独立 overlay
工作空间中以 Python 3 编译匹配 Melodic 的 `vision_opencv/cv_bridge`；仅给本仓库的
`catkin_make` 传 `PYTHON_EXECUTABLE` 不能改变系统中已经编译好的 Python 扩展。

仓库提供了已验证的构建脚本：

```bash
python3 -m pip install --user 'catkin_pkg<1' 'rospkg<2' 'empy==3.3.4' defusedxml pyyaml
bash src/tools/build_melodic_python3_cv_bridge.sh
source src/setup_ros.sh
```

`setup_ros.sh` 会在 Melodic 下自动加载工作空间根目录中的
`melodic_py3_cv_bridge_ws`；也可用 `WHEELTEC_MELODIC_PY3_OVERLAY` 指定其他位置。

### AUBO 厂商 SDK 的 protobuf 2.6 ABI

仓库内的 AUBO 预编译 SDK 固定依赖 `libprotobuf.so.9`。Ubuntu 18.04/20.04 的当前
protobuf 不能通过软链接冒充该 ABI。若系统尚无 `libprotobuf9v5`，从 Ubuntu 官方
归档安装兼容包：

```bash
curl -L --fail -o /tmp/libprotobuf9v5_2.6.1-1.3_amd64.deb \
  http://archive.ubuntu.com/ubuntu/pool/main/p/protobuf/libprotobuf9v5_2.6.1-1.3_amd64.deb
echo '8178472ae72a3242f4e1c78f38bfd137c5aadadcefc1980aee2ba9820be1d192  /tmp/libprotobuf9v5_2.6.1-1.3_amd64.deb' | sha256sum -c -
sudo apt install /tmp/libprotobuf9v5_2.6.1-1.3_amd64.deb
```

`aubo_sdk` 会在 CMake 配置阶段检查该 SONAME，缺失时直接给出明确错误。

## 系统依赖替代 vendor 源码

以下目录已放置 `CATKIN_IGNORE`，不会进入当前支持的 catkin 构建图：

- `navigation-melodic/`
- `teb_local_planner-melodic-devel/`
- `depthimage_to_laserscan-melodic-devel/`
- `realsense-ros-development/`
- `slam_karto/`
- `ros_astra_camera/`
- `ros_tensorflow/`
- `bodyreader/`
- `xf_mic_asr_offline/`、`xf_mic_asr_offline_circle/`、`tts_make/`
- `qt_ros_test/`

前五项由 rosdep/apt 提供对应发行版的软件包。RealSense D435i 是唯一支持的物理
RGB-D 相机，`turn_on_wheeltec_robot/launch/wheeltec_camera.launch` 直接启动
`realsense2_camera`。旧 Astra、USB 相机和语音/TensorFlow 示例不属于支持范围。

这些目录目前只做可逆隔离，尚未物理删除。确认没有未提交资产后，可以在单独提交
中删除，以便清楚审查第三方代码移除和仓库体积变化。

## Python 3 规则

- 活跃包的可执行 Python 脚本使用 `#!/usr/bin/env python3`。
- manifest 使用 `python3-numpy`、`python3-opencv`、`python3-sklearn` 等 rosdep key。
- 不按 ROS 发行版维护 Python 2/Python 3 两套分支。
- 旧第三方节点只有在 Python 3 语法检查和实际节点测试通过后才视为受支持。

`darknet_ros` 不再作为仓库子模块。`aubo_perception` 的 Darknet 消息适配器保留为
可选兼容入口，缺少 `darknet_ros_msgs` 时会给出明确错误；颜色/RGB-D 感知和其余
构建不再被该旧后端阻塞。`wheeltec_yolo_action` 中直接导入
`darknet_ros_msgs` 的旧节点仍属于 legacy，需要后续迁移到实际选定的 Python 3
检测后端后再做运行验证。

## 已完成的兼容实验

| 检查项 | Ubuntu 20.04 / Noetic | Ubuntu 18.04 / Melodic 容器 |
| --- | --- | --- |
| 活动包数量 | 41 | 41 |
| rosdep key 可解析 | 通过 | 通过 |
| Python 3 全量 `catkin_make` | 100% 通过 | 100% 通过 |
| OpenCV | 4.2，KCF 通过 | 3.2，KCF 通过 |
| Python 3 `rospy` | 通过 | 通过 |
| Python 3 `cv_bridge` 图像往返 | 系统包通过 | 官方 melodic 源码 overlay 通过 |
| WheelTec GMapping launch 展开 | 通过 | 通过 |
| RealSense launch 展开 | 通过 | 通过 |

Noetic 还执行了 12 秒真实 roslaunch 冒烟测试：ROS master、底盘节点、TF、
joint/robot state publisher 和 robot_pose_ekf 均持续运行；使用不存在的测试串口时，
底盘驱动按配置重连且未崩溃。

尚未完成的是需要实物的验收：RealSense D435i 图像/深度对齐、具体雷达串口、WheelTec
控制器通信以及 AUBO 真机运动。无硬件的编译和冒烟实验不能替代这些安全验收。

仓库中的 `lsn10`、`lsn10p` 是没有 `.gitmodules` 配置的空 gitlink，当前 checkout
不会得到源码；如硬件确实使用这两个驱动，需要补齐可复现的子模块地址或将经过
验证的源码正式纳入仓库。
