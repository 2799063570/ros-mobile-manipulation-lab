# 仓库清理审计

审计日期：2026-10-05

## 本次执行范围（2026-10-05）

本文件下方保留清理前的审计数据；本节记录用户确认的实际执行范围。

- 迁出 `qt_ros_test/`、`xf_mic_asr_offline/`、`xf_mic_asr_offline_circle/`、
  `bodyreader/`、`tts_make/` 和 `ros_astra_camera/`，共约 359.97 MB。
- 这六个目录先分别生成 ZIP 归档，逐文件比较数量、大小和 SHA-256 后，再移除原目录。
  归档位于本次任务的仓库外附件目录 `wheeltec-legacy-archives/`，包含完整原始内容，
  也包含旧目录中的生成物。归档只保存在当前机器，没有上传到远端。
- 删除当前树中的已跟踪 `.pyc`、日志、临时 PCM/语音归档及 `.tmp_queue_test` 编译产物。
- 删除 `fdilink_ahrs/data/gps_data.bag` 和 `lsx10/lslidar_driver/pcap/1.txt`。
- 保留 Navigation、RealSense、Karto、TEB、depthimage_to_laserscan 源码快照，
  也保留 `navigation-melodic/amcl.zip`。`ros_tensorflow` 除 Python 缓存外保持原样。
- 演示视频、AUBO SDK 和活动源码保持原样；未改写 Git 历史。

恢复 legacy 目录时，将对应 ZIP 解压到仓库根目录（ZIP 已包含目录名）。
删除的已跟踪生成物和测试数据也可从清理前的 Git 提交恢复。

## 清理前审计

## 结论

- 12 个包含 `CATKIN_IGNORE` 的目录合计约 712 MB、2,864 个已跟踪文件。
- 其中约 709 MB 是 Git 当前版本中的文件；两种统计的少量差异来自工作树中的子仓库/未跟踪内容。
- 这些目录已不进入当前 catkin 构建图。活动代码对 Navigation、TEB、Karto、
  `depthimage_to_laserscan` 和 RealSense 的引用是 ROS 包名引用，当前设计要求由
  rosdep/apt 提供，而不是使用仓库中的旧源码快照。
- `ros_tensorflow`、旧 Astra/BodyReader、语音、TTS 和 Qt 工具没有被活动运行路径依赖，
  可以优先迁出。
- 活动区域仍有少量大型资产和构建产物，不能与 legacy 目录一起无条件删除，需要分别处理。

## 1. 当前版本的清理候选

### A. 可直接清理的生成物

执行删除前仍应单独提交并检查 diff。

| 类别 | 数量 | 约占空间 | 说明 |
| --- | ---: | ---: | --- |
| Python bytecode | 97 | 0.88 MB | 全部位于已隔离目录；源码可重新生成 |
| 日志 | 17 | 27.32 MB | 全部位于已隔离目录；包括重复的 `msc.log` |
| 临时 PCM | 4 | 34.96 MB | 全部位于已隔离目录；两个 17.48 MB 的 `all.pcm` 内容相同 |
| 临时归档 | 3 | 15.41 MB | 全部位于已隔离目录；包括重复的 `system.tar` |
| 本机构建产物 | 9 | 约 1.75 MB | `.tmp_queue_test/*.exe`、`*.obj` 及旧语音 sample 的 `*.o` |

建议同步补充 `.gitignore`：

```gitignore
*.o
*.obj
*.exe
*.log
.tmp_queue_test/
```

仓库已经忽略 `*.py[cod]` 和 `__pycache__/`，但历史上已经跟踪的文件仍需通过
`git rm --cached` 或删除提交解除跟踪。

### B. 应迁到外部资产存储的候选

| 文件/类别 | 约占空间 | 建议 |
| --- | ---: | --- |
| `fdilink_ahrs/data/gps_data.bag` | 61.97 MB | 当前活动代码没有文本引用；确认不是回归测试输入后，迁到 Release/测试数据存储 |
| `aubo/video_or_img/sorting_process.mp4` | 22.70 MB | README 只把它作为高清演示；迁到 Release，仓库保留 3.9 MB 预览版 |
| `lsx10/lslidar_driver/pcap/1.txt` | 6.31 MB | 只在 launch 的注释中出现；迁到测试数据存储或删除 |

### C. 暂时保留的预编译库

活动区域中约 111.54 MB 的预编译库主要来自 `aubo/aubo_sdk`。当前 CMake 会直接链接
这些库，因此在引入可复现的 SDK 下载/校验机制前不能删除。

同一个 AUBO 库的多个 `.so` 文件内容相同，但它们模拟 Linux SONAME 符号链接。应先在
Linux 中确认实际文件类型和 SDK 装载行为，再决定是否用真正的符号链接替代，不能在
Windows checkout 中直接去重。

## 2. `CATKIN_IGNORE` 目录评估

### 第一批：可以优先迁出

这些目录没有被活动运行路径依赖。少数命中仅来自兼容性文档或旧使用说明。

| 目录 | 已跟踪文件 | 当前约占空间 | 处理建议 |
| --- | ---: | ---: | --- |
| `ros_tensorflow/` | 468 | 307.82 MB | 迁到 legacy 仓库；其中约 291 MB 是模型/检查点 |
| `qt_ros_test/` | 1,022 | 100.13 MB | 迁到 legacy 仓库；包含重复包、旧 Python 2 代码和无法解析的 launch XML |
| `xf_mic_asr_offline_circle/` | 165 | 67.03 MB | 与另一语音目录有大量重复二进制和运行数据，迁出 |
| `xf_mic_asr_offline/` | 340 | 64.57 MB | 迁到 legacy/厂商资产包 |
| `bodyreader/` | 143 | 59.60 MB | 当前只在文档中出现，迁到 legacy/厂商资产包 |
| `tts_make/` | 25 | 58.79 MB | 当前只在旧说明中出现，迁到 legacy/厂商资产包 |
| `ros_astra_camera/` | 151 | 9.85 MB | 实机支持已切换到 RealSense；Gazebo 中名为 `astra_camera` 的 xacro 宏不依赖此驱动目录 |

第一批合计约 667.79 MB。

### 第二批：由系统包替代，验证后迁出

这些旧源码目录自身已被隔离，但活动 launch/config 仍引用对应 ROS 包名。活动
`package.xml` 已声明主要运行依赖，预期由 `rosdep install` 和 apt 提供。

| 目录 | 已跟踪文件 | 当前约占空间 | 外部替代 |
| --- | ---: | ---: | --- |
| `realsense-ros-development/` | 89 | 35.99 MB | `realsense2_camera`、`realsense2_description` 系统包 |
| `navigation-melodic/` | 310 | 2.92 MB | ROS Navigation、AMCL、move_base、map_server 等系统包 |
| `slam_karto/` | 63 | 1.75 MB | `slam_karto` 系统包 |
| `teb_local_planner-melodic-devel/` | 69 | 0.74 MB | `teb_local_planner` 系统包 |
| `depthimage_to_laserscan-melodic-devel/` | 19 | 0.05 MB | `depthimage_to_laserscan` 系统包 |

第二批迁出前必须在不包含这些目录的干净工作空间中验证：

1. Melodic、Noetic 分别运行 `rosdep install --from-paths src --ignore-src -r -y`。
2. 分别完成全量 `catkin_make`。
3. 对建图、导航、RealSense、Karto、TEB 和深度图转激光的 launch 做静态展开检查。
4. 至少进行一次无硬件冒烟启动，确认插件和运行时包均由系统环境解析。

## 3. 重复和大文件证据

当前版本中的主要重复项包括：

- 两份完全相同的 17.48 MB `all.pcm`；
- 两份完全相同的 9.33 MB 语音资源 `common.jet`；
- 两份完全相同的 8.39 MB `msc.log`；
- 两份完全相同的 7.67 MB `system.tar`；
- 三份完全相同的 x64 `libmsc.so`；
- Qt 旧工程中多组完全相同的 STL 模型。

最大的单个当前文件包括：

| 文件 | 约占空间 | 状态 |
| --- | ---: | --- |
| `ros_tensorflow/.../ssd_inception_v2.pb` | 97.74 MB | 已隔离 |
| `ros_tensorflow/.../classify_image_graph_def.pb` | 91.24 MB | 已隔离 |
| `fdilink_ahrs/data/gps_data.bag` | 61.97 MB | 活动目录，未发现引用 |
| `bodyreader/.../libOrbbecBodyTracking.so` | 47.75 MB | 已隔离 |
| TensorFlow MNIST checkpoint（两份） | 各 37.10 MB | 已隔离 |

## 4. 推荐提交拆分

为便于回滚和审查，后续实际清理建议拆成独立提交：

1. `chore: ignore local build and runtime artifacts`
2. `chore: remove tracked generated files from legacy packages`
3. `chore: move unsupported perception and voice stacks to legacy storage`
4. `chore: replace vendored ROS stacks with rosdep dependencies`
5. `docs: point large demonstrations and test data to release assets`

以上只处理当前树。是否使用 `git filter-repo` 清除历史大对象，应在所有协作者完成分支
同步之后另行决策；它会改写提交哈希，不应与上述普通清理提交同时执行。
