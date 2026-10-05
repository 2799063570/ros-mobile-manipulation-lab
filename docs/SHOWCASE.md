# AUBO 复合机器人演示指南

## 一条命令启动

在 Ubuntu 的 Melodic/Noetic 工作空间完成 README 中的依赖安装、编译和环境加载后：

```bash
source src/setup_ros.sh
roslaunch aubo_mobile_bringup showcase.launch scenario:=nav_sorting
```

该入口用于 **Gazebo 仿真展示**。所有模式使用颜色检测和仓库已有世界文件，无需下载
YOLO 权重；不连接实机、不会启动 OpenVLA，也不下载模型。启动前退出其他演示的
roslaunch，避免重复的 Gazebo、控制器、TF 和任务服务。

| 场景 | 展示内容 | 启动后的操作 | RViz 固定坐标系 |
| --- | --- | --- | --- |
| `perception` | 俯视 RGB 相机、颜色检测框、对齐深度三维定位 | 等待相机图像；不启动运动任务 | `odom` |
| `sorting` | 腕部 RGB-D 识别、MoveIt 规划、抓取及分类放置 | 等待初始化，面板进入观察位后开始分拣 | `odom` |
| `nav_sorting` | 四工位地图导航、精靠、观察、分拣及返回 | 等待初始化，使用导航分拣面板开始完整任务 | `map` |

切换场景前使用 `Ctrl+C` 退出当前 launch，再执行对应命令：

```bash
roslaunch aubo_mobile_bringup showcase.launch scenario:=perception
roslaunch aubo_mobile_bringup showcase.launch scenario:=sorting
roslaunch aubo_mobile_bringup showcase.launch scenario:=nav_sorting
```

## 启动参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `scenario` | `nav_sorting` | 只接受上述三个场景；拼写错误直接报错 |
| `gui` | `true` | 显示 Gazebo 场景与物体；关闭后仍运行仿真 |
| `rviz` | `true` | 显示一个统一 RViz 窗口 |
| `paused` | `false` | 是否暂停启动；暂停时图像、控制器和初始化可能等待仿真时间 |
| `auto_start` | `false` | 默认由操作员开始；开启后 sorting 自动进入观察位并分拣，nav_sorting 自动执行总任务；perception 忽略此参数 |
| `rviz_config` | 随场景选择 | 可替换为自己的 RViz 配置绝对路径 |

例如仅展示 RViz、隐藏 Gazebo 窗口：

```bash
roslaunch aubo_mobile_bringup showcase.launch scenario:=nav_sorting gui:=false
```

无人值守仿真（不适合讲解时逐步演示）：

```bash
roslaunch aubo_mobile_bringup showcase.launch \
  scenario:=nav_sorting gui:=false rviz:=false auto_start:=true
```

此入口固定使用颜色检测。需要 YOLO 时，继续使用原有 `sorting_gazebo.launch` 或
`four_tables_gazebo.launch`，按各功能包说明配置模型环境；展示入口不假设个人路径可用。

## 建议的讲解流程

以下是演示顺序，不保证完整四工位任务在 3～5 分钟内完成；初始化与运动时间取决于
硬件性能、实时因子和规划结果。

1. **感知片段**：启动 `perception`。在 Gazebo 介绍三色方块和分类区；在 RViz 对比
   Camera RGB 与 Detection overlay。说明颜色轮廓结合对齐深度得到机器人坐标中的
   目标位置，不能把检测框直接当成机械臂运动指令。
2. **抓取片段**：退出后启动 `sorting`。面板默认等待操作；点击“1. 移动到相机观察位”，待运动
   完成、检测结果稳定后点击“开始分拣”。观察 MoveIt 场景和规划轨迹、机械臂动作及
   识别结果。面板提供实际分拣状态和检测摘要。
3. **导航片段**：退出后启动 `nav_sorting`。待地图、机器人定位和控制器就绪，点击
   “开始导航分拣”。介绍绿色全局路径、橙色 TEB 局部路径、工位进度与任务状态。
   腕部相机在导航过程中不一定看到目标；到工位进入观察姿态后再讲解识别与抓取。
4. **结束**：完整任务按现有四工位配置执行并返回起点。若时间有限，用面板停止任务，
   等待停止确认后结束演示；不要将中途停止描述成任务完成。

Gazebo 显示真实仿真桌面、方块和放置动作；RViz 显示机器人、规划场景、地图与图像。
RViz 不会自动渲染 Gazebo 世界中的全部实体，因此演示时推荐同时保留两个窗口。

## 开始、停止与重置

优先使用当前场景面板。没有 RViz 时，可在另一个已加载环境的终端调用已有服务：

```bash
# sorting：先进入观察位，等待 /sorting/state 返回就绪，再开始。
rosservice call /sorting/move_to_observation "{}"
rostopic echo /sorting/state
# 上一条是持续订阅，Ctrl+C 退出订阅后执行下一条。
rosservice call /sorting/start "{}"
rosservice call /sorting/stop "{}"

# nav_sorting：由总任务管理导航和机械臂，不要同时单独触发 /sorting/start。
rosservice call /nav_sorting/start "{}"
rosservice call /nav_sorting/stop "{}"
```

`perception` 不启动以上任务服务。停止命令是异步任务取消，应观察面板/状态话题确认
已经停止；软件停止服务不能替代实机急停。

**可靠恢复初始场景**：先停止任务并等待确认，`Ctrl+C` 退出 launch，等待所有节点退出，
再使用同一命令启动。重新创建 Gazebo 世界会恢复方块位置，同时重建任务队列、状态和
定位。不要在任务运行中直接调用 `/gazebo/reset_world` 或 `/gazebo/reset_simulation`：
这些服务不会同步清理任务状态、MoveIt 场景或 TF 时间缓存。

## 展示布局说明

`aubo_mobile_bringup/rviz/showcase_*.rviz` 保存初始视角和 1440×900 窗口尺寸：

- 感知：机器人、俯视原图、标注结果，不加载运动控制面板。
- 分拣：机器人、MoveIt 规划场景/轨迹、腕部原图、标注结果、已有分拣面板。
- 多工位：地图、机器人、融合激光、Navfn/TEB 路径、腕部原图、标注结果、已有总任务面板。

不加载旧配置中的 RTAB-Map 插件，避免额外插件依赖；不提供直接给 move_base 发目标的
RViz 工具，避免演示过程中绕过总任务控制。没有写入与某台机器屏幕绑定的 Qt 窗口状态
二进制，首次打开时可以拖动图像/面板停靠位置；当前版本固定的是内容、视角和窗口尺寸。

## 常见问题

| 现象 | 检查方式 |
| --- | --- |
| 启动后没有图像或初始化不结束 | 确认 Gazebo 未暂停，`rostopic hz /clock` 有输出；首次加载需等待控制器和 TF 就绪 |
| perception 没有检测结果 | `rostopic hz /workspace_camera/color/image_raw`，再检查相机对齐深度与日志中的 TF 错误；不依赖腕部相机姿态 |
| sorting 图像中没有方块 | 点击进入观察位；检查 `/hand_camera/image_raw` 和 `/sorting/debug_image` |
| RViz 面板或 MotionPlanning 加载失败 | 重新编译并 source 工作空间；确认 `aubo_mobile_sorting`、`aubo_mobile_nav_sorting` 和 `moveit_ros_visualization` 可发现 |
| 地图正常但机器人/路径不显示 | 检查 map 到 odom 的定位 TF、`/scan` 和 move_base 日志；不要通过改变 Fixed Frame 掩盖定位故障 |
| 重复节点、服务冲突、物体位置未恢复 | 完整退出上一轮 Gazebo/launch，再重新启动；同一 ROS master 只运行一套演示 |
| 多工位任务停止未确认 | 查看 `/nav_sorting/state` 和原任务包的停止恢复说明；不要直接重新开始 |

## 验证

新增测试通过 ROS 的真实 launch 解析器展开三个场景，检查单一 RViz、相机话题、任务
启停参数、无界面模式，以及未知场景报错；不启动仿真或发送运动命令。

```bash
# 工作空间根目录；安装测试依赖、加载环境后执行。
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin_make run_tests_aubo_mobile_bringup
catkin_test_results --verbose build/test_results

# 无启动的节点展开检查。
roslaunch --nodes aubo_mobile_bringup showcase.launch scenario:=perception
roslaunch --nodes aubo_mobile_bringup showcase.launch scenario:=sorting
roslaunch --nodes aubo_mobile_bringup showcase.launch scenario:=nav_sorting
```

静态解析不能替代 Gazebo 验收。正式展示前应分别运行三个场景，确认面板可见、图像有
内容、分拣完成、多工位导航完成并返回，以及停止后重启确实恢复物体位置。
