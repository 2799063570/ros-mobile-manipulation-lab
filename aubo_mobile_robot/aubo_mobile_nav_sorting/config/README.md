# 导航分拣配置说明

本目录保存单桌与四工作台任务的导航、感知和机械臂参数。配置只负责提供参数，
实际感知算法位于 `aubo_perception`，机械臂分拣逻辑位于 `aubo_sorting_core`，
导航与工位任务逻辑位于 `aubo_mobile_nav_sorting`。

## 文件用途

| 文件 | 用途 | 主要加载位置 |
| --- | --- | --- |
| [scenario.yaml](scenario.yaml) | 单桌导航任务、近场候选停靠、任务超时与返回起点 | `mission.launch` 的默认任务配置，加载到 `/nav_sorting_mission` |
| [four_tables.yaml](four_tables.yaml) | 四桌颜色任务：工位位置、放置点、精靠与前移恢复 | `four_tables_gazebo.launch perception_mode:=color`，加载到 `/nav_sorting_mission` |
| [four_tables_yolo.yaml](four_tables_yolo.yaml) | 四桌 YOLO 任务：将 bottle/can/box 对应到工位与仿真实体 | `four_tables_gazebo.launch perception_mode:=yolo`，加载到 `/nav_sorting_mission` |
| [four_tables_colors.yaml](four_tables_colors.yaml) | HSV、轮廓过滤、深度顶面定位、感知工作区 | 四桌颜色模式经移动感知入口加载到 `/color_object_detector` |
| [four_tables_navigation.yaml](four_tables_navigation.yaml) | 全局规划频率，以及 TEB/DWA 速度、容差和避障参数 | 四桌入口的导航配置，加载到 `/move_base` |
| [sorting.yaml](sorting.yaml) | 机械臂观察、抓取、放置、夹爪、检测采样与缓存 | 四桌入口及 `mission_gazebo.launch`，加载到 `/color_sorting_task` |

`four_tables_yolo.yaml` 不是 YOLO 网络配置。模型权重和 RGB-D 适配参数由
`aubo_mobile_perception/launch/yolo_rgbd_detector.launch` 及其引用的共享感知入口提供。
YOLO 模式还会在本目录 `sorting.yaml` 之后加载
`aubo_mobile_sorting/config/yolo_sorting.yaml`，其中同名机械臂参数会覆盖基础值。
默认四桌世界仍包含彩色方块；类别名称映射本身不会将方块变成瓶子或易拉罐。

## 启动和加载顺序

```bash
# 默认四桌颜色模式，使用 TEB，等待面板或服务启动任务。
roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch

# 对照 DWA 导航。
roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch local_planner:=dwa

# 使用 YOLO 类别任务配置，需匹配的模型、相机输入及物体外观。
roslaunch aubo_mobile_nav_sorting four_tables_gazebo.launch perception_mode:=yolo

# 单桌导航分拣仿真。
roslaunch aubo_mobile_nav_sorting mission_gazebo.launch
```

四桌入口根据 `perception_mode` 选择任务文件，可通过 `mission_config:=绝对路径`
替换任务配置；该参数不会同时替换机械臂、颜色检测或导航配置。
启动文件先加载 YAML，再设置部分显式参数，例如 `auto_start`，最终值以运行时参数为准。
切换工位时，任务会调用 `/sorting/configure_workspace` 更新桌体、放置点和实体名称，
因此 `sorting.yaml` 中的单桌默认位置不是四桌全过程使用的固定位置。

## 坐标、单位和高度

- 位姿 `[x, y, yaw]`：位置单位为米，航向单位为弧度；三维位置为 `[x, y, z]`。
- `pre_dock_goal` 和 `navigation_goal`：分别为预停点和最终工作点，使用 `navigation_goal_frame`。
- `table_center` 和 `table_size`：桌体中心、长宽高，使用 `table_frame`。
- `place_targets`：各类别的放置 XY，使用 `place_frame`。
- 四桌仿真固定场景使用 `odom`，其与 Gazebo 世界重合；导航和返回起点可使用 `map`。
- `table_z`、感知工作区和抓取目标使用 `base_link`，不要直接填入桌面世界高度。
- `projection_plane_z = table_z + object_height` 是预期方块顶面高度；
  `object_center_z = table_z + object_height / 2` 是发布的物块中心高度。
- `pregrasp_height`、`lift_height`、`preplace_height` 是相对桌面的高度；
  `grasp_height_offset` 则是相对物块中心的抓取高度补偿。

当前桌面在世界中高约 0.45 m，在 `base_link` 下为 0.14 m；
4 cm 方块的顶面和中心分别为 0.18 m、0.16 m。
更改桌高或物高时，应同步核对感知配置、机械臂配置和工位配置。

## 停靠与失败恢复

TEB/DWA 预停点位置容差当前为 0.01 m，任务最终精靠的平面位置容差暂放宽为 0.07 m，
最终航向容差为 0.04 rad，用于容纳目前的停靠残差；这不会消除实际位置误差。
预抓取恢复的 3 cm 前移仍使用独立的 0.01 m 到位容差，避免被宽容差直接跳过。
精靠控制器通过小幅转向持续修正航向及横向误差，并在接近目标时减速。
差速底盘不能主动横移，预停点的侧向误差过大仍可能导致精靠失败。
`near_field_direct_dock_lateral_tolerance` 是允许的侧向误差上限，
不能将它当作“侧向自动修正量”。航向对正、无进展超时和总超时分别由对应参数控制。
控制器使用 heading_speed 限制角速度，并将目标转向偏置限制在航向走廊内。
激光安全层对不超过 0.04 m/s、0.12 rad/s 的行驶纠偏检查反应与制动期间的
保守矩形扫掠包围盒；原地转向及更快转向仍使用原旋转安全距离。
越界、无进展和超时日志包含对应失败原因；无进展时同时检查安全层是否拦停。

两套四桌任务均启用 `pregrasp_forward_recovery_enabled`。
仅预抓取位姿规划失败、操作已结束且底盘解锁时，执行：

```text
收臂至 transport → 按实际位姿闭环前移 → 重新观察和检测 → 重试未完成物块
```

当前代码限定每次目标前移 0.03 m、最多两次，底盘中心到轴对齐桌边至少 0.35 m，
且前移必须使机器人更接近桌子。这三个限制目前在任务代码中定义，并非 YAML 参数。
执行失败、抓取/放置阶段失败或停止请求不会触发该重试；收臂失败也禁止移动底盘。
这是一种有限的可达性重试，不会自动区分所有逆解失败和碰撞原因。

`base_recovery_enabled: false` 和 `base_recovery_steps: []` 关闭的是旧版通用搜索，
与上述独立的前移恢复开关不同。`post_sort_retreat_enabled` 则控制每桌成功分拣后
收臂并后退离桌，当前后退目标距离为 0.30 m。

## 深度定位与检测

`four_tables_colors.yaml` 开启 `use_depth` 和 `require_depth`，使用腕部对齐深度：
`/hand_camera/aligned_depth_to_color/image_raw`。相机话题由移动感知 launch 设置。
共享感知节点反投影轮廓内的深度点，只保留预期顶面附近的点来计算 XY 中心，
发布的 Z 仍采用配置的物块中心高度。

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `max_depth_age` | 0.06 s | 彩色图与深度图允许的最大时间差 |
| `top_surface_tolerance` | 0.004 m | 相对预期顶面的高度筛选范围 |
| `min_top_surface_points` | 30 | 计算中心所需的最少有效顶面点数 |
| `top_surface_percentile` | 5.0 | 用 5% 和 95% 分位边界的中点估计中心 |
| `detection_samples`（sorting.yaml） | 8 | 抓取前的位置平均采样数 |
| `target_cache_fallback_enabled`（sorting.yaml） | false | 禁止以旧缓存代替新检测 |

HSV 色相范围为 0～179，饱和度和明度范围为 0～255；轮廓面积单位为平方像素。
`max_contour_area` 用于排除较大的放置色块，改变相机分辨率后应重新核对面积阈值。
深度无效或过期时跳过目标，不退回纯 RGB 质心定位。
TF 等待上限 `tf_wait_timeout` 默认 0.2 s，来自共享感知节点默认值，本目录未显式设置。
调试标签 `red-D`、`green-D`、`blue-D` 表示使用了深度结果。

## 生效与排查

修改 YAML 后通常需要重启对应 launch/节点；直接修改文件不会更新运行中的参数。
任务支持的动态参数只能在空闲时调整，并非所有字段都支持动态重配置。
只修改配置和注释不需要重新编译；相机传感器类型或 URDF 变化需要重启仿真并重新生成机器人。

```bash
rosparam get /nav_sorting_mission/near_field_direct_dock_goal_tolerance
rosparam get /nav_sorting_mission/pregrasp_forward_recovery_enabled
rosparam get /color_object_detector/require_depth
rostopic hz /hand_camera/aligned_depth_to_color/image_raw
rostopic echo -n 1 /sorting/detection_summary
rostopic echo -n 1 /sorting/failure
rostopic echo -n 1 /nav_sorting/state
```

`green pre-grasp` 表示到绿块上方的位姿规划失败；`green grasp` 才是直线下降阶段。
颜色计数为零时先检查相机、深度和 TF，规划失败时再检查实际停靠距离、目标位姿及碰撞。
完整任务说明见 [上级 README](../README.md)。
