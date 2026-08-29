# AUBO 移动机器人统一启动入口

该包只负责组合启动，不包含模型、控制算法、地图或分拣参数。仿真统一使用：

```bash
roslaunch aubo_mobile_bringup simulation.launch mode:=sorting
```

`mode` 可选：

- `robot`：只启动模型、Gazebo 和控制器；
- `navigation`：启动训练场景、定位导航和 RViz；
- `exploration`：启动 GMapping、move_base 和 RRT 自主探索建图；
- `sorting`：启动工作台视觉分拣；
- `mission`：启动导航到工位并分拣的完整任务。

可继续传入 `gui`、`rviz`、`debug_view`、`paused` 和 `auto_start`。原有各功能包
launch 文件继续保留，便于单独调试；日常使用优先从本包进入。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `mode` | `sorting` | `robot`、`navigation`、`exploration`、`sorting` 或 `mission` |
| `gui` | `true` | 是否显示 Gazebo 界面 |
| `rviz` | `true` | 是否启动对应 RViz 配置 |
| `debug_view` | `false` | 是否显示视觉调试图 |
| `paused` | `false` | Gazebo 是否暂停启动 |
| `auto_start` | `false` | 分拣或完整任务是否自动开始 |

例如无界面运行完整任务：

```bash
roslaunch aubo_mobile_bringup simulation.launch \
  mode:=mission gui:=false rviz:=false debug_view:=false auto_start:=true
```

启动 RRT 自主探索建图：

```bash
roslaunch aubo_mobile_bringup simulation.launch mode:=exploration
```

地图出现后，在 RViz 中使用 **Publish Point** 依次发布四个探索边界角点和一个
机器人附近的 RRT 起点。探索结束后使用
`roslaunch aubo_mobile_navigation map_saver.launch` 保存地图。

## 与原入口的关系

统一入口内部仍调用以下功能包 launch，因此这些命令没有被删除：

- `aubo_mobile_robot/gazebo.launch`；
- `aubo_mobile_navigation/navigation_gazebo.launch`；
- `aubo_mobile_navigation/rrt_exploration_gazebo.launch`；
- `aubo_mobile_sorting/sorting_gazebo.launch`；
- `aubo_mobile_nav_sorting/mission_gazebo.launch`。

手动建图入口仍使用 `aubo_mobile_navigation/mapping_gazebo.launch` 或
`aubo_mobile_nav_sorting/mapping_gazebo.launch`；自主探索建图使用独立的
`exploration` 模式，避免与加载已有地图的定位导航混淆。
