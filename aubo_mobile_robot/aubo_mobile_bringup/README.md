# AUBO 移动机器人统一启动入口

仿真统一使用：

```bash
roslaunch aubo_mobile_bringup simulation.launch mode:=sorting
```

`mode` 可选：

- `robot`：只启动模型、Gazebo 和控制器；
- `navigation`：启动训练场景、定位导航和 RViz；
- `sorting`：启动工作台视觉分拣；
- `mission`：启动导航到工位并分拣的完整任务。

可继续传入 `gui`、`rviz`、`debug_view`、`paused` 和 `auto_start`。原有各功能包
launch 文件继续保留，便于单独调试；日常使用优先从本包进入。
