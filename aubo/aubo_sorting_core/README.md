# AUBO 通用分拣核心

该包只维护一份 MoveIt 抓取分拣状态机。固定底座和移动底盘的控制器名称、桌面
坐标、观察姿态、放置区域及 Gazebo 抓取辅助开关都由各场景 YAML 参数传入。

请从 `aubo_color_sorting` 或 `aubo_mobile_sorting` 的 launch 文件启动，不要直接
启动核心脚本。

## 输入与输出

- 订阅 `/sorting/detections`，类型为
  `aubo_perception/DetectedObjectArray`；
- 连接 MoveIt 的 `aubo_i5` 规划组和平台配置的夹爪轨迹 action；
- 发布 `/sorting/state` 与 `/sorting/detection_summary`；
- 发布 `/sorting/failure`；无法生成非空 MoveIt 轨迹时以
  `PLANNING_FAILED | <目标>` 明确上报，执行器失败不会混为规划失败；
- 提供 `/sorting/move_to_observation`、`/sorting/prepare_work`、
  `/sorting/start`、`/sorting/stop`、`/sorting/open_gripper`、`/sorting/home` 和
  `/sorting/configure_workspace`。

`prepare_work` 主要供移动机器人到达工位后的任务编排使用，固定平台通常不调用。
`configure_workspace` 在节点空闲时从 `/sorting/workspace_config` 读取当前桌子的
碰撞体、抓取高度和放置点。切换桌子会清空本桌完成颜色记录；同一桌规划失败后重试
则跳过已经完成的颜色，避免重复抓取。

## 参数边界

以下差异必须由场景 YAML 提供，不能写进核心包：

- `gripper_action`、规划组和末端链接；
- `table_frame`、桌面碰撞体和目标坐标系；
- 观察、运输、工作准备及结束姿态；
- 抓取高度、放置点、速度和加速度；
- 是否启用 Gazebo 抓取辅助插件。

固定平台加载 `aubo_color_sorting/config/sorting.yaml`，移动平台加载
`aubo_mobile_sorting/config/sorting.yaml`；导航分拣场景可用
`aubo_mobile_nav_sorting/config/sorting.yaml` 覆盖移动平台默认值。
