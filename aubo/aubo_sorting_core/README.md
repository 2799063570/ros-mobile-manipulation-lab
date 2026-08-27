# AUBO 通用分拣核心

该包只维护一份 MoveIt 抓取分拣状态机。固定底座和移动底盘的控制器名称、桌面
坐标、观察姿态、放置区域及 Gazebo 抓取辅助开关都由各场景 YAML 参数传入。

请从 `aubo_color_sorting` 或 `aubo_mobile_sorting` 的 launch 文件启动，不要直接
启动核心脚本。
