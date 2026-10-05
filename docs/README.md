# 文档导航

第一次了解项目，先阅读[项目首页](../README.md)，再按目的选择：

| 目的 | 文档 |
| --- | --- |
| 查每个功能包的职责、运行入口、前置条件 | [功能包总览与运行指南](FUNCTION_PACKAGES.md) |
| 展示感知、分拣、导航分拣，了解界面和任务操作 | [项目演示指南](SHOWCASE.md) |
| 了解保留 / 迁出的源码及生成物清理范围 | [仓库清理记录](repository_cleanup_audit.md) |
| 在实机上切换车型、雷达和相机 | [设备切换说明](车型_雷达_相机的切换.txt) |
| 查找历史 WheelTec 命令 | [ROS 常用功能命令](ROS常用功能命令5.5.txt)、[Common function command](Common%20function%20command-WHEELTEC-ROS5.5.txt) |
| 研究 OpenVLA 的隔离环境与模型服务 | [Gazebo 部署计划](OpenVLA_Ubuntu20.04_Gazebo部署计划.md)、[部署实测记录](OpenVLA_Ubuntu20.04_部署实测记录.md) |

历史命令文档可能涉及已迁出包、旧设备或开发机绝对路径；当前入口和运行边界以
[功能包指南](FUNCTION_PACKAGES.md)及对应包内 README 为准。
文档中的“完整入口”“组件入口”“实机入口”不能互换，执行前请先确认是否会启动运动。
