# aubo_sdk

该功能包引入了 [`aubo_perception_planning/aubo_sdk`](https://github.com/2799063570/aubo_perception_planning/tree/main/aubo_sdk) 在提交 `e7d7558cb907f4a6cef632e302d612b7157172ec` 时的头文件、控制器库和运行配置。上游资源保持不变，仅针对当前工作空间调整了 Catkin 元数据和演示程序。

## 测试

连接测试为只读操作：它只登录控制器并输出诊断信息、关节位置和 TCP 位姿，不会启动或移动机械臂。

```bash
rosrun aubo_sdk sdk_test 192.168.1.2
```

运动演示带有强制保护，必须显式传入 `--execute`，并给出六个以弧度为单位的关节目标：

```bash
rosrun aubo_sdk joint_move_demo --execute 192.168.1.2 q1 q2 q3 q4 q5 q6
```

执行运动前必须检查工作区、示教器模式和急停按钮状态。
