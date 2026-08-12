---
tags: [跟随, 巡线, 应用]
---

# simple_follower

## 定位

提供巡线、激光跟随、视觉跟随和 AR 标签跟随，是学习“感知结果转换成底盘控制”的最佳应用包。

## 启动入口

```bash
roslaunch simple_follower line_follower.launch
roslaunch simple_follower laser_follower.launch
roslaunch simple_follower visual_follower.launch
roslaunch simple_follower ar_follower.launch
```

## 关键源码

- `scripts/line_follow.py`
- `scripts/laserTracker.py`
- `scripts/laser_follow.py`
- `scripts/visualTracker.py`
- `scripts/visual_follow.py`
- `scripts/ar_follow.py`
- `src/avoidance.cpp`

## 阅读问题

1. 传感器原始数据由哪个节点接收？
2. 目标位置使用什么消息传递？
3. 距离误差和方向误差怎样转换为速度？
4. 是否限制最大速度？
5. 目标消失或障碍物过近时怎样停车？

关联：[[分类/04-跟随与自主行为]]、[[04-系统数据流]]。

