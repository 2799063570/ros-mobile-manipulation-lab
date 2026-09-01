# teb_local_planner ROS 功能包

`teb_local_planner` 是 ROS 二维导航栈中 `base_local_planner` 的插件。其核心方法为时间弹性带（Timed Elastic Band，TEB），运行时会综合轨迹执行时间、障碍物距离和运动学/动力学约束，对机器人局部轨迹进行优化。

使用说明和教程见 [ROS Wiki](http://wiki.ros.org/teb_local_planner)。

Melodic 分支构建状态：[![Melodic 构建状态](http://build.ros.org/buildStatus/icon?job=Mdev__teb_local_planner__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__teb_local_planner__ubuntu_bionic_amd64/)

## 学术引用

如果在研究工作中使用该规划器，请至少引用以下论文之一：

- C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, 2012, pp. 74–79.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Efficient trajectory optimization using a sparse model. Proc. IEEE European Conference on Mobile Robots, 2013, pp. 138–143.
- C. Rösmann, F. Hoffmann and T. Bertram: Planning of Multiple Robot Trajectories in Distinctive Topologies, Proc. IEEE European Conference on Mobile Robots, 2015.
- C. Rösmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory Optimization and Control for Car-Like Robots, IEEE/RSJ IROS, 2017.

## 演示视频

左侧视频介绍功能包特性以及仿真和真实机器人示例，右侧视频演示 0.2 版新增的类汽车机器人和代价地图转换功能。建议先观看左侧视频。

<a href="http://www.youtube.com/watch?feature=player_embedded&v=e1Bw6JOgHME" target="_blank"><img src="http://img.youtube.com/vi/e1Bw6JOgHME/0.jpg" alt="移动机器人最优轨迹规划器" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=o5wnRCzdUMo" target="_blank"><img src="http://img.youtube.com/vi/o5wnRCzdUMo/0.jpg" alt="类汽车机器人与代价地图转换" width="240" height="180" border="10" /></a>

## 许可证

`teb_local_planner` 采用 BSD 许可证。依赖的 ROS 功能包列在 `package.xml` 中，也采用 BSD 许可证。第三方依赖采用各自许可证：Eigen 为 MPL-2.0，g2o 为 BSD，其 `csparse_extension` 为 LGPL-3.0+，Boost 使用 Boost Software License。详细条款以各项目附带的许可证文件为准。

## 安装依赖

```bash
rosdep install teb_local_planner
```
