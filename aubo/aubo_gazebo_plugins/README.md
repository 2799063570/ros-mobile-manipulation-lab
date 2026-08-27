# AUBO Gazebo 通用插件

`aubo_grasp_attach_plugin` 在夹爪闭合后为末端与目标物体建立临时固定关节，并在
放置时解除。机器人模型、末端链接、目标链接、距离阈值和 ROS 话题全部通过世界
文件中的 SDF 参数配置，因此固定机械臂与移动机械臂共用同一实现。

## 世界文件配置

```xml
<plugin name="aubo_grasp_attach" filename="libaubo_grasp_attach_plugin.so">
  <robot_model>aubo_mobile_robot</robot_model>
  <palm_link>wrist3_Link</palm_link>
  <object_link>block_link</object_link>
  <max_attach_distance>0.18</max_attach_distance>
  <attach_topic>/sorting/grasp/attach</attach_topic>
  <detach_topic>/sorting/grasp/detach</detach_topic>
  <status_topic>/sorting/grasp/status</status_topic>
</plugin>
```

固定机械臂把 `robot_model` 设置为 `aubo_i5`，移动平台设置为
`aubo_mobile_robot`。插件收到目标模型名称后只在距离不超过阈值时创建固定关节，
状态通过 `/sorting/grasp/status` 发布。

该插件只用于 Gazebo 小物体接触稳定，不参与视觉定位和 MoveIt 规划，也不能用于
真实机械臂。修改插件后需要重新执行 `catkin_make --force-cmake` 并重新加载工作空间。
