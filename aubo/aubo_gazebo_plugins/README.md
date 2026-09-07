# AUBO Gazebo 通用插件

`aubo_grasp_attach_plugin` 收到吸附请求后为末端与目标物体建立临时固定关节，并在
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

## 四桌场景的抓取外观优化

四桌世界将 `max_attach_distance` 从 0.18 m 收紧为 0.06 m，并设置
`max_lateral_offset=0.008` m，拒绝相对两指原点中点沿闭合轴偏移超过 8 mm 的物块。
未配置横向阈值时默认 0（关闭附加检查），其他场景保持兼容。启用后如果手指链接
缺失或闭合轴异常，插件拒绝吸附。状态 `error:object_off_center:<model>` 表示偏心，
日志包含实际误差，便于调整感知或 TCP，而非强行吸住物块。

当前任务先吸附再闭合，吸附期间仍关闭物块碰撞以避免固定约束与位置控制互相挤压。
导航分拣的 40 mm 方块配置将闭合角改为 0.20 rad；按实际 STL 与抓取高度带计算，
最小间隙约 42.45 mm，原 0.28 rad 只有 36.87 mm。该标定针对当前 40 mm 方块，
并不是物体宽度自适应或真实接触抓取；8 mm 门限只阻止明显偏心，也不保证所有
允许偏心下都无穿透。轻微重叠仍需依据实际偏心日志校准。

几何回归检查（无需 Gazebo）：

```bash
python3 src/aubo/aubo_gazebo_plugins/test/test_gripper_clearance.py
```

检查对 STL 三角形按物块高度区间裁切后计算间隙，防止以后改闭合角或抓取高度时
再次让理想居中的方块与手指重叠。修改世界参数和插件后需重启 Gazebo 才能生效。
