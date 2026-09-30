# launch 文件说明

这个目录下主要提供了 4 个与手眼标定相关的启动文件，分别用于“标定”和“发布标定结果”。

## 1. `calibrate.launch`

用于执行手眼标定的主流程。它会根据机器人末端的 `tf` 和 ArUco 标记的 `tf` 采样数据，并计算出相机与机器人之间的固定外参。

适用场景：

- 重新做一次手眼标定
- 更换相机安装位置后重新计算外参
- 更换标记板、机器人末端工具或坐标系定义后重新标定

## 2. `publish.launch`

用于加载已经保存好的标定结果，并将结果持续发布到 `tf` 中。

它本身不负责采样或计算，只负责把已有的标定 YAML 重新读出来并发布出去。

适用场景：

- 系统启动时自动恢复手眼外参
- 标定完成后让 MoveIt、视觉算法或碰撞检测直接使用结果

## 3. `aubo_realsense_calibration.launch`

这是面向 Aubo + Realsense + ArUco 的标定入口文件，主要负责把整套标定环境拉起来，包括：

- 启动 Realsense 相机
- 启动 ArUco 识别节点
- 传入机器人基座、末端、相机和 marker 的 frame 名称
- 进入 `easy_handeye` 的标定界面

这个文件更像“项目级标定配置入口”，你只需要在这里改 frame 名称、marker 编号、marker 尺寸等参数。

## 4. `aubo_realsense_publish_only.launch`

这是面向 Aubo + Realsense 的“只发布结果”入口文件。

它的作用是：

- 读取之前保存好的标定结果
- 发布对应的手眼外参到 `tf`
- 不再重新采样，不再重新计算标定

适用场景：

- 日常启动系统时直接加载已有标定结果
- 不需要重新标定，只想让视觉和机器人继续使用同一套外参

## 推荐流程

1. 先运行 `aubo_realsense_calibration.launch` 做标定
2. 标定成功后保存结果
3. 日常使用时运行 `aubo_realsense_publish_only.launch`，直接发布已有结果

## 说明

- 文件名里 `realsense` 的拼写建议统一，不要混用 `realsence`。
- 如果机器人末端、相机安装方式或 marker 位置发生变化，应重新运行标定流程。