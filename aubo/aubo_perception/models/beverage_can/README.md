# 饮料罐外观资源

供 `yolo_perception.world` 和 `yolo_sorting.world` 共用的可乐易拉罐网格与原始贴图。
资源来自本机 Gazebo 模型库 `coke_can`（model.config 作者 John Hsu / OSRF；DAE 贡献者 VCGLab，MeshLab 导出）。保留原始 UV 和贴图，无需运行时下载或依赖用户的 `~/.gazebo` 缓存。

网格已转换为米制、Z 轴为罐体轴线、几何中心为原点。原始顶点包围盒为
`[-38.01, 29.0] × [-38.94, 28.08] × [182.5, 306.4]`（毫米）。
XY 按最大径向距离等比缩放到半径 0.025 m，Z 缩放到总长 0.10 m，法向量使用逆转置变换并归一化。

两个场景仍使用原来的圆柱碰撞体、质量、惯量、横放位置和 `beverage_can::can_link` 名称。
外观资源跟随 aubo_perception 的 models 目录安装；Gazebo ROS 使用 package 模型路径解析资源，贴图通过 DAE 的相对路径加载。

修改后需要重启 Gazebo 场景。识别效果需要用实际相机图像和当前 YOLO 权重验证。
