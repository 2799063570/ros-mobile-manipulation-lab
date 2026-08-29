# 项目演示视频与图片

该目录集中保存 AUBO 机械臂及复合移动机器人演示素材。算法、模型和启动配置仍放在
各自ROS功能包中，文档通过相对链接引用这里的媒体文件。

## 分拣流程

[▶ 在线预览 `preview.mp4`（约 3.9 MB）](preview.mp4)

[下载高清版本 `sorting_process.mp4`（约 22.7 MB）](sorting_process.mp4)

内容包括目标识别、机械臂运动规划、抓取以及分类放置流程。GitHub若未直接显示
播放器，点击链接后可在浏览器中打开或下载观看。`preview.mp4`用于README和网页
快速预览，`sorting_process.mp4`保留较高画质。

README展示图统一生成到 `readme/`：画布为960×600，原图按比例居中并使用浅色留白，
因此不同截图不会被拉伸或裁掉。新增或替换图片后，在仓库根目录运行：

```powershell
.\tools\create_readme_thumbnails.ps1
```

## 图片索引

| 文件 | 展示内容 |
| --- | --- |
| [`simple_diff_gazebo.png`](simple_diff_gazebo.png) | 差速机器人及Gazebo迷宫环境 |
| [`mapping.png`](mapping.png) | 双雷达建图结果 |
| [`mapping_nav.png`](mapping_nav.png) | 地图、机器人定位和导航轨迹 |
| [`rrt_exploration_view.png`](rrt_exploration_view.png) | RRT/frontier自主探索过程 |
| [`opencv_detector.png`](opencv_detector.png) | 红、绿、蓝目标识别和机器人坐标定位 |
| [`object_detector.png`](object_detector.png) | YOLO OBB目标检测和抓取参数输出 |
| [`octmap.png`](octmap.png) | MoveIt PlanningScene中的OctoMap环境建模 |
| [`sorting_control.png`](sorting_control.png) | RViz分拣控制面板和MoveIt交互 |
| [`grasp_gazebo.png`](grasp_gazebo.png) | Gazebo中的机械臂抓取过程 |

## 素材维护约定

- 文件名使用小写英文和下划线，例如 `octomap_validation.mp4`；
- 新增视频时在本文件补充用途、运行环境和对应启动命令；
- 优先保留压缩后的 MP4，避免提交无损录屏和临时导出文件；
- 真机视频需注明是否启用了轨迹执行、碰撞检查和急停保护。
