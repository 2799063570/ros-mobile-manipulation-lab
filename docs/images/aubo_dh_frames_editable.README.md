# AUBO DH坐标系：可编辑矢量图

- `aubo_dh_frames_editable.svg`：可编辑源文件。机械臂、夹爪、箭头、原点和文字均为矢量对象，没有嵌入位图。
- `aubo_dh_frames_editable_preview.png`：同一SVG的预览，不能作为可编辑源文件使用。
- `../../tools/draw_aubo_dh_frames.py`：生成脚本，只依赖Python标准库。运行 `python tools/draw_aubo_dh_frames.py` 重新生成SVG；会覆盖同名SVG，手工修改前请另存副本。

在Inkscape或Illustrator中打开SVG，展开对象面板或取消组合，即可分别修改轮廓、颜色、箭头与文字。文字保留为文本，未转成路径。

图层：`Background`、`Robot and gripper`、`Axis extensions`、`DH coordinate frames`。各机械部件有独立分组；坐标系按 `frame-0` 至 `frame-6` 和 `frame-TCP` 分组，每根轴还有独立分组。

数据约定：采用文档第6.7.2节的标准DH手算近似，六轴q=0，{0}与base_link重合；长度从arm.xacro和jiazhua.urdf读取。第6坐标系到TCP沿局部+Z偏移120 mm。图形轮廓是简化结构示意，不是机械加工图；DH坐标系也不是URDF全部link坐标系。原点位置与轴方向由矩阵连乘生成，不靠手工拖动估计。
