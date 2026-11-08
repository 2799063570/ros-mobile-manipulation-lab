# AUBO DH坐标系论文简洁版

生成方式：内置image_gen；日期：2026-11-04。

单台机械臂、夹爪及DH坐标系，零位示意。轴方向依据文档标准DH近似模型，结构图非比例，精确几何由URDF定义。

## 初始提示词

Use case: scientific-educational. Edit the provided technical figure into a clean, minimal academic paper illustration.
KEEP ONLY ONE COMPLETE AUBO SIX-AXIS ROBOT WITH ITS TWO-FINGER GRIPPER AND COORDINATE TRIADS. The main robot in the left of the source is the subject to retain and enlarge. Remove all surrounding content: absolutely NO title, subtitle, table, orientation-list, wrist inset, second robot, dimension arrows, dimension numbers, legend, footer, panels, boxes, explanatory prose, or decorative elements. Portrait canvas, high resolution, pure white opaque background, generous but balanced margin around a single centered robot. Robot is gray/black fine engineering line art with very light gray shading, restrained detail, no glossy rendering and no cast shadows. Coordinate arrows are thin and sharp, x muted red, y green, z blue. Mathematical labels italic serif, consistent small size, clear subscripts. Black origin dots small and consistent; TCP dot orange. No label bounding boxes.

Preserve the source robot mechanical shape and its zero-angle pose: bottom-right base, bottom-left shoulder, long vertical first link on left to elbow, second link vertical on right to high wrist, top wrist extends toward left, two fingers at far top-left. Keep the GRIPPER and a single TCP orange origin BETWEEN its fingertips. No extra tools or camera.

There must be EXACTLY EIGHT coordinate triads: frames 0,1,2,3,4,5,6 and TCP. Every triad has THREE labelled arrows x_i, y_i, z_i, starting at exactly one common point. Use small unboxed O₀,O₁,O₂,O₃,O₄,O₅,O₆ next to black dots; the orange TCP dot can simply carry small “TCP”. Label arrows x_TCP, y_TCP,z_TCP in subscript style, no prose. No duplicate TCP dot.

CRITICAL SCIENTIFIC RIG -- preserve positions and axis directions:
Projection: global +X diagonal down-left, +Y horizontal right, +Z vertical up.
O0: base_link origin on vertical base axis at bottom.
O1: above O0 by122 mm, at shoulder rotation axis level.
O2: above O1 by408 mm, at elbow level on extension of elbow's horizontal rotation axis. O2 is NOT necessarily at elbow casing center. 
O3: above O2 by376 mm, at upper right wrist1 axis.
O0,O1,O2,O3 must all lie on ONE EXACT VERTICAL LINE.
O4: horizontal LEFT of O3 by121.5 mm, at wrist2 axis base, SAME HEIGHT as O3.
O5: vertical ABOVE O4 by102.5 mm at wrist2/wrist3 intersection.
O6: horizontal LEFT of O5 by94 mm, flange/gripper mounting plane.
TCP: horizontal LEFT of O6 by120 mm, at midpoint BETWEEN the two open fingers.
O5,O6,TCP must be at SAME HEIGHT. Dimensions above are construction instructions ONLY; never print them. Keep subtle short gray dashed rotation-axis extensions where needed to locate O2/O3; no dashed decoration.

Arrow directions, MANDATORY and unchanged:
frame0: x0 red down-left, y0 green right, z0 blue UP.
frame1: x1 red down-left, y1 green UP, z1 blue LEFT.
frame2: x2 red UP, y2 green down-left, z2 blue RIGHT.
frame3: x3 red UP, y3 green up-right, z3 blue LEFT.
frame4: x4 red up-right, y4 green LEFT, z4 blue UP.
frame5: x5 red down-left, y5 green UP, z5 blue LEFT.
frame6: x6 red down-left, y6 green UP, z6 blue LEFT.
TCP: xTCP red down-left, yTCP green UP, zTCP blue LEFT.
All labels must agree with arrows. Keep frame2 and3 particularly correct: their upward arrows are RED X axes, not blue Z axes.
Frame1,5,6,TCP have identical arrow orientations.
Keep upper wrist triads compact with no text collisions; separate origin positions physically, do not move individual triads off their actual origins just for label placement.
An elegant restrained journal figure, just robot geometry and coordinate labels. No text other than the coordinate and origin labels described.

## 原点标签修订

Edit the attached minimal academic robot illustration. Make ONE text correction only, preserving every pixel of mechanical shape, all coordinate arrows, axis labels, origin dot positions, white background, layout and typography otherwise.
At the TOP WRIST there are currently TWO origin labels O₅. The LEFT black dot under the y₆ green axis and at the intersection of x₆ red and z₆ blue axes is the SIXTH frame origin. Its current label “O₅” must be replaced by “O₆” in the same italic black font and same location. The RIGHT wrist dot under y₅ correctly labelled O₅ MUST stay O₅. All other labels stay unchanged. Final upper origins from left to right are TCP, O₆, O₅; lower are O₄,O₃,O₂,O₁,O₀.
Do not move anything, do not add any title, prose, table, inset or dimensions. Just change that one incorrect O₅ to O₆.

