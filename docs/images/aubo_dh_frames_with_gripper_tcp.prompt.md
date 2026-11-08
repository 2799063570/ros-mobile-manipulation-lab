# AUBO机械臂与夹爪DH坐标系位置图

使用内置image_gen生成。日期：2026-11-04。

依据：文档第6.7.2节标准DH参数；arm.xacro；jiazhua.urdf；aubo_i5_with_camera.xacro。

在q=0的理想化模型下计算坐标原点及三轴方向；夹爪TCP沿wrist3局部+Z偏移0.12m。结构示意非比例，DH坐标系不可作为URDF各link坐标系直接使用。

## 初始提示词

Create a NEW high resolution Chinese scientific educational robot-coordinate-location illustration, using the attached AUBO robot drawing for mechanical body structure. This is NOT a formula infographic. The user's primary request is to SEE WHERE EVERY COORDINATE FRAME LIES ON THE ARM, including the actual two-finger gripper and its TCP. Main subject occupies most of the canvas. White background, black/light-gray engineering line robot, precise vivid coordinate triads, red x axes, green y axes, blue z axes, orange TCP. Excellent readable Chinese labels. Landscape 3:2 or square composition, large main robot on left two-thirds, enlarged wrist/gripper inset on upper right, compact exact origin table lower right. No DH parameter table or big algebra.

Title exact “AUBO机械臂与夹爪｜DH坐标系位置”
Subtitle “理想化零位 q₁=…=q₆=0；以 base_link 为基准”
Use 3D oblique engineering projection where base +X projects diagonally down-left, +Y projects horizontally right, +Z vertically up. Source mechanical form: J1 base at bottom right, J2 lower left shoulder, long first link rising from left shoulder to middle-left elbow, second vertical link on right rising from elbow to upper-right wrist1, wrist2 is on upper-left, wrist3 extends horizontally left; ADD an open parallel two-finger gripper bolted onto the far-left output flange, its fingers extending further left along tool Z. TCP orange point is BETWEEN the two fingertips in the useful contact band, NOT on either finger. No camera needed.

Accurate DH coordinate rig MUST be overlaid on the robot, not floating without anchoring. Solid black or orange dots at every origin O0 O1 O2 O3 O4 O5 O6 OTCP, triad arrows emerge precisely from these dots, label x0/y0/z0 etc by each arrow. These are the document's STANDARD DH frames, NOT original reference's red frame numbers. Here are exact origin positions in meters, base coords X,Y,Z:
{0} O0=(0,0,0) at base_link origin.
{1} O1=(0,0,0.122) vertically above O0 at shoulder-axis height on base axis.
{2} O2=(0,0,0.530) DIRECTLY VERTICALLY ABOVE O1, at elbow height ON THE EXTENSION OF ELBOW ROTATION AXIS toward the right, NOT necessarily elbow casing center.
{3} O3=(0,0,0.906) DIRECTLY VERTICALLY ABOVE O2 on right wrist1 axis.
{4} O4=(0,-0.1215,0.906) LEFT of O3, SAME HEIGHT; at wrist2 vertical axis base.
{5} O5=(0,-0.1215,1.0085) DIRECTLY ABOVE O4 at intersection of wrist2 axis and wrist3 axis.
{6} O6=(0,-0.2155,1.0085) LEFT of O5, SAME HEIGHT; at flange/gripper mounting plane.
{TCP} OTCP=(0,-0.3355,1.0085) LEFT of O6, SAME HEIGHT; between gripper fingertips.
Use dotted axis-extension lines from O2 through elbow casing center, and from O3 toward O4 to make origin placements intelligible. Body robot has axis offset like reference, while DH O2 may fall on extension line. DO NOT move O2 onto the elbow body if it breaks these coordinates.

EXACT AXIS DIRECTIONS for zero position, given as global base direction and projected direction:
Frame0: x0=+X down-left, y0=+Y right, z0=+Z up.
Frame1: x1=+X down-left, y1=+Z up, z1=-Y left.
Frame2: x2=+Z up, y2=+X down-left, z2=+Y right.
Frame3: x3=+Z up, y3=-X up-right, z3=-Y left.
Frame4: x4=-X up-right, y4=-Y left, z4=+Z up.
Frame5: x5=+X down-left, y5=+Z up, z5=-Y left.
Frame6: x6=+X down-left, y6=+Z up, z6=-Y left.
FrameTCP has SAME orientation as frame6: xTCP down-left red, yTCP up green, zTCP left blue.
Use short arrows and clear labels to avoid triad overlap. Render all 8 origins and all triads, can keep 4/5/6 small on main view and clearly repeat/enlarge in inset.

Wrist inset heading “腕部与夹爪TCP放大”.
Show top wrist modules and actual two opposing fingers extending left. Enlarge O4,O5,O6,OTCP and their correct triads. Show horizontal blue z6/zTCP direction left, orange dimension double arrow from O6 to OTCP marked “120 mm · 沿局部 +Z”. Connect O6 labels to callout “{6} = wrist3_Link = gripper_base_link”. Connect orange point between fingers to “TCP = gripper_link = tcp_link”. Small line “两者方向相同，仅沿末轴平移”.
On main view dimension a2=408 mm vertically from O1 height to O2, a3=376 mm from O2 height to O3. Source base height98.5/27 and other dimensions must NOT be copied.
Compact lower right origin table title “DH原点在基坐标中的位置 / m” with columns “坐标系 | X | Y | Z”, exact rows:
“{0} | 0 | 0 | 0”
“{1} | 0 | 0 | 0.122”
“{2} | 0 | 0 | 0.530”
“{3} | 0 | 0 | 0.906”
“{4} | 0 | −0.1215 | 0.906”
“{5} | 0 | −0.1215 | 1.0085”
“{6} | 0 | −0.2155 | 1.0085”
“TCP | 0 | −0.3355 | 1.0085”.
Footer exact: “标准DH坐标系 ≠ URDF各link坐标系；DH原点可位于关节轴延长线上。结构示意，非比例；TCP采用仓库120 mm工具定义。”
Small color legend “X 红 · Y 绿 · Z 蓝 · TCP 橙”.
This must look like a precise physical frame-location diagram with gripper, not an equation/flowchart layout. Ensure no extra frames and no incorrect coincident origins.

## 坐标轴与TCP修订

Edit the attached robotics coordinate diagram. Preserve mechanical robot body, gripper shape, Chinese title, tables, other triads and all labels. Make the following targeted technically essential corrections with precision, not interpretation:

1. Main robot frame {2} at black dot O2 near middle elbow: DELETE the existing blue upward z2 arrow. Draw a RED upward arrow from O2 and label x₂ in RED. Keep its GREEN down-left y₂ arrow. Add a BLUE horizontally RIGHTWARD arrow from O2 labelled z₂ in BLUE. Final directions: x2 up, y2 down-left, z2 right. All three meet at O2.

2. Main robot frame {3} at black dot O3 near upper-right wrist: DELETE blue upward z3 arrow and RED down-left x3 arrow. Draw a RED upward arrow from O3, labelled x₃ in RED. Keep the GREEN upper-right y3 arrow. Draw a BLUE horizontally LEFTWARD arrow from O3 labelled z₃ in BLUE. Final directions: x3 up, y3 upper-right, z3 left. All three meet at O3.

3. TCP must be a SINGLE shared origin. In BOTH the main gripper drawing and the top-right enlarged inset, the orange TCP dot is currently to the left of the triad's black common origin dot. Move the WHOLE TCP triad origin onto the orange TCP dot between the fingers. The GREEN yTCP upward arrow, RED xTCP down-left arrow and BLUE zTCP leftward arrow must all START EXACTLY at the orange dot, with no second black origin dot anywhere between the fingers. Orange leader still targets this same dot. Preserve the TCP orange dot's current position, all other origin points remain fixed. In the inset orange120mm dimension starts below this same TCP point and ends below O6, unchanged.

4. Origin O0 at base should be precisely vertically aligned below O1,O2,O3, since their X,Y base coordinates are all zero. Shift ONLY O0 dot and its entire triad slightly left if needed to have O0 directly under the existing O1/O2/O3 dashed vertical axis. Base z0 blue arrow must point straight up, y0 green right, x0 red down-left.

Do NOT change numerical tables, mechanical geometry, other labels, or triad directions. Re-check corrected triads against the left orientation table, which is already CORRECT. Focus on scientific correctness and clearly visible arrow origins.

## 最终标记修订

Make ONLY two tiny surgical corrections to this otherwise finished engineering illustration. Preserve all robot geometry, all 8 coordinate triads and their directions, all text, table, colors, layout and mathematical values.
A. In the MAIN ROBOT at the top-left gripper there are currently TWO orange dots: the correct LARGE orange dot at the common origin of red xTCP, green yTCP and blue zTCP arrows, and a smaller stray orange dot a little to the LEFT and DOWN of it, attached to the orange leader from the TCP/O_TCP label. ERASE the SMALLER stray orange dot ONLY and reconnect the orange leader from the TCP label directly to the LARGE orange triad origin. The three coordinate arrows remain at the current large dot; do not move the TCP origin or invent any extra dot. Thus there is EXACTLY ONE orange dot between the fingers.
B. In the RIGHT ENLARGED INSET the horizontal orange 120 mm dimension's LEFT extension line currently starts noticeably to the left of the orange TCP dot. Shift the LEFT endpoint of that horizontal dimension and its vertical extension line to be EXACTLY VERTICALLY BELOW the actual orange TCP origin dot. Keep the RIGHT endpoint precisely below the black O6 dot. The dimension then measures from TCP triad origin to O6 triad origin. Keep label “120 mm · 沿局部 +Z” unchanged.
Do not change anything else.

