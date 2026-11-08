# AUBO关节变换关系图生成记录

生成方式：内置 image_gen（imagegen skill），2026-11-04。

参考图：用户提供的AUBO机械臂结构图；结构参考原图，DH参数与公式依据本文档第6.7节。图片为教学示意，不作为比例图或坐标标定依据。

## 初始提示词

Use case: scientific-educational.
Create a high resolution landscape Chinese robotics textbook infographic based on the attached AUBO six-axis robot engineering drawing. Input image is a reference for robot structural shape ONLY. Create a new clean explanatory figure, not a photograph. White background, precise black/gray engineering line drawing, generous spacing, crisp Chinese typography and math. Title: “AUBO 六轴机械臂｜关节坐标变换”. Three clearly separated panels. Dark navy text, orange transformation arrows, x red / y green / z blue coordinate axes. This is a schematic, not a scaled manufacturing drawing.

CRITICAL: the source photo's red coordinate labels are NOT the DH frames used below. Do not copy those coordinate labels or source dimensions 98.5, 27, 71.5, 140.5,19. Use reconstructed standard DH frames consistent with our model. Footer verbatim: “结构参考原图；DH坐标系重新选取，与原图轴标号不直接等同。示意图非比例，长度单位：m。”

LEFT PANEL heading “① 机械结构与关节顺序”.
Draw a slender simplified vertical robot line drawing faithful to reference structure: base at bottom right, shoulder at bottom left, long first link rising to elbow on left, second link rises on right, two wrist modules on top, flange projects to top left. Avoid dense axis triads here. Six clear external leader labels from bottom to top: J1 基座旋转, J2 肩关节, J3 肘关节, J4 腕1, J5 腕2, J6 腕3. J1 on base joint, J2 on low left joint, J3 on middle left elbow, J4 upper right horizontal wrist elbow, J5 top central vertical wrist, J6 top left horizontal output cylinder. Bottom-left note “Jᵢ：第i个活动关节”. Only show link length dimensions “a₂=0.408” and “a₃=0.376”, no other structural dimensions needed.

MIDDLE PANEL heading “② 相邻DH坐标系的变换”.
At top show abstract two DH frames {i−1} and {i} with right-handed axis triads and orange curved arrow labelled Aᵢ. These are separate from structural panel. Use a conventional clear DH construction with z axes skew and xᵢ common normal, label θᵢ around old z, dᵢ along old z, aᵢ along new x, αᵢ around new x. Formula in prominent box:
“Aᵢ =  ⁱ⁻¹Tᵢ”
“Aᵢ = Rz(θᵢ) Tz(dᵢ) Tx(aᵢ) Rx(αᵢ)”
“θᵢ = qᵢ + θ₀ᵢ”
Under it compact parameter table headed “本模型 · 标准DH” with columns “i | aᵢ | αᵢ | dᵢ | θᵢ” and EXACT rows:
“1 | 0 | π/2 | 0.122 | q₁”
“2 | 0.408 | π | 0 | q₂+π/2”
“3 | 0.376 | π | 0 | q₃”
“4 | 0 | π/2 | 0.1215 | q₄+π/2”
“5 | 0 | π/2 | 0.1025 | q₅+π”
“6 | 0 | 0 | 0.094 | q₆”
Do not substitute alternate robot DH data.

RIGHT PANEL heading “③ 两关节串联 → 六轴模型”.
A simple coordinate-mapping chain with orange arrows, directed from right to left:
“{0} ← A₁ — {1} ← A₂ — {2}”
Below it label “将{2}中的点坐标转换到基座{0}”.
Clear formulas:
“A₁ = Rz(q₁) Tz(0.122) Rx(π/2)”
“A₂ = Rz(q₂+π/2) Tx(0.408) Rx(π)”
“⁰T₂ = A₁A₂”
A prominent equation using block matrix:
“⁰T₂ = [ R₁R₂   p₁+R₁p₂ ; 0   1 ]”
and highlighted explanatory text “先将p₂旋转到基座，再与p₁相加”.
Then a box “前两级DH原点的位置”:
“x₂ = −0.408 cos(q₁) sin(q₂)”
“y₂ = −0.408 sin(q₁) sin(q₂)”
“z₂ = 0.122 + 0.408 cos(q₂)”
Then final bottom box:
“⁰T_TCP = A₁A₂A₃A₄A₅A₆ Tz(0.12)”
“= [ R(q)   p(q) ; 0   1 ]”
“平移列 → XYZ     旋转矩阵 → roll / pitch / yaw”
Small note: “p₂为第2级局部平移；DH原点不一定是关节外壳中心。”
Keep all equations correct and readable, no added claims, no watermark, no decorative icons. This is a polished educational diagram intended for a technical Markdown document.

## 最终修订提示词

Edit this existing infographic with ONE targeted correction. Preserve the left robot illustration, title, right column, parameter table, footer, fonts and overall layout exactly. Replace ONLY the top geometric coordinate-axis diagram in the middle column (the area between its heading and the formula box). The current sketch incorrectly suggests parallel z axes with nonzero alpha and incorrectly located d; remove that entire sketch. Replace it with a clean 2 by 2 grid of four small step cards showing standard DH coordinate-frame construction via postmultiplication. Each card contains a simple unambiguous arrow or rotation icon and these exact Chinese text labels:
Top left “1  绕当前Z转 θᵢ” with “Rz(θᵢ)”
Top right “2  沿当前Z移 dᵢ” with “Tz(dᵢ)”
Bottom left “3  沿当前X移 aᵢ” with “Tx(aᵢ)”
Bottom right “4  绕当前X转 αᵢ” with “Rx(αᵢ)”.
Use orange rotation arrows and blue translation arrows. Do not draw any before/after pair of 3D axes or common-normal geometry. Small caption below cards: “依次右乘，建立相邻坐标系关系”.
Ensure all other equations and numerical DH entries remain precisely unchanged. This is a textbook scientific diagram, clarity and correctness are mandatory.

