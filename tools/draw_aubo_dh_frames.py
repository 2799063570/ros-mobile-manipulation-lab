"""Draw an entirely editable SVG of the nominal AUBO DH frames and gripper.

Geometry lengths come from the repository URDF. Fixed DH rotations use exact pi
as in section 6.7.2 of the execution document. The robot body is schematic.
Only Python's standard library is required; no raster artwork is embedded.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/images/aubo_dh_frames_editable.svg"
SVG = "http://www.w3.org/2000/svg"
INK = "http://www.inkscape.org/namespaces/inkscape"
ET.register_namespace("", SVG)
ET.register_namespace("inkscape", INK)


def element(parent, tag, **attrs):
    return ET.SubElement(parent, "{" + SVG + "}" + tag,
                         {k.replace("_", "-"): str(v) for k, v in attrs.items()})


def group(parent, name):
    return element(parent, "g", id=name)


def path(parent, d, fill="#f1f2f3", **attrs):
    return element(parent, "path", d=d, fill=fill, **attrs)


def ellipse(parent, x, y, rx, ry, **attrs):
    return element(parent, "ellipse", cx=x, cy=y, rx=rx, ry=ry, **attrs)


def line(parent, x1, y1, x2, y2, **attrs):
    return element(parent, "line", x1=x1, y1=y1, x2=x2, y2=y2, **attrs)


def text(parent, x, y, content, **attrs):
    node = element(parent, "text", x=x, y=y, **attrs)
    node.text = content
    return node


def symbol(parent, x, y, letter, subscript, color="#17191c", size=21):
    node = text(parent, x, y, letter, fill=color, stroke="none",
                font_family="Times New Roman, STIX Two Text, serif",
                font_size=size, font_style="italic")
    sub = element(node, "tspan", baseline_shift="sub", font_size="70%")
    sub.text = str(subscript)
    return node


def screw(parent, x, y):
    ellipse(parent, x, y, 2.7, 2.7, fill="#d5d8db", stroke_width="0.9")
    line(parent, x-1.3, y, x+1.3, y, stroke_width="0.7")


def cylinder_v(parent, name, x, top, bottom, width):
    g = group(parent, name)
    r, depth = width/2, 9
    path(g, f"M {x-r},{top} A {r},{depth} 0 0 1 {x+r},{top} "
         f"L {x+r},{bottom} A {r},{depth} 0 0 1 {x-r},{bottom} Z")
    ellipse(g, x, top, r, depth, fill="#fafafa")
    path(g, f"M {x-r+7},{top+5} L {x-r+7},{bottom-3}", fill="none",
         stroke="#c2c6cb", stroke_width="1")
    path(g, f"M {x+r-7},{top+5} L {x+r-7},{bottom-3}", fill="none",
         stroke="#c2c6cb", stroke_width="1")
    path(g, f"M {x-r},{bottom-7} A {r},{depth} 0 0 0 {x+r},{bottom-7}",
         fill="none", stroke="#8c939a", stroke_width="0.9")
    return g


def cylinder_h(parent, name, left, right, y, height):
    g = group(parent, name)
    r, depth = height/2, 10
    path(g, f"M {left},{y-r} L {right},{y-r} "
         f"A {depth},{r} 0 0 1 {right},{y+r} L {left},{y+r} "
         f"A {depth},{r} 0 0 1 {left},{y-r} Z")
    ellipse(g, right, y, depth, r, fill="#e5e8eb")
    line(g, left+5, y-r+7, right-5, y-r+7, stroke="#bac0c5", stroke_width="0.9")
    line(g, left+5, y+r-7, right-5, y+r-7, stroke="#bac0c5", stroke_width="0.9")
    return g


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def dh(a, alpha, d, theta):
    c, s, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return [[c, -s*ca, s*sa, a*c], [s, c*ca, -c*sa, a*s],
            [0, sa, ca, d], [0, 0, 0, 1]]


def xyz(tree, name):
    joint = tree.find(f"joint[@name='{name}']")
    return [float(v) for v in joint.find("origin").get("xyz").split()]


arm = ET.parse(ROOT / "aubo/aubo_description/urdf/arm.xacro").getroot()
gripper = ET.parse(ROOT / "aubo/aubo_description/urdf/jiazhua.urdf").getroot()
d1 = xyz(arm, "shoulder_joint")[2]
a2 = xyz(arm, "foreArm_joint")[0]
a3 = xyz(arm, "wrist1_joint")[0]
d4 = xyz(arm, "upperArm_joint")[1]
d5 = xyz(arm, "wrist2_joint")[1]
d6 = -xyz(arm, "wrist3_joint")[1]
tool_xyz = xyz(gripper, "gripper_joint")
assert tool_xyz[:2] == [0, 0]
tool = tool_xyz[2]
assert xyz(gripper, "tcp_joint") == [0, 0, 0]
assembly = ET.parse(ROOT / "aubo/aubo_description/urdf/aubo_i5_with_camera.xacro").getroot()
assert xyz(assembly, "ee_link_joint") == [0, 0, 0]
for tree, name in [(assembly, "ee_link_joint"), (gripper, "gripper_joint"),
                   (gripper, "tcp_joint")]:
    rpy = tree.find(f"joint[@name='{name}']/origin").get("rpy", "0 0 0")
    assert all(float(v) == 0 for v in rpy.split()), "Tool rotation changed; redraw the tool rig."
parameters = [(0, math.pi/2, d1, 0), (a2, math.pi, 0, math.pi/2),
              (a3, math.pi, 0, 0), (0, math.pi/2, d4, math.pi/2),
              (0, math.pi/2, d5, math.pi), (0, 0, d6, 0)]
identity = [[float(i == j) for j in range(4)] for i in range(4)]
transforms = [identity]
for row in parameters:
    transforms.append(multiply(transforms[-1], dh(*row)))
transforms.append(multiply(transforms[-1], dh(0, 0, tool, 0)))

# Oblique projection: base X down-left, Y right, Z up. Units: 1000 px/m.
def project(p):
    return (650+1000*(-0.78*p[0]+p[1]), 1250+1000*(0.62*p[0]-p[2]))


points = [project([t[i][3] for i in range(3)]) for t in transforms]
(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5), (x6, y6), (xt, yt) = points
root = ET.Element("{"+SVG+"}svg", {
    "width": "140mm", "height": "252mm", "viewBox": "150 100 700 1260",
    "version": "1.1", "role": "img", "aria-labelledby": "figure-title figure-description"})
title = element(root, "title", id="figure-title")
title.text = "AUBO nominal DH frames with gripper TCP — editable vector drawing"
desc = element(root, "desc", id="figure-description")
desc.text = ("Zero joint angles; DH frame 0 equals base_link. All visible elements are "
             "editable vector paths, lines, polygons and live text. No raster images. "
             "Geometry from URDF, exact pi nominal fixed rotations; body is schematic.")
background = group(root, "background")
element(background, "rect", x=150, y=100, width=700, height=1260, fill="white")
body = group(root, "robot-and-gripper")
body.set("stroke", "#343a40")
body.set("stroke-width", "1.6")
body.set("stroke-linejoin", "round")
body.set("stroke-linecap", "round")

# Base and first joint.
g = group(body, "base-pedestal")
path(g, f"M {x0-86},{y0-9} A 86,17 0 0 1 {x0+86},{y0-9} "
     f"L {x0+86},{y0+10} A 86,17 0 0 1 {x0-86},{y0+10} Z", fill="#e5e8eb")
ellipse(g, x0, y0-9, 86, 17, fill="#fafafa")
for dx in [-65, 65]:
    screw(g, x0+dx, y0-8)
cylinder_v(body, "joint-1-base", x1, y1-40, y0-17, 105)

# Lower horizontal shoulder assembly and link 2.
cylinder_h(body, "joint-2-shoulder", x4-18, x1+12, y1, 108)
g = group(body, "shoulder-cover")
ellipse(g, x4-18, y1, 18, 52, fill="#e7eaed")
ellipse(g, x4-23, y1, 12, 42, fill="#f8f9fa")
for dy in [-30, 30]:
    screw(g, x4-25, y1+dy)
cylinder_v(body, "link-2-upper-arm", x4, y2+50, y1-52, 66)

# Elbow body, offset toward the left of its DH origin.
g = group(body, "joint-3-elbow")
path(g, f"M {x4-48},{y2-33} Q {x4-43},{y2-61} {x4-9},{y2-61} "
     f"L {x2+28},{y2-61} Q {x2+53},{y2-41} {x2+53},{y2+7} "
     f"Q {x2+47},{y2+56} {x2-5},{y2+58} L {x4-5},{y2+58} "
     f"Q {x4-51},{y2+52} {x4-48},{y2-33} Z")
ellipse(g, x4-42, y2, 13, 46, fill="#e0e4e7")
ellipse(g, x4-46, y2, 8, 36, fill="#f8f9fa")
path(g, f"M {x2-35},{y2-48} Q {x2-53},{y2+23} {x4+4},{y2+44}",
     fill="none", stroke="#9ea5ac", stroke_width="1")
for dy in [-26, 26]:
    screw(g, x4-48, y2+dy)
cylinder_v(body, "link-3-forearm", x3, y3+51, y2-57, 68)

# Wrist 1 runs horizontally, wrist 2 vertically, wrist 3 horizontally.
cylinder_h(body, "joint-4-wrist-1", x4-19, x3+25, y3, 100)
g = group(body, "wrist-1-seams")
for dx in [x3+18, x3+23]:
    ellipse(g, dx, y3, 10, 46, fill="none", stroke="#7d858d", stroke_width="0.8")
cylinder_v(body, "joint-5-wrist-2", x4, y5-26, y4+22, 90)
cylinder_h(body, "joint-6-wrist-3", x6+5, x5+16, y5, 76)
g = group(body, "wrist-3-rings")
for xx in [x6+16, x6+21, x5-19, x5-14]:
    ellipse(g, xx, y5, 7, 37, fill="none", stroke="#8d959d", stroke_width="0.9")

# Flange and open two-finger gripper. TCP remains in the central contact band.
g = group(body, "flange")
path(g, f"M {x6-5},{y6-43} L {x6+7},{y6-43} L {x6+7},{y6+43} "
     f"L {x6-5},{y6+43} Z", fill="#d7dce1")
for yy in [y6-32, y6+32]:
    screw(g, x6+1, yy)
g = group(body, "gripper-housing")
path(g, f"M {x6-62},{y6-52} L {x6-7},{y6-45} L {x6-7},{y6+45} "
     f"L {x6-62},{y6+52} L {x6-72},{y6+37} L {x6-72},{y6-37} Z", fill="#e2e6ea")
path(g, f"M {x6-53},{y6-42} L {x6-22},{y6-36} L {x6-22},{y6+36} "
     f"L {x6-53},{y6+42} Z", fill="#fafafa", stroke="#879099", stroke_width="0.9")
for yy in [y6-33, y6+33]:
    screw(g, x6-59, yy)
for side, sign in [("upper", -1), ("lower", 1)]:
    g = group(body, "gripper-finger-"+side)
    near, far = yt+sign*29, yt+sign*52
    path(g, f"M {x6-63},{near} L {xt-35},{near} L {xt-51},{near+sign*4} "
         f"L {xt-51},{far-2*sign} L {xt-25},{far} L {x6-62},{far} Z",
         fill="#d0d6dc")
    path(g, f"M {xt-48},{near+sign*5} L {xt-10},{near+sign*5} "
         f"L {xt-10},{near+sign*15} L {xt-48},{near+sign*15} Z",
         fill="#9ca5ad", stroke="none")

construction = group(root, "axis-extensions")
for xx, yy, xxx, yyy in [(x4, y2, x2, y2), (x4, y3, x3, y3)]:
    line(construction, xx, yy, xxx, yyy, stroke="#8e969f", stroke_width="1",
         stroke_dasharray="5 5", opacity="0.65")

frames = group(root, "coordinate-frames")
colors = ["#c53737", "#218244", "#2858b5"]
for index, (matrix, (ox, oy)) in enumerate(zip(transforms, points)):
    label = str(index) if index < 7 else "TCP"
    g = group(frames, "frame-"+label)
    g.set("data-origin-base-m", " ".join(f"{matrix[i][3]:.9f}" for i in range(3)))
    g.set("data-frame-convention", "standard-DH-nominal-zero")
    length = 45 if index in [4, 5, 6] else 53
    for col, (letter, color) in enumerate(zip("xyz", colors)):
        axis = [matrix[row][col] for row in range(3)]
        dx, dy = -0.78*axis[0]+axis[1], 0.62*axis[0]-axis[2]
        norm = math.hypot(dx, dy)
        ux, uy = dx/norm, dy/norm
        ex, ey = ox+length*ux, oy+length*uy
        axis_group = group(g, "axis-"+label+"-"+letter)
        line(axis_group, ox, oy, ex-7*ux, ey-7*uy, stroke=color, stroke_width="2.6")
        element(axis_group, "polygon", points=(f"{ex},{ey} {ex-10*ux-3.3*uy},{ey-10*uy+3.3*ux} "
                                               f"{ex-10*ux+3.3*uy},{ey-10*uy-3.3*ux}"), fill=color)
        lx, ly = ex+4, ey-5
        if ux < -0.5:
            lx, ly = ex-8, ey-8
        if uy > 0.3:
            lx, ly = ex-4, ey+17
        if index == 0 and letter == "y":
            ly -= 8
        symbol(axis_group, lx, ly, letter, label, color)
    ellipse(g, ox, oy, 4.1, 4.1, fill="#e78b23" if index == 7 else "#1b2025",
            stroke="white", stroke_width="0.8", id="origin-"+label)
    if index < 7:
        symbol(g, ox+8, oy+22, "O", label, size=20)
    else:
        text(g, ox+7, oy+21, "TCP", font_family="Times New Roman, serif",
             font_style="italic", font_size="19", fill="#20252a")

# Inkscape-compatible layers. Each frame, axis, label and mechanical component
# remains its own editable group within the layers.
for layer, label in [(background, "Background"), (body, "Robot and gripper"),
                     (construction, "Axis extensions"), (frames, "DH coordinate frames")]:
    layer.set("{"+INK+"}groupmode", "layer")
    layer.set("{"+INK+"}label", label)
ET.indent(root, space="  ")
OUT.parent.mkdir(parents=True, exist_ok=True)
ET.ElementTree(root).write(OUT, encoding="utf-8", xml_declaration=True)
print(OUT)
