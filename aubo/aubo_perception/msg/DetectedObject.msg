string color
geometry_msgs/Pose pose
float32 contour_area
int32 pixel_x
int32 pixel_y
# 通用类别；color 保留为旧分拣程序的类别键（不局限于颜色）。
string class_name
float32 confidence
# 物体中心位姿；以下角度是 header.frame_id 的 XY 平面短边方向。
float32 grasp_width
float32 grasp_angle
bool grasp_geometry_valid
# 深度投影/所选高度模式校验有效时为 true；table 模式为 false。
bool depth_valid
# 自身物高；bounded_depth 下是受类别厚度上限约束的估计，不含下层支撑。
float32 object_height
# Convex detector rectangle projected onto the object's horizontal surface.
# Coordinates use DetectedObjectArray.header.frame_id; empty means unavailable.
geometry_msgs/Polygon footprint
