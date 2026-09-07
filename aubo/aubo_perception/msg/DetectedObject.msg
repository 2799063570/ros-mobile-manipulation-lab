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
# 深度高度经过桌面 + 物高校验才为 true；table 模式为 false。
bool depth_valid
float32 object_height
