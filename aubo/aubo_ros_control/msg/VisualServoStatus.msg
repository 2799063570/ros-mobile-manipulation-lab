# Read-only geometry monitor; validity is NOT motion authorization.
std_msgs/Header header
string servo_mode
string state
string error_frame
string detail
bool feedback_valid
bool target_valid
bool error_valid
# ROS input topic/header ages; not cabinet sampling timestamps or SDK RPC times.
float64 feedback_age
float64 target_age
float64 position_deadband
geometry_msgs/Point tcp_position
geometry_msgs/Point observed_position
geometry_msgs/Point goal_position
geometry_msgs/Vector3 position_error
float64 position_error_norm
