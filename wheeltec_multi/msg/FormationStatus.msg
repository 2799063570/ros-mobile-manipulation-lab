uint8 WAITING=0
uint8 TRACKING=1
uint8 RECOVERING=2
uint8 AVOIDING=3

std_msgs/Header header
uint8 state
uint32 leader_sequence
float32 leader_age
bool localization_initialized
geometry_msgs/Pose2D target
geometry_msgs/Pose2D follower
geometry_msgs/Vector3 error_body
geometry_msgs/Twist command
