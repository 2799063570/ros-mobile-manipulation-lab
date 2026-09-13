std_msgs/Header header
aubo_perception/DetectedObject[] objects
# False means unavailable/partial geometry, never proof that the table is empty.
bool observation_valid
# Optical frame at header.stamp, used to exclude the gripper's table projection.
string sensor_frame
