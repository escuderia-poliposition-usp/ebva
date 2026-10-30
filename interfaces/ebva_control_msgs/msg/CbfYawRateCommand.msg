# Gated yaw_rate CBF result. No actuator mapping or model selection is implied.
# header is the exact original autonomous nominal header.
std_msgs/Header header
# Original LaserScan acquisition header in the declared aligned model frame.
std_msgs/Header scan_header
# Original Odometry header (pose parent); twist basis is velocity_child_frame_id.
std_msgs/Header velocity_header
string velocity_child_frame_id
float64 longitudinal_velocity_mps
float64 acceleration_mps2
float64 yaw_rate_rps
