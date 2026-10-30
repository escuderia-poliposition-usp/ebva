# Filtered steering-tangent model; never yaw rate or steering angle.
# All three acquisition headers and the body velocity actually consumed.
std_msgs/Header header
std_msgs/Header scan_header
std_msgs/Header velocity_header
string velocity_child_frame_id
float64 longitudinal_velocity_mps
float64 acceleration_mps2
float64 steering_tangent
