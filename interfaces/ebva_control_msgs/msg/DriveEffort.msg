# Atomic dimensionless pair. contract identifies its producer semantics.
# This is neither speed/steering angle nor acceleration/yaw rate.
# Header is the original measurement, not the adapter receipt time.
std_msgs/Header header
string contract
float32 longitudinal
float32 steering
