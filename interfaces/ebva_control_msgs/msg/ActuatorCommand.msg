# Atomic request in actuator units; NOT m/s or steering radians.
# stamp is source generation time. Receiver also expires silence locally.
std_msgs/Header header
float64 erpm
# Normalized servo position [0,1].
float64 servo
