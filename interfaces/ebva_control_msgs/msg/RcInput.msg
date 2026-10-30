# stamp is host reception time of this serial frame, not PWM measurement time.
std_msgs/Header header
uint64 sequence
float64 throttle_us
float64 steering_us
bool valid
uint8 RF_UNKNOWN=0
uint8 RF_VALID=1
uint8 RF_LOST=2
# Two-channel serial protocol can only report RF_UNKNOWN.
uint8 rf_state
string reason
