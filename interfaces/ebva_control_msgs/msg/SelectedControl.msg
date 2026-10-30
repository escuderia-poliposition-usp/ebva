std_msgs/Header header
uint64 sequence
uint8 STOP=0
uint8 MANUAL=1
uint8 AUTONOMY=2
uint8 mode
string reason
# Tagged selection: backend maps rc only in MANUAL and autonomy only in AUTONOMY.
RcInput rc
ActuatorCommand autonomy
