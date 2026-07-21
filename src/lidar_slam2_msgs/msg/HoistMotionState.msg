uint8 STATE_UNKNOWN=0
uint8 STATE_STOPPED=1
uint8 STATE_UP=2
uint8 STATE_DOWN=3

std_msgs/Header header
bool valid
bool fresh
uint8 state
float32 speed_mps
string reason
