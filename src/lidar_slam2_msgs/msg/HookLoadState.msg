uint16 SCHEMA_VERSION=1

uint8 STATE_UNKNOWN=0
uint8 STATE_INHIBIT=1
uint8 STATE_EMPTY=2
uint8 STATE_LOADED=3

std_msgs/Header header
uint16 schema_version
bool valid
bool fresh
uint8 state
float32 voltage
uint32 stable_samples
string reason
