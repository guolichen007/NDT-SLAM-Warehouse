# Evidence and requested alarm level consumed by the fail-safe heartbeat node.
uint16 SCHEMA_VERSION=2

int32 ALARM_CLEAR=14
int32 ALARM_INNER_WARNING=17
int32 ALARM_OUTER_OR_INVALID=18

uint8 HOOK_UNKNOWN=0
uint8 HOOK_INHIBIT=1
uint8 HOOK_EMPTY=2
uint8 HOOK_LOADED=3

std_msgs/Header header
uint16 schema_version
bool valid
bool cargo_valid
uint32 cargo_track_id
uint8 cargo_source
int32 requested_alarm_code

bool hook_signal_valid
uint8 hook_load_state
float32 hook_voltage
bool no_cargo_confirmed

float32 cargo_bottom_z_map
float32 cargo_bottom_uncertainty_m
bool obstacle_valid
uint32 obstacle_count
float32 nearest_obstacle_distance_m
float32 obstacle_top_z_map
float32 obstacle_uncertainty_m
float32 conservative_vertical_clearance_m
float32 confidence
string reason
