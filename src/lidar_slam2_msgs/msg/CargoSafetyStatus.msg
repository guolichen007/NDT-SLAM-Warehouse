# Evidence and requested alarm level consumed by the fail-safe heartbeat node.
uint16 SCHEMA_VERSION=1

int32 ALARM_CLEAR=14
int32 ALARM_INNER_WARNING=17
int32 ALARM_OUTER_OR_INVALID=18

std_msgs/Header header
uint16 schema_version
bool valid
bool cargo_valid
uint32 cargo_track_id
uint8 cargo_source
int32 requested_alarm_code

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
