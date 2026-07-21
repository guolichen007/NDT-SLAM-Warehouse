uint32 SCHEMA_VERSION=1

uint8 OBSERVATION_INVALID=0
uint8 OBSERVATION_CURRENT_MEASUREMENT=1
uint8 OBSERVATION_SHORT_GAP_HOLD=2
uint8 OBSERVATION_STALE=3
uint8 OBSERVATION_TRACK_CHANGED=4
uint8 OBSERVATION_TIMESTAMP_ROLLBACK=5

uint8 SWAY_NOT_EVALUATED=0
uint8 SWAY_NORMAL=1
uint8 SWAY_DETECTED=2
uint8 SWAY_WARNING=3
uint8 SWAY_ALARM=4
uint8 SWAY_SETTLING=5

uint8 SKEW_NOT_EVALUATED=0
uint8 SKEW_NONE=1
uint8 SKEW_PULL_SUSPECTED=2
uint8 SKEW_PULL_ALARM=3

uint8 TORSION_NOT_EVALUATED=0
uint8 TORSION_NORMAL=1
uint8 TORSION_DETECTED=2
uint8 TORSION_WARNING=3
uint8 TORSION_ALARM=4

uint8 ACTION_NONE=0
uint8 ACTION_WATCH=1
uint8 ACTION_REDUCE_TRAVEL_SPEED=2
uint8 ACTION_STOP_TRAVEL=3
uint8 ACTION_INHIBIT_HOIST_UP=4
uint8 ACTION_STOP_AND_SETTLE=5

std_msgs/Header header
uint32 schema_version
bool valid
uint8 observation_state
uint8 sway_state
uint8 skew_pull_state
uint8 torsion_state
uint64 cargo_lifecycle_id
uint64 track_segment_id
uint32 track_id
bool hook_anchor_valid
string hook_anchor_source
uint8 hoist_state
bool hoist_state_fresh
bool hoist_up_confirmed
bool alarm_inhibited
float32 offset_x_m
float32 offset_y_m
float32 offset_m
float32 rope_length_m
float32 angle_deg
float32 horizontal_speed_mps
float32 radial_speed_mps
float32 oscillation_amplitude_m
float32 direction_consistency
int32 zero_crossings
float32 yaw_error_deg
float32 observation_age_sec
float32 state_duration_sec
uint8 recommended_action
string reason
