# Authoritative cargo safety evidence and final status consumed by the main
# controller.  Warning codes describe physical hazards only; system faults use
# a separate code range and fault mask.
uint16 SCHEMA_VERSION=6

int32 CODE_CLEAR=14
int32 CODE_LEVEL1_WARNING=17
int32 CODE_LEVEL2_WARNING=18

int32 CODE_SYSTEM_NOT_READY=30
int32 CODE_LOCALIZATION_INVALID=31
int32 CODE_GRAVITY_INVALID=32
int32 CODE_CARGO_INVALID=33
int32 CODE_OBSTACLE_INVALID=34
int32 CODE_INTERNAL_ERROR=35

uint32 FAULT_STATUS_STALE=1
uint32 FAULT_LOCALIZATION=2
uint32 FAULT_GRAVITY=4
uint32 FAULT_CARGO=8
uint32 FAULT_OBSTACLE=16
uint32 FAULT_INTERNAL=32

uint8 HOOK_UNKNOWN=0
uint8 HOOK_INHIBIT=1
uint8 HOOK_EMPTY=2
uint8 HOOK_LOADED=3

uint8 HOOK_ROLE_DISABLED=0
uint8 HOOK_ROLE_REQUIRED=1
uint8 HOOK_ROLE_AUXILIARY=2

# Typed evidence lifecycle. Consumers must not infer safety semantics by
# searching the free-form reason string.
uint8 EVIDENCE_UNKNOWN=0
uint8 EVIDENCE_CLEAR=1
uint8 EVIDENCE_HAZARD_CONFIRMED=2
uint8 EVIDENCE_HAZARD_CANDIDATE=3
uint8 EVIDENCE_TRACK_CONFIRMATION_PENDING=4
uint8 EVIDENCE_SPARSE_PENDING=5
uint8 EVIDENCE_SOURCE_UNRESOLVED=6
uint8 EVIDENCE_HARD_FAULT=7

uint8 PROVENANCE_NONE=0
uint8 PROVENANCE_OUTSIDE_CARGO_SHELL_ONLY=1
uint8 PROVENANCE_PRE_CARGO_OCCUPANCY=2
uint8 PROVENANCE_STATIC_MAP_MATCH=3
uint8 PROVENANCE_CARGO_MOVED_AWAY_PERSISTENCE=4
uint8 PROVENANCE_DUAL_LIDAR_CONSENSUS=5

std_msgs/Header header
uint16 schema_version

bool valid
bool warning_valid

int32 requested_alarm_code
int32 warning_code
int32 fault_code
uint32 fault_mask

bool localization_valid
uint8 hook_signal_role
bool hook_signal_valid
bool hook_signal_conflict
bool cargo_valid
uint32 cargo_track_id
uint32 obstacle_track_id
uint32 obstacle_validated_streak
uint32 obstacle_static_provenance_streak
float32 obstacle_static_age_sec
bool obstacle_large_geometry_valid
uint8 obstacle_provenance_type
bool obstacle_provenance_valid
float32 obstacle_track_cell_overlap
float32 obstacle_track_iou
float32 obstacle_association_cost
string obstacle_association_reset_reason
uint8 cargo_source
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
uint8 evidence_state
string reason
