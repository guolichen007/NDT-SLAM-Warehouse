# Versioned, frame-explicit cargo bottom estimate.
uint16 SCHEMA_VERSION=2

uint8 SOURCE_INVALID=0
uint8 SOURCE_POINTS=1
uint8 SOURCE_MAP_DIFF=2
uint8 SOURCE_MAP_STATIC=3
uint8 SOURCE_RECENT_STABLE=4
uint8 SOURCE_ORIGIN_HEIGHT=5

std_msgs/Header header
uint16 schema_version
bool valid
uint32 track_id
uint8 track_state
uint8 source
string source_detail
string invalid_reason

# Height evidence in the observation/base frame and in map.
float32 bottom_z_base
float32 top_z_base
float32 bottom_z_map
float32 top_z_map
float32 height_m
float32 uncertainty_m
float32 confidence

# Robust vertical distribution of the supporting cargo samples.
float32 z02_base
float32 z05_base
float32 z50_base
float32 z95_base
uint32 sample_count
uint32 support_point_count
float32 support_ratio

# Fixed ordering in both arrays: four bottom corners followed by four top corners.
geometry_msgs/Point[8] corners_base
geometry_msgs/Point[8] corners_map
