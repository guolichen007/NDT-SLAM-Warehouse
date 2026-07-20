#pragma once

#include <cstdint>

namespace ndt_slam {

enum class PendingOriginAction : std::uint8_t {
    NONE = 0,
    KEEP_WAITING_FOR_LIDAR_TIME,
    ATTACH,
    DISCARD_EXPIRED,
    DISCARD_SPATIAL_MISMATCH,
    DISCARD_INVALID
};

struct PendingOriginBindingInput {
    bool pending_valid = false;
    double lidar_stamp_sec = 0.0;
    double origin_stamp_sec = 0.0;
    double max_age_sec = 2.0;
    double future_stamp_tolerance_sec = 0.05;
    bool track_center_finite = false;
    bool origin_center_finite = false;
    double center_distance_m = 0.0;
    double maximum_center_distance_m = 0.50;
};

PendingOriginAction evaluatePendingOriginBinding(
    const PendingOriginBindingInput& input);

const char* pendingOriginActionName(PendingOriginAction action);

}  // namespace ndt_slam
