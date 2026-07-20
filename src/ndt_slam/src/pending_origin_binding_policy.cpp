#include "ndt_slam/pending_origin_binding_policy.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {

PendingOriginAction evaluatePendingOriginBinding(
    const PendingOriginBindingInput& input) {
    if (!input.pending_valid) return PendingOriginAction::NONE;

    if (!std::isfinite(input.lidar_stamp_sec) ||
        !std::isfinite(input.origin_stamp_sec) ||
        !std::isfinite(input.max_age_sec) || input.max_age_sec < 0.0 ||
        !std::isfinite(input.future_stamp_tolerance_sec) ||
        input.future_stamp_tolerance_sec < 0.0 ||
        !std::isfinite(input.maximum_center_distance_m) ||
        input.maximum_center_distance_m < 0.0) {
        return PendingOriginAction::DISCARD_INVALID;
    }

    const double age = input.lidar_stamp_sec - input.origin_stamp_sec;
    if (!std::isfinite(age)) {
        return PendingOriginAction::DISCARD_INVALID;
    }
    if (age < -input.future_stamp_tolerance_sec) {
        return PendingOriginAction::KEEP_WAITING_FOR_LIDAR_TIME;
    }

    const double effective_age = std::max(0.0, age);
    if (effective_age > input.max_age_sec) {
        return PendingOriginAction::DISCARD_EXPIRED;
    }
    if (!input.track_center_finite || !input.origin_center_finite ||
        !std::isfinite(input.center_distance_m)) {
        return PendingOriginAction::DISCARD_INVALID;
    }
    if (input.center_distance_m > input.maximum_center_distance_m) {
        return PendingOriginAction::DISCARD_SPATIAL_MISMATCH;
    }
    return PendingOriginAction::ATTACH;
}

const char* pendingOriginActionName(PendingOriginAction action) {
    switch (action) {
        case PendingOriginAction::NONE:
            return "none";
        case PendingOriginAction::KEEP_WAITING_FOR_LIDAR_TIME:
            return "keep_waiting_for_lidar_time";
        case PendingOriginAction::ATTACH:
            return "attach";
        case PendingOriginAction::DISCARD_EXPIRED:
            return "discard_expired";
        case PendingOriginAction::DISCARD_SPATIAL_MISMATCH:
            return "discard_spatial_mismatch";
        case PendingOriginAction::DISCARD_INVALID:
            return "discard_invalid";
    }
    return "unknown";
}

}  // namespace ndt_slam
