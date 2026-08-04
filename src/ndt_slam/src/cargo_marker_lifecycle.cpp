#include "ndt_slam/cargo_marker_lifecycle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {

void CargoMarkerLifecycle::setConfig(
    const CargoMarkerLifecycleConfig& config) {
    config_ = config;
    config_.invalid_hold_sec = std::max(0.0, config_.invalid_hold_sec);
    config_.timestamp_epsilon_sec =
        std::max(0.0, config_.timestamp_epsilon_sec);
}

void CargoMarkerLifecycle::reset() {
    has_last_stamp_ = false;
    last_stamp_sec_ = 0.0;
    has_last_geometry_ = false;
    last_geometry_stamp_sec_ = 0.0;
    last_geometry_ = CargoBoxGeometry{};
}

CargoMarkerLifecycleDecision CargoMarkerLifecycle::update(
    const CargoMarkerLifecycleInput& input) {
    CargoMarkerLifecycleDecision decision;
    if (!std::isfinite(input.stamp_sec)) {
        decision.reason = "nonfinite_stamp";
        return decision;
    }
    if (has_last_stamp_ &&
        input.stamp_sec + config_.timestamp_epsilon_sec < last_stamp_sec_) {
        // A bag replay or sensor epoch restart must not inherit a marker from
        // the previous time axis.
        reset();
    }
    has_last_stamp_ = true;
    last_stamp_sec_ = input.stamp_sec;

    if (input.explicit_empty) {
        has_last_geometry_ = false;
        last_geometry_ = CargoBoxGeometry{};
        decision.reason = "explicit_empty";
        return decision;
    }

    if (!input.localization_valid) {
        // Map-frame geometry cannot be predicted safely without a valid map
        // pose. Delete immediately instead of freezing the last map position
        // and presenting it as a current cargo box in RViz.
        has_last_geometry_ = false;
        last_geometry_ = CargoBoxGeometry{};
        decision.reason = "localization_invalid_delete";
        return decision;
    }

    if (input.geometry_valid && input.geometry.valid) {
        has_last_geometry_ = true;
        last_geometry_stamp_sec_ = input.stamp_sec;
        last_geometry_ = input.geometry;
        decision.show = true;
        decision.geometry = input.geometry;
        decision.style = input.safety_height_valid
            ? CargoMarkerStyle::VALID
            : CargoMarkerStyle::HEIGHT_DEGRADED;
        decision.reason = input.safety_height_valid
            ? "current_geometry_and_height"
            : "current_geometry_height_invalid";
        return decision;
    }

    const double age_sec = has_last_geometry_
        ? input.stamp_sec - last_geometry_stamp_sec_
        : std::numeric_limits<double>::infinity();
    if (has_last_geometry_ && age_sec >= -config_.timestamp_epsilon_sec &&
        age_sec <= config_.invalid_hold_sec + config_.timestamp_epsilon_sec) {
        decision.show = true;
        decision.using_last_good_geometry = true;
        decision.geometry = last_geometry_;
        decision.style = CargoMarkerStyle::HEIGHT_DEGRADED;
        decision.reason = "last_good_height_hold";
        return decision;
    }

    has_last_geometry_ = false;
    last_geometry_ = CargoBoxGeometry{};
    decision.reason = "invalid_hold_expired";
    return decision;
}

}  // namespace ndt_slam
