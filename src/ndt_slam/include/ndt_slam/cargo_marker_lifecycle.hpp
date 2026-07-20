#pragma once

#include "ndt_slam/cargo_bottom_fusion.hpp"

#include <string>

namespace ndt_slam {

enum class CargoMarkerStyle {
    VALID,
    HEIGHT_DEGRADED,
    LOCALIZATION_DEGRADED,
};

struct CargoMarkerLifecycleConfig {
    double invalid_hold_sec = 2.0;
    double timestamp_epsilon_sec = 1.0e-6;
};

struct CargoMarkerLifecycleInput {
    double stamp_sec = 0.0;
    bool explicit_empty = false;
    bool localization_valid = true;
    bool geometry_valid = false;
    bool safety_height_valid = false;
    CargoBoxGeometry geometry;
};

struct CargoMarkerLifecycleDecision {
    bool show = false;
    bool using_last_good_geometry = false;
    CargoMarkerStyle style = CargoMarkerStyle::HEIGHT_DEGRADED;
    CargoBoxGeometry geometry;
    std::string reason = "no_geometry";
};

// Visualization-only lifecycle. It deliberately does not alter CargoBottom,
// CargoSafetyStatus, removal authorization, or any alarm validity flag.
class CargoMarkerLifecycle {
public:
    void setConfig(const CargoMarkerLifecycleConfig& config);
    void reset();
    CargoMarkerLifecycleDecision update(
        const CargoMarkerLifecycleInput& input);

private:
    CargoMarkerLifecycleConfig config_;
    bool has_last_stamp_ = false;
    double last_stamp_sec_ = 0.0;
    bool has_last_geometry_ = false;
    double last_geometry_stamp_sec_ = 0.0;
    CargoBoxGeometry last_geometry_;
};

}  // namespace ndt_slam
