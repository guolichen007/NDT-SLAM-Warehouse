#include <ndt_slam/tracking_ephemeral_shadow.hpp>

#include <algorithm>
#include <cmath>

namespace ndt_slam {

bool TrackingEphemeralShadow::configure(
    const TrackingEphemeralShadowConfig& config, std::string* reason) {
    config_ = config;
    config_.ttl_sec = std::max(0.1, config_.ttl_sec);
    if (config_.bind_to_ndt_target) {
        config_.enabled = false;
        config_.bind_to_ndt_target = false;
        if (reason) *reason = "ephemeral_ndt_binding_forbidden";
        return false;
    }
    if (reason) *reason = config_.enabled
        ? "shadow_diagnostics_only" : "disabled";
    return true;
}

void TrackingEphemeralShadow::observe(
    std::size_t points, double stamp_sec) {
    if (!config_.enabled || !std::isfinite(stamp_sec)) return;
    ++observations_;
    latest_points_ = points;
    latest_stamp_sec_ = stamp_sec;
}

TrackingEphemeralShadowStatus TrackingEphemeralShadow::status(
    double now_sec) const {
    TrackingEphemeralShadowStatus result;
    result.enabled = config_.enabled;
    result.isolated_from_ndt_target = !config_.bind_to_ndt_target;
    result.observations = observations_;
    result.latest_points = latest_points_;
    result.latest_stamp_sec = latest_stamp_sec_;
    if (!config_.enabled) {
        result.reason = "disabled";
    } else if (!std::isfinite(now_sec) || latest_stamp_sec_ <= 0.0 ||
               now_sec - latest_stamp_sec_ > config_.ttl_sec) {
        result.reason = "shadow_stale";
    } else {
        result.reason = "shadow_current";
    }
    return result;
}

}  // namespace ndt_slam
