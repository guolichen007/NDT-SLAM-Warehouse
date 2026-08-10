#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace ndt_slam {

struct TrackingEphemeralShadowConfig {
    bool enabled = false;
    bool bind_to_ndt_target = false;
    bool shadow_diagnostics_only = true;
    double ttl_sec = 2.0;
};

struct TrackingEphemeralShadowStatus {
    bool enabled = false;
    bool isolated_from_ndt_target = true;
    std::uint64_t observations = 0U;
    std::size_t latest_points = 0U;
    double latest_stamp_sec = 0.0;
    std::string reason = "disabled";
};

// First-production implementation intentionally keeps only shadow metrics.
// There is no cloud accessor and therefore no API that can bind this data to
// the NDT target. Enabling target binding is rejected by configure().
class TrackingEphemeralShadow {
public:
    bool configure(const TrackingEphemeralShadowConfig& config,
                   std::string* reason);
    void observe(std::size_t points, double stamp_sec);
    TrackingEphemeralShadowStatus status(double now_sec) const;

private:
    TrackingEphemeralShadowConfig config_;
    std::uint64_t observations_ = 0U;
    std::size_t latest_points_ = 0U;
    double latest_stamp_sec_ = 0.0;
};

}  // namespace ndt_slam
