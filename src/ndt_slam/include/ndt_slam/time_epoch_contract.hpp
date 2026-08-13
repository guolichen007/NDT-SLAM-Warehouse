#pragma once

#include <cmath>
#include <cstdint>

namespace ndt_slam {

inline bool isSourceTimestampRollback(
    double previous_stamp_sec,
    double current_stamp_sec,
    double tolerance_sec = 1.0e-6) noexcept {
    return std::isfinite(previous_stamp_sec) &&
           std::isfinite(current_stamp_sec) &&
           current_stamp_sec + tolerance_sec < previous_stamp_sec;
}

inline bool isMapCommitLifecycleCurrent(
    std::uint64_t job_lifecycle_epoch,
    std::uint64_t current_lifecycle_epoch) noexcept {
    return job_lifecycle_epoch == current_lifecycle_epoch;
}

}  // namespace ndt_slam
