#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ndt_slam {

struct PendingStaticHazardTrackerConfig {
  int minimum_confirmations = 3;
  double maximum_observation_gap_sec = 0.60;
  float minimum_cell_overlap = 0.20F;
};

struct PendingStaticHazardObservation {
  double stamp_sec = 0.0;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t map_generation = 0U;
  bool authority_valid = false;
  bool query_valid = false;
  bool query_bounded = false;
  bool hazard = false;
  std::int32_t warning_code = 0;
  std::vector<std::int64_t> matched_cell_keys;
};

struct PendingStaticHazardDecision {
  bool valid = false;
  bool hazard_observed = false;
  bool authorized = false;
  std::int32_t warning_code = 0;
  std::uint32_t obstacle_id = 0U;
  int confirmations = 0;
  float cell_overlap = 0.0F;
  std::string reason = "not_evaluated";
};

// Confirms that a pending-cargo static hazard belongs to one stable map
// region. The tracker never grants CLEAR and never reuses the live-obstacle
// track namespace.
class PendingStaticHazardTracker {
 public:
  explicit PendingStaticHazardTracker(
      const PendingStaticHazardTrackerConfig& config =
          PendingStaticHazardTrackerConfig{});

  void setConfig(const PendingStaticHazardTrackerConfig& config);
  const PendingStaticHazardTrackerConfig& config() const noexcept {
    return config_;
  }
  void reset();
  PendingStaticHazardDecision update(
      PendingStaticHazardObservation observation);

 private:
  PendingStaticHazardTrackerConfig config_;
  bool active_ = false;
  double last_stamp_sec_ = 0.0;
  std::uint64_t cargo_lifecycle_id_ = 0U;
  std::uint64_t map_generation_ = 0U;
  std::uint32_t obstacle_id_ = 0U;
  int confirmations_ = 0;
  std::vector<std::int64_t> matched_cell_keys_;
};

}  // namespace ndt_slam
