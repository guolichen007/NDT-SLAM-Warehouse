#pragma once

#include <cstdint>
#include <string>

namespace ndt_slam {

struct MapWriteRearmConfig {
  int minimum_accepted_frames = 20;
  double minimum_duration_sec = 2.5;
};

struct MapWriteRearmEvidence {
  double stamp_sec = 0.0;
  bool pose_finite = false;
  bool ndt_accepted = false;
  bool prediction_only = true;
  bool relocalization_job_active = true;
  std::string map_uuid;
  std::uint64_t map_generation = 0U;
  std::uint64_t continuity_generation = 0U;
};

struct MapWriteRearmDecision {
  bool map_write_rearmed = false;
  int accepted_frames = 0;
  double stable_duration_sec = 0.0;
  std::string reason = "not_evaluated";
};

class MapWriteRearmPolicy {
 public:
  explicit MapWriteRearmPolicy(const MapWriteRearmConfig& config = {});
  void configure(const MapWriteRearmConfig& config);
  void reset(const std::string& reason = "reset");
  MapWriteRearmDecision update(const MapWriteRearmEvidence& evidence);
  const MapWriteRearmDecision& decision() const { return decision_; }

 private:
  MapWriteRearmConfig config_;
  MapWriteRearmDecision decision_;
  std::string map_uuid_;
  std::uint64_t map_generation_ = 0U;
  std::uint64_t continuity_generation_ = 0U;
  double first_stamp_sec_ = 0.0;
  double previous_stamp_sec_ = 0.0;
};

}  // namespace ndt_slam
