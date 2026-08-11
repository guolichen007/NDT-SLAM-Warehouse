#include "ndt_slam/map_write_rearm_policy.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {

MapWriteRearmPolicy::MapWriteRearmPolicy(const MapWriteRearmConfig& config) {
  configure(config);
}

void MapWriteRearmPolicy::configure(const MapWriteRearmConfig& config) {
  config_ = config;
  config_.minimum_accepted_frames =
      std::max(1, config_.minimum_accepted_frames);
  config_.minimum_duration_sec = std::max(0.0, config_.minimum_duration_sec);
  reset("configured");
}

void MapWriteRearmPolicy::reset(const std::string& reason) {
  decision_ = {};
  decision_.reason = reason;
  map_uuid_.clear();
  map_generation_ = 0U;
  continuity_generation_ = 0U;
  first_stamp_sec_ = 0.0;
  previous_stamp_sec_ = 0.0;
}

MapWriteRearmDecision MapWriteRearmPolicy::update(
    const MapWriteRearmEvidence& evidence) {
  if (decision_.map_write_rearmed) return decision_;
  const bool sample_valid = std::isfinite(evidence.stamp_sec) &&
      evidence.stamp_sec > 0.0 && evidence.pose_finite &&
      evidence.ndt_accepted && !evidence.prediction_only &&
      !evidence.relocalization_job_active && !evidence.map_uuid.empty() &&
      evidence.continuity_generation > 0U;
  if (!sample_valid) {
    reset("stabilization_sample_invalid");
    return decision_;
  }
  if (decision_.accepted_frames == 0) {
    map_uuid_ = evidence.map_uuid;
    map_generation_ = evidence.map_generation;
    continuity_generation_ = evidence.continuity_generation;
    first_stamp_sec_ = evidence.stamp_sec;
  } else if (evidence.stamp_sec <= previous_stamp_sec_ ||
             evidence.map_uuid != map_uuid_ ||
             evidence.map_generation != map_generation_ ||
             evidence.continuity_generation != continuity_generation_) {
    reset("stabilization_identity_or_time_changed");
    return decision_;
  }
  previous_stamp_sec_ = evidence.stamp_sec;
  ++decision_.accepted_frames;
  decision_.stable_duration_sec = evidence.stamp_sec - first_stamp_sec_;
  if (decision_.accepted_frames >= config_.minimum_accepted_frames &&
      decision_.stable_duration_sec >= config_.minimum_duration_sec) {
    decision_.map_write_rearmed = true;
    decision_.reason = "map_write_rearmed";
  } else {
    decision_.reason = "readonly_stabilizing";
  }
  return decision_;
}

}  // namespace ndt_slam
