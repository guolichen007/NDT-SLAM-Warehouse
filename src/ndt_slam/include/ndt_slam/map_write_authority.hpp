#pragma once

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class MapWriteAuthorityPhase : std::uint8_t {
  SUBMIT_CURRENT = 0,
  ASYNC_COMPLETE = 1,
};

// Immutable evidence captured with one AcceptedPose. Per-frame pose
// generation and stamp must be current when submitted. An asynchronous worker
// may finish after newer accepted poses exist, so completion validates the
// continuity/map/static identities instead of comparing with the latest pose.
struct MapWriteAuthorityEvidence {
  MapWriteAuthorityPhase phase = MapWriteAuthorityPhase::SUBMIT_CURRENT;
  bool accepted_pose_valid = false;
  bool ndt_accepted = false;
  bool prediction_only = true;
  bool map_commit_quality_valid = false;
  bool localization_quarantined = true;
  bool pose_finite = false;
  std::uint64_t source_pose_generation = 0U;
  std::uint64_t latest_pose_generation = 0U;
  double source_stamp_sec = 0.0;
  double latest_stamp_sec = 0.0;
  std::uint64_t source_continuity_generation = 0U;
  std::uint64_t current_continuity_generation = 0U;
  std::uint64_t source_map_generation = 0U;
  std::uint64_t current_map_generation = 0U;
  std::string source_map_uuid;
  std::string current_map_uuid;
  std::uint64_t source_lifecycle_epoch = 0U;
  std::uint64_t current_lifecycle_epoch = 0U;
  std::uint64_t source_static_epoch = 0U;
  std::uint64_t current_static_epoch = 0U;
};

struct MapWriteAuthorityDecision {
  bool authorized = false;
  std::string reason = "not_evaluated";
};

MapWriteAuthorityDecision evaluateMapWriteAuthority(
    const MapWriteAuthorityEvidence& evidence);

}  // namespace ndt_slam
