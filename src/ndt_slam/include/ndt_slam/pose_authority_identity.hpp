#pragma once

#include <cstdint>
#include <cmath>
#include <string>

namespace ndt_slam {

struct PoseAuthorityIdentity {
  std::uint64_t map_rebuild_generation = 0U;
  std::uint64_t keyframe_pose_version = 0U;
  std::uint64_t yaw_authority_generation = 0U;
  std::string map_frame_uuid;
  std::string yaw_reference_hash;
  std::uint64_t target_snapshot_id = 0U;

  bool validForRail() const noexcept {
    return !map_frame_uuid.empty() && !yaw_reference_hash.empty() &&
        yaw_authority_generation != 0U && target_snapshot_id != 0U;
  }
};

inline bool samePoseAuthorityIdentity(const PoseAuthorityIdentity& lhs,
                                      const PoseAuthorityIdentity& rhs) {
  return lhs.map_rebuild_generation == rhs.map_rebuild_generation &&
      lhs.keyframe_pose_version == rhs.keyframe_pose_version &&
      lhs.yaw_authority_generation == rhs.yaw_authority_generation &&
      lhs.map_frame_uuid == rhs.map_frame_uuid &&
      lhs.yaw_reference_hash == rhs.yaw_reference_hash &&
      lhs.target_snapshot_id == rhs.target_snapshot_id;
}

// Provenance carried by an actual temporal evidence owner. The source stamp
// is the stamp of the physical evidence, not the frame that happens to consume
// a held result later.
struct TemporalEvidenceAuthority {
  bool valid = false;
  PoseAuthorityIdentity pose_identity;
  double source_stamp_sec = 0.0;
};

inline TemporalEvidenceAuthority bindTemporalEvidenceAuthority(
    const PoseAuthorityIdentity& identity, double source_stamp_sec) {
  TemporalEvidenceAuthority authority;
  authority.valid = true;
  authority.pose_identity = identity;
  authority.source_stamp_sec = source_stamp_sec;
  return authority;
}

inline bool sameTemporalEvidenceAuthority(
    const TemporalEvidenceAuthority& lhs,
    const TemporalEvidenceAuthority& rhs) {
  return lhs.valid && rhs.valid &&
      samePoseAuthorityIdentity(lhs.pose_identity, rhs.pose_identity);
}

// Binds a newly produced evidence stamp to the current frame, while an
// unchanged stamp retains the identity of the frame that produced it. A
// historical stamp that appears without an existing binding is deliberately
// rejected instead of being relabelled with the consuming frame.
inline TemporalEvidenceAuthority advanceTemporalEvidenceAuthority(
    const TemporalEvidenceAuthority& previous,
    double evidence_stamp_sec,
    const PoseAuthorityIdentity& current_identity,
    double current_frame_stamp_sec,
    double stamp_epsilon_sec = 1.0e-4) {
  if (!std::isfinite(evidence_stamp_sec) || evidence_stamp_sec <= 0.0 ||
      !std::isfinite(current_frame_stamp_sec) ||
      current_frame_stamp_sec <= 0.0) {
    return TemporalEvidenceAuthority{};
  }
  if (std::abs(evidence_stamp_sec - current_frame_stamp_sec) <=
      stamp_epsilon_sec) {
    return bindTemporalEvidenceAuthority(
        current_identity, evidence_stamp_sec);
  }
  if (previous.valid &&
      std::abs(previous.source_stamp_sec - evidence_stamp_sec) <=
          stamp_epsilon_sec) {
    return previous;
  }
  return TemporalEvidenceAuthority{};
}

}  // namespace ndt_slam
