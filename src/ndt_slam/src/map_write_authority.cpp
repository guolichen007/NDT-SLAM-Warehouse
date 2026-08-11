#include "ndt_slam/map_write_authority.hpp"

#include <cmath>

namespace ndt_slam {

MapWriteAuthorityDecision evaluateMapWriteAuthority(
    const MapWriteAuthorityEvidence& evidence) {
  MapWriteAuthorityDecision result;
  if (!evidence.accepted_pose_valid) {
    result.reason = "accepted_pose_invalid";
    return result;
  }
  if (!evidence.ndt_accepted || evidence.prediction_only) {
    result.reason = "ndt_rejected_or_prediction_only";
    return result;
  }
  if (!evidence.map_commit_quality_valid) {
    result.reason = "map_commit_quality_invalid";
    return result;
  }
  if (evidence.localization_quarantined) {
    result.reason = "localization_quarantined";
    return result;
  }
  if (!evidence.startup_recovery_verified) {
    result.reason = "startup_recovery_not_verified";
    return result;
  }
  if (!evidence.map_write_rearmed) {
    result.reason = "map_write_not_rearmed";
    return result;
  }
  if (!evidence.pose_finite ||
      !std::isfinite(evidence.source_stamp_sec) ||
      evidence.source_stamp_sec <= 0.0 ||
      evidence.source_pose_generation == 0U) {
    result.reason = "accepted_pose_identity_invalid";
    return result;
  }
  if (evidence.source_continuity_generation == 0U ||
      evidence.source_continuity_generation !=
          evidence.current_continuity_generation) {
    result.reason = "localization_continuity_mismatch";
    return result;
  }
  if (evidence.source_map_generation != evidence.current_map_generation ||
      evidence.source_map_uuid.empty() ||
      evidence.source_map_uuid != evidence.current_map_uuid) {
    result.reason = "localization_map_identity_mismatch";
    return result;
  }
  if (evidence.source_lifecycle_epoch != evidence.current_lifecycle_epoch) {
    result.reason = "map_lifecycle_epoch_mismatch";
    return result;
  }
  if (evidence.source_static_epoch != evidence.current_static_epoch) {
    result.reason = "static_evidence_epoch_mismatch";
    return result;
  }
  if (evidence.phase == MapWriteAuthorityPhase::SUBMIT_CURRENT) {
    if (evidence.source_pose_generation !=
        evidence.latest_pose_generation) {
      result.reason = "accepted_pose_not_current";
      return result;
    }
    if (!std::isfinite(evidence.latest_stamp_sec) ||
        std::abs(evidence.source_stamp_sec - evidence.latest_stamp_sec) >
            1.0e-6) {
      result.reason = "accepted_pose_stamp_not_current";
      return result;
    }
  }
  result.authorized = true;
  result.reason = evidence.phase == MapWriteAuthorityPhase::SUBMIT_CURRENT
      ? "accepted_pose_current" : "accepted_pose_lineage_current";
  return result;
}

}  // namespace ndt_slam
