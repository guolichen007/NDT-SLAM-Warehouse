#pragma once

#include "ndt_slam/static_obstacle_evidence_index.hpp"

#include <string>

namespace ndt_slam {

// One typed authority decision is shared by origin binding, thickness fusion,
// official hazard publication and clear authorization. Unverified clean-map
// geometry remains available for diagnostics only.
struct StaticEvidenceAuthorization {
  bool diagnostic_height_allowed = true;
  bool formal_origin_authorized = false;
  bool formal_thickness_authorized = false;
  bool official_static_risk_authorized = false;
  bool official_clear_authorized = false;
};

inline StaticEvidenceAuthorization authorizeStaticEvidence(
    StaticEvidenceAuthority authority) noexcept {
  StaticEvidenceAuthorization result;
  const bool formal =
      authority == StaticEvidenceAuthority::RUNTIME_MATURE ||
      authority == StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE;
  result.formal_origin_authorized = formal;
  result.formal_thickness_authorized = formal;
  result.official_static_risk_authorized = formal;
  result.official_clear_authorized = formal;
  return result;
}

// Guard C static-conflict context identity.  The static snapshot generation is
// bound to the STATIC-EVIDENCE epoch, not the pose map-rebuild generation: a
// source-time rollback advances the pose generation while the spatial static
// evidence (and its epoch) is deliberately preserved, so the two generations
// must never be compared across domains.  Pose authority is validated
// independently and is a hard precondition here.
struct StaticConflictContextAuthority {
  bool valid = false;
  std::string reason = "static_conflict_context_missing";
};

inline StaticConflictContextAuthority evaluateStaticConflictContextAuthority(
    bool snapshot_present, std::uint64_t snapshot_generation,
    std::uint64_t expected_static_evidence_epoch,
    StaticEvidenceAuthority snapshot_authority,
    bool pose_authority_valid) noexcept {
  StaticConflictContextAuthority result;
  if (!pose_authority_valid) {
    result.reason = "pose_authority_invalid";
    return result;
  }
  if (!snapshot_present) {
    result.reason = "static_snapshot_missing";
    return result;
  }
  if (snapshot_authority == StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN) {
    result.reason = "static_authority_unverified";
    return result;
  }
  if (snapshot_generation != expected_static_evidence_epoch) {
    result.reason = "static_evidence_epoch_mismatch";
    return result;
  }
  result.valid = true;
  result.reason = "none";
  return result;
}

}  // namespace ndt_slam
