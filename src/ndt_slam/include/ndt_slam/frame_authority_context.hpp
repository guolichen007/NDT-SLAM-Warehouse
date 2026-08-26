#pragma once

#include "ndt_slam/pose_authority_identity.hpp"
#include "ndt_slam/rail_localization_authority.hpp"

#include <sophus/se3.hpp>

namespace ndt_slam {

// Created exactly once after the frame's final EKF/runtime pose is known.
// Map-frame consumers receive this value instead of rebuilding provenance
// independently from mutable node members.
struct FrameAuthorityContext {
  Sophus::SE3d runtime_pose;
  PoseAuthorityIdentity pose_identity;
  LocalizationAuthorityHealth localization_health;
  LocalizationFailureClass failure_class = LocalizationFailureClass::NONE;
  double source_stamp_sec = 0.0;
  bool rail_authority_mode = false;
  bool observability_map_mutation_authorized = true;

  bool safetyAuthorized() const noexcept {
    return localization_health.safety_localization_authorized &&
        (!rail_authority_mode || pose_identity.validForRail());
  }

  bool mapMutationAuthorized() const noexcept {
    return localization_health.map_mutation_authorized &&
        observability_map_mutation_authorized &&
        (!rail_authority_mode || pose_identity.validForRail());
  }
};

inline bool safetyFrameAuthorityMatches(
    const FrameAuthorityContext& frame,
    const TemporalEvidenceAuthority& cargo_authority,
    const TemporalEvidenceAuthority& obstacle_authority) {
  return frame.safetyAuthorized() &&
      cargo_authority.valid && obstacle_authority.valid &&
      samePoseAuthorityIdentity(
          frame.pose_identity, cargo_authority.pose_identity) &&
      samePoseAuthorityIdentity(
          frame.pose_identity, obstacle_authority.pose_identity);
}

}  // namespace ndt_slam
