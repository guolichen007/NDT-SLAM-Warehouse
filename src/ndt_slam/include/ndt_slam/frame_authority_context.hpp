#pragma once

#include "ndt_slam/rail_localization_authority.hpp"

#include <sophus/se3.hpp>

#include <cstdint>
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

  bool safetyAuthorized() const noexcept {
    return localization_health.safety_localization_authorized &&
        (!rail_authority_mode || pose_identity.validForRail());
  }

  bool mapMutationAuthorized() const noexcept {
    return localization_health.map_mutation_authorized &&
        (!rail_authority_mode || pose_identity.validForRail());
  }
};

}  // namespace ndt_slam
