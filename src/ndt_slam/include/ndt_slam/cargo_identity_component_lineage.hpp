#pragma once

#include "ndt_slam/avoidance_map_mutation.hpp"
#include "ndt_slam/pose_authority_identity.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace ndt_slam {

// Compact, frame-owned description of one accepted S4 component.  This is an
// identity-only input: it deliberately contains no points, vertical evidence,
// geometry authority, ownership or map-mutation fields.
struct CargoIdentityComponentDescriptor {
  std::uint64_t component_id = 0U;
  std::uint64_t exact_seed_frame_group_id = 0U;
  double source_stamp_sec = 0.0;
  Eigen::Vector2d center_base = Eigen::Vector2d::Zero();
  double robust_x05 = 0.0;
  double robust_x95 = 0.0;
  double robust_y05 = 0.0;
  double robust_y95 = 0.0;
  Eigen::Vector2d robust_xy_extent = Eigen::Vector2d::Zero();
};

enum class CargoIdentityLineageState : std::uint8_t {
  UNAVAILABLE = 0,
  MATCHED,
  AMBIGUOUS,
  WORLD_STATIC_VETO,
};

// A temporal correspondence between accepted components in two adjacent
// source frames.  "Lineage" is not an intra-frame component merge and never
// carries vertical, point, exact-owner, geometry or safety authority.
struct CargoIdentitySupportLineageObservation {
  bool valid = false;
  CargoIdentityLineageState state = CargoIdentityLineageState::UNAVAILABLE;
  double previous_source_stamp_sec = 0.0;
  double source_stamp_sec = 0.0;
  std::uint64_t previous_component_id = 0U;
  std::uint64_t current_component_id = 0U;
  std::uint64_t exact_seed_frame_group_id = 0U;
  Eigen::Vector2d robust_xy_center = Eigen::Vector2d::Zero();
  Eigen::Vector2d robust_xy_extent = Eigen::Vector2d::Zero();
  double robust_x05 = 0.0;
  double robust_x95 = 0.0;
  double robust_y05 = 0.0;
  double robust_y95 = 0.0;
  double base_step_m = 0.0;
  double map_step_m = 0.0;
  double extent_step = 0.0;
};

struct CargoIdentityComponentLineageConfig {
  double maximum_xy_step_m = 0.30;
  double maximum_size_relative_step = 0.60;
  double maximum_observation_gap_sec = 0.50;
  double ambiguity_cost_margin = 0.08;
};

struct CargoIdentityComponentLineageFrame {
  double source_stamp_sec = 0.0;
  std::uint64_t lifecycle_id = 0U;
  SourceFrameIdentity source_frame_identity;
  PoseAuthorityIdentity pose_identity;
  Eigen::Matrix4d pose_map_base = Eigen::Matrix4d::Identity();
  std::vector<CargoIdentityComponentDescriptor> components;
};

struct CargoIdentityComponentLineageResult {
  std::vector<CargoIdentitySupportLineageObservation> observations;
  std::size_t pair_count = 0U;
  std::size_t match_count = 0U;
  std::size_t ambiguous_count = 0U;
  std::size_t world_static_veto_count = 0U;
  std::string reset_reason = "none";
};

// Owns exactly one previous source frame of compact descriptors.  It owns no
// Cargo identity: CargoPhysicalIdentityAuthority remains the sole temporal
// owner of Cargo histories, pre-lift reference, lift and validation state.
class CargoIdentityComponentLineage {
 public:
  explicit CargoIdentityComponentLineage(
      const CargoIdentityComponentLineageConfig& config = {});

  void setConfig(const CargoIdentityComponentLineageConfig& config);
  void reset(const std::string& reason = "explicit_reset");
  CargoIdentityComponentLineageResult update(
      const CargoIdentityComponentLineageFrame& frame);

 private:
  CargoIdentityComponentLineageConfig config_;
  CargoIdentityComponentLineageFrame previous_;
  bool has_previous_ = false;
  std::string reset_reason_ = "constructed";
};

}  // namespace ndt_slam
