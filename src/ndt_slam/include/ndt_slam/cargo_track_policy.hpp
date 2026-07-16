#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ndt_slam/hook_load_evidence_policy.hpp"

namespace ndt_slam {

enum class CargoLockAuthoritySource : std::uint8_t {
  NONE = 0,
  GRAVITY_LOADED = 1,
  LIDAR_SUSPENDED = 2,
  LIFT_FROM_ORIGIN = 3
};

const char* cargoLockAuthoritySourceName(CargoLockAuthoritySource source);

struct CargoPhysicalLockAuthorityInput {
  HookLoadSignalRole signal_role = HookLoadSignalRole::AUXILIARY;
  bool gravity_valid = false;
  HookLoadState gravity_state = HookLoadState::UNKNOWN;
  float ground_clearance_m = 0.0F;
  float minimum_ground_clearance_m = 0.30F;
  float lift_from_origin_m = 0.0F;
  float minimum_lift_from_origin_m = 0.25F;
  int suspension_confirm_frames = 0;
  int lift_confirm_frames = 0;
  int required_lidar_confirm_frames = 3;
};

struct CargoPhysicalLockAuthorityDecision {
  bool allowed = false;
  CargoLockAuthoritySource source = CargoLockAuthoritySource::NONE;
  bool gravity_conflict = false;
  std::string reason = "physical_authority_missing";
};

CargoPhysicalLockAuthorityDecision evaluateCargoPhysicalLockAuthority(
    const CargoPhysicalLockAuthorityInput& input);

struct CargoRearmInput {
  bool empty_confirmed = false;
  double rearm_age_sec = 0.0;
  double minimum_empty_confirm_sec = 1.0;
  bool gravity_valid = false;
  HookLoadState gravity_state = HookLoadState::UNKNOWN;
  HookLoadState gravity_state_at_clear = HookLoadState::UNKNOWN;
  bool candidate_valid = false;
  bool independent_suspension_evidence = false;
  float candidate_score_margin = 0.0F;
  float minimum_score_margin = 0.08F;
  float retired_identity_confidence = 1.0F;
  float maximum_retired_identity_confidence = 0.62F;
};

struct CargoRearmDecision {
  bool allowed = false;
  std::string reason = "rearm_evidence_missing";
};

CargoRearmDecision evaluateCargoRearm(const CargoRearmInput& input);

struct CargoCandidateDescriptor {
  int component_id = -1;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f size = Eigen::Vector3f::Zero();
  float yaw_rad = 0.0F;
  float orientation_confidence = 0.0F;
  std::size_t point_count = 0U;
  bool suspension_evidence = false;
};

struct CargoCandidateIdentityContext {
  Eigen::Vector2f hook_center = Eigen::Vector2f::Zero();
  float hook_region_radius_m = 1.5F;
  std::size_t strong_point_count = 80U;
  bool predicted_track_valid = false;
  Eigen::Vector3f predicted_center = Eigen::Vector3f::Zero();
  Eigen::Vector3f predicted_size = Eigen::Vector3f::Zero();
  float predicted_yaw_rad = 0.0F;
  float association_radius_m = 0.65F;
};

struct CargoCandidateIdentityScore {
  bool valid = false;
  int component_id = -1;
  float hook_distance_score = 0.0F;
  float predicted_center_score = 0.0F;
  float overlap_score = 0.0F;
  float shape_confidence = 0.0F;
  float motion_confidence = 0.0F;
  float suspension_confidence = 0.0F;
  float point_support_confidence = 0.0F;
  float identity_confidence = 0.0F;
  float overall_lock_confidence = 0.0F;
  std::string reason = "not_evaluated";
};

struct CargoCandidateRanking {
  bool valid = false;
  CargoCandidateIdentityScore top1;
  CargoCandidateIdentityScore top2;
  float top1_rank = 0.0F;
  float top2_rank = 0.0F;
  float margin = 0.0F;
};

CargoCandidateRanking rankCargoCandidateIdentityScores(
    const std::vector<CargoCandidateIdentityScore>& scores);

struct CargoAssociationInput {
  CargoCandidateDescriptor candidate;
  Eigen::Vector3f previous_center = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f locked_size = Eigen::Vector3f::Zero();
  float locked_yaw_rad = 0.0F;
  double sensor_dt_sec = 0.0;
  float base_center_gate_m = 0.65F;
  float max_xy_speed_mps = 2.0F;
  float max_z_speed_mps = 1.5F;
  float horizontal_uncertainty_m = 0.0F;
  float vertical_uncertainty_m = 0.0F;
  float horizontal_tracking_residual_m = 0.0F;
  float vertical_tracking_residual_m = 0.0F;
  float velocity_model_uncertainty_mps = 0.05F;
  float maximum_xy_gate_m = 0.80F;
  float maximum_z_gate_m = 0.90F;
  float minimum_overlap_ratio = 0.30F;
  float maximum_shape_relative_error = 0.60F;
  float maximum_axial_yaw_error_rad = 0.35F;
  bool use_shape_as_hard_gate = true;
  bool use_yaw_as_hard_gate = true;
  bool strict_reacquisition = false;
};

struct CargoAssociationDecision {
  bool accepted = false;
  Eigen::Vector3f predicted_center = Eigen::Vector3f::Zero();
  float dynamic_xy_gate_m = 0.0F;
  float dynamic_z_gate_m = 0.0F;
  float center_residual_xy_m = 0.0F;
  float center_residual_z_m = 0.0F;
  float overlap_ratio = 0.0F;
  float length_relative_error = 0.0F;
  float width_relative_error = 0.0F;
  float height_relative_error = 0.0F;
  float axial_yaw_error_rad = 0.0F;
  bool yaw_used_as_hard_gate = true;
  std::string reason = "not_evaluated";
};

struct CargoFrozenObbSupportInput {
  std::vector<Eigen::Vector3f> points;
  Eigen::Vector3f predicted_center = Eigen::Vector3f::Zero();
  Eigen::Vector3f locked_size = Eigen::Vector3f::Zero();
  float locked_yaw_rad = 0.0F;
  float horizontal_margin_m = 0.15F;
  float vertical_margin_m = 0.20F;
};

struct CargoFrozenObbSupport {
  bool valid = false;
  std::size_t finite_points = 0U;
  std::size_t inside_points = 0U;
  float inside_ratio = 0.0F;
  float long_axis_coverage_ratio = 0.0F;
  float short_axis_coverage_ratio = 0.0F;
  float vertical_coverage_ratio = 0.0F;
  std::string reason = "not_evaluated";
};

CargoFrozenObbSupport evaluateCargoFrozenObbSupport(
    const CargoFrozenObbSupportInput& input);

struct CargoProvisionalLockConfig {
  std::size_t minimum_frames = 3U;
  float maximum_center_step_m = 0.30F;
  float maximum_shape_cv = 0.18F;
  float minimum_orientation_concentration = 0.70F;
  float minimum_identity_confidence = 0.65F;
  float minimum_overall_lock_confidence = 0.70F;
};

struct CargoProvisionalLockSummary {
  bool formal_lock_allowed = false;
  Eigen::Vector3f median_center = Eigen::Vector3f::Zero();
  Eigen::Vector3f median_size = Eigen::Vector3f::Zero();
  float axial_yaw_rad = 0.0F;
  float orientation_confidence = 0.0F;
  float shape_confidence = 0.0F;
  float motion_confidence = 0.0F;
  float identity_confidence = 0.0F;
  float suspension_confidence = 0.0F;
  float overall_lock_confidence = 0.0F;
  std::string reason = "not_evaluated";
};

float cargoOrientedOverlapRatio(
    const CargoCandidateDescriptor& lhs,
    const CargoCandidateDescriptor& rhs);

CargoCandidateIdentityScore scoreCargoCandidateIdentity(
    const CargoCandidateDescriptor& candidate,
    const CargoCandidateIdentityContext& context);

CargoAssociationDecision evaluateCargoPredictedAssociation(
    const CargoAssociationInput& input);

CargoProvisionalLockSummary summarizeCargoProvisionalLock(
    const std::vector<CargoCandidateDescriptor>& observations,
    const std::vector<CargoCandidateIdentityScore>& scores,
    const CargoProvisionalLockConfig& config);

/** Snap an axial OBB yaw to the nearest installation axis (0 or +/- pi/2). */
float quantizeCargoAxialYawToOrthogonal(float yaw_rad);

}  // namespace ndt_slam
