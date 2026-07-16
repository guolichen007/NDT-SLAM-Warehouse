#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace ndt_slam {

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
  float minimum_overlap_ratio = 0.30F;
  float maximum_shape_relative_error = 0.60F;
  float maximum_axial_yaw_error_rad = 0.35F;
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
  std::string reason = "not_evaluated";
};

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

}  // namespace ndt_slam
