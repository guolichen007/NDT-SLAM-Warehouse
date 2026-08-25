#pragma once

#include "ndt_slam/cargo_avoidance_fusion.hpp"
#include "ndt_slam/cargo_physical_identity_authority.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace ndt_slam {

struct CargoShadowGeometryConfig {
  std::size_t minimum_point_support = 20U;
  int formal_confirm_frames = 3;
  double maximum_observation_gap_sec = 0.50;
  double maximum_xy_step_m = 0.30;
  double maximum_z_speed_mps = 1.50;
  double z_step_margin_m = 0.05;
  double minimum_length_m = 0.10;
  double maximum_length_m = 20.0;
  double minimum_width_m = 0.10;
  double maximum_width_m = 10.0;
  double minimum_height_m = 0.10;
  double maximum_height_m = 5.0;
  double maximum_size_cv = 0.25;
  double minimum_axial_orientation_concentration = 0.70;
};

struct CargoShadowResolvedGeometryObservation {
  bool valid = false;
  double source_stamp_sec = 0.0;
  Eigen::Vector2d footprint_center_base = Eigen::Vector2d::Zero();
  double physical_anchor_z = 0.0;
  double vertical_uncertainty_m = 0.20;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  double yaw_rad = 0.0;
  std::size_t point_support = 0U;
};

struct CargoPhysicalGroupEvidenceSnapshot {
  bool valid = false;
  std::uint64_t physical_history_id = 0U;
  std::uint64_t frame_group_id = 0U;
  std::uint64_t load_epoch = 0U;
  std::uint64_t lifecycle_id = 0U;
  double source_stamp_sec = 0.0;
  std::vector<Eigen::Vector3f> union_points_base;
  Eigen::Vector3d stable_anchor = Eigen::Vector3d::Zero();
  CargoGroupVerticalMode vertical_mode = CargoGroupVerticalMode::INVALID;
  CargoVerticalEvidenceSource vertical_source =
      CargoVerticalEvidenceSource::COMPONENT_UNION;
  bool supported_top_valid = false;
  double supported_top_z = std::numeric_limits<double>::quiet_NaN();
  double vertical_uncertainty_m = std::numeric_limits<double>::quiet_NaN();
  std::string vertical_reject_reason = "not_evaluated";
  bool geometry_resolved = false;
  CargoShadowResolvedGeometryObservation resolved_geometry;
  std::vector<std::uint64_t> member_component_ids;
};

// Binds a validated identity directly to its current frame-local physical
// group. Candidate ids are intentionally absent from this ownership boundary.
CargoPhysicalGroupEvidenceSnapshot bindCargoPhysicalGroupEvidence(
    const std::vector<CargoPhysicalGroupObservation>& groups,
    const CargoPhysicalIdentityDecision& identity,
    std::uint64_t lifecycle_id);

bool cargoPhysicalGroupEvidenceOwnerMatches(
    const CargoPhysicalGroupEvidenceSnapshot& snapshot,
    const CargoPhysicalIdentityDecision& identity,
    std::uint64_t lifecycle_id) noexcept;

struct CargoShadowGeometryInput {
  double stamp_sec = 0.0;
  CargoPhysicalIdentityDecision identity;
  CargoShadowResolvedGeometryObservation geometry;
};

struct CargoShadowGeometryDecision {
  bool identity_validated = false;
  bool pending_envelope_valid = false;
  bool formal_geometry_valid = false;
  bool formal_clear_authorized = false;
  bool reference_independent = true;
  bool geometry_resolved = false;
  std::uint64_t physical_history_id = 0U;
  int confirm_count = 0;
  double source_stamp_sec = 0.0;
  double window_start_stamp_sec = 0.0;
  Eigen::Vector3d median_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d median_size = Eigen::Vector3d::Zero();
  double axial_orientation_concentration = 0.0;
  std::string reject_reason = "not_evaluated";
};

class CargoShadowGeometryAuthority {
 public:
  explicit CargoShadowGeometryAuthority(
      const CargoShadowGeometryConfig& config = CargoShadowGeometryConfig{});
  void setConfig(const CargoShadowGeometryConfig& config);
  void reset(const std::string& reason = "explicit_reset");
  CargoShadowGeometryDecision update(const CargoShadowGeometryInput& input);
  const CargoShadowGeometryDecision& decision() const noexcept {
    return decision_;
  }

 private:
  CargoShadowGeometryConfig config_;
  CargoShadowGeometryDecision decision_;
  std::deque<CargoShadowResolvedGeometryObservation> window_;
  std::uint64_t history_id_ = 0U;
  double last_stamp_sec_ = 0.0;
  std::string reset_reason_ = "constructed";
};

struct CargoShadowThicknessProvenance {
  bool valid = false;
  std::uint64_t physical_history_id = 0U;
  std::uint64_t load_epoch = 0U;
  std::uint64_t lifecycle_id = 0U;
  std::string source;
  double source_stamp_sec = 0.0;
};

bool shadowThicknessAuthorized(
    const CargoShadowThicknessProvenance& provenance,
    const CargoPhysicalIdentityDecision& identity,
    std::uint64_t lifecycle_id) noexcept;

struct CargoShadowThicknessState {
  float frozen_thickness_m = 0.0F;
  CargoShadowThicknessProvenance provenance;

  void reset() noexcept;
  bool freezeFromFormalGeometry(
      const CargoPhysicalGroupEvidenceSnapshot& snapshot,
      const CargoPhysicalIdentityDecision& identity,
      const CargoShadowGeometryDecision& geometry,
      float minimum_height_m,
      float maximum_height_m);
};

struct CargoShadowFusionProjection {
  bool pending = false;
  bool formal = false;
  bool bottom_valid = false;
  bool clear_authorized = false;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_track_id = 0U;
  CargoAvoidanceSourceRisk live;
  CargoAvoidanceSourceRisk static_map;
};

// Copies every obstacle/provenance/history/authority field from the canonical
// snapshot and replaces Cargo-owned projection fields only.
CargoAvoidanceFusionInput projectShadowCargoOntoCanonicalFusion(
    const CargoAvoidanceFusionInput& canonical,
    const CargoShadowFusionProjection& shadow);

struct CargoShadowPhysicalDistanceTiming {
  double load_edge_stamp_sec = 0.0;
  double identity_validation_stamp_sec = 0.0;
  double pending_ready_stamp_sec = 0.0;
  double formal_ready_stamp_sec = 0.0;
  double first_obstacle_8m_stamp_sec = 0.0;
  double first_obstacle_5m_stamp_sec = 0.0;
  bool identity_validated_before_8m = false;
  bool pending_or_lock_ready_before_5m = false;
};

void updateShadowPhysicalDistanceTiming(
    double stamp_sec, double physical_distance_m,
    double level2_distance_m, double far_distance_m,
    bool identity_validated, bool pending_ready, bool formal_ready,
    CargoShadowPhysicalDistanceTiming* timing);

}  // namespace ndt_slam
