#include "ndt_slam/product_cargo_context.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {

ProductCargoContext buildLegacyProductCargoContext(
    const LegacyProductCargoContextInput& input) {
  ProductCargoContext output;
  output.mode = CargoAuthorityMode::LEGACY;
  output.valid_input = true;
  output.identity_authorized = input.active_track;
  output.geometry_authorized = input.frozen_geometry_ready &&
      input.geometry.valid;
  output.bottom_authorized = input.bottom.valid &&
      input.bottom.geometry_valid;
  output.safety_authorized = output.identity_authorized &&
      output.geometry_authorized && output.bottom_authorized &&
      input.formal_use.formal_safety_valid;
  output.clear_authorized = output.safety_authorized &&
      input.clear_authorized;
  output.self_removal_authorized = output.safety_authorized &&
      input.self_removal_authorized;
  output.map_mutation_authorized = output.self_removal_authorized;
  output.source_stamp_sec = input.bottom.stamp_sec;
  output.cargo_lifecycle_id = input.cargo_lifecycle_id;
  output.cargo_id = input.cargo_id;
  output.geometry = input.geometry;
  output.bottom = input.bottom;
  output.formal_use = input.formal_use;
  output.temporal_authority = input.bottom.pose_authority;
  output.reason = output.safety_authorized
      ? "legacy_authorized" : "legacy_formal_gate_closed";
  return output;
}

ProductCargoContext buildV6ProductCargoContext(
    const V6ProductCargoContextInput& input) {
  ProductCargoContext output;
  output.mode = CargoAuthorityMode::V6_AUTHORITY;
  output.valid_input = input.canonical.valid_input;
  output.source_stamp_sec = input.canonical.source_stamp_sec;
  output.cargo_lifecycle_id = input.identity.physical_cargo_epoch_id;
  output.cargo_id = input.identity.physical_history_id;
  // The canonical snapshot proves the identity/geometry permissions while
  // BottomFusion remains the sole vertical estimator. Preserve its full
  // result rather than synthesizing a second vertical algorithm.
  output.bottom = input.bottom;
  output.reason = input.canonical.reason;
  if (!input.canonical.cargo_safety_authorized ||
      !input.canonical.safety_geometry.valid ||
      !input.bottom.valid || !input.bottom.geometry_valid ||
      !input.pose_map_base.translation().allFinite() ||
      !input.pose_map_base.so3().matrix().allFinite() ||
      !std::isfinite(input.evaluation_stamp_sec) ||
      input.evaluation_stamp_sec <= 0.0 ||
      !std::isfinite(input.horizontal_uncertainty_m) ||
      input.horizontal_uncertainty_m < 0.0F) {
    return output;
  }

  const CanonicalCargoGeometry& canonical =
      input.canonical.safety_geometry;
  LockedCargoShape shape;
  shape.valid = true;
  shape.length_m = canonical.footprint_base.length_m;
  shape.width_m = canonical.footprint_base.width_m;
  shape.height_m = canonical.top_z_base - canonical.bottom_z_base;
  shape.yaw_base_rad = canonical.footprint_base.yaw_base_rad;
  shape.orientation_confidence = 1.0F;
  LiveCargoPose live;
  live.valid = shape.valid && std::isfinite(shape.height_m) &&
      shape.height_m > 0.0F;
  live.center_base = Eigen::Vector3f(
      canonical.footprint_base.center_base.x(),
      canonical.footprint_base.center_base.y(),
      0.5F * (canonical.bottom_z_base + canonical.top_z_base));
  live.evidence_stamp_sec = input.canonical.source_stamp_sec;
  live.evaluation_stamp_sec = input.evaluation_stamp_sec;
  live.source = CargoPoseSource::CURRENT_ASSOCIATED_LIDAR;
  live.vertical_source = CargoVerticalPoseSource::DIRECT_TOP;
  live.position_uncertainty_m = input.horizontal_uncertainty_m;
  if (!live.valid || !live.center_base.allFinite()) {
    output.reason = "v6_geometry_nonfinite";
    return output;
  }

  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.matrix() = input.pose_map_base.matrix().cast<float>();
  output.geometry = buildCurrentRigidCargoGeometry(
      shape, live, transform, input.identity.physical_history_id,
      input.horizontal_uncertainty_m,
      std::max(0.0F, input.bottom.uncertainty));
  output.geometry.pose_evidence_stamp_sec =
      input.canonical.source_stamp_sec;
  output.geometry.height_evidence_stamp_sec =
      input.canonical.source_stamp_sec;
  output.geometry.evaluation_stamp_sec = input.evaluation_stamp_sec;
  output.geometry.reason = output.geometry.valid
      ? "v6_current_owner_geometry" : "v6_geometry_build_failed";
  output.identity_authorized = output.geometry.valid;
  output.geometry_authorized = output.geometry.valid &&
      input.geometry_authority.formal_geometry_valid;
  output.bottom_authorized = input.bottom.valid &&
      input.bottom.geometry_valid &&
      input.bottom.track_id == input.identity.physical_history_id;
  output.safety_authorized = output.identity_authorized &&
      output.geometry_authorized && output.bottom_authorized;
  output.clear_authorized = output.safety_authorized &&
      input.geometry_authority.formal_clear_authorized;
  output.self_removal_authorized =
      input.canonical.cargo_map_mutation_authorized;
  output.map_mutation_authorized =
      input.canonical.cargo_map_mutation_authorized;
  output.formal_use.display_valid = output.geometry.valid;
  output.formal_use.formal_safety_valid = output.safety_authorized;
  output.formal_use.formal_removal_valid =
      output.self_removal_authorized;
  output.formal_use.horizontal_uncertainty_m =
      input.horizontal_uncertainty_m;
  output.formal_use.reason = output.safety_authorized
      ? "v6_authorized" : "v6_formal_gate_closed";
  output.temporal_authority = input.bottom.pose_authority;
  output.reason = output.safety_authorized
      ? "v6_authorized" : output.formal_use.reason;
  return output;
}

ProductCargoSelection selectProductCargoContext(
    CargoAuthorityMode mode,
    const ProductCargoContext& legacy,
    const ProductCargoContext& v6,
    bool legacy_positive_hazard,
    bool legacy_hazard_same_authority) {
  ProductCargoSelection output;
  if (mode == CargoAuthorityMode::LEGACY ||
      mode == CargoAuthorityMode::V6_SHADOW) {
    output.product = legacy;
    output.product.mode = mode;
    output.reason = mode == CargoAuthorityMode::LEGACY
        ? "legacy_product" : "shadow_legacy_product";
    return output;
  }
  output.product = v6;
  output.product.mode = CargoAuthorityMode::V6_AUTHORITY;
  if (v6.safety_authorized) {
    output.reason = "v6_product";
    return output;
  }
  output.product.clear_authorized = false;
  output.product.self_removal_authorized = false;
  output.product.map_mutation_authorized = false;
  output.legacy_clear_rejected = legacy.clear_authorized;
  output.legacy_positive_hazard_retained =
      legacy_positive_hazard && legacy_hazard_same_authority;
  output.reason = output.legacy_positive_hazard_retained
      ? "v6_invalid_legacy_positive_only_retained"
      : "v6_invalid_fail_closed";
  return output;
}

bool v6LiveSelfRemovalAuthorityCurrent(
    const ProductCargoContext& product,
    const CanonicalCargoAuthoritySnapshot& canonical,
    const SourceFrameIdentity& source_frame_identity,
    const PoseAuthorityIdentity& pose_identity,
    double source_stamp_sec) noexcept {
  constexpr double kStampEpsilonSec = 1.0e-6;
  return product.mode == CargoAuthorityMode::V6_AUTHORITY &&
      product.valid_input && product.safety_authorized &&
      product.self_removal_authorized &&
      product.map_mutation_authorized &&
      canonical.valid_input && canonical.cargo_safety_authorized &&
      canonical.cargo_map_mutation_authorized &&
      canonical.map_mutation.authorized &&
      canonical.map_mutation.tight_geometry_valid &&
      canonical.physical_history_id != 0U &&
      canonical.physical_history_id == product.cargo_id &&
      canonical.physical_cargo_epoch_id != 0U &&
      canonical.physical_cargo_epoch_id == product.cargo_lifecycle_id &&
      std::isfinite(source_stamp_sec) && source_stamp_sec > 0.0 &&
      std::abs(product.source_stamp_sec - source_stamp_sec) <=
          kStampEpsilonSec &&
      std::abs(canonical.source_stamp_sec - source_stamp_sec) <=
          kStampEpsilonSec &&
      source_frame_identity.valid() &&
      std::abs(source_frame_identity.sensor_source_stamp_sec -
               source_stamp_sec) <= kStampEpsilonSec &&
      samePoseAuthorityIdentity(canonical.pose_identity, pose_identity) &&
      sameSourceFrameIdentity(
          canonical.map_mutation.owner_points.source_frame_identity,
          source_frame_identity);
}

bool shouldRemoveV6LiveCargoSelfPoint(
    const ProductCargoContext& product,
    const CanonicalCargoAuthoritySnapshot& canonical,
    const SourceFrameIdentity& source_frame_identity,
    const PoseAuthorityIdentity& pose_identity,
    double source_stamp_sec,
    const pcl::PointXYZ& point) noexcept {
  return v6LiveSelfRemovalAuthorityCurrent(
             product, canonical, source_frame_identity, pose_identity,
             source_stamp_sec) &&
      canonical.map_mutation.ownsCurrentPoint(point);
}

float cargoLiveSelfRemovalMargin(
    CargoAuthorityMode mode,
    float base_margin,
    float maximum_margin,
    float legacy_tracking_residual,
    float product_uncertainty) noexcept {
  const float finite_base = std::isfinite(base_margin)
      ? std::max(0.0F, base_margin) : 0.0F;
  const float finite_maximum = std::isfinite(maximum_margin)
      ? std::max(finite_base, maximum_margin) : finite_base;
  const float finite_uncertainty = std::isfinite(product_uncertainty)
      ? std::max(0.0F, product_uncertainty) : 0.0F;
  const float finite_legacy_residual =
      mode == CargoAuthorityMode::V6_AUTHORITY
      ? 0.0F
      : (std::isfinite(legacy_tracking_residual)
             ? std::max(0.0F, legacy_tracking_residual) : 0.0F);
  return std::min(
      finite_maximum,
      finite_base + finite_legacy_residual + finite_uncertainty);
}

}  // namespace ndt_slam
