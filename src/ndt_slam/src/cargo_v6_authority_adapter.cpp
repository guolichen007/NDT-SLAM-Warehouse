#include "ndt_slam/cargo_v6_authority_adapter.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

constexpr double kStampEpsilonSec = 1.0e-4;

std::string upper(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char character) {
                   return static_cast<char>(std::toupper(character));
                 });
  return value;
}

bool finiteResolvedGeometry(
    const CargoShadowResolvedGeometryObservation& geometry) {
  return geometry.valid && geometry.footprint_center_base.allFinite() &&
      geometry.size.allFinite() && std::isfinite(geometry.yaw_rad) &&
      geometry.size.x() > 0.0 && geometry.size.y() > 0.0;
}

}  // namespace

const char* cargoAuthorityModeName(CargoAuthorityMode mode) noexcept {
  switch (mode) {
    case CargoAuthorityMode::LEGACY:
      return "LEGACY";
    case CargoAuthorityMode::V6_SHADOW:
      return "V6_SHADOW";
    case CargoAuthorityMode::V6_AUTHORITY:
      return "V6_AUTHORITY";
  }
  return "LEGACY";
}

bool parseCargoAuthorityMode(const std::string& value,
                             CargoAuthorityMode* mode) noexcept {
  if (mode == nullptr) return false;
  const std::string normalized = upper(value);
  if (normalized == "LEGACY") {
    *mode = CargoAuthorityMode::LEGACY;
    return true;
  }
  if (normalized == "V6_SHADOW" || normalized == "SHADOW") {
    *mode = CargoAuthorityMode::V6_SHADOW;
    return true;
  }
  if (normalized == "V6_AUTHORITY" || normalized == "AUTHORITY") {
    *mode = CargoAuthorityMode::V6_AUTHORITY;
    return true;
  }
  return false;
}

CanonicalCargoAuthoritySnapshot buildCanonicalCargoAuthoritySnapshot(
    const CanonicalCargoAuthorityInput& input) {
  CanonicalCargoAuthoritySnapshot output;
  output.source_stamp_sec = input.source_stamp_sec;
  output.pose_identity = input.pose_identity;
  output.physical_history_id = input.identity.physical_history_id;
  output.physical_cargo_epoch_id =
      input.identity.physical_cargo_epoch_id;

  if (!std::isfinite(input.source_stamp_sec) ||
      input.source_stamp_sec <= 0.0 ||
      !input.source_frame_identity.valid() ||
      std::abs(input.source_stamp_sec -
               input.source_frame_identity.sensor_source_stamp_sec) >
          kStampEpsilonSec ||
      !std::isfinite(input.owner_voxel_size_m) ||
      input.owner_voxel_size_m <= 0.0F) {
    output.reason = "invalid_source";
    return output;
  }
  output.valid_input = true;

  const bool identity_current = input.identity.valid_input &&
      input.identity.identity == CargoPhysicalIdentityState::VALIDATED &&
      input.identity.lift_confirmed &&
      input.identity.current_candidate_fresh &&
      input.identity.geometry_resolved &&
      input.identity.physical_history_id != 0U;
  if (!identity_current) {
    output.reason = "identity_not_current_validated";
    return output;
  }
  if (!cargoPhysicalGroupEvidenceOwnerMatches(
          input.group, input.identity,
          input.identity.physical_cargo_epoch_id) ||
      std::abs(input.group.source_stamp_sec - input.source_stamp_sec) >
          kStampEpsilonSec ||
      !input.group.supported_top_valid ||
      !input.group.geometry_resolved ||
      !finiteResolvedGeometry(input.group.resolved_geometry)) {
    output.reason = "current_group_owner_or_geometry_invalid";
    return output;
  }
  if (!input.geometry.formal_geometry_valid ||
      input.geometry.physical_history_id !=
          input.identity.physical_history_id ||
      !input.bottom.valid || !input.bottom.geometry_valid ||
      input.bottom.track_id != input.identity.physical_history_id ||
      std::abs(input.bottom.stamp_sec - input.source_stamp_sec) >
          kStampEpsilonSec) {
    output.reason = "formal_geometry_or_bottom_invalid";
    return output;
  }

  const auto& resolved = input.group.resolved_geometry;
  const float bottom_z = input.bottom.geometry.bottom_z_base;
  const float top_z = static_cast<float>(input.group.supported_top_z);
  if (!std::isfinite(bottom_z) || !std::isfinite(top_z) ||
      bottom_z >= top_z || input.group.union_points_base.empty()) {
    output.reason = "vertical_geometry_invalid";
    return output;
  }

  output.safety_geometry.valid = true;
  output.safety_geometry.bottom_z_base = bottom_z;
  output.safety_geometry.top_z_base = top_z;
  output.safety_geometry.footprint_base.valid = true;
  output.safety_geometry.footprint_base.center_base =
      resolved.footprint_center_base.cast<float>();
  output.safety_geometry.footprint_base.length_m =
      static_cast<float>(resolved.size.x());
  output.safety_geometry.footprint_base.width_m =
      static_cast<float>(resolved.size.y());
  output.safety_geometry.footprint_base.min_z = bottom_z;
  output.safety_geometry.footprint_base.max_z = top_z;
  output.safety_geometry.footprint_base.yaw_base_rad =
      static_cast<float>(resolved.yaw_rad);
  output.would_authorize_safety = true;

  CargoMapMutationSnapshot mutation;
  mutation.owner_points.valid = true;
  mutation.owner_points.source_frame_identity =
      input.source_frame_identity;
  mutation.owner_points.voxel_size_m = input.owner_voxel_size_m;
  for (const Eigen::Vector3f& point : input.group.union_points_base) {
    if (!point.allFinite()) continue;
    const pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
    SourcePointKey exact_key;
    if (makeSourcePointKey(pcl_point, &exact_key)) {
      mutation.owner_points.exact_points.insert(exact_key);
    }
    PointOwnershipVoxel voxel;
    if (makePointOwnershipVoxel(
            pcl_point, input.owner_voxel_size_m, &voxel)) {
      mutation.owner_points.voxels.insert(voxel);
    }
  }
  mutation.tight_geometry_valid =
      !mutation.owner_points.exact_points.empty();
  mutation.center_x = output.safety_geometry.footprint_base.center_base.x();
  mutation.center_y = output.safety_geometry.footprint_base.center_base.y();
  mutation.min_z = bottom_z;
  mutation.max_z = top_z;
  mutation.half_length =
      0.5F * output.safety_geometry.footprint_base.length_m;
  mutation.half_width =
      0.5F * output.safety_geometry.footprint_base.width_m;
  mutation.yaw_rad = output.safety_geometry.footprint_base.yaw_base_rad;
  output.would_authorize_map_mutation =
      mutation.tight_geometry_valid &&
      !input.independent_static_provenance_conflict;

  const bool product_mode =
      input.mode == CargoAuthorityMode::V6_AUTHORITY;
  output.cargo_safety_authorized =
      product_mode && output.would_authorize_safety;
  output.cargo_map_mutation_authorized =
      product_mode && output.would_authorize_map_mutation;
  mutation.authorized = output.cargo_map_mutation_authorized;
  output.map_mutation = std::move(mutation);
  if (input.independent_static_provenance_conflict) {
    output.reason = "independent_static_provenance_conflict";
  } else if (!product_mode) {
    output.reason = input.mode == CargoAuthorityMode::V6_SHADOW
        ? "shadow_only" : "legacy_mode";
  } else {
    output.reason = "authorized";
  }
  return output;
}

CargoRegistrationHygieneShadow evaluateCargoRegistrationHygieneShadow(
    const pcl::PointCloud<pcl::PointXYZ>& registration_source,
    bool legacy_authorized,
    const CargoObbFootprint& legacy_footprint,
    const CanonicalCargoAuthoritySnapshot& v6) {
  CargoRegistrationHygieneShadow output;
  output.source_points = registration_source.size();
  output.legacy_authorized = legacy_authorized && legacy_footprint.valid;
  output.v6_proposed_authorized = v6.would_authorize_safety &&
      v6.map_mutation.tight_geometry_valid &&
      v6.map_mutation.owner_points.valid;
  if ((!output.legacy_authorized && !output.v6_proposed_authorized) ||
      registration_source.empty()) {
    output.valid_input = true;
    output.reason = "no_registration_removal_authority";
    return output;
  }

  output.valid_input = true;
  for (const pcl::PointXYZ& point : registration_source.points) {
    const bool legacy_owned = output.legacy_authorized &&
        containsPointInCargoObbBase(
            Eigen::Vector3f(point.x, point.y, point.z), legacy_footprint,
            0.10F, 0.10F);
    const bool v6_owned = output.v6_proposed_authorized &&
        v6.map_mutation.ownsCurrentPoint(point);
    if (legacy_owned) ++output.legacy_removed_points;
    if (v6_owned) ++output.v6_proposed_removed_points;
    if (legacy_owned && v6_owned) ++output.intersection_points;
    if (legacy_owned && !v6_owned) ++output.legacy_only_points;
    if (!legacy_owned && v6_owned) ++output.v6_only_points;
  }
  if (v6.would_authorize_safety && !v6.would_authorize_map_mutation) {
    output.v6_proposed_points_on_static_conflict_frame =
        output.v6_proposed_removed_points;
  }
  output.reason = output.v6_proposed_authorized
      ? "v6_exact_registration_shadow" : "legacy_only_registration";
  return output;
}

bool cargoGroupOverlapsMatureStaticEvidence(
    const CargoPhysicalGroupEvidenceSnapshot& group,
    const Sophus::SE3d& pose_map_base,
    const PoseAuthorityIdentity& pose_identity,
    const std::shared_ptr<const StaticEvidenceSnapshot>& static_snapshot) {
  if (!group.valid || !pose_map_base.translation().allFinite() ||
      !pose_map_base.so3().matrix().allFinite() || !static_snapshot ||
      static_snapshot->map_generation !=
          pose_identity.map_rebuild_generation ||
      !std::isfinite(static_snapshot->cell_size_m) ||
      static_snapshot->cell_size_m <= 0.0F) {
    return true;
  }
  for (const Eigen::Vector3f& point_base : group.union_points_base) {
    if (!point_base.allFinite()) continue;
    const Eigen::Vector3d point_map =
        pose_map_base * point_base.cast<double>();
    if (!point_map.allFinite()) continue;
    const auto x = static_cast<std::int32_t>(std::floor(
        point_map.x() / static_snapshot->cell_size_m));
    const auto y = static_cast<std::int32_t>(std::floor(
        point_map.y() / static_snapshot->cell_size_m));
    const auto item = static_snapshot->cells.find(
        packStaticEvidenceCell(x, y));
    if (item == static_snapshot->cells.end()) continue;
    const StaticEvidenceCell& cell = item->second;
    if (cell.clean_map_confirmed && cell.temporally_mature &&
        point_map.z() >= cell.min_z && point_map.z() <= cell.max_z) {
      return true;
    }
  }
  return false;
}

}  // namespace ndt_slam
