#pragma once

#include "ndt_slam/avoidance_map_mutation.hpp"
#include "ndt_slam/cargo_bottom_fusion.hpp"
#include "ndt_slam/integrated_cargo_identity_shadow.hpp"
#include "ndt_slam/static_obstacle_evidence_index.hpp"

#include <sophus/se3.hpp>

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class CargoAuthorityMode : std::uint8_t {
  LEGACY = 0,
  V6_SHADOW = 1,
  V6_AUTHORITY = 2,
};

const char* cargoAuthorityModeName(CargoAuthorityMode mode) noexcept;
bool parseCargoAuthorityMode(const std::string& value,
                             CargoAuthorityMode* mode) noexcept;

struct CanonicalCargoGeometry {
  bool valid = false;
  CargoObbFootprint footprint_base;
  float bottom_z_base = 0.0F;
  float top_z_base = 0.0F;
};

struct CanonicalCargoAuthorityInput {
  CargoAuthorityMode mode = CargoAuthorityMode::LEGACY;
  double source_stamp_sec = 0.0;
  SourceFrameIdentity source_frame_identity;
  PoseAuthorityIdentity pose_identity;
  CargoPhysicalIdentityDecision identity;
  CargoPhysicalGroupEvidenceSnapshot group;
  CargoShadowGeometryDecision geometry;
  CargoBottomResult bottom;
  float owner_voxel_size_m = 0.06F;
  bool independent_static_provenance_conflict = true;
};

struct CanonicalCargoAuthoritySnapshot {
  bool valid_input = false;
  bool would_authorize_safety = false;
  bool would_authorize_map_mutation = false;
  bool cargo_safety_authorized = false;
  bool cargo_map_mutation_authorized = false;
  double source_stamp_sec = 0.0;
  PoseAuthorityIdentity pose_identity;
  std::uint64_t physical_history_id = 0U;
  CanonicalCargoGeometry safety_geometry;
  CargoMapMutationSnapshot map_mutation;
  std::string reason = "not_evaluated";
};

// Diagnostic-only comparison. Registration continues to use the commissioned
// Legacy removal path in this release; V6 can only describe its exact-point
// counterfactual and has no registration write authority.
struct CargoRegistrationHygieneShadow {
  bool valid_input = false;
  bool legacy_authorized = false;
  bool v6_proposed_authorized = false;
  std::size_t source_points = 0U;
  std::size_t legacy_removed_points = 0U;
  std::size_t v6_proposed_removed_points = 0U;
  std::size_t intersection_points = 0U;
  std::size_t legacy_only_points = 0U;
  std::size_t v6_only_points = 0U;
  std::size_t static_background_conflict_points = 0U;
  std::string reason = "not_evaluated";
};

CanonicalCargoAuthoritySnapshot buildCanonicalCargoAuthoritySnapshot(
    const CanonicalCargoAuthorityInput& input);

CargoRegistrationHygieneShadow evaluateCargoRegistrationHygieneShadow(
    const pcl::PointCloud<pcl::PointXYZ>& registration_source,
    bool legacy_authorized,
    const CargoObbFootprint& legacy_footprint,
    const CanonicalCargoAuthoritySnapshot& v6);

// Conservative map-only conflict check. A mature cell already present in the
// immutable StaticEvidence snapshot is independent persistent provenance; V6
// may still own Safety, but it cannot erase that cell from the localization
// authority map.
bool cargoGroupOverlapsMatureStaticEvidence(
    const CargoPhysicalGroupEvidenceSnapshot& group,
    const Sophus::SE3d& pose_map_base,
    const PoseAuthorityIdentity& pose_identity,
    const std::shared_ptr<const StaticEvidenceSnapshot>& static_snapshot);

}  // namespace ndt_slam
