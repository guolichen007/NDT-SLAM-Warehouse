#pragma once

#include "ndt_slam/cargo_rigid_geometry.hpp"
#include "ndt_slam/cargo_v6_authority_adapter.hpp"

#include <cstdint>
#include <string>

namespace ndt_slam {

// Integration-only value object. It owns no temporal state and deliberately
// contains every permission consumed by the existing product pipeline so a
// legacy gate cannot accidentally veto or authorize V6.
struct ProductCargoContext {
  CargoAuthorityMode mode = CargoAuthorityMode::LEGACY;
  bool valid_input = false;
  bool identity_authorized = false;
  bool geometry_authorized = false;
  bool bottom_authorized = false;
  bool safety_authorized = false;
  bool clear_authorized = false;
  bool self_removal_authorized = false;
  bool map_mutation_authorized = false;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_id = 0U;
  RigidCargoGeometry geometry;
  CargoBottomResult bottom;
  CargoFormalUseDecision formal_use;
  TemporalEvidenceAuthority temporal_authority;
  std::string reason = "not_evaluated";
};

struct LegacyProductCargoContextInput {
  bool active_track = false;
  bool frozen_geometry_ready = false;
  bool clear_authorized = false;
  bool self_removal_authorized = false;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_id = 0U;
  RigidCargoGeometry geometry;
  CargoBottomResult bottom;
  CargoFormalUseDecision formal_use;
};

ProductCargoContext buildLegacyProductCargoContext(
    const LegacyProductCargoContextInput& input);

struct V6ProductCargoContextInput {
  CanonicalCargoAuthoritySnapshot canonical;
  CargoShadowGeometryDecision geometry_authority;
  CargoPhysicalIdentityDecision identity;
  CargoBottomResult bottom;
  Sophus::SE3d pose_map_base;
  double evaluation_stamp_sec = 0.0;
  float horizontal_uncertainty_m = 0.0F;
};

ProductCargoContext buildV6ProductCargoContext(
    const V6ProductCargoContextInput& input);

struct ProductCargoSelection {
  ProductCargoContext product;
  bool legacy_positive_hazard_retained = false;
  bool legacy_clear_rejected = false;
  std::string reason = "not_evaluated";
};

ProductCargoSelection selectProductCargoContext(
    CargoAuthorityMode mode,
    const ProductCargoContext& legacy,
    const ProductCargoContext& v6,
    bool legacy_positive_hazard,
    bool legacy_hazard_same_authority);

}  // namespace ndt_slam
