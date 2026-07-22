#pragma once

#include "ndt_slam/static_obstacle_evidence_index.hpp"

#include <cstdint>
#include <limits>
#include <string>

namespace ndt_slam {

struct CargoAvoidanceSourceRisk {
  bool available = false;
  bool reliable = false;
  bool hazard = false;
  std::int32_t warning_code = 0;
  float distance_m = std::numeric_limits<float>::infinity();
  float clearance_m = std::numeric_limits<float>::quiet_NaN();
  float coverage = 0.0F;
  std::string reason;
};

struct CargoAvoidanceFusionConfig {
  float minimum_live_coverage_for_clear = 0.05F;
  bool provisional_positive_warning_to_official_code = true;
};

struct CargoAvoidanceFusionInput {
  bool localization_valid = false;
  bool formal_cargo_geometry_valid = false;
  bool formal_cargo_bottom_valid = false;
  bool formal_clear_authorized = false;
  bool pending_envelope_valid = false;
  bool static_session_manifest_valid = false;
  bool static_session_hash_valid = false;
  bool static_session_uuid_valid = false;
  bool static_risk_contract_valid = false;
  bool static_clear_contract_valid = false;
  StaticEvidenceAuthority static_authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  CargoAvoidanceSourceRisk live;
  CargoAvoidanceSourceRisk static_map;
};

struct CargoAvoidanceFusionResult {
  bool official_valid = false;
  std::int32_t official_code = 33;
  std::string reason = "cargo_recognition_or_geometry_invalid";
  float distance_m = std::numeric_limits<float>::infinity();
  float clearance_m = std::numeric_limits<float>::quiet_NaN();
  bool risk_live = false;
  bool risk_static = false;
  bool map_live_conflict = false;
  std::string provisional_status = "UNKNOWN";
};

CargoAvoidanceFusionResult fuseCargoAvoidanceRisk(
    const CargoAvoidanceFusionInput& input,
    const CargoAvoidanceFusionConfig& config =
        CargoAvoidanceFusionConfig{});

}  // namespace ndt_slam
