#pragma once

#include "ndt_slam/pending_cargo_envelope.hpp"
#include "ndt_slam/static_obstacle_evidence_index.hpp"

#include <cstdint>
#include <limits>
#include <string>

namespace ndt_slam {

enum class PendingWarningPromotionPolicy : std::uint8_t {
  DISABLED = 0,
  EVIDENCE_BACKED_ONLY,
  LEGACY_ANY_PENDING,
};

const char* pendingWarningPromotionPolicyName(
    PendingWarningPromotionPolicy policy) noexcept;

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
  PendingWarningPromotionPolicy pending_warning_promotion_policy =
      PendingWarningPromotionPolicy::DISABLED;
  int pending_minimum_obstacle_confirmations = 3;
  float pending_minimum_authority_confidence = 0.55F;
  bool allow_static_only_pending_warning = true;
};

struct CargoAvoidanceFusionInput {
  bool localization_valid = false;
  // Warning authority is withheld while cargo motion/direction is not
  // authoritative. Tracking continues outside this fusion contract.
  bool warning_motion_authorized = true;
  // A code-17 source must belong to a track confirmed outside the 3 m shell.
  bool near_field_history_authorized = true;
  // Raw evaluator candidate used only by the two authorization gates above.
  // It cannot directly become an official warning.
  bool warning_candidate_present = false;
  std::int32_t warning_candidate_code = 0;
  bool formal_cargo_geometry_valid = false;
  bool formal_cargo_bottom_valid = false;
  bool formal_clear_authorized = false;
  bool pending_envelope_valid = false;
  PendingCargoEnvelopeSource pending_envelope_source =
      PendingCargoEnvelopeSource::NONE;
  CargoEnvelopePoseSource pending_pose_source =
      CargoEnvelopePoseSource::NONE;
  bool pending_recognition_state_allows_warning = false;
  bool pending_warning_query_allowed = false;
  bool pending_pose_physically_plausible = false;
  std::string pending_warning_state_reason =
      "recognition_state_not_warning_authorized";
  bool pending_self_evidence_valid = false;
  bool pending_external_separation_valid = false;
  bool pending_external_obstacle_authorized = false;
  std::uint64_t pending_external_obstacle_track_id = 0U;
  int pending_external_obstacle_confirmations = 0;
  bool pending_external_provenance_valid = false;
  bool pending_external_geometry_valid = false;
  float pending_authority_confidence = 0.0F;
  bool pending_static_obstacle_authorized = false;
  std::uint32_t pending_static_obstacle_id = 0U;
  int pending_static_obstacle_confirmations = 0;
  bool pending_static_provenance_valid = false;
  float pending_static_authority_confidence = 0.0F;
  // False when a live cluster first appeared already embedded in the cargo
  // footprint without prior separated-track history or independent static
  // provenance. Such evidence is diagnostic only and must not become 17/18
  // or be interpreted as a clear observation.
  bool live_obstacle_origin_resolved = true;
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
  bool pending_warning_authorized = false;
  bool pending_live_warning_authorized = false;
  bool pending_static_warning_authorized = false;
  std::string pending_authority_reason = "not_evaluated";
  std::string provisional_status = "UNKNOWN";
};

CargoAvoidanceFusionResult fuseCargoAvoidanceRisk(
    const CargoAvoidanceFusionInput& input,
    const CargoAvoidanceFusionConfig& config =
        CargoAvoidanceFusionConfig{});

}  // namespace ndt_slam
