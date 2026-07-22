#pragma once

#include "ndt_slam/cargo_rigid_geometry.hpp"
#include "ndt_slam/cargo_presence_state_machine.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>

namespace ndt_slam {

enum class PendingCargoEnvelopeSource : std::uint8_t {
  NONE = 0,
  CURRENT_CANDIDATE,
  RETIRED_FORMAL_SHAPE,
  LIFT_ORIGIN_CANDIDATE,
  CONFIGURED_CONSERVATIVE,
};

const char* pendingCargoEnvelopeSourceName(
    PendingCargoEnvelopeSource source) noexcept;

struct PendingCargoEnvelopeConfig {
  float configured_length_m = 4.0F;
  float configured_width_m = 3.0F;
  float configured_height_m = 3.0F;
  // base_link Z points upward. The configured cargo center is below the
  // physical hook anchor, so the default offset is negative.
  float configured_center_offset_z_m = -1.50F;
  float horizontal_margin_m = 0.20F;
  float vertical_margin_m = 0.15F;
  double maximum_candidate_age_sec = 0.50;
  double maximum_retired_age_sec = 8.0;
  float maximum_fallback_sway_offset_m = 0.80F;
  float lost_position_uncertainty_per_sec = 0.15F;
  float maximum_lost_position_uncertainty_m = 1.00F;
};

struct PendingCargoEnvelopeCandidate {
  bool valid = false;
  Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
  float length_m = 0.0F;
  float width_m = 0.0F;
  float height_m = 0.0F;
  float yaw_base_rad = 0.0F;
  float horizontal_uncertainty_m = 0.0F;
  float vertical_uncertainty_m = 0.0F;
  double evidence_stamp_sec = 0.0;
  std::uint64_t cargo_lifecycle_id = 0U;
};

struct PendingCargoEnvelopeInput {
  double stamp_sec = 0.0;
  bool hook_loaded = false;
  std::uint64_t cargo_lifecycle_id = 0U;
  PendingCargoEnvelopeCandidate current_candidate;
  PendingCargoEnvelopeCandidate retired_formal_shape;
  PendingCargoEnvelopeCandidate lift_origin_candidate;
  bool hook_anchor_valid = false;
  Eigen::Vector3f hook_anchor_base = Eigen::Vector3f::Zero();
};

struct PendingCargoEnvelope {
  bool valid = false;
  PendingCargoEnvelopeSource source = PendingCargoEnvelopeSource::NONE;
  Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
  float length_m = 0.0F;
  float width_m = 0.0F;
  float height_m = 0.0F;
  float yaw_base_rad = 0.0F;
  float bottom_z_base = std::numeric_limits<float>::quiet_NaN();
  float top_z_base = std::numeric_limits<float>::quiet_NaN();
  float horizontal_uncertainty_m = 0.0F;
  float vertical_uncertainty_m = 0.0F;
  double evidence_stamp_sec = 0.0;
  double evaluation_stamp_sec = 0.0;
  double age_sec = 0.0;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::string reason = "not_built";
};

enum class EffectiveCargoEnvelopeSource : std::uint8_t {
  NONE = 0,
  FORMAL_FROZEN_GEOMETRY,
  CURRENT_ASSOCIATED_TRACK,
  RETIRED_LOCKED_SHAPE,
  STATIC_ORIGIN_COMPONENT,
  CONFIGURED_CONSERVATIVE_DEFAULT,
};

struct CargoEnvelopeResolverConfig {
  float configured_length_m = 4.0F;
  float configured_width_m = 3.0F;
  float configured_height_m = 3.0F;
  float configured_center_z_m = -1.50F;
  float horizontal_uncertainty_m = 1.00F;
  float vertical_uncertainty_m = 0.50F;
};

struct EffectiveCargoEnvelope {
  bool valid = false;
  bool formal = false;
  bool fallback_active = false;
  bool can_authorize_clear = false;
  EffectiveCargoEnvelopeSource source =
      EffectiveCargoEnvelopeSource::NONE;
  CargoObbFootprint footprint;
  float horizontal_uncertainty_m = 0.0F;
  float vertical_uncertainty_m = 0.0F;
  double evidence_stamp_sec = 0.0;
  double age_sec = 0.0;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::string reason = "not_resolved";
};

PendingCargoEnvelope buildPendingCargoEnvelope(
    const PendingCargoEnvelopeInput& input,
    const PendingCargoEnvelopeConfig& config =
        PendingCargoEnvelopeConfig{});

CargoObbFootprint toCargoObbFootprint(
    const PendingCargoEnvelope& envelope);

EffectiveCargoEnvelope resolveEffectiveCargoEnvelope(
    const CargoPresenceResult& presence,
    const RigidCargoGeometry& formal_geometry,
    const PendingCargoEnvelope& pending_geometry,
    const CargoEnvelopeResolverConfig& config =
        CargoEnvelopeResolverConfig{});

const char* effectiveCargoEnvelopeSourceName(
    EffectiveCargoEnvelopeSource source) noexcept;

}  // namespace ndt_slam
