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

enum class CargoEnvelopePoseSource : std::uint8_t {
  NONE = 0,
  CURRENT_ASSOCIATED_LIDAR,
  SHORT_TERM_TRACK_PREDICTION,
  RETIRED_TRACK_PREDICTION,
  HOOK_PLUS_LAST_RELIABLE_OFFSET,
  HOOK_DEFAULT_OFFSET,
};

const char* cargoEnvelopePoseSourceName(
    CargoEnvelopePoseSource source) noexcept;

struct CargoEnvelopePoseCandidate {
  bool valid = false;
  Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
  float yaw_base_rad = 0.0F;
  float horizontal_uncertainty_m = 0.0F;
  float vertical_uncertainty_m = 0.0F;
  double evidence_stamp_sec = 0.0;
  std::uint64_t cargo_lifecycle_id = 0U;
  CargoEnvelopePoseSource source = CargoEnvelopePoseSource::NONE;
};

enum class CargoEnvelopeShapeSource : std::uint8_t {
  NONE = 0,
  FORMAL_FROZEN_GEOMETRY,
  CURRENT_HIGH_QUALITY_LIDAR,
  RETIRED_LOCKED_SHAPE,
  STATIC_ORIGIN_COMPONENT,
  CONFIGURED_CONSERVATIVE_DEFAULT,
};

const char* cargoEnvelopeShapeSourceName(
    CargoEnvelopeShapeSource source) noexcept;

struct CargoEnvelopeShapeCandidate {
  bool valid = false;
  float length_m = 0.0F;
  float width_m = 0.0F;
  float height_m = 0.0F;
  float yaw_rad = 0.0F;
  float uncertainty_m = 0.0F;
  double evidence_stamp_sec = 0.0;
  std::uint64_t cargo_lifecycle_id = 0U;
  CargoEnvelopeShapeSource source = CargoEnvelopeShapeSource::NONE;
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

struct PendingCargoEnvelopeInput {
  double stamp_sec = 0.0;
  bool hook_loaded = false;
  std::uint64_t cargo_lifecycle_id = 0U;
  CargoEnvelopePoseCandidate current_associated_pose;
  CargoEnvelopePoseCandidate short_term_track_pose;
  CargoEnvelopePoseCandidate retired_track_pose;
  CargoEnvelopePoseCandidate hook_last_offset_pose;
  CargoEnvelopePoseCandidate hook_default_pose;
  CargoEnvelopeShapeCandidate formal_frozen_shape;
  CargoEnvelopeShapeCandidate current_high_quality_shape;
  CargoEnvelopeShapeCandidate retired_locked_shape;
  CargoEnvelopeShapeCandidate static_origin_shape;
};

struct PendingCargoEnvelope {
  bool valid = false;
  PendingCargoEnvelopeSource source = PendingCargoEnvelopeSource::NONE;
  CargoEnvelopePoseSource pose_source = CargoEnvelopePoseSource::NONE;
  CargoEnvelopeShapeSource shape_source = CargoEnvelopeShapeSource::NONE;
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
  double maximum_formal_height_age_sec = 0.50;
  double maximum_formal_pose_age_sec = 0.50;
};

struct EffectiveCargoEnvelope {
  bool valid = false;
  bool formal = false;
  bool fallback_active = false;
  bool can_authorize_clear = false;
  EffectiveCargoEnvelopeSource source =
      EffectiveCargoEnvelopeSource::NONE;
  CargoEnvelopePoseSource pose_source = CargoEnvelopePoseSource::NONE;
  CargoEnvelopeShapeSource shape_source = CargoEnvelopeShapeSource::NONE;
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

EffectiveCargoEnvelope composeEffectiveCargoEnvelope(
    const CargoPresenceResult& presence,
    const CargoEnvelopePoseCandidate& pose,
    const CargoEnvelopeShapeCandidate& shape,
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
