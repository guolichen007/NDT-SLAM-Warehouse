#pragma once

#include "ndt_slam/cargo_rigid_geometry.hpp"

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
  float horizontal_margin_m = 0.20F;
  float vertical_margin_m = 0.15F;
  double maximum_candidate_age_sec = 0.50;
  double maximum_retired_age_sec = 8.0;
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
};

struct PendingCargoEnvelopeInput {
  double stamp_sec = 0.0;
  bool hook_loaded = false;
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
  std::string reason = "not_built";
};

PendingCargoEnvelope buildPendingCargoEnvelope(
    const PendingCargoEnvelopeInput& input,
    const PendingCargoEnvelopeConfig& config =
        PendingCargoEnvelopeConfig{});

CargoObbFootprint toCargoObbFootprint(
    const PendingCargoEnvelope& envelope);

}  // namespace ndt_slam
