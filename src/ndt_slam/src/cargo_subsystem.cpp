#include "ndt_slam/cargo_subsystem.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {

CargoContractSource cargoContractSource(
    CargoEnvelopePoseSource source) noexcept {
  switch (source) {
    case CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR:
      return CargoContractSource::CURRENT_ASSOCIATED_LIDAR;
    case CargoEnvelopePoseSource::SHORT_TERM_TRACK_PREDICTION:
    case CargoEnvelopePoseSource::RETIRED_TRACK_PREDICTION:
    case CargoEnvelopePoseSource::HOOK_PLUS_LAST_RELIABLE_OFFSET:
    case CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET:
      return CargoContractSource::MOTION_PREDICTION;
    case CargoEnvelopePoseSource::NONE:
      return CargoContractSource::UNKNOWN;
  }
  return CargoContractSource::UNKNOWN;
}

const CargoSubsystemSnapshot& CargoSubsystem::update(
    const CargoSubsystemFrameInput& input) {
  CargoSubsystemSnapshot next;
  next.track.source_stamp_sec = input.source_stamp_sec;
  next.track.evaluation_stamp_sec = input.evaluation_stamp_sec;
  next.track.frame_id = input.frame_id;
  next.track.cargo_lifecycle_id = input.cargo_lifecycle_id;
  next.track.cargo_track_id = input.cargo_track_id;
  next.track.center = input.envelope.center_base;
  next.track.uncertainty = Eigen::Vector3f(
      input.envelope.horizontal_uncertainty_m,
      input.envelope.horizontal_uncertainty_m,
      input.envelope.vertical_uncertainty_m);
  next.track.source = cargoContractSource(input.envelope.pose_source);
  next.track.valid = input.identity_valid && input.lifecycle_valid &&
      input.envelope.center_base.allFinite();
  next.track.fresh = next.track.valid && input.cloud_fresh;

  next.geometry.source_stamp_sec = input.source_stamp_sec;
  next.geometry.evaluation_stamp_sec = input.evaluation_stamp_sec;
  next.geometry.frame_id = input.frame_id;
  next.geometry.cargo_lifecycle_id = input.cargo_lifecycle_id;
  next.geometry.cargo_track_id = input.cargo_track_id;
  next.geometry.length_m = input.envelope.length_m;
  next.geometry.width_m = input.envelope.width_m;
  next.geometry.yaw_rad = input.envelope.yaw_base_rad;
  next.geometry.top_z_m = input.envelope.top_z_base;
  next.geometry.bottom_z_m = input.envelope.bottom_z_base;
  next.geometry.height_m = input.envelope.height_m;
  next.geometry.uncertainty = next.track.uncertainty;
  next.geometry.source = next.track.source;
  next.geometry.horizontal_valid = next.track.valid &&
      std::isfinite(input.envelope.length_m) &&
      input.envelope.length_m > 0.0F &&
      std::isfinite(input.envelope.width_m) &&
      input.envelope.width_m > 0.0F &&
      std::isfinite(input.envelope.yaw_base_rad);
  next.geometry.vertical_valid = input.vertical_geometry_valid &&
      std::isfinite(input.envelope.bottom_z_base) &&
      std::isfinite(input.envelope.top_z_base) &&
      input.envelope.top_z_base > input.envelope.bottom_z_base;
  next.geometry.vertical_authority = next.geometry.vertical_valid
      ? input.vertical_authority
      : CargoVerticalAuthority::INVALID;
  next.geometry.fresh = next.geometry.horizontal_valid && input.cloud_fresh;

  next.envelope.source_stamp_sec = input.source_stamp_sec;
  next.envelope.evaluation_stamp_sec = input.evaluation_stamp_sec;
  next.envelope.frame_id = input.frame_id;
  next.envelope.cargo_lifecycle_id = input.cargo_lifecycle_id;
  next.envelope.cargo_track_id = input.cargo_track_id;
  next.envelope.center = input.envelope.center_base.head<2>();
  next.envelope.nominal_length_m = input.envelope.length_m;
  next.envelope.nominal_width_m = input.envelope.width_m;
  next.envelope.horizontal_uncertainty_m =
      input.envelope.horizontal_uncertainty_m;
  next.envelope.vertical_uncertainty_m =
      input.envelope.vertical_uncertainty_m;
  next.envelope.conservative_length_m = std::max(
      0.0F, input.envelope.length_m +
          2.0F * input.envelope.horizontal_uncertainty_m);
  next.envelope.conservative_width_m = std::max(
      0.0F, input.envelope.width_m +
          2.0F * input.envelope.horizontal_uncertainty_m);
  next.envelope.yaw_rad = input.envelope.yaw_base_rad;
  next.envelope.safe_bottom_z_m = input.envelope.bottom_z_base -
      input.envelope.vertical_uncertainty_m;
  next.envelope.source = next.track.source;
  next.envelope.horizontal_valid = next.geometry.horizontal_valid;
  next.envelope.vertical_valid = next.geometry.vertical_valid;
  next.envelope.formal = input.formal_geometry_valid;
  next.envelope.fresh = next.geometry.fresh;

  CargoCapabilityInput capability_input;
  capability_input.config_valid = input.config_valid;
  capability_input.external_output_authorized =
      input.external_output_authorized;
  capability_input.cargo_identity_valid = input.identity_valid;
  capability_input.lifecycle_valid = input.lifecycle_valid;
  capability_input.horizontal_envelope_valid =
      next.envelope.horizontal_valid;
  capability_input.vertical_geometry_valid =
      next.envelope.vertical_valid;
  capability_input.cloud_fresh = input.cloud_fresh;
  capability_input.positive_identity_authorized =
      input.positive_identity_authorized;
  capability_input.formal_geometry_valid = input.formal_geometry_valid;
  capability_input.formal_clear_contract_valid =
      input.formal_clear_contract_valid;
  capability_input.formal_removal_contract_valid =
      input.formal_removal_contract_valid;
  next.capability = deriveCargoCapability(capability_input);

  snapshot_ = std::move(next);
  return snapshot_;
}

}  // namespace ndt_slam
