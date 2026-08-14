#include "ndt_slam/cargo_capability.hpp"

namespace ndt_slam {

CargoCapability deriveCargoCapability(const CargoCapabilityInput& input) {
  CargoCapability output;
  if (!input.config_valid) {
    output.perception_reason = "config_invalid";
    output.warning_reason = "config_invalid";
    return output;
  }
  if (!input.external_output_authorized) {
    output.perception_reason = "external_output_not_authorized";
    output.warning_reason = "external_output_not_authorized";
    return output;
  }
  if (!input.lifecycle_valid || !input.cargo_identity_valid) {
    output.perception_reason = "cargo_identity_not_authorized";
    output.warning_reason = "cargo_identity_not_authorized";
    return output;
  }
  if (!input.horizontal_envelope_valid) {
    output.perception_reason = "horizontal_envelope_invalid";
    output.warning_reason = "horizontal_envelope_invalid";
    return output;
  }
  if (!input.cloud_fresh) {
    output.perception_reason = "obstacle_cloud_not_fresh";
    output.warning_reason = "obstacle_cloud_not_fresh";
    return output;
  }
  output.perception = true;
  output.tracking = true;
  output.perception_reason = "physical_perception_authorized";
  if (!input.vertical_geometry_valid) {
    output.warning_reason = "vertical_geometry_invalid";
    return output;
  }
  output.positive_warning = input.positive_identity_authorized;
  output.formal_warning = input.formal_geometry_valid;
  output.clear = input.formal_geometry_valid &&
      input.formal_clear_contract_valid;
  output.cargo_removal = input.formal_geometry_valid &&
      input.formal_removal_contract_valid;
  output.map_eligibility = output.cargo_removal;
  output.warning_reason = output.formal_warning
      ? "formal_warning_authorized"
      : (output.positive_warning
             ? "positive_only_warning_authorized"
             : "warning_identity_not_authorized");
  return output;
}

}  // namespace ndt_slam
