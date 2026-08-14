#pragma once

#include <string>

namespace ndt_slam {

struct CargoCapabilityInput {
  bool config_valid = true;
  bool external_output_authorized = true;
  bool cargo_identity_valid = false;
  bool lifecycle_valid = false;
  bool horizontal_envelope_valid = false;
  bool vertical_geometry_valid = false;
  bool cloud_fresh = false;
  bool positive_identity_authorized = false;
  bool formal_geometry_valid = false;
  bool formal_clear_contract_valid = false;
  bool formal_removal_contract_valid = false;
};

struct CargoCapability {
  bool perception = false;
  bool tracking = false;
  bool positive_warning = false;
  bool formal_warning = false;
  bool clear = false;
  bool cargo_removal = false;
  bool map_eligibility = false;
  std::string perception_reason = "not_evaluated";
  std::string warning_reason = "not_evaluated";
};

CargoCapability deriveCargoCapability(const CargoCapabilityInput& input);

}  // namespace ndt_slam
