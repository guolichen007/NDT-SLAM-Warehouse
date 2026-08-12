#include <gtest/gtest.h>

#include "ndt_slam/cargo_capability.hpp"

namespace ndt_slam {
namespace {

CargoCapabilityInput physicalInput() {
  CargoCapabilityInput input;
  input.config_valid = true;
  input.external_output_authorized = true;
  input.cargo_identity_valid = true;
  input.lifecycle_valid = true;
  input.horizontal_envelope_valid = true;
  input.cloud_fresh = true;
  return input;
}

TEST(CargoCapability, VerticalInvalidPreservesPhysicalTrackingOnly) {
  CargoCapabilityInput input = physicalInput();
  input.vertical_geometry_valid = false;
  input.positive_identity_authorized = true;
  const CargoCapability capability = deriveCargoCapability(input);
  EXPECT_TRUE(capability.perception);
  EXPECT_TRUE(capability.tracking);
  EXPECT_FALSE(capability.positive_warning);
  EXPECT_FALSE(capability.formal_warning);
  EXPECT_FALSE(capability.clear);
  EXPECT_FALSE(capability.cargo_removal);
  EXPECT_EQ(capability.warning_reason, "vertical_geometry_invalid");
}

TEST(CargoCapability, PositiveOnlyCanWarnButCannotClearRemoveOrMap) {
  CargoCapabilityInput input = physicalInput();
  input.vertical_geometry_valid = true;
  input.positive_identity_authorized = true;
  const CargoCapability capability = deriveCargoCapability(input);
  EXPECT_TRUE(capability.positive_warning);
  EXPECT_FALSE(capability.formal_warning);
  EXPECT_FALSE(capability.clear);
  EXPECT_FALSE(capability.cargo_removal);
  EXPECT_FALSE(capability.map_eligibility);
}

TEST(CargoCapability, FormalContractsAuthorizeEachCapabilityIndependently) {
  CargoCapabilityInput input = physicalInput();
  input.vertical_geometry_valid = true;
  input.formal_geometry_valid = true;
  input.formal_clear_contract_valid = true;
  input.formal_removal_contract_valid = false;
  const CargoCapability capability = deriveCargoCapability(input);
  EXPECT_TRUE(capability.formal_warning);
  EXPECT_TRUE(capability.clear);
  EXPECT_FALSE(capability.cargo_removal);
  EXPECT_FALSE(capability.map_eligibility);
}

TEST(CargoCapability, InvalidConfigClosesEveryCapability) {
  CargoCapabilityInput input = physicalInput();
  input.config_valid = false;
  input.vertical_geometry_valid = true;
  input.formal_geometry_valid = true;
  const CargoCapability capability = deriveCargoCapability(input);
  EXPECT_FALSE(capability.perception);
  EXPECT_FALSE(capability.tracking);
  EXPECT_FALSE(capability.formal_warning);
  EXPECT_EQ(capability.warning_reason, "config_invalid");
}

}  // namespace
}  // namespace ndt_slam
