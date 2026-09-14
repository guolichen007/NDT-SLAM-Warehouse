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
  input.vertical_authority = CargoVerticalAuthority::DIRECT_BOTTOM;
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
  input.vertical_authority = CargoVerticalAuthority::DIRECT_BOTTOM;
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
  input.vertical_authority = CargoVerticalAuthority::DIRECT_BOTTOM;
  input.formal_geometry_valid = true;
  const CargoCapability capability = deriveCargoCapability(input);
  EXPECT_FALSE(capability.perception);
  EXPECT_FALSE(capability.tracking);
  EXPECT_FALSE(capability.formal_warning);
  EXPECT_EQ(capability.warning_reason, "config_invalid");
}

// A finite bottom/top is NOT safety authority.  The geometry may be finite
// (vertical_geometry_valid=true) but if the authority is INVALID (no physical
// bottom measurement) the warning must stay closed while tracking survives.
TEST(CargoCapability, FiniteGeometryWithoutAuthorityCannotWarn) {
  CargoCapabilityInput input = physicalInput();
  input.vertical_geometry_valid = true;
  input.vertical_authority = CargoVerticalAuthority::INVALID;
  input.positive_identity_authorized = true;
  const CargoCapability capability = deriveCargoCapability(input);
  EXPECT_TRUE(capability.perception);
  EXPECT_TRUE(capability.tracking);
  EXPECT_FALSE(capability.positive_warning);
  EXPECT_FALSE(capability.formal_warning);
  EXPECT_FALSE(capability.clear);
  EXPECT_EQ(capability.warning_reason, "vertical_authority_not_safety_authorized");
}

TEST(CargoCapability, OnlySafetyAuthorizedAuthoritiesCanWarn) {
  EXPECT_TRUE(isSafetyAuthorizedCargoVerticalAuthority(
      CargoVerticalAuthority::DIRECT_BOTTOM));
  EXPECT_TRUE(isSafetyAuthorizedCargoVerticalAuthority(
      CargoVerticalAuthority::SUPPORTED_TOP_MINUS_FROZEN_HEIGHT));
  EXPECT_TRUE(isSafetyAuthorizedCargoVerticalAuthority(
      CargoVerticalAuthority::FRESH_HELD_FORMAL));
  EXPECT_FALSE(isSafetyAuthorizedCargoVerticalAuthority(
      CargoVerticalAuthority::INVALID));
}

}  // namespace
}  // namespace ndt_slam
