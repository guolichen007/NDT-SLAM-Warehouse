#include "ndt_slam/cargo_avoidance_fusion.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoAvoidanceFusionInput validInput() {
  CargoAvoidanceFusionInput input;
  input.localization_valid = true;
  input.formal_cargo_geometry_valid = true;
  input.formal_cargo_bottom_valid = true;
  input.static_session_manifest_valid = true;
  input.static_session_hash_valid = true;
  input.static_session_uuid_valid = true;
  input.static_risk_contract_valid = true;
  input.static_clear_contract_valid = true;
  input.static_authority = StaticEvidenceAuthority::RUNTIME_MATURE;
  input.live.available = true;
  input.live.reliable = true;
  input.live.coverage = 0.75F;
  input.static_map.available = true;
  input.static_map.reliable = true;
  return input;
}

TEST(CargoAvoidanceFusion, ClearNeedsBothReliableSources) {
  auto input = validInput();
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 14);

  input.static_session_hash_valid = false;
  const auto rejected = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(rejected.official_valid);
  EXPECT_EQ(rejected.official_code, 34);
}

TEST(CargoAvoidanceFusion, StaticHazardSurvivesLiveClearConflict) {
  auto input = validInput();
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  input.static_map.distance_m = 2.0F;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
  EXPECT_TRUE(result.map_live_conflict);
  EXPECT_EQ(result.reason, "MAP_LIVE_CONFLICT_static_hazard_retained");
}

TEST(CargoAvoidanceFusion, StaticHazardSurvivesLiveBlank) {
  auto input = validInput();
  input.live.available = false;
  input.live.reliable = false;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
}

TEST(CargoAvoidanceFusion, PendingEnvelopeCannotGrantClear) {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.pending_envelope_valid = true;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_EQ(result.provisional_status, "CLEAR_NOT_AUTHORIZED");
}

TEST(CargoAvoidanceFusion, PendingHazardWarnsOfficiallyByDefault) {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.formal_cargo_bottom_valid = false;
  input.pending_envelope_valid = true;
  input.live.hazard = true;
  input.live.warning_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
  EXPECT_EQ(result.provisional_status, "NEAR_3M");
}

TEST(CargoAvoidanceFusion, PendingOptInCanOnlyEscalatePositiveHazard) {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.formal_cargo_bottom_valid = false;
  input.pending_envelope_valid = true;
  input.static_map.hazard = true;
  input.static_map.warning_code = 18;
  CargoAvoidanceFusionConfig config;
  config.provisional_positive_warning_to_official_code = true;
  const auto result = fuseCargoAvoidanceRisk(input, config);
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 18);
  EXPECT_NE(result.official_code, 14);
}

TEST(CargoAvoidanceFusion, UnverifiedStaticCannotAuthorizeClearOrHazard) {
  auto input = validInput();
  input.static_authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 34);
}

TEST(CargoAvoidanceFusion, PendingUnverifiedStaticRemainsAdvisoryOnly) {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.formal_cargo_bottom_valid = false;
  input.pending_envelope_valid = true;
  input.static_authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  CargoAvoidanceFusionConfig config;
  config.provisional_positive_warning_to_official_code = true;
  const auto result = fuseCargoAvoidanceRisk(input, config);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_FALSE(result.risk_static);
  EXPECT_EQ(result.provisional_status, "CLEAR_NOT_AUTHORIZED");
}

TEST(CargoAvoidanceFusion, MoreSevereSourceWins) {
  auto input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 18;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
}

}  // namespace
}  // namespace ndt_slam
