#include "ndt_slam/crane_yaw_authority.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace ndt_slam {
namespace {

TEST(CraneYawAuthorityTest, ShadowBuildRejectsRuntimeApplication) {
  CraneYawAuthorityConfig config;
  config.enabled = true;
  config.apply_to_runtime_pose = true;
  config.configured_base_yaw_in_map_rad = 0.0;
  config.map_frame_convention_id = "site_x_east_y_north";
  config.map_frame_convention_description = "approved test convention";
  CraneYawAuthority authority(config);
  const auto result = authority.observe({});
  EXPECT_EQ(result.state, CraneYawAuthorityState::INVALID);
  EXPECT_EQ(result.reason, "PRODUCT_MODE_NOT_IMPLEMENTED_IN_SHADOW_BUILD");
  EXPECT_FALSE(result.product_application_allowed);
}

TEST(CraneYawAuthorityTest, WeakYawAndHealthyRailStayConfigHold) {
  CraneYawAuthorityConfig config;
  config.enabled = true;
  config.configured_base_yaw_in_map_rad = 0.0;
  config.map_frame_convention_id = "site_x_east_y_north";
  config.map_frame_convention_description = "approved test convention";
  config.raw_yaw_threshold_rad = 1.0 * M_PI / 180.0;
  config.rail_fitness_delta_threshold = 0.5;
  config.rail_translation_delta_threshold_m = 0.5;
  config.required_consecutive_frames = 2U;
  CraneYawAuthority authority(config);
  CraneYawEvidence evidence;
  evidence.raw_ndt_yaw_rad = 10.0 * M_PI / 180.0;
  evidence.yaw_observability_strong = false;
  evidence.rail_registration_valid = true;
  evidence.rail_fitness_delta = 0.0;
  evidence.rail_translation_delta_m = 0.1;
  for (int i = 0; i < 10; ++i) {
    const auto result = authority.observe(evidence);
    EXPECT_EQ(result.state, CraneYawAuthorityState::CONFIG_HOLD);
    EXPECT_FALSE(result.composite_conflict_evidence);
    EXPECT_FALSE(result.product_application_allowed);
  }
}

TEST(CraneYawAuthorityTest, RawYawAloneCannotCreateConflict) {
  CraneYawAuthorityConfig config;
  config.enabled = true;
  config.configured_base_yaw_in_map_rad = 0.0;
  config.map_frame_convention_id = "site_x_east_y_north";
  config.map_frame_convention_description = "approved test convention";
  config.raw_yaw_threshold_rad = 1.0 * M_PI / 180.0;
  config.required_consecutive_frames = 2U;
  CraneYawAuthority authority(config);
  CraneYawEvidence evidence;
  evidence.raw_ndt_yaw_rad = 10.0 * M_PI / 180.0;
  evidence.yaw_observability_strong = true;
  evidence.rail_registration_valid = true;
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(authority.observe(evidence).state,
              CraneYawAuthorityState::CONFIG_HOLD);
  }
}

TEST(CraneYawAuthorityTest, ConsecutiveRailFailureProducesEvidenceOnly) {
  CraneYawAuthorityConfig config;
  config.enabled = true;
  config.configured_base_yaw_in_map_rad = 0.0;
  config.map_frame_convention_id = "site_x_east_y_north";
  config.map_frame_convention_description = "approved test convention";
  config.required_consecutive_frames = 2U;
  CraneYawAuthority authority(config);
  CraneYawEvidence evidence;
  evidence.rail_registration_valid = false;
  EXPECT_EQ(authority.observe(evidence).state,
            CraneYawAuthorityState::CONFIG_HOLD);
  const auto decision = authority.observe(evidence);
  EXPECT_EQ(decision.state,
            CraneYawAuthorityState::PHYSICAL_CONFLICT_EVIDENCE);
  EXPECT_FALSE(decision.product_application_allowed);
}

TEST(CraneYawAuthorityTest, FallbackNeedsThreeCircularSamples) {
  CraneYawAuthorityConfig config;
  config.enabled = true;
  config.allow_first_reliable_fallback = true;
  config.fallback_required_reliable_frames = 3U;
  config.map_frame_convention_id = "site_x_east_y_north";
  config.map_frame_convention_description = "approved test convention";
  CraneYawAuthority authority(config);
  CraneYawEvidence evidence;
  evidence.yaw_observability_strong = true;
  evidence.rail_registration_valid = true;
  evidence.raw_ndt_yaw_rad = 179.0 * M_PI / 180.0;
  EXPECT_EQ(authority.observe(evidence).state,
            CraneYawAuthorityState::FALLBACK_BOOTSTRAP);
  evidence.raw_ndt_yaw_rad = -179.0 * M_PI / 180.0;
  EXPECT_EQ(authority.observe(evidence).state,
            CraneYawAuthorityState::FALLBACK_BOOTSTRAP);
  evidence.raw_ndt_yaw_rad = 180.0 * M_PI / 180.0;
  const auto result = authority.observe(evidence);
  EXPECT_EQ(result.state, CraneYawAuthorityState::CONFIG_HOLD);
  EXPECT_NEAR(std::abs(result.authoritative_yaw_rad), M_PI, 0.02);
}

}  // namespace
}  // namespace ndt_slam
