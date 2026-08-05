#include <gtest/gtest.h>

#include "ndt_slam/cargo_preload_baseline_tracker.hpp"

namespace ndt_slam {
namespace {

StaticHeightComponent component(float top = 1.0F,
                                float support = 0.0F) {
  StaticHeightComponent value;
  value.valid = true;
  value.component_id = 7U;
  value.map_generation = 3U;
  value.authority = StaticEvidenceAuthority::RUNTIME_MATURE;
  value.center_map = Eigen::Vector2f(2.0F, 1.0F);
  value.length_m = 2.0F;
  value.width_m = 1.0F;
  value.top_z95_map = top;
  value.support_z_map = support;
  value.uncertainty_m = 0.05F;
  value.members.resize(12U);
  return value;
}

CargoPreloadBaselineInput input(double stamp) {
  CargoPreloadBaselineInput value;
  value.stamp_sec = stamp;
  value.hook_empty = true;
  value.localization_valid = true;
  value.stationary = true;
  value.component = component();
  return value;
}

TEST(CargoPreloadBaselineTracker, RequiresFiveStableEmptyFrames) {
  CargoPreloadBaselineTracker tracker;
  CargoPreloadBaselineResult result;
  for (int index = 0; index < 4; ++index) {
    result = tracker.update(input(1.0 + 0.1 * index));
    EXPECT_FALSE(result.ready);
  }
  result = tracker.update(input(1.4));
  EXPECT_TRUE(result.ready) << result.reason;
  EXPECT_NEAR(result.thickness_m, 1.0F, 1.0e-6F);
  EXPECT_EQ(result.occupied_cells, 12U);
}

TEST(CargoPreloadBaselineTracker, DoesNotUseLoadedOrMovingFrames) {
  CargoPreloadBaselineTracker tracker;
  auto value = input(1.0);
  value.stationary = false;
  EXPECT_FALSE(tracker.update(value).valid);
  value = input(1.1);
  value.hook_empty = false;
  EXPECT_FALSE(tracker.update(value).valid);
}

TEST(CargoPreloadBaselineTracker,
     MaturePreLifecycleMapComponentCanBuildWhileMoving) {
  CargoPreloadBaselineTracker tracker;
  CargoPreloadBaselineResult result;
  for (int index = 0; index < 5; ++index) {
    auto value = input(1.0 + 0.1 * index);
    value.stationary = false;
    value.independently_mature_static = true;
    result = tracker.update(value);
  }
  EXPECT_TRUE(result.ready) << result.reason;
  EXPECT_NEAR(result.thickness_m, 1.0F, 1.0e-6F);
}

TEST(CargoPreloadBaselineTracker,
     UnverifiedMovingComponentCannotBuildBaseline) {
  CargoPreloadBaselineTracker tracker;
  auto value = input(1.0);
  value.stationary = false;
  value.independently_mature_static = false;
  const CargoPreloadBaselineResult result = tracker.update(value);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "base_not_stationary_or_mature");
}

TEST(CargoPreloadBaselineTracker, GapAndRollbackCannotInheritReadiness) {
  CargoPreloadBaselineTracker tracker;
  for (int index = 0; index < 4; ++index) {
    tracker.update(input(1.0 + 0.1 * index));
  }
  EXPECT_FALSE(tracker.update(input(2.0)).ready);
  EXPECT_FALSE(tracker.update(input(1.5)).ready);
}

TEST(CargoPreloadBaselineTracker, MapGenerationChangeRestartsBaseline) {
  CargoPreloadBaselineTracker tracker;
  for (int index = 0; index < 4; ++index) {
    tracker.update(input(1.0 + 0.1 * index));
  }
  auto changed = input(1.4);
  changed.component.map_generation = 4U;
  const CargoPreloadBaselineResult result = tracker.update(changed);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.confirm_frames, 1);
}

TEST(CargoPreloadBaselineTracker, RejectsSpatiallyUncertainAnchorMatch) {
  CargoPreloadBaselineTracker tracker;
  auto value = input(1.0);
  value.component.hook_anchor_distance_m = 0.51F;
  const CargoPreloadBaselineResult result = tracker.update(value);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "anchor_component_spatially_uncertain");
}

TEST(CargoPreloadBaselineTracker, RejectsSparseStaticComponent) {
  CargoPreloadBaselineTracker tracker;
  auto value = input(1.0);
  value.component.members.resize(3U);
  const CargoPreloadBaselineResult result = tracker.update(value);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "baseline_spatial_coverage_insufficient");
}

TEST(CargoPreloadBaselineTracker, ComponentIdentityChangeRestartsWindow) {
  CargoPreloadBaselineTracker tracker;
  for (int index = 0; index < 4; ++index) {
    tracker.update(input(1.0 + 0.1 * index));
  }
  auto changed = input(1.4);
  changed.component.component_id = 8U;
  const CargoPreloadBaselineResult result = tracker.update(changed);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.confirm_frames, 1);
}

}  // namespace
}  // namespace ndt_slam
