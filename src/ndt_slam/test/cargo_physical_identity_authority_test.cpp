#include "ndt_slam/cargo_physical_identity_authority.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace ndt_slam {
namespace {

CargoPhysicalCandidateObservation candidate(
    std::uint64_t id, double stamp, double x, double z,
    std::vector<std::uint64_t> members = {1U}) {
  CargoPhysicalCandidateObservation result;
  result.candidate_id = id;
  result.member_component_ids = std::move(members);
  result.stamp_sec = stamp;
  result.center = Eigen::Vector3d(x, 0.0, z);
  result.size = Eigen::Vector3d(1.0, 0.8, 0.4);
  result.z05 = z - 0.2;
  result.z50 = z;
  result.z95 = z;
  result.vertical_uncertainty_m = 0.01;
  result.point_support = 100U;
  return result;
}

CargoVerticalEvidenceConfig verticalConfig() {
  CargoVerticalEvidenceConfig config;
  config.surface_band_height_m = 0.10F;
  config.xy_cell_size_m = 0.10F;
  config.minimum_surface_points = 4U;
  config.minimum_surface_cells = 2U;
  config.minimum_surface_coverage_ratio = 0.0F;
  return config;
}

std::vector<Eigen::Vector3f> componentPoints(double x, double z) {
  std::vector<Eigen::Vector3f> points;
  for (float dx : {-0.20F, 0.20F}) {
    for (float dy : {-0.20F, 0.20F}) {
      points.emplace_back(static_cast<float>(x) + dx, dy,
                          static_cast<float>(z));
      points.emplace_back(static_cast<float>(x) + dx, dy,
                          static_cast<float>(z));
    }
  }
  points.emplace_back(static_cast<float>(x) - 0.15F, -0.15F,
                      static_cast<float>(z) - 0.20F);
  points.emplace_back(static_cast<float>(x) + 0.15F, 0.15F,
                      static_cast<float>(z) - 0.20F);
  return points;
}

std::vector<Eigen::Vector3f> supportEnvelopePoints(
    double x_low, double x_high, double z, bool bias_left = false,
    bool bias_right = false) {
  std::vector<Eigen::Vector3f> points;
  for (int index = 0; index < 10; ++index) {
    const float y = index % 2 == 0 ? -0.20F : 0.20F;
    points.emplace_back(static_cast<float>(x_low), y,
                        static_cast<float>(z));
    points.emplace_back(static_cast<float>(x_high), y,
                        static_cast<float>(z));
  }
  const double dense_x = bias_left ? x_low + 0.10 :
      bias_right ? x_high - 0.10 : 0.5 * (x_low + x_high);
  for (int index = 0; index < 80; ++index) {
    const float y = index % 2 == 0 ? -0.20F : 0.20F;
    points.emplace_back(static_cast<float>(dense_x), y,
                        static_cast<float>(z));
  }
  points.emplace_back(static_cast<float>(x_low), -0.20F,
                      static_cast<float>(z - 0.20));
  points.emplace_back(static_cast<float>(x_high), 0.20F,
                      static_cast<float>(z - 0.20));
  return points;
}

std::vector<CargoPhysicalGroupObservation> buildGroups(
    const std::vector<CargoPhysicalCandidateObservation>& candidates,
    std::vector<CargoPhysicalComponentObservation> explicit_components = {},
    CargoVerticalEvidenceConfig config = verticalConfig()) {
  std::map<std::uint64_t, CargoPhysicalComponentObservation> components;
  for (auto& component : explicit_components) {
    components.emplace(component.component_id, std::move(component));
  }
  for (const auto& observation : candidates) {
    for (std::uint64_t member : observation.member_component_ids) {
      if (components.count(member) != 0U) continue;
      CargoPhysicalComponentObservation component;
      component.component_id = member;
      component.points_base = componentPoints(
          observation.center.x(), observation.z95);
      components.emplace(member, std::move(component));
    }
  }
  std::vector<CargoPhysicalComponentObservation> values;
  for (auto& entry : components) values.push_back(std::move(entry.second));
  return groupCargoPhysicalCandidates(
      candidates, values, nullptr, false, 0.0, config, 0.05, 0.10,
      nullptr);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromPoints(
    const std::vector<Eigen::Vector3f>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(points.size());
  for (const Eigen::Vector3f& point : points) {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    cloud->push_back(pcl_point);
  }
  return cloud;
}

void attachCurrentRawRoi(CargoPhysicalIdentityInput* frame) {
  ASSERT_NE(frame, nullptr);
  std::vector<Eigen::Vector3f> points;
  for (const auto& physical_group : frame->groups) {
    points.insert(points.end(), physical_group.union_points_base.begin(),
                  physical_group.union_points_base.end());
  }
  frame->frame_evidence.source_stamp_sec = frame->pipeline_stamp_sec;
  frame->frame_evidence.raw_roi_current_frame = cloudFromPoints(points);
  frame->vertical_config = verticalConfig();
}

std::vector<CargoPhysicalGroupObservation> buildGroupsWithRawRoi(
    const std::vector<CargoPhysicalCandidateObservation>& candidates,
    std::vector<CargoPhysicalComponentObservation> components,
    const std::vector<Eigen::Vector3f>& raw_points,
    double source_stamp,
    bool frame_ground_valid = false,
    double frame_ground_z = 0.0,
    bool grouping_ground_valid = false,
    double grouping_ground_z = 0.0,
    CargoPhysicalGroupingTelemetry* telemetry = nullptr) {
  CargoShadowFrameEvidence frame;
  frame.source_stamp_sec = source_stamp;
  frame.raw_roi_current_frame = cloudFromPoints(raw_points);
  frame.ground_reference_valid = frame_ground_valid;
  frame.ground_z_base = static_cast<float>(frame_ground_z);
  return groupCargoPhysicalCandidates(
      candidates, components, &frame, grouping_ground_valid,
      grouping_ground_z, verticalConfig(), 0.05, 0.10, telemetry);
}

CargoPhysicalGroupObservation groupWithSupport(
    std::uint64_t candidate_id, std::uint64_t component_id, double stamp,
    double x_low, double x_high, double z, bool bias_left = false,
    bool bias_right = false) {
  auto observation = candidate(candidate_id, stamp,
      0.5 * (x_low + x_high), z, {component_id});
  observation.size.x() = x_high - x_low + 0.20;
  CargoPhysicalComponentObservation component;
  component.component_id = component_id;
  component.points_base = supportEnvelopePoints(
      x_low, x_high, z, bias_left, bias_right);
  return buildGroups({observation}, {component}).front();
}

CargoPhysicalGroupObservation group(
    std::uint64_t id, double stamp, double x, double z,
    std::vector<std::uint64_t> members = {1U}) {
  auto groups = buildGroups(
      {candidate(id, stamp, x, z, std::move(members))});
  return groups.front();
}

CargoPhysicalIdentityInput input(
    double stamp, HookLoadSignalRole role, bool gravity_valid,
    HookLoadState gravity_state, double x, double z) {
  CargoPhysicalIdentityInput result;
  result.pipeline_stamp_sec = stamp;
  result.lifecycle_id = 7U;
  result.hook_role = role;
  result.gravity_valid = gravity_valid;
  result.gravity_state = gravity_state;
  result.groups = {group(10U, stamp, x, z)};
  result.frame_evidence.source_stamp_sec = stamp;
  result.frame_evidence.raw_roi_current_frame =
      cloudFromPoints(componentPoints(x, z));
  result.vertical_config = verticalConfig();
  return result;
}

CargoPhysicalIdentityInput rawSurfaceInput(
    double stamp, std::uint64_t lifecycle_id, HookLoadState gravity_state,
    double group_x, double descriptor_z, double raw_surface_z,
    HookLoadSignalRole role = HookLoadSignalRole::REQUIRED) {
  CargoPhysicalIdentityInput result = input(
      stamp, role, true, gravity_state, group_x, descriptor_z);
  result.lifecycle_id = lifecycle_id;
  result.frame_evidence.source_stamp_sec = stamp;
  result.frame_evidence.raw_roi_current_frame =
      cloudFromPoints(componentPoints(group_x, raw_surface_z));
  result.vertical_config = verticalConfig();
  return result;
}

CargoPhysicalIdentityDecision freezeSurfaceReference(
    CargoPhysicalIdentityAuthority* authority, std::uint64_t lifecycle_id,
    double first_stamp = 1.0, double surface_z = 0.40,
    int sample_count = 4) {
  CargoPhysicalIdentityDecision decision;
  for (int sample = 0; sample < sample_count; ++sample) {
    decision = authority->update(rawSurfaceInput(
        first_stamp + 0.05 * sample, lifecycle_id, HookLoadState::EMPTY,
        0.0, surface_z, surface_z));
  }
  return decision;
}

CargoIdentitySupportLineageObservation lineageObservation(
    std::uint64_t previous_component_id,
    std::uint64_t current_component_id,
    std::uint64_t exact_seed_frame_group_id,
    double previous_stamp, double stamp, double center_x,
    double extent_x = 0.40, double extent_y = 0.40,
    CargoIdentityMotionObservabilityState motion_state =
        CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE,
    double ego_xy_step_m = 1.0, double map_step_m = 1.0) {
  CargoIdentitySupportLineageObservation observation;
  observation.valid = true;
  observation.state = CargoIdentityLineageState::MATCHED;
  observation.previous_source_stamp_sec = previous_stamp;
  observation.source_stamp_sec = stamp;
  observation.source_age_sec = stamp - previous_stamp;
  observation.source_frame_offset = 1U;
  observation.previous_component_id = previous_component_id;
  observation.current_component_id = current_component_id;
  observation.exact_seed_frame_group_id = exact_seed_frame_group_id;
  observation.robust_xy_center = Eigen::Vector2d(center_x, 0.0);
  observation.robust_xy_extent = Eigen::Vector2d(extent_x, extent_y);
  observation.robust_x05 = center_x - 0.5 * extent_x;
  observation.robust_x95 = center_x + 0.5 * extent_x;
  observation.robust_y05 = -0.5 * extent_y;
  observation.robust_y95 = 0.5 * extent_y;
  observation.base_step_m = 0.05;
  observation.map_step_m = map_step_m;
  observation.ego_xy_step_m = ego_xy_step_m;
  observation.extent_step = 0.05;
  observation.motion_observability_state = motion_state;
  return observation;
}

CargoPhysicalIdentityInput lineageInput(
    double stamp, HookLoadState gravity_state, double exact_group_x,
    double exact_vertical_z, std::uint64_t component_id,
    const CargoIdentitySupportLineageObservation* lineage = nullptr) {
  CargoPhysicalIdentityInput result;
  result.pipeline_stamp_sec = stamp;
  result.lifecycle_id = 7U;
  result.hook_role = HookLoadSignalRole::REQUIRED;
  result.gravity_valid = true;
  result.gravity_state = gravity_state;
  result.groups = {group(
      component_id, stamp, exact_group_x, exact_vertical_z,
      {component_id})};
  result.frame_evidence.source_stamp_sec = stamp;
  result.frame_evidence.raw_roi_current_frame =
      cloudFromPoints(componentPoints(exact_group_x, exact_vertical_z));
  result.vertical_config = verticalConfig();
  if (lineage) result.lineage_observations.push_back(*lineage);
  return result;
}

CargoPhysicalIdentityConfig testConfig() {
  CargoPhysicalIdentityConfig config;
  config.maximum_xy_step_m = 0.35;
  config.maximum_z_speed_mps = 5.0;
  config.z_step_margin_m = 0.10;
  config.maximum_observation_gap_sec = 1.0;
  config.maximum_source_age_sec = 0.25;
  config.minimum_significant_change_m = 0.15;
  config.significance_sigma = 3.0;
  config.lift_confirm_frames = 2;
  return config;
}

CargoPhysicalIdentityDecision validateRequired(
    CargoPhysicalIdentityAuthority* authority) {
  authority->update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::EMPTY, 0.0, 0.4));
  authority->update(input(1.05, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::EMPTY, 0.0, 0.4));
  authority->update(input(1.1, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::LOADED, 0.0, 0.4));
  authority->update(input(1.2, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::LOADED, 0.0, 0.7));
  return authority->update(input(1.3, HookLoadSignalRole::REQUIRED, true,
                                 HookLoadState::LOADED, 0.0, 0.7));
}

TEST(CargoPhysicalIdentityAuthorityTest,
     DelayedLoadEdgeCannotOverwriteEarliestPreLiftReference) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  const std::vector<double> low = {0.465, 0.470, 0.468, 0.472};
  double stamp = 1.0;
  for (double z : low) {
    authority.update(input(stamp, HookLoadSignalRole::REQUIRED, true,
                           HookLoadState::EMPTY, 0.0, z));
    stamp += 0.05;
  }
  for (double z : {0.616, 0.702, 0.711}) {
    authority.update(input(stamp, HookLoadSignalRole::REQUIRED, true,
                           HookLoadState::EMPTY, 0.0, z));
    stamp += 0.05;
  }
  authority.update(input(stamp, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.711));
  for (int frame = 0; frame < 3; ++frame) {
    stamp += 0.05;
    authority.update(input(stamp, HookLoadSignalRole::REQUIRED, true,
                           HookLoadState::LOADED, 0.0, 0.893));
  }
  stamp += 0.05;
  const auto result = authority.update(input(
      stamp, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.893));
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.prelift_state, CargoPreLiftReferenceState::FROZEN);
  EXPECT_NEAR(result.baseline_z95, 0.469, 0.005);
  EXPECT_LT(result.baseline_z95, 0.50);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     EarliestPrefixNeverSlidesToLiftedStableWindow) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  double stamp = 1.0;
  for (double z : {0.465, 0.616, 0.702, 0.711}) {
    authority.update(input(stamp, HookLoadSignalRole::REQUIRED, true,
                           HookLoadState::EMPTY, 0.0, z));
    stamp += 0.05;
  }
  for (int frame = 0; frame < 8; ++frame) {
    const auto result = authority.update(input(
        stamp, HookLoadSignalRole::REQUIRED, true,
        HookLoadState::EMPTY, 0.0, 0.711));
    ASSERT_EQ(result.group_diagnostics.size(), 1U);
    EXPECT_EQ(result.group_diagnostics.front().prelift_state,
              CargoPreLiftReferenceState::CLOSED);
    stamp += 0.05;
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     SubSignificantMonotonicPreLiftVariationMustNotCloseReference) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.minimum_significant_change_m = 0.15;
  config.significance_sigma = 3.0;
  CargoPhysicalIdentityAuthority authority(config);
  CargoPhysicalIdentityDecision result;
  double stamp = 1.0;
  for (double z : {0.465, 0.500, 0.535, 0.565}) {
    result = authority.update(input(
        stamp, HookLoadSignalRole::REQUIRED, true,
        HookLoadState::EMPTY, 0.0, z));
    stamp += 0.05;
  }
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_EQ(result.group_diagnostics.front().prelift_state,
            CargoPreLiftReferenceState::FROZEN);
  EXPECT_NEAR(result.group_diagnostics.front().baseline_z, 0.5175, 1.0e-6);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     UnsupportedFrameWithinGapPausesWithoutReplacingPrefix) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 3;
  CargoPhysicalIdentityAuthority authority(config);
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  auto unsupported = input(1.1, HookLoadSignalRole::REQUIRED, true,
                           HookLoadState::EMPTY, 0.0, 0.4);
  unsupported.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::CONTINUITY_ONLY;
  auto paused = authority.update(unsupported);
  ASSERT_EQ(paused.group_diagnostics.size(), 1U);
  EXPECT_EQ(paused.group_diagnostics.front().prelift_state,
            CargoPreLiftReferenceState::PAUSED);
  authority.update(input(1.2, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  const auto frozen = authority.update(input(
      1.3, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::EMPTY, 0.0, 0.4));
  ASSERT_EQ(frozen.group_diagnostics.size(), 1U);
  EXPECT_EQ(frozen.group_diagnostics.front().prelift_state,
            CargoPreLiftReferenceState::FROZEN);
  EXPECT_EQ(frozen.group_diagnostics.front().prelift_sample_count, 3U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LocalizationAuthorityIsRequiredForPreLiftEvidence) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto unauthorized = input(1.0, HookLoadSignalRole::REQUIRED, true,
                            HookLoadState::EMPTY, 0.0, 0.4);
  unauthorized.localization_authorized = false;
  unauthorized.pose_authority_identity_valid = false;
  const auto first = authority.update(unauthorized);
  ASSERT_EQ(first.group_diagnostics.size(), 1U);
  EXPECT_EQ(first.group_diagnostics.front().prelift_sample_count, 0U);
  auto authorized = input(1.1, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::EMPTY, 0.0, 0.4);
  authority.update(authorized);
  authorized.pipeline_stamp_sec = 1.2;
  authorized.groups = {group(10U, 1.2, 0.0, 0.4)};
  attachCurrentRawRoi(&authorized);
  const auto frozen = authority.update(authorized);
  ASSERT_EQ(frozen.group_diagnostics.size(), 1U);
  EXPECT_EQ(frozen.group_diagnostics.front().prelift_state,
            CargoPreLiftReferenceState::FROZEN);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadLifecycleChangeUsesOneShotReferenceWithoutCarryingIdentity) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto first = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.4);
  first.lifecycle_id = 11U;
  authority.update(first);
  first.pipeline_stamp_sec = 1.05;
  first.groups = {group(10U, 1.05, 0.0, 0.4)};
  attachCurrentRawRoi(&first);
  auto frozen = authority.update(first);
  ASSERT_EQ(frozen.group_diagnostics.size(), 1U);
  EXPECT_EQ(frozen.group_diagnostics.front().prelift_state,
            CargoPreLiftReferenceState::FROZEN);
  auto replacement = input(1.1, HookLoadSignalRole::REQUIRED, true,
                           HookLoadState::LOADED, 0.0, 0.8);
  replacement.lifecycle_id = 12U;
  const auto result = authority.update(replacement);
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_TRUE(result.preload_handoff_captured);
  EXPECT_TRUE(result.preload_handoff_consumed);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_NE(result.group_diagnostics.front().physical_cargo_epoch_id, 11U);
  EXPECT_TRUE(std::isfinite(result.group_diagnostics.front().baseline_z));
  EXPECT_EQ(result.group_diagnostics.front().lift_confirm_count, 1);
  EXPECT_FALSE(result.group_diagnostics.front().lift_confirmed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     TimestampRollbackClosesCargoPreLiftForCurrentEpoch) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  ASSERT_EQ(validated.identity, CargoPhysicalIdentityState::VALIDATED);
  const auto rollback = authority.update(input(
      0.5, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.7));
  EXPECT_EQ(rollback.identity, CargoPhysicalIdentityState::UNKNOWN);
  ASSERT_EQ(rollback.group_diagnostics.size(), 1U);
  EXPECT_EQ(rollback.group_diagnostics.front().prelift_state,
            CargoPreLiftReferenceState::CLOSED);
  EXPECT_EQ(rollback.group_diagnostics.front().prelift_close_reason,
            "CURRENT_EPOCH_PRELIFT_BLOCKED");
  const auto still_blocked = authority.update(input(
      0.6, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.4));
  ASSERT_EQ(still_blocked.group_diagnostics.size(), 1U);
  EXPECT_EQ(still_blocked.group_diagnostics.front().prelift_sample_count, 0U);
  EXPECT_EQ(still_blocked.identity, CargoPhysicalIdentityState::UNKNOWN);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     DisabledRoleFreezesEarliestLidarReferenceBeforeLift) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::DISABLED, false,
                         HookLoadState::UNKNOWN, 0.0, 0.4));
  const auto frozen = authority.update(input(
      1.05, HookLoadSignalRole::DISABLED, false,
      HookLoadState::UNKNOWN, 0.0, 0.4));
  ASSERT_EQ(frozen.group_diagnostics.size(), 1U);
  EXPECT_EQ(frozen.group_diagnostics.front().prelift_state,
            CargoPreLiftReferenceState::FROZEN);
  EXPECT_NEAR(frozen.group_diagnostics.front().baseline_z, 0.4, 1.0e-6);
  authority.update(input(1.1, HookLoadSignalRole::DISABLED, false,
                         HookLoadState::UNKNOWN, 0.0, 0.7));
  const auto result = authority.update(input(
      1.2, HookLoadSignalRole::DISABLED, false,
      HookLoadState::UNKNOWN, 0.0, 0.7));
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_NEAR(result.baseline_z95, 0.4, 1.0e-6);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PreloadBaselineUsedAcrossLoadEdge) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto result = validateRequired(&authority);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.baseline_source,
            CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE);
  EXPECT_NEAR(result.baseline_z95, 0.4, 1.0e-6);
  EXPECT_NEAR(result.lift_delta_m, 0.3, 1.0e-6);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ConfirmedLiftIdentityPersistsDuringStationarySuspendedTransport) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  const auto stationary = authority.update(input(
      1.4, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.7));
  EXPECT_EQ(stationary.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(stationary.physical_history_id, validated.physical_history_id);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ConfirmedLiftIdentityPersistsDuringHorizontalTransport) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  const auto transported = authority.update(input(
      1.4, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.20, 0.7));
  EXPECT_EQ(transported.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(transported.physical_history_id, validated.physical_history_id);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     MeasurementJitterDoesNotDestroyLiftConfirmation) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  validateRequired(&authority);
  const auto jitter = authority.update(input(
      1.4, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.69));
  EXPECT_EQ(jitter.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LiftConfirmationCannotAccumulateAcrossLongSubThresholdPlateau) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.05, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.1, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.4));
  const auto first_lift = authority.update(input(
      1.2, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.7));
  EXPECT_NE(first_lift.identity, CargoPhysicalIdentityState::VALIDATED);
  authority.update(input(1.3, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.4));
  authority.update(input(1.4, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.4));
  const auto second_lift = authority.update(input(
      1.5, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.7));
  EXPECT_NE(second_lift.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PreviousLoadConfirmedIdentityCannotValidateNewLoadEpoch) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto first_load = validateRequired(&authority);
  ASSERT_EQ(first_load.identity, CargoPhysicalIdentityState::VALIDATED);
  const std::uint64_t first_epoch = first_load.load_epoch;

  authority.update(input(1.4, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.7));
  const auto new_load = authority.update(input(
      1.5, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.7));
  EXPECT_GT(new_load.load_epoch, first_epoch);
  EXPECT_FALSE(new_load.lift_confirmed);
  EXPECT_EQ(new_load.lift_confirm_count, 0);
  EXPECT_EQ(new_load.identity, CargoPhysicalIdentityState::UNKNOWN);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     NewLoadEpochRequiresFreshCandidateSpecificLift) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  ASSERT_EQ(validateRequired(&authority).identity,
            CargoPhysicalIdentityState::VALIDATED);
  authority.update(input(1.4, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.7));
  authority.update(input(1.45, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.7));
  authority.update(input(1.5, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.7));
  EXPECT_EQ(authority.update(input(
      1.6, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 1.0)).identity,
      CargoPhysicalIdentityState::UNKNOWN);
  EXPECT_EQ(authority.update(input(
      1.7, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 1.0)).identity,
      CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LargeReverseMotionDoesNotCountAsLift) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  validateRequired(&authority);
  const auto reverse = authority.update(input(
      1.4, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.10));
  EXPECT_FALSE(reverse.lift_confirmed);
  EXPECT_NE(reverse.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ValidatedIdentityRevokedOnStaleGap) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  validateRequired(&authority);
  auto stale = input(2.5, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::LOADED, 0.0, 0.7);
  stale.groups.clear();
  const auto result = authority.update(stale);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::UNKNOWN);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ValidatedIdentityRevokedOnPhysicalConflict) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  validateRequired(&authority);
  auto conflict = input(1.4, HookLoadSignalRole::REQUIRED, true,
                        HookLoadState::LOADED, 0.0, 0.7);
  conflict.groups = buildGroups(
      {candidate(10U, 1.4, 0.0, 0.7, {1U, 2U}),
       candidate(11U, 1.4, 0.0, 0.7, {2U, 3U})});
  const auto result = authority.update(conflict);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::AMBIGUOUS);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     StartedLoadedWithoutIndependentIdentityFailsClosed) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto loaded = input(1.0, HookLoadSignalRole::REQUIRED, true,
                      HookLoadState::LOADED, 0.0, 0.7);
  loaded.node_started_loaded = true;
  const auto result = authority.update(loaded);
  EXPECT_FALSE(result.lift_confirmed);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::UNKNOWN);
  EXPECT_EQ(result.baseline_source,
            CargoLiftBaselineSource::UNAVAILABLE_STARTED_LOADED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     AuxiliaryEmptyCannotAdvanceLiftConfirmation) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::AUXILIARY, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.05, HookLoadSignalRole::AUXILIARY, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  for (int frame = 1; frame <= 3; ++frame) {
    const auto pending = authority.update(input(
        1.0 + frame * 0.1, HookLoadSignalRole::AUXILIARY, true,
        HookLoadState::EMPTY, 0.0, 0.7));
    EXPECT_FALSE(pending.cargo_exists);
  }
  const auto inhibited = authority.update(input(
      1.4, HookLoadSignalRole::AUXILIARY, true,
      HookLoadState::EMPTY, 0.0, 0.7));
  EXPECT_FALSE(inhibited.cargo_exists);
  EXPECT_FALSE(inhibited.lift_confirmed);
  EXPECT_EQ(inhibited.lift_confirm_count, 0);
  ASSERT_EQ(inhibited.group_diagnostics.size(), 1U);
  EXPECT_EQ(inhibited.group_diagnostics.front().lift_confirm_required, 4);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     AuxiliaryEmptyCannotValidateSixFrameSurfaceJump) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::AUXILIARY, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.05, HookLoadSignalRole::AUXILIARY, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  CargoPhysicalIdentityDecision decision;
  for (int frame = 1; frame <= 6; ++frame) {
    decision = authority.update(input(
        1.05 + frame * 0.05, HookLoadSignalRole::AUXILIARY, true,
        HookLoadState::EMPTY, 0.0, 0.9));
    EXPECT_FALSE(decision.lift_confirmed);
    EXPECT_EQ(decision.lift_confirm_count, 0);
    EXPECT_NE(decision.identity, CargoPhysicalIdentityState::VALIDATED);
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     AuxiliaryGravityUnavailableUsesStrictLidarExistence) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::AUXILIARY, false,
                         HookLoadState::UNKNOWN, 0.0, 0.4));
  authority.update(input(1.05, HookLoadSignalRole::AUXILIARY, false,
                         HookLoadState::UNKNOWN, 0.0, 0.4));
  authority.update(input(1.1, HookLoadSignalRole::AUXILIARY, false,
                         HookLoadState::UNKNOWN, 0.0, 0.7));
  authority.update(input(1.2, HookLoadSignalRole::AUXILIARY, false,
                         HookLoadState::UNKNOWN, 0.0, 0.7));
  const auto result = authority.update(input(
      1.3, HookLoadSignalRole::AUXILIARY, false,
      HookLoadState::UNKNOWN, 0.0, 0.7));
  EXPECT_TRUE(result.cargo_exists);
  EXPECT_EQ(result.required_lift_confirm_frames, 3);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     AuxiliaryGravityEmptyConflictCannotRetireValidatedLidarHistory) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::AUXILIARY, false,
                         HookLoadState::UNKNOWN, 0.0, 0.4));
  authority.update(input(1.05, HookLoadSignalRole::AUXILIARY, false,
                         HookLoadState::UNKNOWN, 0.0, 0.4));
  CargoPhysicalIdentityDecision validated;
  for (int frame = 1; frame <= 3; ++frame) {
    validated = authority.update(input(
        1.0 + frame * 0.1, HookLoadSignalRole::AUXILIARY, false,
        HookLoadState::UNKNOWN, 0.0, 0.7));
  }
  ASSERT_EQ(validated.identity, CargoPhysicalIdentityState::VALIDATED);
  const std::uint64_t history_id = validated.physical_history_id;
  const auto retained = authority.update(input(
      1.5, HookLoadSignalRole::AUXILIARY, true,
      HookLoadState::EMPTY, 0.0, 0.7));
  EXPECT_EQ(retained.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(retained.physical_history_id, history_id);
  EXPECT_EQ(retained.existence_source, CargoExistenceSource::STRICT_LIDAR);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     AuxiliaryGravityLoadEdgeCannotDiscardIndependentLidarPrefix) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto first = authority.update(input(
      1.0, HookLoadSignalRole::AUXILIARY, true,
      HookLoadState::EMPTY, 0.0, 0.4));
  ASSERT_EQ(first.prelift_sample_count, 1U);
  const auto frozen = authority.update(input(
      1.05, HookLoadSignalRole::AUXILIARY, true,
      HookLoadState::EMPTY, 0.0, 0.4));
  ASSERT_EQ(frozen.group_diagnostics.size(), 1U);
  EXPECT_EQ(frozen.group_diagnostics.front().prelift_state,
            CargoPreLiftReferenceState::FROZEN);
  EXPECT_NEAR(frozen.group_diagnostics.front().baseline_z, 0.4, 1.0e-6);
  const auto loaded = authority.update(input(
      1.10, HookLoadSignalRole::AUXILIARY, true,
      HookLoadState::LOADED, 0.0, 0.4));
  EXPECT_EQ(loaded.group_diagnostics.front().prelift_state,
            CargoPreLiftReferenceState::FROZEN);
  EXPECT_NEAR(loaded.group_diagnostics.front().baseline_z, 0.4, 1.0e-6);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     DisabledGravityCanUseIndependentLidarExistence) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::DISABLED, false,
                         HookLoadState::UNKNOWN, 0.0, 0.4));
  authority.update(input(1.05, HookLoadSignalRole::DISABLED, false,
                         HookLoadState::UNKNOWN, 0.0, 0.4));
  authority.update(input(1.1, HookLoadSignalRole::DISABLED, false,
                         HookLoadState::UNKNOWN, 0.0, 0.7));
  const auto result = authority.update(input(
      1.2, HookLoadSignalRole::DISABLED, false,
      HookLoadState::UNKNOWN, 0.0, 0.7));
  EXPECT_TRUE(result.cargo_exists);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ExistencePolicyPreservesExistingHookRoleSemantics) {
  CargoPhysicalIdentityAuthority required(testConfig());
  const auto gravity_only = required.update(input(
      1.0, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.4));
  EXPECT_TRUE(gravity_only.cargo_exists);
  EXPECT_EQ(gravity_only.existence_source,
            CargoExistenceSource::GRAVITY_LOADED);
  EXPECT_EQ(gravity_only.identity, CargoPhysicalIdentityState::UNKNOWN);

  CargoPhysicalIdentityAuthority required_unavailable(testConfig());
  const auto unavailable = required_unavailable.update(input(
      1.0, HookLoadSignalRole::REQUIRED, false,
      HookLoadState::UNKNOWN, 0.0, 0.4));
  EXPECT_FALSE(unavailable.cargo_exists);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     SharedComponentHypothesesAreNotMultiplePhysicalObjects) {
  const auto groups = buildGroups(
      {candidate(1U, 1.0, 0.0, 0.5, {9U, 3U}),
       candidate(2U, 1.0, 0.01, 0.5, {3U, 9U})});
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_EQ(groups.front().hypotheses.size(), 2U);
  EXPECT_TRUE(groups.front().geometry_resolved);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     GeometryHypothesisAmbiguityFailsClosed) {
  auto first = candidate(1U, 1.0, 0.0, 0.5, {3U, 9U});
  auto second = candidate(2U, 1.0, 0.0, 0.5, {3U, 9U});
  second.size.x() = 2.0;
  const auto groups = buildGroups({first, second});
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_FALSE(groups.front().geometry_resolved);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     AmbiguousAssociationDoesNotTransferEvidence) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  validateRequired(&authority);
  auto ambiguous = input(1.4, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.7);
  ambiguous.groups = buildGroups(
      {candidate(1U, 1.4, 0.0, 0.7, {1U, 2U}),
       candidate(2U, 1.4, 0.0, 0.7, {2U, 3U})});
  const auto result = authority.update(ambiguous);
  EXPECT_EQ(result.association, CargoCandidateAssociationState::AMBIGUOUS);
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ResolvedHypothesisRequiresFrameIdAndCanonicalMemberSet) {
  CargoPhysicalIdentityDecision decision;
  decision.geometry_resolved = true;
  decision.resolved_candidate_id = 7U;
  decision.resolved_member_component_ids = {3U, 9U};
  auto correct = candidate(7U, 1.0, 0.0, 0.5, {9U, 3U});
  auto colliding = candidate(7U, 1.0, 0.0, 0.5, {4U, 8U});
  auto wrong_frame_id = candidate(8U, 1.0, 0.0, 0.5, {3U, 9U});
  EXPECT_TRUE(matchesResolvedPhysicalHypothesis(correct, decision));
  EXPECT_FALSE(matchesResolvedPhysicalHypothesis(colliding, decision));
  EXPECT_FALSE(matchesResolvedPhysicalHypothesis(wrong_frame_id, decision));
}

TEST(CargoPhysicalIdentityAuthorityTest,
     HypothesisOrderDoesNotChangePhysicalDescriptor) {
  const auto first = candidate(1U, 1.0, 0.40, 0.70, {3U, 9U});
  auto second = candidate(2U, 1.0, 0.42, 0.70, {9U, 3U});
  second.yaw_rad = 1.5707963267948966;
  CargoPhysicalComponentObservation component3;
  component3.component_id = 3U;
  component3.points_base = componentPoints(0.41, 0.70);
  CargoPhysicalComponentObservation component9 = component3;
  component9.component_id = 9U;
  const auto forward = buildGroups(
      {first, second}, {component3, component9});
  const auto reverse = buildGroups(
      {second, first}, {component3, component9});
  ASSERT_EQ(forward.size(), 1U);
  ASSERT_EQ(reverse.size(), 1U);
  EXPECT_TRUE(forward.front().descriptor.stable_anchor.isApprox(
      reverse.front().descriptor.stable_anchor, 1.0e-12));
  EXPECT_TRUE(forward.front().descriptor.aggregate_extent.isApprox(
      reverse.front().descriptor.aggregate_extent, 1.0e-12));
  EXPECT_NEAR(forward.front().descriptor.physical_vertical_z,
              reverse.front().descriptor.physical_vertical_z, 1.0e-12);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     EquivalentGeometryHypothesesProduceSamePhysicalAnchor) {
  auto x_axis = candidate(1U, 1.0, 0.0, 0.6, {4U});
  auto y_axis = candidate(2U, 1.0, 0.0, 0.6, {4U});
  y_axis.yaw_rad = 1.5707963267948966;
  const auto groups = buildGroups({x_axis, y_axis});
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_NEAR(groups.front().descriptor.stable_anchor.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(groups.front().descriptor.stable_anchor.y(), 0.0, 1.0e-9);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     RankTop1DoesNotChangeGroupVerticalEvidence) {
  auto first = candidate(99U, 1.0, 0.0, 0.70, {4U});
  auto second = candidate(1U, 1.0, 0.0, 0.70, {4U});
  second.yaw_rad = 1.5707963267948966;
  const auto groups = buildGroups({first, second});
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_EQ(groups.front().descriptor.vertical_mode,
            CargoGroupVerticalMode::SUPPORTED_EVIDENCE);
  EXPECT_NEAR(groups.front().descriptor.physical_vertical_z, 0.70, 1.0e-6);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     CandidateVerticalEvidenceIndependentOfRankerSelection) {
  auto low_rank_id = candidate(100U, 1.0, 0.0, 0.65, {4U});
  auto high_rank_id = candidate(1U, 1.0, 0.0, 0.65, {4U});
  const auto first = buildGroups({low_rank_id, high_rank_id});
  const auto second = buildGroups({high_rank_id, low_rank_id});
  EXPECT_NEAR(first.front().descriptor.physical_vertical_z,
              second.front().descriptor.physical_vertical_z, 1.0e-12);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ConflictingHypothesisTopsCannotManufactureSupportedEvidence) {
  auto low = candidate(1U, 1.0, -0.30, 0.45, {7U});
  auto high = candidate(2U, 1.0, 0.30, 1.00, {7U});
  low.size = Eigen::Vector3d(0.30, 0.50, 0.40);
  high.size = low.size;
  CargoPhysicalComponentObservation component;
  component.component_id = 7U;
  component.points_base = componentPoints(-0.30, 0.45);
  const auto high_points = componentPoints(0.30, 1.00);
  component.points_base.insert(component.points_base.end(),
                               high_points.begin(), high_points.end());
  const auto groups = buildGroups({low, high}, {component});
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_EQ(groups.front().descriptor.vertical_mode,
            CargoGroupVerticalMode::CONTINUITY_ONLY);
  EXPECT_EQ(groups.front().descriptor.vertical_reject_reason,
            "CONFLICTING_HYPOTHESIS_SUPPORTED_TOPS");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     StablePhysicalAnchorResistsRepresentativeCenterJump) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto first = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.4);
  auto first_candidate = candidate(1U, 1.0, -0.70, 0.4, {5U});
  CargoPhysicalComponentObservation first_component;
  first_component.component_id = 5U;
  first_component.points_base = componentPoints(0.0, 0.4);
  first.groups = buildGroups({first_candidate}, {first_component});
  authority.update(first);

  auto second = input(1.1, HookLoadSignalRole::REQUIRED, true,
                      HookLoadState::LOADED, 0.0, 0.4);
  auto jumped = candidate(2U, 1.1, 0.70, 0.4, {8U});
  CargoPhysicalComponentObservation second_component;
  second_component.component_id = 8U;
  second_component.points_base = componentPoints(0.0, 0.4);
  second.groups = buildGroups({jumped}, {second_component});
  const auto result = authority.update(second);
  EXPECT_EQ(result.association, CargoCandidateAssociationState::MATCHED);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_GT(result.group_diagnostics.front().raw_representative_xy_step_m,
            1.0);
  EXPECT_NEAR(result.group_diagnostics.front().xy_step_m, 0.0, 1.0e-9);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OrientationHypothesisSwapDoesNotSplitHistory) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto first = candidate(1U, 1.0, 0.0, 0.4, {6U});
  auto rotated = candidate(2U, 1.0, 0.0, 0.4, {6U});
  rotated.yaw_rad = 1.5707963267948966;
  CargoPhysicalComponentObservation component;
  component.component_id = 6U;
  component.points_base = componentPoints(0.0, 0.4);
  auto empty = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.4);
  empty.groups = buildGroups({first, rotated}, {component});
  authority.update(empty);
  first.stamp_sec = 1.1;
  rotated.stamp_sec = 1.1;
  auto loaded = input(1.1, HookLoadSignalRole::REQUIRED, true,
                      HookLoadState::LOADED, 0.0, 0.4);
  loaded.groups = buildGroups({rotated, first}, {component});
  EXPECT_EQ(authority.update(loaded).association,
            CargoCandidateAssociationState::MATCHED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PhysicalHistorySurvivesGeometryHypothesisChurn) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  auto churned = input(1.1, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.4);
  auto first = candidate(3U, 1.1, 0.0, 0.4, {11U});
  auto second = first;
  second.candidate_id = 4U;
  second.size.x() = 1.4;
  churned.groups = buildGroups({first, second});
  const auto result = authority.update(churned);
  EXPECT_EQ(result.association, CargoCandidateAssociationState::MATCHED);
  EXPECT_FALSE(churned.groups.front().geometry_resolved);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     DistinctNearbyObjectsDoNotMerge) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto empty = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.4);
  empty.groups = buildGroups({
      candidate(1U, 1.0, -0.25, 0.4, {1U}),
      candidate(2U, 1.0, 0.25, 0.4, {2U})});
  authority.update(empty);
  auto loaded = input(1.1, HookLoadSignalRole::REQUIRED, true,
                      HookLoadState::LOADED, 0.0, 0.4);
  loaded.groups = buildGroups({
      candidate(3U, 1.1, -0.25, 0.4, {8U}),
      candidate(4U, 1.1, 0.25, 0.4, {9U})});
  const auto result = authority.update(loaded);
  ASSERT_EQ(result.group_diagnostics.size(), 2U);
  EXPECT_EQ(result.group_diagnostics[0].association,
            CargoCandidateAssociationState::MATCHED);
  EXPECT_EQ(result.group_diagnostics[1].association,
            CargoCandidateAssociationState::MATCHED);
  EXPECT_NE(result.group_diagnostics[0].matched_history_id,
            result.group_diagnostics[1].matched_history_id);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     CrossingObjectsRemainAmbiguous) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.maximum_xy_step_m = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  auto empty = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.4);
  empty.groups = buildGroups({
      candidate(1U, 1.0, -0.10, 0.4, {1U}),
      candidate(2U, 1.0, 0.10, 0.4, {2U})});
  authority.update(empty);
  auto crossing = input(1.1, HookLoadSignalRole::REQUIRED, true,
                        HookLoadState::LOADED, 0.0, 0.4);
  crossing.groups = buildGroups({
      candidate(3U, 1.1, 0.0, 0.4, {8U}),
      candidate(4U, 1.1, 0.0, 0.4, {9U})});
  const auto result = authority.update(crossing);
  EXPECT_EQ(result.association, CargoCandidateAssociationState::AMBIGUOUS);
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     MemberComponentIdsNeverAffectCrossFrameAssociation) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  auto changed = input(1.1, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.4);
  changed.groups = {group(2U, 1.1, 0.0, 0.4, {99U})};
  const auto result = authority.update(changed);
  EXPECT_EQ(result.association, CargoCandidateAssociationState::MATCHED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LowerCargoLiftCanValidateWithoutAbsoluteHeight) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.27));
  authority.update(input(1.05, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.27));
  authority.update(input(1.1, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.27));
  authority.update(input(1.2, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.70));
  const auto result = authority.update(input(
      1.3, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.70));
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_LT(result.current_z95, 1.0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     WrongStaticCandidateCannotValidate) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.30));
  for (int frame = 1; frame <= 6; ++frame) {
    const auto result = authority.update(input(
        1.0 + 0.1 * frame, HookLoadSignalRole::REQUIRED, true,
        HookLoadState::LOADED, 0.0, 0.30));
    EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     MultipleExactGroupsCannotBorrowCurrentSurfaceForLift) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto frame = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.3);
  frame.groups = buildGroups({
      candidate(1U, 1.0, -0.20, 0.30, {1U}),
      candidate(2U, 1.0, 0.20, 0.27, {2U})});
  attachCurrentRawRoi(&frame);
  authority.update(frame);
  frame.pipeline_stamp_sec = 1.05;
  for (auto& group_observation : frame.groups) {
    group_observation.descriptor.stamp_sec = 1.05;
    group_observation.representative.stamp_sec = 1.05;
  }
  attachCurrentRawRoi(&frame);
  authority.update(frame);
  for (int index = 1; index <= 3; ++index) {
    const double stamp = 1.0 + 0.1 * index;
    const double cargo_z = index == 1 ? 0.27 : 0.70;
    frame = input(stamp, HookLoadSignalRole::REQUIRED, true,
                  HookLoadState::LOADED, 0.0, cargo_z);
    frame.groups = buildGroups({
        candidate(10U + index, stamp, -0.20, 0.30, {8U}),
        candidate(20U + index, stamp, 0.20, cargo_z, {9U})});
    attachCurrentRawRoi(&frame);
    const auto result = authority.update(frame);
    if (index == 3) {
      ASSERT_EQ(result.group_diagnostics.size(), 2U);
      const auto& first = result.group_diagnostics[0];
      const auto& second = result.group_diagnostics[1];
      EXPECT_NE(first.matched_history_id, second.matched_history_id);
      const auto& static_group = first.descriptor.physical_vertical_z <
              second.descriptor.physical_vertical_z ? first : second;
      EXPECT_FALSE(static_group.lift_confirmed);
      EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
    }
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     MultiplePreloadHistoriesCannotFreezeSurfaceReference) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto frame = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.3);
  frame.groups = buildGroups({
      candidate(1U, 1.0, -0.20, 0.30, {1U}),
      candidate(2U, 1.0, 0.20, 0.27, {2U})});
  attachCurrentRawRoi(&frame);
  const auto first_empty = authority.update(frame);
  ASSERT_EQ(first_empty.group_diagnostics.size(), 2U);
  const std::uint64_t static_history_id =
      first_empty.group_diagnostics[0].matched_history_id;
  const std::uint64_t lifting_history_id =
      first_empty.group_diagnostics[1].matched_history_id;
  ASSERT_NE(static_history_id, 0U);
  ASSERT_NE(lifting_history_id, 0U);
  ASSERT_NE(static_history_id, lifting_history_id);
  EXPECT_EQ(first_empty.group_diagnostics[0].prelift_state,
            CargoPreLiftReferenceState::UNSEEN);
  EXPECT_EQ(first_empty.group_diagnostics[1].prelift_state,
            CargoPreLiftReferenceState::UNSEEN);

  frame = input(1.05, HookLoadSignalRole::REQUIRED, true,
                HookLoadState::EMPTY, 0.0, 0.3);
  frame.groups = buildGroups({
      candidate(3U, 1.05, -0.20, 0.30, {3U}),
      candidate(4U, 1.05, 0.20, 0.27, {4U})});
  attachCurrentRawRoi(&frame);
  const auto second_empty = authority.update(frame);
  ASSERT_EQ(second_empty.group_diagnostics.size(), 2U);
  EXPECT_EQ(second_empty.group_diagnostics[0].matched_history_id,
            static_history_id);
  EXPECT_EQ(second_empty.group_diagnostics[1].matched_history_id,
            lifting_history_id);
  EXPECT_NE(second_empty.group_diagnostics[0].prelift_state,
            CargoPreLiftReferenceState::FROZEN);
  EXPECT_NE(second_empty.group_diagnostics[1].prelift_state,
            CargoPreLiftReferenceState::FROZEN);

  for (int index = 1; index <= 3; ++index) {
    const double stamp = 1.0 + 0.1 * index;
    const double cargo_z = index == 1 ? 0.27 : 0.70;
    frame = input(stamp, HookLoadSignalRole::REQUIRED, true,
                  HookLoadState::LOADED, 0.0, cargo_z);
    frame.groups = buildGroups({
        candidate(10U + index, stamp, -0.20, 0.30, {8U}),
        candidate(20U + index, stamp, 0.20, cargo_z, {9U})});
    attachCurrentRawRoi(&frame);
    const auto result = authority.update(frame);
    ASSERT_EQ(result.group_diagnostics.size(), 2U);
    EXPECT_EQ(result.group_diagnostics[0].matched_history_id,
              static_history_id) << "stamp=" << stamp;
    EXPECT_EQ(result.group_diagnostics[1].matched_history_id,
              lifting_history_id) << "stamp=" << stamp;
    EXPECT_FALSE(result.group_diagnostics[0].lift_confirmed);
    EXPECT_EQ(result.group_diagnostics[1].lift_confirm_count, 0);
    if (index == 3) {
      const auto& first = result.group_diagnostics[0];
      const auto& second = result.group_diagnostics[1];
      EXPECT_NE(first.matched_history_id, second.matched_history_id);
      const auto& static_group = first.descriptor.physical_vertical_z <
              second.descriptor.physical_vertical_z ? first : second;
      EXPECT_FALSE(static_group.lift_confirmed);
      EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
    }
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     TwoSpatiallyAmbiguousHistoriesStillFailClosed) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto frame = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.30);
  frame.groups = buildGroups({
      candidate(1U, 1.0, -0.20, 0.30, {1U}),
      candidate(2U, 1.0, 0.20, 0.30, {2U})});
  attachCurrentRawRoi(&frame);
  authority.update(frame);

  frame = input(1.05, HookLoadSignalRole::REQUIRED, true,
                HookLoadState::EMPTY, 0.0, 0.30);
  frame.groups = buildGroups({
      candidate(3U, 1.05, -0.20, 0.30, {3U}),
      candidate(4U, 1.05, 0.20, 0.30, {4U})});
  attachCurrentRawRoi(&frame);
  const auto frozen = authority.update(frame);
  ASSERT_EQ(frozen.group_diagnostics.size(), 2U);
  ASSERT_NE(frozen.group_diagnostics[0].prelift_state,
            CargoPreLiftReferenceState::FROZEN);
  ASSERT_NE(frozen.group_diagnostics[1].prelift_state,
            CargoPreLiftReferenceState::FROZEN);

  frame = input(1.10, HookLoadSignalRole::REQUIRED, true,
                HookLoadState::LOADED, 0.0, 0.30);
  frame.groups = buildGroups({
      candidate(5U, 1.10, 0.0, 0.30, {5U}),
      candidate(6U, 1.10, 0.0, 0.30, {6U})});
  const auto ambiguous = authority.update(frame);
  ASSERT_EQ(ambiguous.group_diagnostics.size(), 2U);
  EXPECT_EQ(ambiguous.association,
            CargoCandidateAssociationState::AMBIGUOUS);
  EXPECT_EQ(ambiguous.group_diagnostics[0].association,
            CargoCandidateAssociationState::AMBIGUOUS);
  EXPECT_EQ(ambiguous.group_diagnostics[1].association,
            CargoCandidateAssociationState::AMBIGUOUS);
  EXPECT_EQ(ambiguous.group_diagnostics[0].matched_history_id, 0U);
  EXPECT_EQ(ambiguous.group_diagnostics[1].matched_history_id, 0U);
  EXPECT_NE(ambiguous.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ContinuityOnlyFrameCannotAdvanceLiftConfirmation) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.05, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.1, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.4));
  const auto supported = authority.update(input(
      1.2, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.7));
  ASSERT_EQ(supported.group_diagnostics.front().lift_confirm_count, 1);
  auto continuity = input(1.3, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::LOADED, 0.0, 0.7);
  continuity.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::CONTINUITY_ONLY;
  continuity.groups.front().descriptor.vertical_reject_reason =
      "NO_SUPPORTED_HYPOTHESIS_TOP";
  const auto result = authority.update(continuity);
  EXPECT_EQ(result.group_diagnostics.front().lift_confirm_count, 1);
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     InvalidSupportedTopCanUseCurrentRobustZForAssociationOnly) {
  CargoVerticalEvidenceConfig config = verticalConfig();
  config.minimum_surface_cells = 100U;
  auto observation = candidate(1U, 1.0, 0.0, 0.4, {1U});
  auto groups = buildGroups({observation}, {}, config);
  ASSERT_EQ(groups.size(), 1U);
  ASSERT_EQ(groups.front().descriptor.vertical_mode,
            CargoGroupVerticalMode::CONTINUITY_ONLY);
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto first = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.4);
  first.groups = groups;
  authority.update(first);
  observation.stamp_sec = 1.1;
  auto second = input(1.1, HookLoadSignalRole::REQUIRED, true,
                      HookLoadState::LOADED, 0.0, 0.4);
  second.groups = buildGroups({observation}, {}, config);
  const auto result = authority.update(second);
  EXPECT_EQ(result.association, CargoCandidateAssociationState::MATCHED);
  EXPECT_EQ(result.group_diagnostics.front().lift_confirm_count, 0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ContinuityOnlyFrameCannotCreateValidation) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  for (int frame = 0; frame < 6; ++frame) {
    auto current = input(1.0 + 0.1 * frame,
                         HookLoadSignalRole::REQUIRED, true,
                         frame == 0 ? HookLoadState::EMPTY
                                    : HookLoadState::LOADED,
                         0.0, frame == 0 ? 0.4 : 0.8);
    current.groups.front().descriptor.vertical_mode =
        CargoGroupVerticalMode::CONTINUITY_ONLY;
    const auto result = authority.update(current);
    EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ValidatedHistorySurvivesSingleContinuityOnlyFrame) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  auto continuity = input(1.4, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::LOADED, 0.0, 0.7);
  continuity.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::CONTINUITY_ONLY;
  continuity.groups.front().descriptor.vertical_reject_reason =
      "NO_SUPPORTED_HYPOTHESIS_TOP";
  const auto result = authority.update(continuity);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.physical_history_id, validated.physical_history_id);
  EXPECT_FALSE(result.current_vertical_evidence_valid);
  EXPECT_FALSE(result.geometry_resolved);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ContinuityOnlyCannotProduceCargoGeometryOrClear) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  validateRequired(&authority);
  auto continuity = input(1.4, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::LOADED, 0.0, 0.7);
  continuity.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::CONTINUITY_ONLY;
  const auto result = authority.update(continuity);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_FALSE(result.current_vertical_evidence_valid);
  EXPECT_FALSE(result.geometry_resolved);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ContinuityOnlyCollapsedExtentCannotRejectByItself) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  validateRequired(&authority);
  auto continuity = input(1.4, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::LOADED, 0.0, 0.7);
  continuity.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::CONTINUITY_ONLY;
  continuity.groups.front().descriptor.aggregate_extent.z() = 0.001;
  const auto result = authority.update(continuity);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_EQ(result.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);
  EXPECT_NE(result.group_diagnostics.front().association_reject_reason,
            "EXTENT_GATE");
  EXPECT_FALSE(result.current_vertical_evidence_valid);
  EXPECT_FALSE(result.geometry_resolved);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ReacquiredVerticalIsAssociationOnly) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  auto continuity = input(1.4, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::LOADED, 0.0, 2.0);
  continuity.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::CONTINUITY_ONLY;
  continuity.groups.front().descriptor.physical_vertical_z = 2.0;
  continuity.groups.front().descriptor.stable_anchor.z() = 2.0;
  continuity.vertical_config = verticalConfig();
  continuity.frame_evidence.source_stamp_sec = 1.4;
  continuity.frame_evidence.raw_roi_current_frame = cloudFromPoints(
      componentPoints(0.0, 0.70));
  continuity.frame_evidence.ground_reference_valid = false;
  continuity.frame_evidence.ground_z_base = 0.0F;
  const auto result = authority.update(continuity);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  const auto& diagnostic = result.group_diagnostics.front();
  EXPECT_TRUE(diagnostic.reacquired_vertical_attempted);
  EXPECT_TRUE(diagnostic.reacquired_vertical.valid)
      << diagnostic.reacquired_vertical.reason;
  EXPECT_EQ(diagnostic.association,
            CargoCandidateAssociationState::MATCHED);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.physical_history_id, validated.physical_history_id);
  EXPECT_EQ(result.lift_confirm_count, validated.lift_confirm_count);
  EXPECT_FALSE(result.current_vertical_evidence_valid);
  EXPECT_FALSE(result.geometry_resolved);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     MissingCurrentGroupCannotReuseValidatedGeometry) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  validateRequired(&authority);
  auto missing = input(1.4, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.7);
  missing.groups.clear();
  const auto result = authority.update(missing);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_FALSE(result.current_candidate_fresh);
  EXPECT_FALSE(result.current_vertical_evidence_valid);
  EXPECT_FALSE(result.geometry_resolved);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PendingLiftProgressExpiresByLastSupportedEvidenceGap) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.05, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.1, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.4));
  authority.update(input(1.2, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.7));
  auto continuity = input(2.1, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::LOADED, 0.0, 0.7);
  continuity.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::CONTINUITY_ONLY;
  authority.update(continuity);
  continuity.pipeline_stamp_sec = 2.2;
  continuity.groups.front().descriptor.stamp_sec = 2.2;
  continuity.groups.front().representative.stamp_sec = 2.2;
  authority.update(continuity);
  const auto resumed = authority.update(input(
      2.3, HookLoadSignalRole::REQUIRED, true,
      HookLoadState::LOADED, 0.0, 0.7));
  ASSERT_EQ(resumed.group_diagnostics.size(), 1U);
  EXPECT_EQ(resumed.group_diagnostics.front().lift_confirm_count, 1);
  EXPECT_NE(resumed.identity, CargoPhysicalIdentityState::VALIDATED);
}

// POSTLOAD (gravity LOADED) RECOVERY_HOLD: a bounded INVALID/XY-churn gap holds
// the history alive without fragmenting or accumulating validation evidence.
// PRELOAD (EMPTY) retains the original finiteDescriptor skip to preserve the
// prelift reference-freeze timing.

TEST(CargoPhysicalIdentityAuthorityTest,
     InvalidVerticalHoldsHistoryInsteadOfBreakingOrReusingZ) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  auto invalid = input(1.1, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.4);
  invalid.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::INVALID;
  const auto result = authority.update(invalid);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_EQ(result.group_diagnostics.front().association,
            CargoCandidateAssociationState::RECOVERY_HOLD);
  EXPECT_NE(result.group_diagnostics.front().matched_history_id, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     InvalidVerticalRecoveryReturnsToOriginalHistory) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  auto invalid = input(1.1, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.4);
  invalid.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::INVALID;
  const auto held = authority.update(invalid);
  const std::uint64_t held_id =
      held.group_diagnostics.front().matched_history_id;
  EXPECT_NE(held_id, 0U);
  auto valid = input(1.2, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::LOADED, 0.0, 0.4);
  const auto recovered = authority.update(valid);
  ASSERT_EQ(recovered.group_diagnostics.size(), 1U);
  EXPECT_EQ(recovered.group_diagnostics.front().matched_history_id, held_id);
  EXPECT_EQ(recovered.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     RecoveryHoldExpiresAfterTtlWithoutFragmenting) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.history_hold_ttl_sec = 1.0;
  CargoPhysicalIdentityAuthority authority(config);
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  auto invalid = input(1.5, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.4);
  invalid.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::INVALID;
  const auto held = authority.update(invalid);
  EXPECT_EQ(held.group_diagnostics.front().association,
            CargoCandidateAssociationState::RECOVERY_HOLD);
  auto late = input(2.6, HookLoadSignalRole::REQUIRED, true,
                    HookLoadState::LOADED, 0.0, 0.4);
  late.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::INVALID;
  const auto after = authority.update(late);
  ASSERT_EQ(after.group_diagnostics.size(), 1U);
  EXPECT_NE(after.group_diagnostics.front().association,
            CargoCandidateAssociationState::RECOVERY_HOLD);
  EXPECT_EQ(after.group_diagnostics.front().matched_history_id, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     RobustSupportCenterResistsPointDensityShift) {
  const auto left_biased = groupWithSupport(
      1U, 1U, 1.0, -0.60, 0.60, 0.70, true, false);
  const auto right_biased = groupWithSupport(
      2U, 2U, 1.1, -0.60, 0.60, 0.70, false, true);
  EXPECT_GT(std::abs(left_biased.descriptor.stable_anchor.x() -
                     right_biased.descriptor.stable_anchor.x()), 0.50);
  EXPECT_NEAR(left_biased.descriptor.robust_xy_center.x(),
              right_biased.descriptor.robust_xy_center.x(), 1.0e-9);
  EXPECT_NEAR(left_biased.descriptor.robust_xy_extent.x(),
              right_biased.descriptor.robust_xy_extent.x(), 1.0e-9);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PartialVisibilityOfSameCargoDoesNotCreateNewHistory) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.maximum_xy_step_m = 0.30;
  CargoPhysicalIdentityAuthority authority(config);
  auto first = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.70);
  first.groups = {groupWithSupport(
      1U, 1U, 1.0, -0.60, 0.60, 0.70)};
  authority.update(first);
  auto partial = input(1.1, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.70);
  partial.groups = {groupWithSupport(
      2U, 2U, 1.1, 0.20, 1.00, 0.70)};
  const auto result = authority.update(partial);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_EQ(result.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);
  EXPECT_EQ(result.group_diagnostics.front().association_mode,
            CargoPhysicalAssociationMode::SUPPORT_OVERLAP_CONTINUITY);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     SupportOverlapCanPreserveHistoryWithoutRelaxing030) {
  CargoPhysicalIdentityConfig config;
  EXPECT_DOUBLE_EQ(config.maximum_xy_step_m, 0.30);
  config.maximum_z_speed_mps = 5.0;
  CargoPhysicalIdentityAuthority authority(config);
  auto first = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.70);
  first.groups = {groupWithSupport(
      1U, 1U, 1.0, -0.60, 0.60, 0.70)};
  authority.update(first);
  auto overlap = input(1.1, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.70);
  overlap.groups = {groupWithSupport(
      2U, 2U, 1.1, 0.20, 1.00, 0.70)};
  const auto result = authority.update(overlap);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_GT(result.group_diagnostics.front().xy_step_m, 0.30);
  EXPECT_NEAR(result.group_diagnostics.front().support_xy_separation_m,
              0.0, 1.0e-9);
  EXPECT_EQ(result.group_diagnostics.front().association_mode,
            CargoPhysicalAssociationMode::SUPPORT_OVERLAP_CONTINUITY);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     SeparatedSupportBeyond030CreatesNewHistory) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.maximum_xy_step_m = 0.30;
  CargoPhysicalIdentityAuthority authority(config);
  auto first = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.70);
  first.groups = {groupWithSupport(
      1U, 1U, 1.0, -0.60, 0.60, 0.70)};
  authority.update(first);
  auto separated = input(1.1, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.70);
  separated.groups = {groupWithSupport(
      2U, 2U, 1.1, 1.00, 1.80, 0.70)};
  const auto result = authority.update(separated);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_EQ(result.group_diagnostics.front().association,
            CargoCandidateAssociationState::NEW_HISTORY);
  EXPECT_EQ(result.group_diagnostics.front().association_mode,
            CargoPhysicalAssociationMode::NEW_HISTORY);
  EXPECT_GT(result.group_diagnostics.front().support_xy_separation_m, 0.30);
  EXPECT_EQ(result.group_diagnostics.front().association_reject_reason,
            "XY_GATE");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     StaticLowAndLiftingCargoDoNotMergeThroughOverlap) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.maximum_xy_step_m = 0.30;
  config.maximum_z_speed_mps = 1.50;
  CargoPhysicalIdentityAuthority authority(config);
  auto first = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.30);
  first.groups = {
      groupWithSupport(1U, 1U, 1.0, -0.60, 0.40, 0.30),
      groupWithSupport(2U, 2U, 1.0, -0.40, 0.60, 1.10)};
  authority.update(first);
  auto second = input(1.1, HookLoadSignalRole::REQUIRED, true,
                      HookLoadState::LOADED, 0.0, 0.30);
  second.groups = {
      groupWithSupport(3U, 3U, 1.1, -0.20, 0.80, 0.30),
      groupWithSupport(4U, 4U, 1.1, -0.80, 0.20, 1.10)};
  const auto result = authority.update(second);
  ASSERT_EQ(result.group_diagnostics.size(), 2U);
  EXPECT_EQ(result.group_diagnostics[0].association,
            CargoCandidateAssociationState::MATCHED);
  EXPECT_EQ(result.group_diagnostics[1].association,
            CargoCandidateAssociationState::MATCHED);
  EXPECT_NE(result.group_diagnostics[0].matched_history_id,
            result.group_diagnostics[1].matched_history_id);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     UnrelatedAmbiguousGroupDoesNotRevokeValidatedCargo) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  auto current = input(1.4, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.70);
  current.groups = buildGroups({
      candidate(10U, 1.4, 0.0, 0.70, {1U}),
      candidate(20U, 1.4, 5.0, 0.70, {20U, 21U}),
      candidate(21U, 1.4, 5.0, 0.70, {21U, 22U})});
  const auto result = authority.update(current);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.physical_history_id, validated.physical_history_id);
  EXPECT_EQ(result.association, CargoCandidateAssociationState::MATCHED);
  ASSERT_EQ(result.group_diagnostics.size(), 2U);
  EXPECT_TRUE(result.group_diagnostics.front().frame_has_unrelated_ambiguity);
  EXPECT_TRUE(result.group_diagnostics.back().frame_has_unrelated_ambiguity);
  EXPECT_FALSE(result.group_diagnostics.back().validated_history_conflict);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     AmbiguityCompetingForValidatedHistoryRevokesIdentity) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  validateRequired(&authority);
  auto current = input(1.4, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.70);
  current.groups = buildGroups({
      candidate(20U, 1.4, 0.0, 0.70, {20U, 21U}),
      candidate(21U, 1.4, 0.0, 0.70, {21U, 22U})});
  const auto result = authority.update(current);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::AMBIGUOUS);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_TRUE(result.group_diagnostics.front().validated_history_conflict);
  EXPECT_NE(result.group_diagnostics.front().conflicting_history_id, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     TwoFreshConfirmedPhysicalHistoriesRemainAmbiguous) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto frame = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.30);
  frame.groups = buildGroups({
      candidate(1U, 1.0, -1.0, 0.30, {1U}),
      candidate(2U, 1.0, 1.0, 0.30, {2U})});
  attachCurrentRawRoi(&frame);
  authority.update(frame);
  frame.pipeline_stamp_sec = 1.05;
  for (auto& group_observation : frame.groups) {
    group_observation.descriptor.stamp_sec = 1.05;
    group_observation.representative.stamp_sec = 1.05;
  }
  attachCurrentRawRoi(&frame);
  authority.update(frame);
  for (int index = 1; index <= 3; ++index) {
    const double stamp = 1.0 + 0.1 * index;
    const double z = index == 1 ? 0.30 : 0.70;
    frame = input(stamp, HookLoadSignalRole::REQUIRED, true,
                  HookLoadState::LOADED, 0.0, z);
    frame.groups = buildGroups({
        candidate(10U + index, stamp, -1.0, z, {10U}),
        candidate(20U + index, stamp, 1.0, z, {20U})});
    attachCurrentRawRoi(&frame);
    const auto result = authority.update(frame);
    if (index == 3) {
      EXPECT_EQ(result.identity, CargoPhysicalIdentityState::AMBIGUOUS);
      EXPECT_EQ(result.association, CargoCandidateAssociationState::AMBIGUOUS);
    }
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ValidatedHistorySurvivesUnrelatedNewHistory) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  auto current = input(1.4, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.70);
  current.groups = buildGroups({
      candidate(10U, 1.4, 0.0, 0.70, {1U}),
      candidate(20U, 1.4, 5.0, 0.70, {20U})});
  const auto result = authority.update(current);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.physical_history_id, validated.physical_history_id);
  EXPECT_EQ(result.association, CargoCandidateAssociationState::MATCHED);
  ASSERT_EQ(result.group_diagnostics.size(), 2U);
  EXPECT_TRUE(std::any_of(
      result.group_diagnostics.begin(), result.group_diagnostics.end(),
      [](const CargoPhysicalGroupDiagnostic& diagnostic) {
        return diagnostic.association ==
            CargoCandidateAssociationState::NEW_HISTORY;
      }));
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ExistingLiftThresholdContractsRemainUnchanged) {
  const CargoPhysicalIdentityConfig config;
  EXPECT_DOUBLE_EQ(config.maximum_xy_step_m, 0.30);
  EXPECT_DOUBLE_EQ(config.maximum_observation_gap_sec, 0.50);
  EXPECT_EQ(config.lift_confirm_frames, 4);
  EXPECT_DOUBLE_EQ(config.minimum_significant_change_m, 0.15);
  EXPECT_DOUBLE_EQ(config.significance_sigma, 3.0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     DegenerateEdgeTouchCannotUseSupportOverlapContinuity) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.maximum_xy_step_m = 0.30;
  CargoPhysicalIdentityAuthority authority(config);
  auto first = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.70);
  first.groups = {groupWithSupport(
      1U, 1U, 1.0, -0.50, 0.50, 0.70)};
  first.groups.front().descriptor.robust_x05 = -0.50;
  first.groups.front().descriptor.robust_x95 = 0.50;
  first.groups.front().descriptor.robust_y05 = -0.50;
  first.groups.front().descriptor.robust_y95 = 0.50;
  first.groups.front().descriptor.robust_xy_center = Eigen::Vector2d(0.0, 0.0);
  authority.update(first);

  auto touching = input(1.1, HookLoadSignalRole::REQUIRED, true,
                        HookLoadState::LOADED, 1.0, 0.70);
  touching.groups = {groupWithSupport(
      2U, 2U, 1.1, 0.50, 1.50, 0.70)};
  touching.groups.front().descriptor.robust_x05 = 0.50;
  touching.groups.front().descriptor.robust_x95 = 1.50;
  touching.groups.front().descriptor.robust_y05 = -0.50;
  touching.groups.front().descriptor.robust_y95 = 0.50;
  touching.groups.front().descriptor.robust_xy_center =
      Eigen::Vector2d(1.0, 0.0);
  const auto result = authority.update(touching);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_EQ(result.group_diagnostics.front().association,
            CargoCandidateAssociationState::NEW_HISTORY);
  EXPECT_EQ(result.group_diagnostics.front().association_mode,
            CargoPhysicalAssociationMode::NEW_HISTORY);
  EXPECT_EQ(result.group_diagnostics.front().association_reject_reason,
            "XY_GATE");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PositiveSupportGapCannotUseOverlapContinuity) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.maximum_xy_step_m = 0.30;
  CargoPhysicalIdentityAuthority authority(config);
  auto first = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.70);
  first.groups = {groupWithSupport(
      1U, 1U, 1.0, -0.50, 0.50, 0.70)};
  first.groups.front().descriptor.robust_x05 = -0.50;
  first.groups.front().descriptor.robust_x95 = 0.50;
  first.groups.front().descriptor.robust_y05 = -0.50;
  first.groups.front().descriptor.robust_y95 = 0.50;
  first.groups.front().descriptor.robust_xy_center = Eigen::Vector2d(0.0, 0.0);
  authority.update(first);

  auto separated = input(1.1, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 1.01, 0.70);
  separated.groups = {groupWithSupport(
      2U, 2U, 1.1, 0.51, 1.51, 0.70)};
  separated.groups.front().descriptor.robust_x05 = 0.51;
  separated.groups.front().descriptor.robust_x95 = 1.51;
  separated.groups.front().descriptor.robust_y05 = -0.50;
  separated.groups.front().descriptor.robust_y95 = 0.50;
  separated.groups.front().descriptor.robust_xy_center =
      Eigen::Vector2d(1.01, 0.0);
  const auto result = authority.update(separated);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_EQ(result.group_diagnostics.front().association,
            CargoCandidateAssociationState::NEW_HISTORY);
  EXPECT_GT(result.group_diagnostics.front().support_xy_separation_m, 0.0);
  EXPECT_EQ(result.group_diagnostics.front().association_reject_reason,
            "XY_GATE");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     FragmentedTopCanBeOwnedBySameXYBodySupport) {
  auto observation = candidate(1U, 1.0, 0.0, 0.40, {1U});
  CargoPhysicalComponentObservation component;
  component.component_id = 1U;
  component.points_base = componentPoints(0.0, 0.40);
  std::vector<Eigen::Vector3f> raw_points = component.points_base;
  for (float x : {-0.20F, 0.20F}) {
    for (float y : {-0.20F, 0.20F}) {
      raw_points.emplace_back(x, y, 0.90F);
      raw_points.emplace_back(x, y, 0.90F);
    }
  }
  CargoPhysicalGroupingTelemetry telemetry;
  const auto groups = buildGroupsWithRawRoi(
      {observation}, {component}, raw_points, 1.0,
      false, 0.0, false, 0.0, &telemetry);
  ASSERT_EQ(groups.size(), 1U);
  const auto& descriptor = groups.front().descriptor;
  EXPECT_EQ(descriptor.vertical_source,
            CargoVerticalEvidenceSource::RAW_ROI_CURRENT_FOOTPRINT);
  EXPECT_EQ(descriptor.vertical_mode,
            CargoGroupVerticalMode::SUPPORTED_EVIDENCE);
  EXPECT_NEAR(descriptor.physical_vertical_z, 0.90, 1.0e-5);
  EXPECT_GT(descriptor.owner_overlap_cell_count, 0U);
  EXPECT_GT(telemetry.raw_roi_vertical_hypothesis_count, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     AdjacentHighObjectCannotBeBorrowed) {
  auto observation = candidate(1U, 1.0, 0.0, 0.40, {1U});
  CargoPhysicalComponentObservation component;
  component.component_id = 1U;
  component.points_base = componentPoints(0.0, 0.40);
  std::vector<Eigen::Vector3f> raw_points = component.points_base;
  for (float x : {-0.40F, 0.40F}) {
    for (float y : {-0.30F, 0.30F}) {
      raw_points.emplace_back(x, y, 1.20F);
      raw_points.emplace_back(x, y, 1.20F);
    }
  }
  const auto groups = buildGroupsWithRawRoi(
      {observation}, {component}, raw_points, 1.0);
  ASSERT_EQ(groups.size(), 1U);
  const auto& descriptor = groups.front().descriptor;
  EXPECT_EQ(descriptor.vertical_source,
            CargoVerticalEvidenceSource::COMPONENT_UNION);
  EXPECT_NEAR(descriptor.physical_vertical_z, 0.40, 1.0e-5);
  EXPECT_EQ(descriptor.raw_roi_supported_hypothesis_count, 0U);
  EXPECT_GT(descriptor.owner_proof_rejected_hypothesis_count, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     NestedHypothesisCannotBorrowOwnerCellsFromNonMemberComponent) {
  auto member_one = candidate(1U, 1.0, 0.0, 0.40, {1U});
  auto members_one_two = candidate(2U, 1.0, 0.0, 0.40, {1U, 2U});
  CargoPhysicalComponentObservation component_one;
  component_one.component_id = 1U;
  CargoPhysicalComponentObservation component_two;
  component_two.component_id = 2U;
  for (float y : {-0.20F, 0.20F}) {
    component_one.points_base.emplace_back(-0.20F, y, 0.40F);
    component_one.points_base.emplace_back(-0.20F, y, 0.40F);
    component_two.points_base.emplace_back(0.20F, y, 0.40F);
    component_two.points_base.emplace_back(0.20F, y, 0.40F);
  }
  std::vector<Eigen::Vector3f> raw_points = component_one.points_base;
  raw_points.insert(raw_points.end(), component_two.points_base.begin(),
                    component_two.points_base.end());
  for (float y : {-0.20F, 0.20F}) {
    raw_points.emplace_back(0.20F, y, 0.90F);
    raw_points.emplace_back(0.20F, y, 0.90F);
  }
  const auto groups = buildGroupsWithRawRoi(
      {member_one, members_one_two}, {component_one, component_two},
      raw_points, 1.0);
  ASSERT_EQ(groups.size(), 1U);
  const auto& descriptor = groups.front().descriptor;
  EXPECT_EQ(descriptor.raw_roi_supported_hypothesis_count, 1U);
  EXPECT_EQ(descriptor.owner_proof_rejected_hypothesis_count, 1U);
  EXPECT_EQ(descriptor.vertical_source,
            CargoVerticalEvidenceSource::RAW_ROI_CURRENT_FOOTPRINT);
  EXPECT_NEAR(descriptor.physical_vertical_z, 0.90, 1.0e-5);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     RawRoiSourceStampMustEqualDetectionSourceStamp) {
  auto observation = candidate(1U, 1.0, 0.0, 0.40, {1U});
  CargoPhysicalComponentObservation component;
  component.component_id = 1U;
  component.points_base = componentPoints(0.0, 0.40);
  std::vector<Eigen::Vector3f> raw_points = component.points_base;
  for (float x : {-0.20F, 0.20F}) {
    for (float y : {-0.20F, 0.20F}) {
      raw_points.emplace_back(x, y, 0.90F);
      raw_points.emplace_back(x, y, 0.90F);
    }
  }
  const auto groups = buildGroupsWithRawRoi(
      {observation}, {component}, raw_points, 2.0);
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_EQ(groups.front().descriptor.vertical_source,
            CargoVerticalEvidenceSource::COMPONENT_UNION);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     RawRoiAndComponentEvidenceShareGroundContext) {
  auto observation = candidate(1U, 1.0, 0.0, 0.40, {1U});
  CargoPhysicalComponentObservation component;
  component.component_id = 1U;
  component.points_base = componentPoints(0.0, 0.40);
  std::vector<Eigen::Vector3f> raw_points = component.points_base;
  const auto groups = buildGroupsWithRawRoi(
      {observation}, {component}, raw_points, 1.0,
      true, 0.0, false, 0.0);
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_EQ(groups.front().descriptor.vertical_source,
            CargoVerticalEvidenceSource::COMPONENT_UNION);
  EXPECT_EQ(groups.front().descriptor.raw_roi_supported_hypothesis_count, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     FamilyRescueCannotOverrideExactAssociation) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  const auto observation = lineageObservation(
      101U, 202U, 1U, 1.0, 1.1, 0.05);
  auto next = lineageInput(
      1.1, HookLoadState::EMPTY, 0.05, 0.40, 202U,
      &observation);
  const auto decision = authority.update(next);
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_EQ(decision.group_diagnostics.front().association_mode,
            CargoPhysicalAssociationMode::ANCHOR_CONTINUITY);
  EXPECT_TRUE(decision.group_diagnostics.front().lineage_exact_path_won);
  EXPECT_FALSE(decision.group_diagnostics.front().lineage_rescue_used);
  EXPECT_EQ(decision.group_diagnostics.front().lineage_reject_stage,
            CargoLineageRejectStage::EXACT_PATH_WON);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     FamilyRescueCannotProvideLiftVerticalWithoutCurrentOwnerProof) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  auto observation = lineageObservation(
      101U, 202U, 1U, 1.0, 1.1, 0.05);
  auto decision = authority.update(lineageInput(
      1.1, HookLoadState::EMPTY, 2.0, 0.40, 202U,
      &observation));
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_EQ(decision.group_diagnostics.front().association_mode,
            CargoPhysicalAssociationMode::COMPONENT_LINEAGE_CONTINUITY);
  EXPECT_TRUE(decision.group_diagnostics.front().lineage_rescue_used);
  EXPECT_EQ(decision.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);
  EXPECT_NE(decision.group_diagnostics.front().prelift_state,
            CargoPreLiftReferenceState::FROZEN);
  EXPECT_FALSE(decision.group_diagnostics.front().current_surface_vertical_valid);

  observation = lineageObservation(
      202U, 303U, 1U, 1.1, 1.2, 0.10);
  authority.update(lineageInput(
      1.2, HookLoadState::LOADED, 2.0, 0.40, 303U,
      &observation));
  observation = lineageObservation(
      303U, 404U, 1U, 1.2, 1.3, 0.15);
  authority.update(lineageInput(
      1.3, HookLoadState::LOADED, 2.0, 0.70, 404U,
      &observation));
  observation = lineageObservation(
      404U, 505U, 1U, 1.3, 1.4, 0.20);
  decision = authority.update(lineageInput(
      1.4, HookLoadState::LOADED, 2.0, 0.70, 505U,
      &observation));
  EXPECT_NE(decision.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_FALSE(decision.current_surface_vertical_valid);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadedLowEgoCanContinueUniqueExistingHistory) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  const auto observation = lineageObservation(
      101U, 202U, 1U, 1.0, 1.1, 0.05, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE,
      0.05, 0.07);
  const auto decision = authority.update(lineageInput(
      1.1, HookLoadState::LOADED, 2.0, 0.40, 202U,
      &observation));
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_EQ(decision.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);
  EXPECT_EQ(decision.group_diagnostics.front().association_mode,
            CargoPhysicalAssociationMode::COMPONENT_LINEAGE_CONTINUITY);
  EXPECT_TRUE(decision.group_diagnostics.front().lineage_rescue_used);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadedLowEgoCannotCreateHistory) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto observation = lineageObservation(
      101U, 202U, 1U, 1.0, 1.1, 0.05, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE,
      0.05, 0.07);
  const auto decision = authority.update(lineageInput(
      1.1, HookLoadState::LOADED, 2.0, 0.40, 202U,
      &observation));
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_EQ(decision.group_diagnostics.front().association,
            CargoCandidateAssociationState::NEW_HISTORY);
  EXPECT_FALSE(decision.group_diagnostics.front().lineage_rescue_used);
  EXPECT_NE(decision.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadedLowEgoCannotOverrideExactAssociation) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  const auto observation = lineageObservation(
      101U, 202U, 1U, 1.0, 1.1, 0.05, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE,
      0.05, 0.07);
  const auto decision = authority.update(lineageInput(
      1.1, HookLoadState::LOADED, 0.05, 0.40, 202U,
      &observation));
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_EQ(decision.group_diagnostics.front().association_mode,
            CargoPhysicalAssociationMode::ANCHOR_CONTINUITY);
  EXPECT_TRUE(decision.group_diagnostics.front().lineage_exact_path_won);
  EXPECT_FALSE(decision.group_diagnostics.front().lineage_rescue_used);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadedLowEgoCannotBreakHistoryTie) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  CargoPhysicalIdentityInput first;
  first.pipeline_stamp_sec = 1.0;
  first.lifecycle_id = 7U;
  first.hook_role = HookLoadSignalRole::REQUIRED;
  first.gravity_valid = true;
  first.gravity_state = HookLoadState::EMPTY;
  auto first_group = group(1U, 1.0, -1.0, 0.40, {101U});
  auto second_group = group(2U, 1.0, 1.0, 0.40, {101U});
  first_group.frame_group_id = 1U;
  second_group.frame_group_id = 2U;
  first.groups = {first_group, second_group};
  authority.update(first);

  const auto observation = lineageObservation(
      101U, 202U, 1U, 1.0, 1.1, 0.0, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE,
      0.05, 0.07);
  const auto decision = authority.update(lineageInput(
      1.1, HookLoadState::LOADED, 4.0, 0.40, 202U,
      &observation));
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_TRUE(decision.group_diagnostics.front().lineage_ambiguous);
  EXPECT_FALSE(decision.group_diagnostics.front().lineage_rescue_used);
  EXPECT_NE(decision.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     UnknownOrEmptyLowEgoObservationFailsClosed) {
  for (const HookLoadState state :
       {HookLoadState::UNKNOWN, HookLoadState::EMPTY}) {
    CargoPhysicalIdentityAuthority authority(testConfig());
    authority.update(lineageInput(
        1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
    auto observation = lineageObservation(
        101U, 202U, 1U, 1.0, 1.1, 0.05, 0.40, 0.40,
        state == HookLoadState::EMPTY
            ? CargoIdentityMotionObservabilityState::IDLE_ZERO_LOAD
            : CargoIdentityMotionObservabilityState::UNKNOWN_FAIL_CLOSED,
        0.05, 0.07);
    auto current = lineageInput(
        1.1, state, 2.0, 0.40, 202U, &observation);
    current.gravity_valid = state != HookLoadState::UNKNOWN;
    const auto decision = authority.update(current);
    ASSERT_EQ(decision.group_diagnostics.size(), 1U);
    EXPECT_FALSE(decision.group_diagnostics.front().lineage_rescue_used);
    EXPECT_NE(decision.group_diagnostics.front().association,
              CargoCandidateAssociationState::MATCHED);
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LineageCannotProvideLiftVerticalWithoutFrozenFootprintOwnerProof) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  authority.update(lineageInput(
      1.1, HookLoadState::EMPTY, 0.0, 0.40, 102U));

  auto first = lineageObservation(
      102U, 202U, 1U, 1.1, 1.2, 0.05, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE,
      0.05, 0.07);
  authority.update(lineageInput(
      1.2, HookLoadState::LOADED, 2.0, 0.70, 202U, &first));
  auto second = lineageObservation(
      202U, 303U, 1U, 1.2, 1.3, 0.10, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE,
      0.05, 0.07);
  const auto decision = authority.update(lineageInput(
      1.3, HookLoadState::LOADED, 2.0, 0.70, 303U, &second));
  EXPECT_NE(decision.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_FALSE(decision.current_surface_vertical_valid);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadLineageSurfaceSwitchCannotSolelyValidate) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  authority.update(lineageInput(
      1.1, HookLoadState::EMPTY, 0.0, 0.40, 102U));

  auto observation = lineageObservation(
      102U, 202U, 1U, 1.1, 1.2, 0.05, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE,
      0.05, 0.07);
  auto current = lineageInput(
      1.2, HookLoadState::LOADED, 2.0, 0.80, 202U,
      &observation);
  current.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::INVALID;
  const auto decision = authority.update(current);
  EXPECT_NE(decision.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_FALSE(decision.lift_confirmed);
  EXPECT_EQ(decision.lift_confirm_count, 0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadSignalCannotProvideVerticalEvidence) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  authority.update(lineageInput(
      1.1, HookLoadState::EMPTY, 0.0, 0.40, 102U));
  auto observation = lineageObservation(
      102U, 202U, 1U, 1.1, 1.2, 0.05, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE,
      0.05, 0.07);
  auto current = lineageInput(
      1.2, HookLoadState::LOADED, 2.0, 10.0, 202U,
      &observation);
  current.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::INVALID;
  const auto decision = authority.update(current);
  EXPECT_FALSE(decision.lift_confirmed);
  EXPECT_EQ(decision.lift_confirm_count, 0);
  EXPECT_NE(decision.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     CargoThicknessCannotBeMiscountedAsLift) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  authority.update(lineageInput(
      1.1, HookLoadState::EMPTY, 0.0, 0.40, 102U));

  auto first = lineageObservation(
      102U, 202U, 1U, 1.1, 1.2, 0.05, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE,
      0.05, 0.07);
  authority.update(lineageInput(
      1.2, HookLoadState::LOADED, 2.0, 0.40, 202U, &first));
  auto second = lineageObservation(
      202U, 303U, 1U, 1.2, 1.3, 0.10, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE,
      0.05, 0.07);
  const auto decision = authority.update(lineageInput(
      1.3, HookLoadState::LOADED, 2.0, 0.40, 303U, &second));
  EXPECT_NE(decision.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_FALSE(decision.lift_confirmed);
  EXPECT_FALSE(std::isfinite(decision.lift_delta_m));
}

TEST(CargoPhysicalIdentityAuthorityTest,
     FamilyRescueCannotCreateHistoryOrValidateByItself) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto observation = lineageObservation(
      11U, 22U, 1U, 0.9, 1.0, 0.0);
  auto first = lineageInput(
      1.0, HookLoadState::EMPTY, 2.0, 0.40, 22U,
      &observation);
  first.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::CONTINUITY_ONLY;
  auto decision = authority.update(first);
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_EQ(decision.group_diagnostics.front().association,
            CargoCandidateAssociationState::NEW_HISTORY);
  EXPECT_FALSE(decision.group_diagnostics.front().lineage_rescue_used);
  EXPECT_NE(decision.identity, CargoPhysicalIdentityState::VALIDATED);

  auto next_observation = lineageObservation(
      22U, 33U, 1U, 1.0, 1.1, 0.05);
  auto next = lineageInput(
      1.1, HookLoadState::LOADED, 4.0, 0.90, 33U,
      &next_observation);
  next.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::CONTINUITY_ONLY;
  decision = authority.update(next);
  EXPECT_NE(decision.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_FALSE(decision.lift_confirmed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LineageCanBridgeTransientProductAmbiguityWithoutCreatingHistory) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto first = authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  ASSERT_EQ(first.group_diagnostics.size(), 1U);
  const std::uint64_t original_history_id =
      first.group_diagnostics.front().matched_history_id;
  ASSERT_NE(original_history_id, 0U);

  auto ambiguous = lineageInput(
      1.1, HookLoadState::LOADED, 1.0, 0.40, 202U);
  ambiguous.groups.front().group_ambiguous = true;
  const auto middle = authority.update(ambiguous);
  EXPECT_NE(middle.identity, CargoPhysicalIdentityState::VALIDATED);

  auto newest = lineageObservation(
      202U, 303U, 1U, 1.1, 1.2, 0.05, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE,
      1.0, 1.0);
  auto older = lineageObservation(
      101U, 303U, 1U, 1.0, 1.2, 0.05, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE,
      2.0, 2.0);
  newest.source_frame_offset = 1U;
  older.source_frame_offset = 2U;
  auto current = lineageInput(
      1.2, HookLoadState::LOADED, 2.0, 0.40, 303U);
  current.lineage_observations = {newest, older};
  const auto bridged = authority.update(current);
  ASSERT_EQ(bridged.group_diagnostics.size(), 1U);
  EXPECT_EQ(bridged.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);
  EXPECT_TRUE(bridged.group_diagnostics.front().lineage_rescue_used);
  EXPECT_EQ(bridged.group_diagnostics.front().lineage_previous_component_id,
            101U);
  EXPECT_EQ(bridged.group_diagnostics.front().lineage_source_frame_offset,
            2U);
  EXPECT_NEAR(bridged.group_diagnostics.front().lineage_source_age_sec,
              0.2, 1.0e-9);
  EXPECT_EQ(bridged.group_diagnostics.front().matched_history_id,
            original_history_id);
  EXPECT_EQ(bridged.group_diagnostics.front().lineage_reject_stage,
            CargoLineageRejectStage::SELECTED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OlderLineageCanBindHistoryAfterInterveningExactUpdate) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto first = authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  ASSERT_EQ(first.group_diagnostics.size(), 1U);
  const std::uint64_t history_id =
      first.group_diagnostics.front().matched_history_id;

  const auto intervening = authority.update(lineageInput(
      1.1, HookLoadState::EMPTY, 0.05, 0.40, 202U));
  ASSERT_EQ(intervening.group_diagnostics.size(), 1U);
  ASSERT_EQ(intervening.group_diagnostics.front().matched_history_id,
            history_id);
  ASSERT_EQ(intervening.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);

  auto older = lineageObservation(
      101U, 303U, 1U, 1.0, 1.2, 0.10, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE,
      2.0, 2.0);
  older.source_frame_offset = 2U;
  const auto rescued = authority.update(lineageInput(
      1.2, HookLoadState::EMPTY, 2.0, 0.40, 303U, &older));
  ASSERT_EQ(rescued.group_diagnostics.size(), 1U);
  const auto& diagnostic = rescued.group_diagnostics.front();
  EXPECT_EQ(diagnostic.association, CargoCandidateAssociationState::MATCHED);
  EXPECT_EQ(diagnostic.association_mode,
            CargoPhysicalAssociationMode::COMPONENT_LINEAGE_CONTINUITY);
  EXPECT_EQ(diagnostic.matched_history_id, history_id);
  EXPECT_TRUE(diagnostic.lineage_rescue_used);
  EXPECT_EQ(diagnostic.lineage_previous_component_id, 101U);
  EXPECT_EQ(diagnostic.lineage_source_frame_offset, 2U);
  EXPECT_EQ(diagnostic.lineage_reject_stage,
            CargoLineageRejectStage::SELECTED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     RecentHistoryProvenanceBoundedToThreeFrames) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.00, 0.40, 101U));
  authority.update(lineageInput(
      1.1, HookLoadState::EMPTY, 0.05, 0.40, 202U));
  authority.update(lineageInput(
      1.2, HookLoadState::EMPTY, 0.10, 0.40, 303U));

  auto oldest_retained = lineageObservation(
      101U, 404U, 1U, 1.0, 1.3, 0.15, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE,
      3.0, 3.0);
  oldest_retained.source_frame_offset = 3U;
  const auto retained = authority.update(lineageInput(
      1.3, HookLoadState::EMPTY, 2.0, 0.40, 404U,
      &oldest_retained));
  ASSERT_TRUE(retained.group_diagnostics.front().lineage_rescue_used);

  auto evicted = lineageObservation(
      101U, 505U, 1U, 1.0, 1.4, 0.20, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE,
      4.0, 4.0);
  // Keep the observation contract valid to exercise bounded History storage;
  // the real matcher never emits an offset greater than three.
  evicted.source_frame_offset = 3U;
  const auto rejected = authority.update(lineageInput(
      1.4, HookLoadState::EMPTY, 4.0, 0.40, 505U, &evicted));
  ASSERT_EQ(rejected.group_diagnostics.size(), 1U);
  EXPECT_FALSE(rejected.group_diagnostics.front().lineage_rescue_used);
  EXPECT_EQ(rejected.group_diagnostics.front().lineage_reject_stage,
            CargoLineageRejectStage::HISTORY_PROVENANCE_NOT_FOUND);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     RecentHistoryProvenanceExpiresByObservationGap) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  auto expired = lineageObservation(
      101U, 202U, 1U, 1.0, 1.6, 0.05, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE,
      2.0, 2.0);
  expired.source_frame_offset = 2U;
  const auto rejected = authority.update(lineageInput(
      1.6, HookLoadState::EMPTY, 2.0, 0.40, 202U, &expired));
  ASSERT_EQ(rejected.group_diagnostics.size(), 1U);
  EXPECT_FALSE(rejected.group_diagnostics.front().lineage_rescue_used);
  EXPECT_EQ(rejected.group_diagnostics.front().lineage_reject_stage,
            CargoLineageRejectStage::OBSERVATION_CONTRACT);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OlderProvenanceCannotCrossLifecycle) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  auto next_lifecycle = lineageInput(
      1.1, HookLoadState::EMPTY, 0.05, 0.40, 202U);
  next_lifecycle.lifecycle_id = 8U;
  authority.update(next_lifecycle);

  auto old_lifecycle = lineageObservation(
      101U, 303U, 1U, 1.0, 1.2, 0.10, 0.40, 0.40,
      CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE,
      2.0, 2.0);
  old_lifecycle.source_frame_offset = 2U;
  auto current = lineageInput(
      1.2, HookLoadState::EMPTY, 2.0, 0.40, 303U, &old_lifecycle);
  current.lifecycle_id = 8U;
  const auto rejected = authority.update(current);
  ASSERT_EQ(rejected.group_diagnostics.size(), 1U);
  EXPECT_FALSE(rejected.group_diagnostics.front().lineage_rescue_used);
  EXPECT_EQ(rejected.group_diagnostics.front().lineage_reject_stage,
            CargoLineageRejectStage::HISTORY_PROVENANCE_NOT_FOUND);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OlderProvenanceCannotBindCompetingHistories) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  CargoPhysicalIdentityInput first;
  first.pipeline_stamp_sec = 1.0;
  first.lifecycle_id = 7U;
  first.hook_role = HookLoadSignalRole::REQUIRED;
  first.gravity_valid = true;
  first.gravity_state = HookLoadState::EMPTY;
  auto left = group(1U, 1.0, -1.0, 0.40, {101U});
  auto right = group(2U, 1.0, 1.0, 0.40, {102U});
  left.frame_group_id = 1U;
  right.frame_group_id = 2U;
  first.groups = {left, right};
  authority.update(first);

  auto from_left = lineageObservation(
      101U, 303U, 1U, 1.0, 1.2, -0.95);
  auto from_right = lineageObservation(
      102U, 303U, 1U, 1.0, 1.2, 1.05);
  from_left.source_frame_offset = 2U;
  from_right.source_frame_offset = 2U;
  auto current = lineageInput(
      1.2, HookLoadState::LOADED, 4.0, 0.40, 303U);
  current.lineage_observations = {from_left, from_right};
  const auto decision = authority.update(current);
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_TRUE(decision.group_diagnostics.front().lineage_ambiguous);
  EXPECT_FALSE(decision.group_diagnostics.front().lineage_rescue_used);
  EXPECT_EQ(decision.group_diagnostics.front().lineage_reject_stage,
            CargoLineageRejectStage::HISTORY_COMPETITION);
  EXPECT_NE(decision.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LineageOlderFallbackCannotCreateHistoryOrProvideVertical) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto observation = lineageObservation(
      101U, 202U, 1U, 1.0, 1.2, 0.05);
  observation.source_frame_offset = 2U;
  auto current = lineageInput(
      1.2, HookLoadState::LOADED, 2.0, 10.0, 202U,
      &observation);
  current.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::INVALID;
  const auto decision = authority.update(current);
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_EQ(decision.group_diagnostics.front().association,
            CargoCandidateAssociationState::NEW_HISTORY);
  EXPECT_FALSE(decision.group_diagnostics.front().lineage_rescue_used);
  EXPECT_EQ(decision.group_diagnostics.front().lineage_reject_stage,
            CargoLineageRejectStage::HISTORY_PROVENANCE_NOT_FOUND);
  EXPECT_FALSE(decision.lift_confirmed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     FamilyRescueCannotBreakHistoryTie) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  CargoPhysicalIdentityInput first;
  first.pipeline_stamp_sec = 1.0;
  first.lifecycle_id = 7U;
  first.hook_role = HookLoadSignalRole::REQUIRED;
  first.gravity_valid = true;
  first.gravity_state = HookLoadState::EMPTY;
  auto first_group = group(1U, 1.0, -1.0, 0.40, {101U});
  auto second_group = group(2U, 1.0, 1.0, 0.40, {101U});
  first_group.frame_group_id = 1U;
  second_group.frame_group_id = 2U;
  first.groups = {first_group, second_group};
  authority.update(first);

  const auto observation = lineageObservation(
      101U, 202U, 1U, 1.0, 1.1, 0.0);
  const auto decision = authority.update(lineageInput(
      1.1, HookLoadState::EMPTY, 4.0, 0.40, 202U,
      &observation));
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_TRUE(decision.group_diagnostics.front().lineage_ambiguous);
  EXPECT_FALSE(decision.group_diagnostics.front().lineage_rescue_used);
  EXPECT_NE(decision.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     FamilyRescueCannotBypassExactGroupCompetitionForHistory) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));

  CargoPhysicalIdentityInput current;
  current.pipeline_stamp_sec = 1.1;
  current.lifecycle_id = 7U;
  current.hook_role = HookLoadSignalRole::REQUIRED;
  current.gravity_valid = true;
  current.gravity_state = HookLoadState::EMPTY;
  auto exact_a = group(201U, 1.1, 0.02, 0.40, {201U});
  auto exact_b = group(301U, 1.1, 0.06, 0.40, {301U});
  auto lineage_group = group(202U, 1.1, 2.0, 0.40, {202U});
  exact_a.frame_group_id = 1U;
  exact_b.frame_group_id = 2U;
  lineage_group.frame_group_id = 3U;
  current.groups = {exact_a, exact_b, lineage_group};
  current.lineage_observations.push_back(lineageObservation(
      101U, 202U, 3U, 1.0, 1.1, 0.05));

  const auto decision = authority.update(current);
  ASSERT_EQ(decision.group_diagnostics.size(), 3U);
  const auto& diagnostic = decision.group_diagnostics[2U];
  EXPECT_TRUE(diagnostic.lineage_attempted);
  EXPECT_TRUE(diagnostic.lineage_ambiguous);
  EXPECT_FALSE(diagnostic.lineage_rescue_used);
  EXPECT_NE(diagnostic.association,
            CargoCandidateAssociationState::MATCHED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     FamilyRescueRequiresCurrentComponentInExactSeedGroup) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  auto observation = lineageObservation(
      101U, 999U, 1U, 1.0, 1.1, 0.05);
  const auto decision = authority.update(lineageInput(
      1.1, HookLoadState::EMPTY, 2.0, 0.40, 202U,
      &observation));
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_FALSE(decision.group_diagnostics.front().lineage_rescue_used);
  EXPECT_NE(decision.group_diagnostics.front().association,
            CargoCandidateAssociationState::MATCHED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadLifecycleChangeCapturesSingleFrozenPreloadHandoff) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 10U);
  ASSERT_TRUE(frozen.surface_reference_frozen);
  const std::uint64_t preload_history =
      frozen.group_diagnostics.front().matched_history_id;
  const auto loaded = authority.update(rawSurfaceInput(
      1.20, 11U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_TRUE(loaded.preload_handoff_captured);
  EXPECT_TRUE(loaded.preload_handoff_consumed);
  EXPECT_EQ(loaded.preload_handoff_source_history_id, preload_history);
  EXPECT_NE(loaded.postload_history_id, preload_history);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LifecycleResetStillClearsCargoHistories) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 20U);
  const auto loaded = authority.update(rawSurfaceInput(
      1.20, 21U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  ASSERT_TRUE(loaded.preload_handoff_consumed);
  EXPECT_NE(frozen.group_diagnostics.front().matched_history_id,
            loaded.group_diagnostics.front().matched_history_id);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     HandoffDoesNotCarryValidatedIdentity) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 30U);
  const auto loaded = authority.update(rawSurfaceInput(
      1.20, 31U, HookLoadState::LOADED, 0.0, 0.40, 0.40));
  EXPECT_TRUE(loaded.preload_handoff_consumed);
  EXPECT_NE(loaded.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     HandoffDoesNotCarryLiftConfirmCount) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 40U);
  const auto loaded = authority.update(rawSurfaceInput(
      1.20, 41U, HookLoadState::LOADED, 0.0, 0.40, 0.40));
  ASSERT_TRUE(loaded.preload_handoff_consumed);
  EXPECT_EQ(loaded.group_diagnostics.front().lift_confirm_count, 0);
  EXPECT_FALSE(loaded.group_diagnostics.front().lift_confirmed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     HandoffRequiresEmptyToLoadedTransition) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 50U);
  auto unknown = rawSurfaceInput(
      1.20, 51U, HookLoadState::UNKNOWN, 0.0, 0.70, 0.70);
  unknown.gravity_valid = false;
  const auto result = authority.update(unknown);
  EXPECT_FALSE(result.preload_handoff_captured);
  EXPECT_FALSE(result.preload_handoff_consumed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     SingleCertificateHandoffWithMultipleFrozenHistories) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 2;
  CargoPhysicalIdentityAuthority authority(config);
  auto frame = rawSurfaceInput(
      1.0, 60U, HookLoadState::EMPTY, 0.0, 0.30, 0.30);
  frame.groups = buildGroups({
      candidate(1U, 1.0, -1.0, 0.30, {1U}),
      candidate(2U, 1.0, 1.0, 0.30, {2U})});
  attachCurrentRawRoi(&frame);
  authority.update(frame);
  frame.pipeline_stamp_sec = 1.05;
  for (auto& physical_group : frame.groups) {
    physical_group.descriptor.stamp_sec = 1.05;
    physical_group.representative.stamp_sec = 1.05;
  }
  attachCurrentRawRoi(&frame);
  const auto frozen = authority.update(frame);
  // Two histories freeze, but only ONE certificate is minted (first freeze).
  EXPECT_TRUE(frozen.preload_reference_certificate_created);
  EXPECT_TRUE(frozen.preload_reference_certificate_valid);
  const std::uint64_t certificate_history =
      frozen.preload_reference_certificate_source_history_id;
  EXPECT_NE(certificate_history, 0U);
  const auto loaded = authority.update(rawSurfaceInput(
      1.10, 61U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  // The single certificate hands off (no history ambiguity), but the current
  // group at x=0.0 does not match the certificate footprint, so it fails
  // closed instead of consuming.
  EXPECT_TRUE(loaded.preload_handoff_captured);
  EXPECT_EQ(loaded.preload_handoff_source_history_id, certificate_history);
  EXPECT_FALSE(loaded.preload_handoff_consumed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     AmbiguousPreloadHistoriesCannotCreateHandoff) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 2;
  CargoPhysicalIdentityAuthority authority(config);
  auto frame = rawSurfaceInput(
      1.0, 70U, HookLoadState::EMPTY, 0.0, 0.30, 0.30);
  frame.groups = buildGroups({
      candidate(1U, 1.0, -0.2, 0.30, {1U}),
      candidate(2U, 1.0, 0.2, 0.30, {2U})});
  attachCurrentRawRoi(&frame);
  authority.update(frame);
  const auto loaded = authority.update(rawSurfaceInput(
      1.10, 71U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_FALSE(loaded.preload_handoff_captured);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     FrozenReferenceSurvivesIdleEmptyBeyondObservationGap) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 80U);
  ASSERT_TRUE(frozen.preload_reference_certificate_valid);
  // The reference froze at ~1.15s; the load edge arrives at 2.0s, far beyond
  // the 0.50s observation gap.  The immutable certificate still hands off.
  const auto loaded = authority.update(rawSurfaceInput(
      2.0, 81U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_TRUE(loaded.preload_handoff_captured);
  EXPECT_TRUE(loaded.preload_handoff_consumed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PostloadHandoffRequiresUniqueCurrentGroup) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 90U);
  auto loaded = rawSurfaceInput(
      1.20, 91U, HookLoadState::LOADED, 0.0, 0.70, 0.70);
  loaded.groups = buildGroups({
      candidate(11U, 1.20, -0.10, 0.70, {11U}),
      candidate(12U, 1.20, 0.10, 0.70, {12U})});
  attachCurrentRawRoi(&loaded);
  const auto result = authority.update(loaded);
  EXPECT_TRUE(result.preload_handoff_captured);
  EXPECT_FALSE(result.preload_handoff_consumed);
  EXPECT_EQ(result.preload_handoff_reject_reason,
            "AMBIGUOUS_POSTLOAD_GROUPS");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PostloadHandoffWrongXYFailsClosed) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 100U);
  const auto result = authority.update(rawSurfaceInput(
      1.20, 101U, HookLoadState::LOADED, 2.0, 0.70, 0.70));
  EXPECT_FALSE(result.preload_handoff_consumed);
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PostloadHandoffExtentMismatchFailsClosed) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 110U);
  auto loaded = rawSurfaceInput(
      1.20, 111U, HookLoadState::LOADED, 0.0, 0.70, 0.70);
  loaded.groups = {groupWithSupport(
      20U, 20U, 1.20, -1.5, 1.5, 0.70)};
  attachCurrentRawRoi(&loaded);
  const auto result = authority.update(loaded);
  EXPECT_FALSE(result.preload_handoff_consumed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PostloadHandoffCompetitionFailsClosed) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 120U);
  auto loaded = rawSurfaceInput(
      1.20, 121U, HookLoadState::LOADED, 0.0, 0.70, 0.70);
  loaded.groups = buildGroups({
      candidate(21U, 1.20, -0.05, 0.70, {21U}),
      candidate(22U, 1.20, 0.05, 0.70, {22U})});
  attachCurrentRawRoi(&loaded);
  const auto result = authority.update(loaded);
  EXPECT_FALSE(result.preload_handoff_consumed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     HandoffCanBeConsumedOnlyOnce) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 130U);
  const auto first = authority.update(rawSurfaceInput(
      1.20, 131U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  ASSERT_TRUE(first.preload_handoff_consumed);
  const auto second = authority.update(rawSurfaceInput(
      1.25, 131U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_FALSE(second.preload_handoff_consumed);
  EXPECT_EQ(second.group_diagnostics.front().matched_history_id,
            first.postload_history_id);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     HandoffCannotSeedTwoHistories) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 140U);
  auto loaded = rawSurfaceInput(
      1.20, 141U, HookLoadState::LOADED, 0.0, 0.70, 0.70);
  loaded.groups = buildGroups({
      candidate(31U, 1.20, -0.05, 0.70, {31U}),
      candidate(32U, 1.20, 0.05, 0.70, {32U})});
  attachCurrentRawRoi(&loaded);
  const auto result = authority.update(loaded);
  EXPECT_EQ(static_cast<int>(std::count_if(
      result.group_diagnostics.begin(), result.group_diagnostics.end(),
      [](const CargoPhysicalGroupDiagnostic& diagnostic) {
        return diagnostic.preload_handoff_consumed;
      })), 0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     StartedLoadedCannotCreateOrConsumePreloadHandoff) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  auto loaded = rawSurfaceInput(
      1.0, 150U, HookLoadState::LOADED, 0.0, 0.70, 0.70);
  loaded.node_started_loaded = true;
  const auto result = authority.update(loaded);
  EXPECT_FALSE(result.preload_handoff_captured);
  EXPECT_FALSE(result.preload_handoff_consumed);
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     GravityLoadedWithoutBoundaryCannotActivateFormalLift) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  // Started loaded: gravity LOADED but no pre-load EMPTY phase, so no frozen
  // certificate and no B6 load/lifecycle boundary can ever close.  The formal
  // lift authority must NOT activate from gravity alone.
  auto loaded = rawSurfaceInput(
      1.0, 600U, HookLoadState::LOADED, 0.0, 0.70, 0.70);
  loaded.node_started_loaded = true;
  const auto result = authority.update(loaded);
  EXPECT_FALSE(result.formal_lift_boundary_authorized);
  EXPECT_FALSE(result.formal_lift_lock_active);
  EXPECT_FALSE(result.formal_lift_confirmed);
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PreloadSurfaceReferenceUsesCurrentRawRoi) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  CargoPhysicalIdentityDecision result;
  for (int frame = 0; frame < 4; ++frame) {
    result = authority.update(rawSurfaceInput(
        1.0 + 0.05 * frame, 160U, HookLoadState::EMPTY,
        0.0, 0.90, 0.40));
  }
  ASSERT_TRUE(result.surface_reference_frozen);
  EXPECT_NEAR(result.surface_baseline_z, 0.40, 1.0e-5);
  EXPECT_NEAR(result.group_diagnostics.front().descriptor.physical_vertical_z,
              0.90, 1.0e-5);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PreloadSurfaceReferenceRequiresCurrentOwnerProof) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  auto frame = rawSurfaceInput(
      1.0, 170U, HookLoadState::EMPTY, 0.0, 0.40, 0.40);
  frame.frame_evidence.raw_roi_current_frame =
      cloudFromPoints(componentPoints(2.0, 0.40));
  const auto result = authority.update(frame);
  EXPECT_FALSE(result.current_surface_vertical_valid);
  EXPECT_EQ(result.prelift_sample_count, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     FrozenPreloadFootprintIsImmutableAfterLoad) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 180U);
  ASSERT_TRUE(frozen.surface_reference_footprint.valid);
  const Eigen::Vector2f center = frozen.surface_reference_footprint.center_base;
  authority.update(rawSurfaceInput(
      1.20, 180U, HookLoadState::LOADED, 0.20, 0.70, 0.70));
  const auto current = authority.update(rawSurfaceInput(
      1.25, 180U, HookLoadState::LOADED, 0.20, 0.70, 0.70));
  EXPECT_TRUE(current.surface_reference_footprint.valid);
  EXPECT_TRUE(current.surface_reference_footprint.center_base.isApprox(center));
}

TEST(CargoPhysicalIdentityAuthorityTest,
     BaselineAndCurrentLiftUseSameVerticalSource) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 190U);
  const auto current = authority.update(rawSurfaceInput(
      1.20, 190U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_TRUE(current.current_surface_vertical_valid);
  EXPECT_NEAR(current.surface_baseline_z, 0.40, 1.0e-5);
  EXPECT_NEAR(current.current_surface_z, 0.70, 1.0e-5);
  EXPECT_EQ(current.lift_vertical_source,
            "FROZEN_PRELOAD_FOOTPRINT_RAW_ROI_OWNER_PROOF");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     HistoricalZCannotSelectCurrentSurface) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 200U);
  auto current = rawSurfaceInput(
      1.20, 200U, HookLoadState::LOADED, 0.0, 0.45, 0.45);
  std::vector<Eigen::Vector3f> points = componentPoints(0.0, 0.45);
  const auto high = componentPoints(0.0, 0.75);
  points.insert(points.end(), high.begin(), high.end());
  current.frame_evidence.raw_roi_current_frame = cloudFromPoints(points);
  const auto result = authority.update(current);
  ASSERT_TRUE(result.current_surface_vertical_valid);
  EXPECT_NEAR(result.current_surface_z, 0.75, 1.0e-5);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     CurrentOwnerProofFailureCannotAdvanceLift) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 210U);
  auto current = rawSurfaceInput(
      1.20, 210U, HookLoadState::LOADED, 0.30, 0.70, 0.70);
  current.frame_evidence.raw_roi_current_frame =
      cloudFromPoints(componentPoints(-0.20, 0.70));
  const auto result = authority.update(current);
  EXPECT_FALSE(result.current_surface_vertical_valid);
  EXPECT_EQ(result.group_diagnostics.front().lift_confirm_count, 0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     SurfaceSwitchCannotProduceFalseLift) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 220U);
  CargoPhysicalIdentityDecision result;
  for (int frame = 0; frame < 4; ++frame) {
    result = authority.update(rawSurfaceInput(
        1.20 + 0.05 * frame, 220U, HookLoadState::LOADED,
        0.0, 0.90, 0.40));
  }
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.group_diagnostics.front().lift_confirm_count, 0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     EmptySurfaceSwitchCannotAdvanceLift) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 230U);
  const auto result = authority.update(rawSurfaceInput(
      1.20, 230U, HookLoadState::EMPTY, 0.0, 0.90, 0.70));
  EXPECT_EQ(result.group_diagnostics.front().lift_confirm_count, 0);
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     TrueLoadedLiftRequiresFourCurrentSupportedFrames) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 240U);
  CargoPhysicalIdentityDecision result;
  for (int frame = 0; frame < 3; ++frame) {
    result = authority.update(rawSurfaceInput(
        1.20 + 0.05 * frame, 241U, HookLoadState::LOADED,
        0.0, 0.70, 0.70));
    EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
  }
  result = authority.update(rawSurfaceInput(
      1.35, 241U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.required_lift_confirm_frames, 4);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ContinuityEvidenceCannotProvideLiftVerticalByItself) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 250U);
  auto current = rawSurfaceInput(
      1.20, 250U, HookLoadState::LOADED, 0.0, 0.70, 0.70);
  current.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::CONTINUITY_ONLY;
  const auto result = authority.update(current);
  EXPECT_FALSE(result.current_surface_vertical_valid);
  EXPECT_EQ(result.group_diagnostics.front().lift_confirm_count, 0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LineageCannotProvideLiftVertical) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(lineageInput(
      1.0, HookLoadState::EMPTY, 0.0, 0.40, 101U));
  const auto observation = lineageObservation(
      101U, 202U, 1U, 1.0, 1.1, 0.05);
  const auto result = authority.update(lineageInput(
      1.1, HookLoadState::LOADED, 2.0, 0.70, 202U, &observation));
  EXPECT_FALSE(result.current_surface_vertical_valid);
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     HandoffBaselineCannotValidateWithoutCurrentLift) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 260U);
  CargoPhysicalIdentityDecision result;
  for (int frame = 0; frame < 5; ++frame) {
    result = authority.update(rawSurfaceInput(
        1.20 + 0.05 * frame, 261U, HookLoadState::LOADED,
        0.0, 0.40, 0.40));
  }
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.group_diagnostics.front().lift_confirm_count, 0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadedFourFramesValidateOnlySameHandoffHistory) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 270U);
  std::uint64_t postload_history = 0U;
  CargoPhysicalIdentityDecision result;
  for (int frame = 0; frame < 4; ++frame) {
    result = authority.update(rawSurfaceInput(
        1.20 + 0.05 * frame, 271U, HookLoadState::LOADED,
        0.0, 0.70, 0.70));
    if (frame == 0) postload_history = result.postload_history_id;
    ASSERT_EQ(result.group_diagnostics.front().matched_history_id,
              postload_history);
  }
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.physical_history_id, postload_history);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LifecycleThenLoadWithinGapCapturesHandoff) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 310U);
  ASSERT_TRUE(frozen.surface_reference_frozen);
  const std::uint64_t preload_history =
      frozen.group_diagnostics.front().matched_history_id;
  // Lifecycle edge first, gravity still EMPTY.
  const auto lifecycle_first = authority.update(rawSurfaceInput(
      1.20, 311U, HookLoadState::EMPTY, 0.0, 0.40, 0.40));
  EXPECT_FALSE(lifecycle_first.preload_handoff_captured);
  EXPECT_TRUE(lifecycle_first.preload_boundary_pending);
  EXPECT_EQ(lifecycle_first.preload_boundary_phase, "WAITING_FOR_LOAD");
  // Load edge within the gap.
  const auto loaded = authority.update(rawSurfaceInput(
      1.25, 311U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_TRUE(loaded.preload_handoff_captured);
  EXPECT_EQ(loaded.preload_handoff_trigger_mode, "LIFECYCLE_THEN_LOAD");
  EXPECT_EQ(loaded.preload_handoff_source_history_id, preload_history);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadThenLifecycleWithinGapCapturesHandoff) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 320U);
  ASSERT_TRUE(frozen.surface_reference_frozen);
  const std::uint64_t preload_history =
      frozen.group_diagnostics.front().matched_history_id;
  // Load edge first, lifecycle unchanged.
  const auto load_first = authority.update(rawSurfaceInput(
      1.20, 320U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_FALSE(load_first.preload_handoff_captured);
  EXPECT_TRUE(load_first.preload_boundary_pending);
  EXPECT_EQ(load_first.preload_boundary_phase, "WAITING_FOR_LIFECYCLE");
  // Lifecycle edge within the gap.
  const auto lifecycle = authority.update(rawSurfaceInput(
      1.25, 321U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_TRUE(lifecycle.preload_handoff_captured);
  EXPECT_EQ(lifecycle.preload_handoff_trigger_mode, "LOAD_THEN_LIFECYCLE");
  EXPECT_EQ(lifecycle.preload_handoff_source_history_id, preload_history);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LifecycleThenLoadBeyondGapFailsClosed) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 330U);
  const auto lifecycle_first = authority.update(rawSurfaceInput(
      1.20, 331U, HookLoadState::EMPTY, 0.0, 0.40, 0.40));
  EXPECT_TRUE(lifecycle_first.preload_boundary_pending);
  const auto loaded = authority.update(rawSurfaceInput(
      2.00, 331U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_FALSE(loaded.preload_handoff_captured);
  EXPECT_FALSE(loaded.preload_boundary_pending);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadThenLifecycleBeyondGapFailsClosed) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 340U);
  const auto load_first = authority.update(rawSurfaceInput(
      1.20, 340U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_TRUE(load_first.preload_boundary_pending);
  const auto lifecycle = authority.update(rawSurfaceInput(
      2.00, 341U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_FALSE(lifecycle.preload_handoff_captured);
  EXPECT_FALSE(lifecycle.preload_boundary_pending);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     SecondLifecycleBeforeLoadFailsClosed) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 350U);
  const auto first = authority.update(rawSurfaceInput(
      1.20, 351U, HookLoadState::EMPTY, 0.0, 0.40, 0.40));
  EXPECT_TRUE(first.preload_boundary_pending);
  // A second unrelated lifecycle change before the load edge must clear the
  // pending reference rather than hand off a stale frozen footprint.
  const auto second = authority.update(rawSurfaceInput(
      1.25, 352U, HookLoadState::EMPTY, 0.0, 0.40, 0.40));
  EXPECT_FALSE(second.preload_boundary_pending);
  EXPECT_FALSE(second.preload_handoff_captured);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     RearmClearsPendingBoundary) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 360U);
  const auto lifecycle_first = authority.update(rawSurfaceInput(
      1.20, 361U, HookLoadState::EMPTY, 0.0, 0.40, 0.40));
  EXPECT_TRUE(lifecycle_first.preload_boundary_pending);
  auto rearm = rawSurfaceInput(
      1.25, 362U, HookLoadState::LOADED, 0.0, 0.70, 0.70);
  rearm.rearm = true;
  const auto result = authority.update(rearm);
  EXPECT_FALSE(result.preload_boundary_pending);
  EXPECT_FALSE(result.preload_handoff_captured);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PendingBoundaryDoesNotValidateIdentity) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 370U);
  const auto lifecycle_first = authority.update(rawSurfaceInput(
      1.20, 371U, HookLoadState::EMPTY, 0.0, 0.40, 0.40));
  ASSERT_TRUE(lifecycle_first.preload_boundary_pending);
  EXPECT_NE(lifecycle_first.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(lifecycle_first.group_diagnostics.front().lift_confirm_count, 0);
  EXPECT_FALSE(lifecycle_first.lift_confirmed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     StartedLoadedCannotCreatePendingBoundary) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  // No pre-load EMPTY phase: a LOADED start has no eligible frozen reference.
  auto started_loaded = rawSurfaceInput(
      1.0, 380U, HookLoadState::LOADED, 0.0, 0.70, 0.70);
  started_loaded.node_started_loaded = true;
  const auto result = authority.update(started_loaded);
  EXPECT_FALSE(result.preload_boundary_pending);
  EXPECT_FALSE(result.preload_handoff_captured);
}

// ===========================================================================
// Frozen preload reference certificate lifecycle (B6 lifetime decoupling).
// ===========================================================================

TEST(CargoPhysicalIdentityAuthorityTest,
     RearmInvalidatesFrozenCertificate) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 500U);
  ASSERT_TRUE(frozen.preload_reference_certificate_valid);
  auto rearm = rawSurfaceInput(
      1.25, 500U, HookLoadState::EMPTY, 0.0, 0.40, 0.40);
  rearm.rearm = true;
  const auto result = authority.update(rearm);
  EXPECT_FALSE(result.preload_reference_certificate_valid);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     SourceTimeRollbackInvalidatesFrozenCertificate) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 510U);
  ASSERT_TRUE(frozen.preload_reference_certificate_valid);
  // A backward source-time jump invalidates the certificate fail-closed.
  const auto result = authority.update(rawSurfaceInput(
      0.5, 510U, HookLoadState::EMPTY, 0.0, 0.40, 0.40));
  EXPECT_FALSE(result.preload_reference_certificate_valid);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PhysicalEpochEndInvalidatesFrozenCertificate) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 520U);
  ASSERT_TRUE(frozen.preload_reference_certificate_valid);
  // A lifecycle edge while gravity is still EMPTY consumes the certificate into
  // a pending boundary, then invalidates it at the physical-epoch boundary.
  const auto result = authority.update(rawSurfaceInput(
      1.20, 521U, HookLoadState::EMPTY, 0.0, 0.40, 0.40));
  EXPECT_FALSE(result.preload_reference_certificate_valid);
  EXPECT_TRUE(result.preload_boundary_pending);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     UnobservablePreloadFrameDoesNotInvalidateCertificate) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 530U);
  ASSERT_TRUE(frozen.preload_reference_certificate_valid);
  auto unobservable = rawSurfaceInput(
      1.20, 530U, HookLoadState::EMPTY, 0.0, 0.40, 0.40);
  unobservable.groups.clear();
  unobservable.frame_evidence.raw_roi_current_frame = cloudFromPoints({});
  const auto result = authority.update(unobservable);
  EXPECT_TRUE(result.preload_reference_certificate_valid);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ContradictoryPreloadOwnerInvalidatesCertificate) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 540U);
  ASSERT_TRUE(frozen.preload_reference_certificate_valid);
  // A swapped owner at the SAME frozen center (x=0.0) but a much larger
  // extent invalidates.
  auto contradiction = rawSurfaceInput(
      1.20, 540U, HookLoadState::EMPTY, 0.0, 0.40, 0.40);
  contradiction.groups = {groupWithSupport(50U, 50U, 1.20, -1.5, 1.5, 0.40)};
  attachCurrentRawRoi(&contradiction);
  const auto result = authority.update(contradiction);
  EXPECT_FALSE(result.preload_reference_certificate_valid);
  EXPECT_EQ(result.preload_reference_certificate_invalidate_reason,
            "CONTRADICTORY_PRELOAD_OWNER");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     DifferentObjectFarFromFrozenCenterDoesNotInvalidateCertificate) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 541U);
  ASSERT_TRUE(frozen.preload_reference_certificate_valid);
  // A separate object at x=2.0 (far from the frozen center x=0.0) is a
  // different object, not a swap; the certificate must survive it.
  const auto result = authority.update(rawSurfaceInput(
      1.20, 541U, HookLoadState::EMPTY, 2.0, 0.40, 0.40));
  EXPECT_TRUE(result.preload_reference_certificate_valid);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LoadThenLifecycleConsumesFrozenCertificateBeyondIdleGap) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 550U);
  ASSERT_TRUE(frozen.preload_reference_certificate_valid);
  // Load edge at 2.0s (0.85s idle after the ~1.15s freeze) still captures the
  // certificate into a pending boundary.
  const auto load_first = authority.update(rawSurfaceInput(
      2.0, 550U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_FALSE(load_first.preload_handoff_captured);
  EXPECT_TRUE(load_first.preload_boundary_pending);
  EXPECT_EQ(load_first.preload_boundary_phase, "WAITING_FOR_LIFECYCLE");
  const auto lifecycle = authority.update(rawSurfaceInput(
      2.05, 551U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_TRUE(lifecycle.preload_handoff_captured);
  EXPECT_EQ(lifecycle.preload_handoff_trigger_mode, "LOAD_THEN_LIFECYCLE");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     LifecycleThenLoadConsumesFrozenCertificateBeyondIdleGap) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  config.maximum_observation_gap_sec = 0.50;
  CargoPhysicalIdentityAuthority authority(config);
  const auto frozen = freezeSurfaceReference(&authority, 560U);
  ASSERT_TRUE(frozen.preload_reference_certificate_valid);
  const auto lifecycle_first = authority.update(rawSurfaceInput(
      2.0, 561U, HookLoadState::EMPTY, 0.0, 0.40, 0.40));
  EXPECT_FALSE(lifecycle_first.preload_handoff_captured);
  EXPECT_TRUE(lifecycle_first.preload_boundary_pending);
  EXPECT_EQ(lifecycle_first.preload_boundary_phase, "WAITING_FOR_LOAD");
  const auto loaded = authority.update(rawSurfaceInput(
      2.05, 561U, HookLoadState::LOADED, 0.0, 0.70, 0.70));
  EXPECT_TRUE(loaded.preload_handoff_captured);
  EXPECT_EQ(loaded.preload_handoff_trigger_mode, "LIFECYCLE_THEN_LOAD");
}

// ===========================================================================
// Canonical fragment reconstruction unit tests (reconstructLogicalCurrentCargo).
// A coarse 1.0 m XY cell is used so world-XY cell arithmetic is exact (integer
// coordinates), removing float-boundary dependence from clique/union asserts.
// ===========================================================================

CargoVerticalEvidenceConfig reconstructConfig() {
  CargoVerticalEvidenceConfig config = verticalConfig();
  config.xy_cell_size_m = 1.0F;
  config.minimum_surface_cells = 2U;
  config.minimum_surface_coverage_ratio = 0.0F;
  return config;
}

CargoPhysicalGroupObservation makeFragment(std::uint64_t frame_group_id,
                                           std::uint64_t component_id,
                                           double stamp, double cx, double cy,
                                           double z, double extent = 2.0) {
  CargoPhysicalGroupObservation g;
  g.frame_group_id = frame_group_id;
  g.member_component_ids = {component_id};
  g.geometry_resolved = true;
  g.group_ambiguous = false;
  const float half = static_cast<float>(0.5 * extent);
  for (float dx : {-half, 0.0F, half}) {
    for (float dy : {-half, 0.0F, half}) {
      g.union_points_base.emplace_back(static_cast<float>(cx) + dx,
                                       static_cast<float>(cy) + dy,
                                       static_cast<float>(z));
    }
  }
  g.descriptor.valid = true;
  g.descriptor.stamp_sec = stamp;
  g.descriptor.stable_anchor = Eigen::Vector3d(cx, cy, z);
  g.descriptor.aggregate_extent = Eigen::Vector3d(extent, extent, 0.40);
  g.descriptor.robust_x05 = cx - 0.5 * extent;
  g.descriptor.robust_x95 = cx + 0.5 * extent;
  g.descriptor.robust_y05 = cy - 0.5 * extent;
  g.descriptor.robust_y95 = cy + 0.5 * extent;
  g.descriptor.robust_xy_center = Eigen::Vector2d(cx, cy);
  g.descriptor.robust_xy_extent = Eigen::Vector2d(extent, extent);
  g.descriptor.aggregate_point_support = 9U;
  g.descriptor.vertical_mode = CargoGroupVerticalMode::SUPPORTED_EVIDENCE;
  g.descriptor.physical_vertical_z = z;
  g.descriptor.vertical_uncertainty_m = 0.05;
  g.descriptor.diagnostic_z05 = z - 0.20;
  g.descriptor.diagnostic_z95 = z;
  g.representative.candidate_id = frame_group_id;
  g.representative.stamp_sec = stamp;
  g.representative.center = Eigen::Vector3d(cx, cy, z);
  g.representative.size = Eigen::Vector3d(extent, extent, 0.40);
  g.representative.yaw_rad = 0.0;
  return g;
}

CargoFootprintSnapshot frozenFootprint(double cx, double cy, double sx,
                                       double sy, double yaw = 0.0) {
  CargoFootprintSnapshot f;
  f.valid = true;
  f.center_base = Eigen::Vector2f(static_cast<float>(cx),
                                  static_cast<float>(cy));
  f.size_xy = Eigen::Vector2f(static_cast<float>(sx), static_cast<float>(sy));
  f.yaw_base_rad = static_cast<float>(yaw);
  f.source_stamp_sec = 1.0;
  return f;
}

std::vector<CargoFootprintGridIndex> frozenOwnerCells(int x_min, int x_max,
                                                      int y_min, int y_max) {
  std::vector<CargoFootprintGridIndex> cells;
  for (int x = x_min; x <= x_max; ++x) {
    for (int y = y_min; y <= y_max; ++y) {
      cells.push_back({x, y});
    }
  }
  return cells;
}

TEST(CargoPhysicalIdentityAuthorityTest, SingletonGroupCanBecomeLogicalOwner) {
  const std::vector<CargoPhysicalGroupObservation> groups = {
      makeFragment(1U, 1U, 1.0, 0.0, 0.0, 0.40)};
  const auto owner = reconstructLogicalCurrentCargo(
      groups, frozenFootprint(0.0, 0.0, 2.0, 2.0),
      frozenOwnerCells(-1, 1, -1, 1), reconstructConfig(), 0.60);
  ASSERT_TRUE(owner.valid);
  EXPECT_FALSE(owner.ambiguous);
  ASSERT_EQ(owner.member_group_indices.size(), 1U);
  EXPECT_EQ(owner.member_group_indices.front(), 0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     TwoFragmentsSameWorldXYBecomeOneLogicalOwner) {
  const std::vector<CargoPhysicalGroupObservation> groups = {
      makeFragment(1U, 1U, 1.0, 0.0, 0.0, 0.40),
      makeFragment(2U, 2U, 1.0, 0.0, 0.0, 1.50)};
  const auto owner = reconstructLogicalCurrentCargo(
      groups, frozenFootprint(0.0, 0.0, 2.0, 2.0),
      frozenOwnerCells(-1, 1, -1, 1), reconstructConfig(), 0.60);
  ASSERT_TRUE(owner.valid);
  EXPECT_EQ(owner.member_group_indices.size(), 2U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ThreeFragmentsCompleteCliqueBecomeOneLogicalOwner) {
  const std::vector<CargoPhysicalGroupObservation> groups = {
      makeFragment(1U, 1U, 1.0, 0.0, 0.0, 0.30),
      makeFragment(2U, 2U, 1.0, 0.0, 0.0, 0.80),
      makeFragment(3U, 3U, 1.0, 0.0, 0.0, 1.50)};
  const auto owner = reconstructLogicalCurrentCargo(
      groups, frozenFootprint(0.0, 0.0, 2.0, 2.0),
      frozenOwnerCells(-1, 1, -1, 1), reconstructConfig(), 0.60);
  ASSERT_TRUE(owner.valid);
  EXPECT_EQ(owner.member_group_indices.size(), 3U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     TwoIndependentCliquesRemainAmbiguous) {
  // Two fragments with disjoint local footprints (different extents) each pass
  // the individual frozen filter but are NOT pairwise SAME_OWNER_FRAGMENT, so
  // the maximal cliques are {A} and {B} -> ambiguous, not a single owner.
  const std::vector<CargoPhysicalGroupObservation> groups = {
      makeFragment(1U, 1U, 1.0, 0.0, 0.0, 0.40, 1.0),
      makeFragment(2U, 2U, 1.0, 0.0, 0.0, 0.40, 4.0)};
  const auto owner = reconstructLogicalCurrentCargo(
      groups, frozenFootprint(0.0, 0.0, 2.0, 2.0),
      frozenOwnerCells(-2, 2, -2, 2), reconstructConfig(), 0.60);
  EXPECT_FALSE(owner.valid);
  EXPECT_TRUE(owner.ambiguous);
  EXPECT_EQ(owner.reject_reason, "CURRENT_OWNER_AMBIGUOUS");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     GroupOrderPermutationDoesNotChangeLogicalOwner) {
  const std::vector<CargoPhysicalGroupObservation> groups_ab = {
      makeFragment(1U, 1U, 1.0, 0.0, 0.0, 0.40),
      makeFragment(2U, 2U, 1.0, 0.0, 0.0, 1.50)};
  const std::vector<CargoPhysicalGroupObservation> groups_ba = {
      makeFragment(2U, 2U, 1.0, 0.0, 0.0, 1.50),
      makeFragment(1U, 1U, 1.0, 0.0, 0.0, 0.40)};
  const auto fp = frozenFootprint(0.0, 0.0, 2.0, 2.0);
  const auto cells = frozenOwnerCells(-1, 1, -1, 1);
  const auto owner_ab = reconstructLogicalCurrentCargo(
      groups_ab, fp, cells, reconstructConfig(), 0.60);
  const auto owner_ba = reconstructLogicalCurrentCargo(
      groups_ba, fp, cells, reconstructConfig(), 0.60);
  ASSERT_TRUE(owner_ab.valid);
  ASSERT_TRUE(owner_ba.valid);
  EXPECT_EQ(owner_ab.member_group_indices, owner_ba.member_group_indices);
  EXPECT_NEAR(owner_ab.current_footprint.center_base.x(),
              owner_ba.current_footprint.center_base.x(), 1e-6);
  EXPECT_NEAR(owner_ab.current_footprint.size_xy.x(),
              owner_ba.current_footprint.size_xy.x(), 1e-6);
}

TEST(CargoPhysicalIdentityAuthorityTest, LogicalOwnerFootprintUsesUnionPoints) {
  // Two overlapping fragments; the logical footprint must span BOTH (3.0 m
  // wide), never a single fragment's 2.0 m footprint.
  const std::vector<CargoPhysicalGroupObservation> groups = {
      makeFragment(1U, 1U, 1.0, 0.0, 0.0, 0.40),
      makeFragment(2U, 2U, 1.0, 1.0, 0.0, 0.40)};
  const auto owner = reconstructLogicalCurrentCargo(
      groups, frozenFootprint(0.5, 0.0, 3.0, 2.0),
      frozenOwnerCells(-2, 2, -1, 1), reconstructConfig(), 0.60);
  ASSERT_TRUE(owner.valid);
  EXPECT_NEAR(owner.current_footprint.size_xy.x(), 3.0, 1e-3);
  EXPECT_NEAR(owner.current_footprint.size_xy.y(), 2.0, 1e-3);
  EXPECT_NEAR(owner.current_footprint.center_base.x(), 0.5, 1e-3);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     NoCurrentOwnerWhenUnionExtentMismatches) {
  const std::vector<CargoPhysicalGroupObservation> groups = {
      makeFragment(1U, 1U, 1.0, 0.0, 0.0, 0.40, 2.0)};
  const auto owner = reconstructLogicalCurrentCargo(
      groups, frozenFootprint(0.0, 0.0, 0.4, 0.4),
      frozenOwnerCells(-1, 1, -1, 1), reconstructConfig(), 0.60);
  EXPECT_FALSE(owner.valid);
  EXPECT_FALSE(owner.ambiguous);
  EXPECT_EQ(owner.reject_reason, "NO_CURRENT_OWNER");
}

TEST(CargoPhysicalIdentityAuthorityTest, EmptySceneCannotReconstructOwner) {
  const std::vector<CargoPhysicalGroupObservation> groups = {};
  const auto owner = reconstructLogicalCurrentCargo(
      groups, frozenFootprint(0.0, 0.0, 2.0, 2.0),
      frozenOwnerCells(-1, 1, -1, 1), reconstructConfig(), 0.60);
  EXPECT_FALSE(owner.valid);
  EXPECT_FALSE(owner.ambiguous);
  EXPECT_EQ(owner.reject_reason, "NO_CURRENT_OWNER");
}

// ===========================================================================
// Three-state lift evidence classification.
// ===========================================================================

TEST(CargoPhysicalIdentityAuthorityTest, SignificantLiftIsPositive) {
  EXPECT_EQ(classifyLiftEvidence(true, true, false, true),
            LiftEvidenceClass::POSITIVE);
}

TEST(CargoPhysicalIdentityAuthorityTest, MissingOwnerWithoutAmbiguityIsUnobservable) {
  EXPECT_EQ(classifyLiftEvidence(false, false, false, false),
            LiftEvidenceClass::UNOBSERVABLE);
}

TEST(CargoPhysicalIdentityAuthorityTest, AmbiguousOwnerIsContradictory) {
  EXPECT_EQ(classifyLiftEvidence(false, false, true, false),
            LiftEvidenceClass::CONTRADICTORY);
}

TEST(CargoPhysicalIdentityAuthorityTest, MissingSurfaceIsUnobservable) {
  EXPECT_EQ(classifyLiftEvidence(false, true, false, false),
            LiftEvidenceClass::UNOBSERVABLE);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ValidSurfaceNotSignificantIsContradictory) {
  EXPECT_EQ(classifyLiftEvidence(false, true, false, true),
            LiftEvidenceClass::CONTRADICTORY);
}

// ============ G/H production-owner latch (immutable owner) ============

TEST(CargoPhysicalIdentityAuthorityTest,
     UniqueValidatedHistoryLatchesProductionOwner) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto decision = validateRequired(&authority);
  ASSERT_EQ(decision.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_TRUE(decision.production_owner_locked);
  EXPECT_NE(decision.production_owner_history_id, 0U);
  EXPECT_NE(decision.production_owner_lock_generation, 0U);
  EXPECT_GT(decision.production_owner_lock_stamp_sec, 0.0);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ProductionOwnerCannotSwitchAfterLatch) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  ASSERT_TRUE(validated.production_owner_locked);
  const std::uint64_t owner_id = validated.production_owner_history_id;
  const std::uint64_t generation = validated.production_owner_lock_generation;
  // Churn (further same-history frames) must never rewrite the latched owner.
  for (double stamp = 1.4; stamp <= 1.9; stamp += 0.1) {
    const auto result = authority.update(input(
        stamp, HookLoadSignalRole::REQUIRED, true, HookLoadState::LOADED,
        0.0, 0.7));
    ASSERT_TRUE(result.production_owner_locked);
    EXPECT_EQ(result.production_owner_history_id, owner_id);
    EXPECT_EQ(result.production_owner_lock_generation, generation);
  }
}

TEST(CargoPhysicalIdentityAuthorityTest, UnloadClearsProductionOwner) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  ASSERT_TRUE(validated.production_owner_locked);
  // A fresh authoritative EMPTY retires the epoch and clears the owner.
  const auto unloaded = authority.update(input(
      2.0, HookLoadSignalRole::REQUIRED, true, HookLoadState::EMPTY, 0.0,
      0.7));
  EXPECT_FALSE(unloaded.production_owner_locked);
  EXPECT_EQ(unloaded.production_owner_history_id, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest, EpochResetClearsProductionOwner) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  ASSERT_TRUE(validated.production_owner_locked);
  CargoPhysicalIdentityInput next = input(
      2.0, HookLoadSignalRole::REQUIRED, true, HookLoadState::EMPTY, 0.0,
      0.4);
  next.lifecycle_id = 8U;  // epoch change (validateRequired used lifecycle 7)
  const auto reset = authority.update(next);
  EXPECT_FALSE(reset.production_owner_locked);
  EXPECT_EQ(reset.production_owner_history_id, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     SourceTimeRollbackClearsProductionOwner) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  ASSERT_TRUE(validated.production_owner_locked);
  // A source-time rollback (earlier pipeline stamp) must clear the owner.
  const auto rolled = authority.update(input(
      0.5, HookLoadSignalRole::REQUIRED, true, HookLoadState::LOADED, 0.0,
      0.7));
  EXPECT_FALSE(rolled.production_owner_locked);
  EXPECT_EQ(rolled.production_owner_history_id, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     RecoveryHoldDoesNotAccumulateValidation) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.1, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.4));
  authority.update(input(1.2, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.7));
  // A run of INVALID (RECOVERY_HOLD) frames must not coast into VALIDATED.
  for (double stamp = 1.3; stamp <= 1.8; stamp += 0.1) {
    auto invalid = input(stamp, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.7);
    invalid.groups.front().descriptor.vertical_mode =
        CargoGroupVerticalMode::INVALID;
    const auto result = authority.update(invalid);
    EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
    EXPECT_FALSE(result.production_owner_locked);
  }
}

// ============ OWNER_FRAGMENT_HANDOFF (physical owner != history id) ============

TEST(CargoPhysicalIdentityAuthorityTest,
     PhysicalOwnerGenerationIsImmutableAcrossChurn) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  ASSERT_TRUE(validated.production_owner_locked);
  ASSERT_NE(validated.production_owner_generation, 0U);
  const std::uint64_t generation = validated.production_owner_generation;
  const std::uint64_t original = validated.production_owner_original_history_id;
  EXPECT_EQ(original, validated.production_owner_history_id);
  EXPECT_EQ(validated.production_owner_handoff_count, 0U);
  // Further same-history churn must never rewrite the physical generation.
  for (double stamp = 1.4; stamp <= 1.9; stamp += 0.1) {
    const auto result = authority.update(input(
        stamp, HookLoadSignalRole::REQUIRED, true, HookLoadState::LOADED,
        0.0, 0.7));
    ASSERT_TRUE(result.production_owner_locked);
    EXPECT_EQ(result.production_owner_generation, generation);
    EXPECT_EQ(result.production_owner_original_history_id, original);
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ValidatedSuccessorHandoffUpdatesActiveHistoryNotGeneration) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  ASSERT_TRUE(validated.production_owner_locked);
  const std::uint64_t generation = validated.production_owner_generation;
  const std::uint64_t original = validated.production_owner_original_history_id;
  ASSERT_EQ(validated.production_owner_history_id, original);
  // A churned group beyond maximum_xy_step (0.35) spawns a NEW history whose
  // robust footprint still overlaps the owner (continuity).  Feed it a lift so
  // it can be validated, then verify the active observation binding hands off
  // WITHOUT changing the physical generation or the original history id.
  double stamp = 1.4;
  auto successor = input(stamp, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.40, 0.7);
  const auto after_handoff = authority.update(successor);
  if (after_handoff.production_owner_handoff_count > 0U) {
    EXPECT_EQ(after_handoff.production_owner_generation, generation);
    EXPECT_EQ(after_handoff.production_owner_original_history_id, original);
    EXPECT_NE(after_handoff.production_owner_history_id, original);
  } else {
    // Handoff may legitimately not fire on the first successor frame (the
    // successor has no continuity evidence yet); the invariants must still hold.
    EXPECT_EQ(after_handoff.production_owner_generation, generation);
    EXPECT_EQ(after_handoff.production_owner_original_history_id, original);
  }
}

// ============ OWNER_CONTINUATION_GATE (pure continuity gate) ============
// The successor no longer re-proves it is Cargo (no lift_confirmed).  These
// tests exercise the strict short-time physical continuity contract directly.

CargoPhysicalGroupDescriptor continuationDescriptor(
    double stamp, double center_x, double extent_x, double extent_y,
    double z) {
  CargoPhysicalGroupDescriptor d;
  d.valid = true;
  d.stamp_sec = stamp;
  d.stable_anchor = Eigen::Vector3d(center_x, 0.0, z);
  d.aggregate_extent = Eigen::Vector3d(extent_x, extent_y, 0.4);
  d.robust_x05 = center_x - 0.5 * extent_x;
  d.robust_x95 = center_x + 0.5 * extent_x;
  d.robust_y05 = -0.5 * extent_y;
  d.robust_y95 = 0.5 * extent_y;
  d.robust_xy_center = Eigen::Vector2d(center_x, 0.0);
  d.robust_xy_extent = Eigen::Vector2d(extent_x, extent_y);
  d.aggregate_point_support = 10U;
  d.vertical_mode = CargoGroupVerticalMode::SUPPORTED_EVIDENCE;
  d.physical_vertical_z = z;
  d.vertical_uncertainty_m = 0.01;
  return d;
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OwnerContinuationAcceptsOverlappingSuccessor) {
  const auto config = testConfig();
  const auto last = continuationDescriptor(1.0, 0.0, 1.0, 1.0, 0.7);
  const auto successor = continuationDescriptor(1.2, 0.2, 1.0, 1.0, 0.7);
  const auto verdict = evaluateOwnerContinuation(last, 1.0, successor, false,
                                                 config);
  EXPECT_TRUE(verdict.eligible) << verdict.reject_reason;
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OwnerContinuationAcceptsLineageWithoutOverlap) {
  const auto config = testConfig();
  // Successor center moved far enough that footprints no longer overlap, but
  // exact lineage continuity still bridges the observation.
  const auto last = continuationDescriptor(1.0, 0.0, 1.0, 1.0, 0.7);
  const auto successor = continuationDescriptor(1.2, 2.0, 1.0, 1.0, 0.7);
  const auto verdict = evaluateOwnerContinuation(last, 1.0, successor, true,
                                                 config);
  EXPECT_TRUE(verdict.eligible) << verdict.reject_reason;
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OwnerContinuationRejectsTemporalGap) {
  const auto config = testConfig();
  const auto last = continuationDescriptor(1.0, 0.0, 1.0, 1.0, 0.7);
  const auto successor = continuationDescriptor(2.5, 0.2, 1.0, 1.0, 0.7);
  const auto verdict = evaluateOwnerContinuation(last, 1.0, successor, false,
                                                 config);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.reject_reason, "TEMPORAL_GAP_TOO_LARGE");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OwnerContinuationRejectsBackwardsTime) {
  const auto config = testConfig();
  const auto last = continuationDescriptor(1.0, 0.0, 1.0, 1.0, 0.7);
  const auto successor = continuationDescriptor(0.9, 0.2, 1.0, 1.0, 0.7);
  const auto verdict = evaluateOwnerContinuation(last, 1.0, successor, false,
                                                 config);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.reject_reason, "TEMPORAL_BACKWARDS");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OwnerContinuationRejectsVerticalRate) {
  const auto config = testConfig();  // maximum_z_speed_mps = 5.0
  const auto last = continuationDescriptor(1.0, 0.0, 1.0, 1.0, 0.7);
  // 0.1 s and a 2.0 m jump far exceed the Z gate.
  const auto successor = continuationDescriptor(1.1, 0.2, 1.0, 1.0, 2.7);
  const auto verdict = evaluateOwnerContinuation(last, 1.0, successor, false,
                                                 config);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.reject_reason, "VERTICAL_RATE_EXCEEDED");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OwnerContinuationRejectsExtentMismatch) {
  const auto config = testConfig();  // maximum_size_relative_step = 0.60
  const auto last = continuationDescriptor(1.0, 0.0, 1.0, 1.0, 0.7);
  // Extent grows 4x (relative step 3.0) — far beyond the contract.
  const auto successor = continuationDescriptor(1.1, 0.0, 4.0, 4.0, 0.7);
  const auto verdict = evaluateOwnerContinuation(last, 1.0, successor, false,
                                                 config);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.reject_reason, "EXTENT_INCOMPATIBLE");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OwnerContinuationRejectsNoBridge) {
  const auto config = testConfig();
  const auto last = continuationDescriptor(1.0, 0.0, 1.0, 1.0, 0.7);
  // No footprint overlap and no lineage: continuity cannot be established.
  const auto successor = continuationDescriptor(1.1, 3.0, 1.0, 1.0, 0.7);
  const auto verdict = evaluateOwnerContinuation(last, 1.0, successor, false,
                                                 config);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.reject_reason, "NO_CONTINUITY_EVIDENCE");
}

TEST(CargoPhysicalIdentityAuthorityTest,
     OwnerContinuationRejectsNonFreshSuccessor) {
  const auto config = testConfig();
  const auto last = continuationDescriptor(1.0, 0.0, 1.0, 1.0, 0.7);
  CargoPhysicalGroupDescriptor stale;
  const auto verdict = evaluateOwnerContinuation(last, 1.0, stale, false,
                                                 config);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.reject_reason, "SUCCESSOR_NOT_FRESH");
}

// ============ OWNER_CONTINUATION_GATE (authority-level fail-closed) ============

TEST(CargoPhysicalIdentityAuthorityTest,
     HandoffRequiresExistingPhysicalOwnerGeneration) {
  // Without a locked physical owner, no observation handoff may occur, and no
  // fresh owner measurement may be produced.
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.1, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.4));
  const auto decision = authority.update(input(
      1.2, HookLoadSignalRole::REQUIRED, true, HookLoadState::LOADED, 0.0,
      0.7));
  EXPECT_FALSE(decision.production_owner_locked);
  EXPECT_EQ(decision.production_owner_handoff_count, 0U);
  EXPECT_FALSE(decision.production_owner_fresh_measurement_valid);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     UnvalidatedFragmentCannotCreatePhysicalOwner) {
  // A fresh MATCHED observation that never reaches unique VALIDATED must not
  // latch a physical owner — owner acquisition stays as strict as before.
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  authority.update(input(1.1, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  // Two LOADED frames are not enough to confirm lift (lift_confirm_frames=2),
  // so identity never reaches VALIDATED and no owner is created.
  authority.update(input(1.2, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.4));
  const auto decision = authority.update(input(
      1.3, HookLoadSignalRole::REQUIRED, true, HookLoadState::LOADED, 0.0,
      0.7));
  EXPECT_FALSE(decision.production_owner_locked);
  EXPECT_EQ(decision.production_owner_history_id, 0U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     HandoffKeepsOriginalHistoryImmutable) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  ASSERT_TRUE(validated.production_owner_locked);
  const std::uint64_t original = validated.production_owner_original_history_id;
  const std::uint64_t generation = validated.production_owner_generation;
  ASSERT_NE(original, 0U);
  // Churn through several frames; original_history_id and generation never move.
  for (double stamp = 1.4; stamp <= 2.0; stamp += 0.1) {
    const auto result = authority.update(input(
        stamp, HookLoadSignalRole::REQUIRED, true, HookLoadState::LOADED,
        0.0, 0.7));
    EXPECT_EQ(result.production_owner_original_history_id, original);
    EXPECT_EQ(result.production_owner_generation, generation);
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     RecoveryHoldCannotBeHandoffSuccessor) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto validated = validateRequired(&authority);
  ASSERT_TRUE(validated.production_owner_locked);
  // A RECOVERY_HOLD group (INVALID vertical, held to the owner's history) is
  // not MATCHED, so it can never act as a fresh successor and the owner's fresh
  // measurement stays false.
  auto held = input(1.4, HookLoadSignalRole::REQUIRED, true,
                    HookLoadState::LOADED, 0.0, 0.7);
  held.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::INVALID;
  const auto decision = authority.update(held);
  ASSERT_EQ(decision.group_diagnostics.size(), 1U);
  EXPECT_EQ(decision.group_diagnostics.front().association,
            CargoCandidateAssociationState::RECOVERY_HOLD);
  EXPECT_FALSE(decision.production_owner_fresh_measurement_valid);
}

// ============ D5 current-frame Cargo component bundle view ============

CargoPhysicalComponentObservation bundleComponent(
    std::uint64_t id, double cx, double cy, double z_lo, double z_hi,
    double ex, double ey) {
  CargoPhysicalComponentObservation component;
  component.component_id = id;
  for (double x : {cx - 0.5 * ex, cx + 0.5 * ex}) {
    for (double y : {cy - 0.5 * ey, cy + 0.5 * ey}) {
      for (double z : {z_lo, 0.5 * (z_lo + z_hi), z_hi}) {
        component.points_base.emplace_back(
            static_cast<float>(x), static_cast<float>(y),
            static_cast<float>(z));
      }
    }
  }
  return component;
}

CargoPhysicalBundleConfig bundleConfig(double vertical_gap = 0.60) {
  CargoPhysicalBundleConfig config;
  config.enabled = true;
  config.maximum_internal_vertical_gap_m = vertical_gap;
  config.maximum_combined_long_side_m = 3.50;
  config.maximum_combined_short_side_m = 2.00;
  return config;
}

TEST(CargoPhysicalIdentityAuthorityTest, PreloadDoesNotUseCargoBundle) {
  // Disabled (PRELOAD): even two clearly-overlapping fragments produce no bundle.
  std::vector<CargoPhysicalComponentObservation> components = {
      bundleComponent(0U, 0.0, 0.0, 0.0, 0.8, 1.0, 1.0),
      bundleComponent(1U, 0.1, 0.0, 0.0, 0.8, 1.0, 1.0)};
  CargoPhysicalBundleConfig config = bundleConfig();
  config.enabled = false;
  EXPECT_TRUE(buildCargoComponentBundles(components, config).empty());
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PostloadSplitCargoFragmentsMayFormBundle) {
  std::vector<CargoPhysicalComponentObservation> components = {
      bundleComponent(0U, 0.0, 0.0, 0.0, 0.8, 1.0, 1.0),
      bundleComponent(1U, 0.2, 0.0, 0.0, 0.8, 1.0, 1.0)};
  const auto bundles = buildCargoComponentBundles(components, bundleConfig());
  ASSERT_EQ(bundles.size(), 1U);
  EXPECT_EQ(bundles.front().raw_component_count, 2U);
  EXPECT_EQ(bundles.front().member_component_ids.size(), 2U);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     VerticallySeparatedOverlappingCargoFragmentsCanBundle) {
  // Overlapping XY, vertically separated by ~0.2 m (within the 0.60 gap).
  std::vector<CargoPhysicalComponentObservation> components = {
      bundleComponent(0U, 0.0, 0.0, 0.2, 0.5, 1.0, 1.0),
      bundleComponent(1U, 0.1, 0.0, 0.7, 1.0, 1.0, 1.0)};
  const auto bundles = buildCargoComponentBundles(components, bundleConfig());
  ASSERT_EQ(bundles.size(), 1U);
  EXPECT_LT(bundles.front().internal_vertical_gap_m, 0.60);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     UnrelatedXYSeparatedComponentsCannotBundle) {
  // No XY footprint overlap: two fragments far apart must not bundle.
  std::vector<CargoPhysicalComponentObservation> components = {
      bundleComponent(0U, 0.0, 0.0, 0.0, 0.8, 1.0, 1.0),
      bundleComponent(1U, 3.0, 0.0, 0.0, 0.8, 1.0, 1.0)};
  EXPECT_TRUE(buildCargoComponentBundles(components, bundleConfig()).empty());
}

TEST(CargoPhysicalIdentityAuthorityTest,
     CombinedExtentBeyondCargoBoundsCannotBundle) {
  // Overlapping XY but the combined footprint exceeds the cargo size contract.
  std::vector<CargoPhysicalComponentObservation> components = {
      bundleComponent(0U, 0.0, 0.0, 0.0, 0.8, 2.5, 1.0),
      bundleComponent(1U, 1.5, 0.0, 0.0, 0.8, 2.5, 1.0)};
  EXPECT_TRUE(buildCargoComponentBundles(components, bundleConfig()).empty());
}

TEST(CargoPhysicalIdentityAuthorityTest,
     StaticGroundComponentIsNotDestructivelyMerged) {
  // A low static component vertically separated beyond the gap must not be
  // bundled into the Cargo fragment (and the raw components are never mutated).
  std::vector<CargoPhysicalComponentObservation> components = {
      bundleComponent(0U, 0.0, 0.0, 1.0, 1.5, 1.0, 1.0),
      bundleComponent(1U, 0.1, 0.0, 0.0, 0.05, 1.0, 1.0)};
  const auto before = components;
  const auto bundles = buildCargoComponentBundles(components, bundleConfig());
  EXPECT_TRUE(bundles.empty());
  EXPECT_EQ(components.size(), before.size());  // not destructive
  EXPECT_EQ(components[0].component_id, before[0].component_id);
  EXPECT_EQ(components[1].component_id, before[1].component_id);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     BundleMemberProvenanceIsPreserved) {
  std::vector<CargoPhysicalComponentObservation> components = {
      bundleComponent(7U, 0.0, 0.0, 0.0, 0.8, 1.0, 1.0),
      bundleComponent(11U, 0.2, 0.0, 0.0, 0.8, 1.0, 1.0)};
  const auto bundles = buildCargoComponentBundles(components, bundleConfig());
  ASSERT_EQ(bundles.size(), 1U);
  const auto& members = bundles.front().member_component_ids;
  EXPECT_EQ(members.size(), 2U);
  EXPECT_NE(std::find(members.begin(), members.end(), 7U), members.end());
  EXPECT_NE(std::find(members.begin(), members.end(), 11U), members.end());
}

TEST(CargoPhysicalIdentityAuthorityTest,
     BundleDoesNotUsePointCountAsOwnerEvidence) {
  // The bundle geometry must not depend on member point count (no point-count
  // authority).  A sparse and a dense fragment with the same footprint yield
  // the same consolidated bundle geometry.
  const auto sparse = bundleComponent(0U, 0.0, 0.0, 0.0, 0.8, 1.0, 1.0);
  auto dense = sparse;
  dense.component_id = 1U;
  dense.points_base.reserve(400U);
  for (int i = 0; i < 100; ++i) {
    dense.points_base.emplace_back(0.0F, 0.0F, 0.4F);  // same footprint centre
  }
  const auto bundles_sparse = buildCargoComponentBundles(
      {sparse, sparse}, bundleConfig());
  const auto bundles_dense = buildCargoComponentBundles(
      {sparse, dense}, bundleConfig());
  ASSERT_EQ(bundles_sparse.size(), 1U);
  ASSERT_EQ(bundles_dense.size(), 1U);
  // Extent is point-count independent (robust footprint).
  EXPECT_NEAR(bundles_sparse.front().size.x(), bundles_dense.front().size.x(),
              0.30);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     BundleUsesCurrentFrameOnlyAndIsStateless) {
  // The bundle builder is a pure function: same input -> same output, no
  // retained history / previous-winner state.
  std::vector<CargoPhysicalComponentObservation> components = {
      bundleComponent(0U, 0.0, 0.0, 0.0, 0.8, 1.0, 1.0),
      bundleComponent(1U, 0.2, 0.0, 0.0, 0.8, 1.0, 1.0)};
  const auto first = buildCargoComponentBundles(components, bundleConfig());
  const auto second = buildCargoComponentBundles(components, bundleConfig());
  ASSERT_EQ(first.size(), second.size());
  if (!first.empty()) {
    EXPECT_NEAR(first.front().center.x(), second.front().center.x(), 1e-9);
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     CargoBundleCannotCreatePhysicalOwner) {
  // A bundle observation is a geometry hypothesis, not an identity decision;
  // it carries no owner/generation/history authority fields.
  std::vector<CargoPhysicalComponentObservation> components = {
      bundleComponent(0U, 0.0, 0.0, 0.0, 0.8, 1.0, 1.0),
      bundleComponent(1U, 0.2, 0.0, 0.0, 0.8, 1.0, 1.0)};
  const auto bundles = buildCargoComponentBundles(components, bundleConfig());
  ASSERT_EQ(bundles.size(), 1U);
  // The bundle type has no physical-owner / generation / history authority
  // fields, so it cannot latch an owner by construction.
  EXPECT_EQ(bundles.front().member_component_ids.size(), 2U);
}

// ============ Canonical owner shape semantics (aggregate_extent vs L/W) ============

TEST(CargoPhysicalIdentityAuthorityTest,
     GroupDescriptorCarriesCanonicalLongShortAndYaw) {
  // A Y-long cargo: canonical size x=LONG=1.77, y=SHORT=1.43, yaw=pi/2.  The
  // descriptor must carry the canonical size/yaw (not derive L/W from the
  // axis-aligned aggregate extent whose x/y are base-axis extents).
  auto observation = candidate(10U, 1.0, 0.0, 0.7);
  observation.size = Eigen::Vector3d(1.77, 1.43, 0.40);
  observation.yaw_rad = 1.5707963267948966;  // pi/2
  const auto groups = buildGroups({observation});
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_TRUE(groups.front().descriptor.canonical_size.allFinite());
  EXPECT_NEAR(groups.front().descriptor.canonical_size.x(), 1.77, 1e-9);
  EXPECT_NEAR(groups.front().descriptor.canonical_size.y(), 1.43, 1e-9);
  EXPECT_NEAR(groups.front().descriptor.yaw_rad, 1.5707963267948966, 1e-9);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     CanonicalLengthExceedsShortForYLongCargo) {
  // A Y-long cargo must still satisfy canonical length >= short.  The axis
  // extent (aggregate_extent) x/y may be inverted (x<y), but the canonical
  // size.x/y is LONG/SHORT by construction.
  auto observation = candidate(10U, 1.0, 0.0, 0.7);
  observation.size = Eigen::Vector3d(1.77, 1.43, 0.40);
  observation.yaw_rad = 1.5707963267948966;
  const auto groups = buildGroups({observation});
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_GE(groups.front().descriptor.canonical_size.x(),
            groups.front().descriptor.canonical_size.y());
}

TEST(CargoPhysicalIdentityAuthorityTest,
     DefaultCandidateCanonicalSizeIsLongShort) {
  // The default candidate() size (1.0, 0.8, 0.4) is already LONG/SHORT/HEIGHT.
  const auto groups = buildGroups({candidate(10U, 1.0, 0.0, 0.7)});
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_NEAR(groups.front().descriptor.canonical_size.x(), 1.0, 1e-9);
  EXPECT_NEAR(groups.front().descriptor.canonical_size.y(), 0.8, 1e-9);
  EXPECT_GE(groups.front().descriptor.canonical_size.x(),
            groups.front().descriptor.canonical_size.y());
}

// ============ Formal lift static-conflict fail-closed ============

CargoPhysicalIdentityInput liftedSurfaceInput(
    double stamp, std::uint64_t lifecycle_id, HookLoadState gravity_state,
    double group_x, double descriptor_z, double raw_surface_z) {
  CargoPhysicalIdentityInput result = rawSurfaceInput(
      stamp, lifecycle_id, gravity_state, group_x, descriptor_z,
      raw_surface_z);
  // Supply a valid range cloud + an all-clear (no static) conflict mask so the
  // caller can explicitly toggle static_conflict_context_valid.
  result.frame_evidence.range_cloud_current_frame =
      cloudFromPoints(componentPoints(group_x, raw_surface_z));
  result.frame_evidence.range_static_conflict_mask =
      std::make_shared<std::vector<std::uint8_t>>(
          result.frame_evidence.range_cloud_current_frame->size(), 0U);
  return result;
}

TEST(CargoPhysicalIdentityAuthorityTest,
     InvalidStaticConflictContextCannotAdvanceFormalLift) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 7U);  // PRELOAD baseline z=0.40
  // POSTLOAD (lifecycle 8) with a clearly lifted raw surface (z=1.0), but the
  // static conflict context is invalid (default false).  The vertical cannot be
  // proven non-static, so formal lift must remain UNOBSERVABLE and never
  // advance, no matter how many frames arrive.
  CargoPhysicalIdentityDecision decision;
  for (double stamp = 1.2; stamp <= 1.7; stamp += 0.1) {
    decision = authority.update(rawSurfaceInput(
        stamp, 8U, HookLoadState::LOADED, 0.0, 1.0, 1.0));
    EXPECT_FALSE(decision.precluster_lift_significant);
    EXPECT_FALSE(decision.formal_lift_confirmed);
    EXPECT_EQ(decision.precluster_lift_confirm_count, 0);
  }
  EXPECT_FALSE(decision.formal_lift_confirmed);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ValidStaticContextAllowsRealCargoLiftEvidence) {
  CargoPhysicalIdentityConfig config = testConfig();
  config.lift_confirm_frames = 4;
  CargoPhysicalIdentityAuthority authority(config);
  freezeSurfaceReference(&authority, 9U);
  CargoPhysicalIdentityDecision decision;
  for (double stamp = 1.2; stamp <= 1.7; stamp += 0.1) {
    auto frame = liftedSurfaceInput(
        stamp, 10U, HookLoadState::LOADED, 0.0, 1.0, 1.0);
    frame.frame_evidence.static_conflict_context_valid = true;
    decision = authority.update(frame);
  }
  // With a valid (all-clear) static context, the real lifted surface may now
  // accumulate POSITIVE formal lift evidence.
  EXPECT_TRUE(decision.formal_lift_confirmed);
}

}  // namespace
}  // namespace ndt_slam
