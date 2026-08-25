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
     AuxiliaryGravityEmptyPreservesStrictLidarExistenceSemantics) {
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
  const auto confirmed = authority.update(input(
      1.4, HookLoadSignalRole::AUXILIARY, true,
      HookLoadState::EMPTY, 0.0, 0.7));
  EXPECT_TRUE(confirmed.cargo_exists);
  EXPECT_EQ(confirmed.existence_source, CargoExistenceSource::STRICT_LIDAR);
  EXPECT_EQ(confirmed.required_lift_confirm_frames, 4);
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
     StaticLowAndLiftingCargoNeverShareHistory) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  auto frame = input(1.0, HookLoadSignalRole::REQUIRED, true,
                     HookLoadState::EMPTY, 0.0, 0.3);
  frame.groups = buildGroups({
      candidate(1U, 1.0, -0.20, 0.30, {1U}),
      candidate(2U, 1.0, 0.20, 0.27, {2U})});
  authority.update(frame);
  for (int index = 1; index <= 3; ++index) {
    const double stamp = 1.0 + 0.1 * index;
    const double cargo_z = index == 1 ? 0.27 : 0.70;
    frame = input(stamp, HookLoadSignalRole::REQUIRED, true,
                  HookLoadState::LOADED, 0.0, cargo_z);
    frame.groups = buildGroups({
        candidate(10U + index, stamp, -0.20, 0.30, {8U}),
        candidate(20U + index, stamp, 0.20, cargo_z, {9U})});
    const auto result = authority.update(frame);
    if (index == 3) {
      ASSERT_EQ(result.group_diagnostics.size(), 2U);
      const auto& first = result.group_diagnostics[0];
      const auto& second = result.group_diagnostics[1];
      EXPECT_NE(first.matched_history_id, second.matched_history_id);
      const auto& static_group = first.descriptor.physical_vertical_z <
              second.descriptor.physical_vertical_z ? first : second;
      EXPECT_FALSE(static_group.lift_confirmed);
      EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
    }
  }
}

TEST(CargoPhysicalIdentityAuthorityTest,
     ContinuityOnlyFrameCannotAdvanceLiftConfirmation) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
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

TEST(CargoPhysicalIdentityAuthorityTest,
     InvalidRobustZBreaksHistoryInsteadOfUsingPreviousZ) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  authority.update(input(1.0, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::EMPTY, 0.0, 0.4));
  auto invalid = input(1.1, HookLoadSignalRole::REQUIRED, true,
                       HookLoadState::LOADED, 0.0, 0.4);
  invalid.groups.front().descriptor.valid = false;
  invalid.groups.front().descriptor.vertical_mode =
      CargoGroupVerticalMode::INVALID;
  const auto result = authority.update(invalid);
  EXPECT_EQ(result.association, CargoCandidateAssociationState::NEW_HISTORY);
  ASSERT_EQ(result.group_diagnostics.size(), 1U);
  EXPECT_EQ(result.group_diagnostics.front().association_reject_reason,
            "VERTICAL_INVALID");
  EXPECT_EQ(result.group_diagnostics.front().matched_history_id, 0U);
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
  authority.update(frame);
  for (int index = 1; index <= 3; ++index) {
    const double stamp = 1.0 + 0.1 * index;
    const double z = index == 1 ? 0.30 : 0.70;
    frame = input(stamp, HookLoadSignalRole::REQUIRED, true,
                  HookLoadState::LOADED, 0.0, z);
    frame.groups = buildGroups({
        candidate(10U + index, stamp, -1.0, z, {10U}),
        candidate(20U + index, stamp, 1.0, z, {20U})});
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

}  // namespace
}  // namespace ndt_slam
