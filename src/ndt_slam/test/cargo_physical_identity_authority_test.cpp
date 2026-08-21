#include "ndt_slam/cargo_physical_identity_authority.hpp"

#include <gtest/gtest.h>

#include <cstdint>
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

CargoPhysicalGroupObservation group(
    std::uint64_t id, double stamp, double x, double z,
    std::vector<std::uint64_t> members = {1U}) {
  auto groups = groupCargoPhysicalCandidates(
      {candidate(id, stamp, x, z, std::move(members))}, 0.05, 0.10);
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
  authority->update(input(1.1, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::LOADED, 0.0, 0.4));
  authority->update(input(1.2, HookLoadSignalRole::REQUIRED, true,
                          HookLoadState::LOADED, 0.0, 0.7));
  return authority->update(input(1.3, HookLoadSignalRole::REQUIRED, true,
                                 HookLoadState::LOADED, 0.0, 0.7));
}

TEST(CargoPhysicalIdentityAuthorityTest,
     PreloadBaselineUsedAcrossLoadEdge) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  const auto result = validateRequired(&authority);
  EXPECT_EQ(result.identity, CargoPhysicalIdentityState::VALIDATED);
  EXPECT_EQ(result.baseline_source,
            CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE);
  EXPECT_NEAR(result.baseline_z95, 0.4, 1.0e-9);
  EXPECT_NEAR(result.lift_delta_m, 0.3, 1.0e-9);
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
  conflict.groups = groupCargoPhysicalCandidates(
      {candidate(10U, 1.4, 0.0, 0.7, {1U, 2U}),
       candidate(11U, 1.4, 0.0, 0.7, {2U, 3U})}, 0.05, 0.10);
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
  const auto groups = groupCargoPhysicalCandidates(
      {candidate(1U, 1.0, 0.0, 0.5, {9U, 3U}),
       candidate(2U, 1.0, 0.01, 0.5, {3U, 9U})}, 0.05, 0.10);
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_EQ(groups.front().hypotheses.size(), 2U);
  EXPECT_TRUE(groups.front().geometry_resolved);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     GeometryHypothesisAmbiguityFailsClosed) {
  auto first = candidate(1U, 1.0, 0.0, 0.5, {3U, 9U});
  auto second = candidate(2U, 1.0, 0.0, 0.5, {3U, 9U});
  second.size.x() = 2.0;
  const auto groups = groupCargoPhysicalCandidates(
      {first, second}, 0.05, 0.10);
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_FALSE(groups.front().geometry_resolved);
}

TEST(CargoPhysicalIdentityAuthorityTest,
     AmbiguousAssociationDoesNotTransferEvidence) {
  CargoPhysicalIdentityAuthority authority(testConfig());
  validateRequired(&authority);
  auto ambiguous = input(1.4, HookLoadSignalRole::REQUIRED, true,
                         HookLoadState::LOADED, 0.0, 0.7);
  ambiguous.groups = groupCargoPhysicalCandidates(
      {candidate(1U, 1.4, 0.0, 0.7, {1U, 2U}),
       candidate(2U, 1.4, 0.0, 0.7, {2U, 3U})}, 0.05, 0.10);
  const auto result = authority.update(ambiguous);
  EXPECT_EQ(result.association, CargoCandidateAssociationState::AMBIGUOUS);
  EXPECT_NE(result.identity, CargoPhysicalIdentityState::VALIDATED);
}

}  // namespace
}  // namespace ndt_slam
