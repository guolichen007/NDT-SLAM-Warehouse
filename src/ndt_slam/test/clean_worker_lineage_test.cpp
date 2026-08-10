#include "ndt_slam/clean_worker_lineage.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CleanWorkerLineage lineage() {
  CleanWorkerLineage value;
  value.localization_continuity_generation = 2U;
  value.localization_map_generation = 3U;
  value.localization_map_uuid = "map-a";
  value.lifecycle_epoch = 4U;
  value.static_evidence_epoch = 5U;
  value.source_accepted_pose_generation = 6U;
  value.source_objects_version = 7U;
  return value;
}

TEST(CleanWorkerLineageTest, CurrentObjectsCanInstall) {
  const auto value = lineage();
  const auto decision = evaluateCleanWorkerLineage(true, value, value);
  EXPECT_EQ(decision.action,
            CleanWorkerLineageAction::INSTALL_CURRENT);
  EXPECT_TRUE(decision.static_observation_authorized);
}

TEST(CleanWorkerLineageTest, InitialLifecycleEpochIsAValidDomainValue) {
  auto value = lineage();
  value.lifecycle_epoch = 0U;
  const auto decision = evaluateCleanWorkerLineage(true, value, value);
  EXPECT_EQ(decision.action,
            CleanWorkerLineageAction::INSTALL_CURRENT);
  EXPECT_TRUE(decision.static_observation_authorized);
}

TEST(CleanWorkerLineageTest, NewAcceptedPoseDoesNotStarveHistory) {
  const auto source = lineage();
  auto current = source;
  current.source_accepted_pose_generation = 100U;
  current.source_objects_version = 8U;
  const auto decision = evaluateCleanWorkerLineage(
      true, source, current);
  EXPECT_EQ(decision.action,
            CleanWorkerLineageAction::PUBLISH_SNAPSHOT_ONLY);
  EXPECT_TRUE(decision.static_observation_authorized);
}

TEST(CleanWorkerLineageTest, IdentityMismatchHasZeroMutationAuthority) {
  const auto source = lineage();
  auto current = source;
  current.localization_map_uuid = "map-b";
  const auto decision = evaluateCleanWorkerLineage(
      true, source, current);
  EXPECT_EQ(decision.action, CleanWorkerLineageAction::DISCARD);
  EXPECT_FALSE(decision.static_observation_authorized);
}

TEST(CleanWorkerLineageTest, FutureObjectsVersionIsRejected) {
  auto source = lineage();
  const auto current = source;
  ++source.source_objects_version;
  const auto decision = evaluateCleanWorkerLineage(
      true, source, current);
  EXPECT_EQ(decision.action, CleanWorkerLineageAction::DISCARD);
  EXPECT_FALSE(decision.static_observation_authorized);
}

}  // namespace
}  // namespace ndt_slam
