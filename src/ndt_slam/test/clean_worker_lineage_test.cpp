#include "ndt_slam/clean_worker_lineage.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CleanWorkerLineage lineage() {
  CleanWorkerLineage value;
  value.mapping_authority_epoch = 7U;
  value.localization_continuity_generation = 11U;
  value.localization_map_generation = 13U;
  value.localization_map_uuid = "map-uuid";
  value.map_rebuild_epoch = 17U;
  value.static_evidence_epoch = 19U;
  value.lifecycle_epoch = 23U;
  value.source_accepted_pose_generation = 29U;
  value.source_objects_version = 31U;
  return value;
}

TEST(CleanWorkerLineageTest, NewerAcceptedPoseDoesNotStarveHistory) {
  const CleanWorkerLineage source = lineage();
  CleanWorkerLineage current = source;
  current.source_accepted_pose_generation = 999U;
  current.source_objects_version = 32U;

  const auto decision = evaluateCleanWorkerLineage(true, source, current);
  EXPECT_EQ(decision.action,
            CleanWorkerLineageAction::PUBLISH_SNAPSHOT_ONLY);
  EXPECT_TRUE(decision.static_observation_authorized);
}

TEST(CleanWorkerLineageTest, CurrentObjectsCanInstallWorkingLayer) {
  const CleanWorkerLineage source = lineage();
  const auto decision = evaluateCleanWorkerLineage(true, source, source);
  EXPECT_EQ(decision.action, CleanWorkerLineageAction::INSTALL_CURRENT);
  EXPECT_TRUE(decision.static_observation_authorized);
}

TEST(CleanWorkerLineageTest, AuthorityAndContinuityAreStrict) {
  const CleanWorkerLineage source = lineage();
  CleanWorkerLineage current = source;
  ++current.mapping_authority_epoch;
  EXPECT_EQ(evaluateCleanWorkerLineage(true, source, current).action,
            CleanWorkerLineageAction::DISCARD);

  current = source;
  ++current.localization_continuity_generation;
  EXPECT_EQ(evaluateCleanWorkerLineage(true, source, current).action,
            CleanWorkerLineageAction::DISCARD);
}

TEST(CleanWorkerLineageTest, FutureObjectsVersionIsRejected) {
  CleanWorkerLineage source = lineage();
  const CleanWorkerLineage current = source;
  ++source.source_objects_version;
  const auto decision = evaluateCleanWorkerLineage(true, source, current);
  EXPECT_EQ(decision.action, CleanWorkerLineageAction::DISCARD);
  EXPECT_FALSE(decision.static_observation_authorized);
}

}  // namespace
}  // namespace ndt_slam
