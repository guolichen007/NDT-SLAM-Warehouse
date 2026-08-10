#include "ndt_slam/map_write_authority.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

MapWriteAuthorityEvidence validEvidence() {
  MapWriteAuthorityEvidence evidence;
  evidence.accepted_pose_valid = true;
  evidence.ndt_accepted = true;
  evidence.prediction_only = false;
  evidence.map_commit_quality_valid = true;
  evidence.localization_quarantined = false;
  evidence.pose_finite = true;
  evidence.source_pose_generation = 10U;
  evidence.latest_pose_generation = 10U;
  evidence.source_stamp_sec = 1.0;
  evidence.latest_stamp_sec = 1.0;
  evidence.source_continuity_generation = 2U;
  evidence.current_continuity_generation = 2U;
  evidence.source_map_generation = 3U;
  evidence.current_map_generation = 3U;
  evidence.source_map_uuid = "map-a";
  evidence.current_map_uuid = "map-a";
  evidence.source_lifecycle_epoch = 4U;
  evidence.current_lifecycle_epoch = 4U;
  evidence.source_static_epoch = 5U;
  evidence.current_static_epoch = 5U;
  return evidence;
}

TEST(MapWriteAuthorityTest, CurrentAcceptedPoseCanSubmit) {
  EXPECT_TRUE(evaluateMapWriteAuthority(validEvidence()).authorized);
}

TEST(MapWriteAuthorityTest, PredictionAndRejectNeverWrite) {
  auto evidence = validEvidence();
  evidence.prediction_only = true;
  EXPECT_FALSE(evaluateMapWriteAuthority(evidence).authorized);
  evidence.prediction_only = false;
  evidence.ndt_accepted = false;
  EXPECT_FALSE(evaluateMapWriteAuthority(evidence).authorized);
}

TEST(MapWriteAuthorityTest, AsyncOlderPoseRemainsValidWithinContinuity) {
  auto evidence = validEvidence();
  evidence.phase = MapWriteAuthorityPhase::ASYNC_COMPLETE;
  evidence.latest_pose_generation = 12U;
  evidence.latest_stamp_sec = 1.2;
  const auto decision = evaluateMapWriteAuthority(evidence);
  EXPECT_TRUE(decision.authorized);
  EXPECT_EQ(decision.reason, "accepted_pose_lineage_current");
}

TEST(MapWriteAuthorityTest, AsyncIdentityMismatchCannotWrite) {
  auto evidence = validEvidence();
  evidence.phase = MapWriteAuthorityPhase::ASYNC_COMPLETE;
  evidence.current_map_uuid = "map-b";
  EXPECT_FALSE(evaluateMapWriteAuthority(evidence).authorized);
  evidence.current_map_uuid = "map-a";
  ++evidence.current_continuity_generation;
  EXPECT_FALSE(evaluateMapWriteAuthority(evidence).authorized);
}

}  // namespace
}  // namespace ndt_slam
