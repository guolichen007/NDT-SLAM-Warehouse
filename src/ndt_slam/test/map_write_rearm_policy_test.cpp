#include "ndt_slam/map_write_rearm_policy.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

MapWriteRearmEvidence evidence(double stamp) {
  MapWriteRearmEvidence value;
  value.stamp_sec = stamp;
  value.pose_finite = true;
  value.ndt_accepted = true;
  value.prediction_only = false;
  value.relocalization_job_active = false;
  value.map_uuid = "map-a";
  value.map_generation = 4U;
  value.continuity_generation = 2U;
  return value;
}

TEST(MapWriteRearmPolicyTest, RequiresFramesDurationAndStableIdentity) {
  MapWriteRearmPolicy policy({3, 2.0});
  EXPECT_FALSE(policy.update(evidence(1.0)).map_write_rearmed);
  EXPECT_FALSE(policy.update(evidence(2.0)).map_write_rearmed);
  EXPECT_TRUE(policy.update(evidence(3.0)).map_write_rearmed);
}

TEST(MapWriteRearmPolicyTest, RelocalizationOrIdentityChangeResetsWindow) {
  MapWriteRearmPolicy policy({2, 1.0});
  policy.update(evidence(1.0));
  auto invalid = evidence(2.0);
  invalid.relocalization_job_active = true;
  EXPECT_EQ(policy.update(invalid).accepted_frames, 0);
  policy.update(evidence(3.0));
  auto changed = evidence(4.0);
  changed.map_uuid = "map-b";
  EXPECT_EQ(policy.update(changed).accepted_frames, 0);
}

}  // namespace
}  // namespace ndt_slam
