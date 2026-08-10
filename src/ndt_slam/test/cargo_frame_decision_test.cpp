#include "ndt_slam/cargo_frame_decision.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoFrameDecision warningDecision() {
  CargoFrameDecision decision;
  decision.stamp_sec = 1.0;
  decision.cargo_identity_confirmed_this_frame = true;
  decision.cargo_identity_authorized = true;
  decision.cargo_lifecycle_id = 7U;
  decision.cargo_track_id = 11U;
  decision.positive_warning_confirmed_this_frame = true;
  decision.warning_code = 18;
  decision.authoritative_hazard_valid = true;
  decision.authoritative_warning_code = 18;
  decision.warning_cargo_lifecycle_id = 7U;
  decision.warning_cargo_track_id = 11U;
  decision.obstacle_track_id = 13U;
  return decision;
}

TEST(CargoFrameDecisionTest, IdentityAndDangerCommitAtomically) {
  const auto result = commitCargoFrameDecision(warningDecision());
  EXPECT_TRUE(result.authorized);
  EXPECT_EQ(result.status_code, 18);
  EXPECT_EQ(result.reason,
            "identity_and_positive_warning_committed_same_frame");
}

TEST(CargoFrameDecisionTest, MismatchedIdentityBecomesCode35) {
  CargoFrameDecision decision = warningDecision();
  decision.warning_cargo_track_id = 99U;
  const auto result = commitCargoFrameDecision(decision);
  EXPECT_FALSE(result.authorized);
  EXPECT_EQ(result.status_code, 35);
  EXPECT_EQ(result.reason,
            "authoritative_hazard_cargo_identity_mismatch");
}

TEST(CargoFrameDecisionTest, NonWarningPathDoesNotRequireIdentity) {
  CargoFrameDecision decision;
  decision.stamp_sec = 1.0;
  decision.warning_code = 34;
  const auto result = commitCargoFrameDecision(decision);
  EXPECT_TRUE(result.authorized);
  EXPECT_EQ(result.status_code, 34);
}

}  // namespace
}  // namespace ndt_slam
