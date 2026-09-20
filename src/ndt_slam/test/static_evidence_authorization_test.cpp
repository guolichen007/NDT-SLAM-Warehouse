#include "ndt_slam/static_evidence_authorization.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

TEST(StaticEvidenceAuthorizationTest,
     UnverifiedStaticCannotBindFormalOrigin) {
  const auto gate = authorizeStaticEvidence(
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN);
  EXPECT_TRUE(gate.diagnostic_height_allowed);
  EXPECT_FALSE(gate.formal_origin_authorized);
}

TEST(StaticEvidenceAuthorizationTest,
     UnverifiedStaticCannotCountAsIndependentThicknessSource) {
  EXPECT_FALSE(authorizeStaticEvidence(
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN)
      .formal_thickness_authorized);
}

TEST(StaticEvidenceAuthorizationTest,
     UnverifiedStaticCannotProduceOfficialHazard) {
  EXPECT_FALSE(authorizeStaticEvidence(
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN)
      .official_static_risk_authorized);
}

TEST(StaticEvidenceAuthorizationTest,
     UnverifiedStaticCannotAuthorizeClear) {
  EXPECT_FALSE(authorizeStaticEvidence(
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN)
      .official_clear_authorized);
}

TEST(StaticEvidenceAuthorizationTest, FormalAuthoritiesEnableAllGates) {
  for (const auto authority : {
           StaticEvidenceAuthority::RUNTIME_MATURE,
           StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE}) {
    const auto gate = authorizeStaticEvidence(authority);
    EXPECT_TRUE(gate.formal_origin_authorized);
    EXPECT_TRUE(gate.formal_thickness_authorized);
    EXPECT_TRUE(gate.official_static_risk_authorized);
    EXPECT_TRUE(gate.official_clear_authorized);
  }
}

// Guard C static-conflict context identity is bound to the static-evidence
// epoch, never to the pose map-rebuild generation. These tests pin that the
// two lifecycle domains are not cross-compared.

TEST(StaticEvidenceAuthorizationTest, MatchingStaticEvidenceEpochIsAccepted) {
  const auto ctx = evaluateStaticConflictContextAuthority(
      /*snapshot_present=*/true, /*snapshot_generation=*/7,
      /*expected_static_evidence_epoch=*/7,
      StaticEvidenceAuthority::RUNTIME_MATURE,
      /*pose_authority_valid=*/true);
  EXPECT_TRUE(ctx.valid);
  EXPECT_EQ(ctx.reason, "none");
}

TEST(StaticEvidenceAuthorizationTest, StaticEvidenceEpochMismatchFailsClosed) {
  const auto ctx = evaluateStaticConflictContextAuthority(
      true, 7, 8, StaticEvidenceAuthority::RUNTIME_MATURE, true);
  EXPECT_FALSE(ctx.valid);
  EXPECT_EQ(ctx.reason, "static_evidence_epoch_mismatch");
}

TEST(StaticEvidenceAuthorizationTest, MissingStaticSnapshotFailsClosed) {
  const auto ctx = evaluateStaticConflictContextAuthority(
      false, 0, 7, StaticEvidenceAuthority::RUNTIME_MATURE, true);
  EXPECT_FALSE(ctx.valid);
  EXPECT_EQ(ctx.reason, "static_snapshot_missing");
}

TEST(StaticEvidenceAuthorizationTest, UnverifiedLoadedCleanStillFailsClosed) {
  const auto ctx = evaluateStaticConflictContextAuthority(
      true, 7, 7, StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN, true);
  EXPECT_FALSE(ctx.valid);
  EXPECT_EQ(ctx.reason, "static_authority_unverified");
}

TEST(StaticEvidenceAuthorizationTest, InvalidPoseAuthorityStillFailsClosed) {
  const auto ctx = evaluateStaticConflictContextAuthority(
      true, 7, 7, StaticEvidenceAuthority::RUNTIME_MATURE, false);
  EXPECT_FALSE(ctx.valid);
  EXPECT_EQ(ctx.reason, "pose_authority_invalid");
}

}  // namespace
}  // namespace ndt_slam
