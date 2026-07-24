#include "ndt_slam/pending_cargo_self_evidence.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoObbFootprint identityBox() {
  CargoObbFootprint box;
  box.valid = true;
  box.center_base = Eigen::Vector2f::Zero();
  box.length_m = 2.0F;
  box.width_m = 1.0F;
  box.min_z = 0.0F;
  box.max_z = 1.0F;
  return box;
}

PendingCargoEnvelope envelope() {
  PendingCargoEnvelope value;
  value.valid = true;
  value.center_base = Eigen::Vector3f(0.0F, 0.0F, 0.5F);
  value.length_m = 2.4F;
  value.width_m = 1.4F;
  value.height_m = 1.4F;
  value.bottom_z_base = -0.2F;
  value.top_z_base = 1.2F;
  return value;
}

PendingCargoSelfEvidenceInput retiredInput() {
  PendingCargoSelfEvidenceInput input;
  input.stamp_sec = 10.0;
  input.evidence_stamp_sec = 9.5;
  input.source = PendingCargoEnvelopeSource::RETIRED_FORMAL_SHAPE;
  input.cargo_lifecycle_id = 4U;
  input.retired_cargo_lifecycle_id = 4U;
  input.retired_track_was_locked = true;
  input.retired_formal_obb_authorized = true;
  input.tight_identity_obb = identityBox();
  return input;
}

TEST(PendingCargoSelfEvidenceTest, RetiredObbOnlyCannotRemoveInteriorPoint) {
  const auto evidence = buildPendingCargoSelfEvidence(retiredInput());
  ASSERT_TRUE(evidence.valid);
  ASSERT_TRUE(evidence.formal_obb_only_authorized);
  EXPECT_FALSE(evidence.positive_warning_identity_authorized);
  const auto classified = classifyPendingCargoPoint(
      Eigen::Vector3f(0.0F, 0.0F, 0.5F), envelope(), evidence, 2.0F);
  EXPECT_EQ(classified.classification,
            PendingPointClass::UNRESOLVED_INSIDE_PENDING);
}

TEST(PendingCargoSelfEvidenceTest, RetiredIdentityPointCanBeRemoved) {
  auto input = retiredInput();
  input.identity_points_base.emplace_back(0.0F, 0.0F, 0.5F);
  const auto evidence = buildPendingCargoSelfEvidence(input);
  ASSERT_TRUE(evidence.valid);
  EXPECT_FALSE(evidence.formal_obb_only_authorized);
  EXPECT_TRUE(evidence.positive_warning_identity_authorized);
  const auto classified = classifyPendingCargoPoint(
      Eigen::Vector3f(0.02F, 0.0F, 0.5F), envelope(), evidence, 2.0F);
  EXPECT_EQ(classified.classification, PendingPointClass::IDENTITY_SELF);
}

TEST(PendingCargoSelfEvidenceTest,
     CargoVerticalBoundaryFragmentCannotBecomeZeroDistanceObstacle) {
  auto input = retiredInput();
  input.identity_points_base.emplace_back(0.0F, 0.0F, 0.5F);
  const auto evidence = buildPendingCargoSelfEvidence(input);
  ASSERT_TRUE(evidence.valid);
  auto pending = envelope();
  pending.vertical_uncertainty_m = 0.05F;
  const auto classified = classifyPendingCargoPoint(
      Eigen::Vector3f(0.0F, 0.0F, 1.31F), pending, evidence, 2.0F);
  EXPECT_EQ(classified.classification,
            PendingPointClass::UNRESOLVED_INSIDE_PENDING);
  EXPECT_FLOAT_EQ(classified.envelope_distance_m, 0.0F);
}

}  // namespace
}  // namespace ndt_slam
