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

}  // namespace
}  // namespace ndt_slam
