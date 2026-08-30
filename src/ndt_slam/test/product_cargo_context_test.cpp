#include "ndt_slam/product_cargo_context.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

ProductCargoContext authorized(CargoAuthorityMode mode,
                               std::uint64_t id) {
  ProductCargoContext context;
  context.mode = mode;
  context.valid_input = true;
  context.identity_authorized = true;
  context.geometry_authorized = true;
  context.bottom_authorized = true;
  context.safety_authorized = true;
  context.clear_authorized = true;
  context.self_removal_authorized = true;
  context.map_mutation_authorized = true;
  context.cargo_id = id;
  return context;
}

TEST(ProductCargoContext, V6BypassesLegacyFormalGateInAuthorityMode) {
  ProductCargoContext legacy;
  legacy.mode = CargoAuthorityMode::LEGACY;
  legacy.reason = "legacy_formal_gate_closed";
  const ProductCargoContext v6 =
      authorized(CargoAuthorityMode::V6_AUTHORITY, 42U);

  const auto selected = selectProductCargoContext(
      CargoAuthorityMode::V6_AUTHORITY, legacy, v6, false, false);

  EXPECT_TRUE(selected.product.safety_authorized);
  EXPECT_EQ(selected.product.cargo_id, 42U);
  EXPECT_EQ(selected.reason, "v6_product");
}

TEST(ProductCargoContext, V6InvalidCannotFallbackToLegacyClear) {
  const ProductCargoContext legacy =
      authorized(CargoAuthorityMode::LEGACY, 7U);
  ProductCargoContext v6;
  v6.mode = CargoAuthorityMode::V6_AUTHORITY;
  v6.reason = "identity_not_current_validated";

  const auto selected = selectProductCargoContext(
      CargoAuthorityMode::V6_AUTHORITY, legacy, v6, false, true);

  EXPECT_FALSE(selected.product.clear_authorized);
  EXPECT_FALSE(selected.product.map_mutation_authorized);
  EXPECT_TRUE(selected.legacy_clear_rejected);
  EXPECT_EQ(selected.reason, "v6_invalid_fail_closed");
}

TEST(ProductCargoContext, LegacyModeKeepsExactExistingGate) {
  const ProductCargoContext legacy =
      authorized(CargoAuthorityMode::LEGACY, 11U);
  const ProductCargoContext v6 =
      authorized(CargoAuthorityMode::V6_AUTHORITY, 22U);

  const auto selected = selectProductCargoContext(
      CargoAuthorityMode::LEGACY, legacy, v6, false, false);

  EXPECT_EQ(selected.product.cargo_id, 11U);
  EXPECT_TRUE(selected.product.clear_authorized);
  EXPECT_EQ(selected.reason, "legacy_product");
}

TEST(ProductCargoContext, ShadowModeCannotChangeProductDecision) {
  ProductCargoContext legacy;
  legacy.mode = CargoAuthorityMode::LEGACY;
  legacy.reason = "legacy_formal_gate_closed";
  const ProductCargoContext v6 =
      authorized(CargoAuthorityMode::V6_AUTHORITY, 22U);

  const auto selected = selectProductCargoContext(
      CargoAuthorityMode::V6_SHADOW, legacy, v6, false, false);

  EXPECT_FALSE(selected.product.safety_authorized);
  EXPECT_EQ(selected.product.cargo_id, 0U);
  EXPECT_EQ(selected.reason, "shadow_legacy_product");
}

TEST(ProductCargoContext, V6InvalidMayRetainSameAuthorityPositiveOnly) {
  ProductCargoContext legacy;
  legacy.clear_authorized = true;
  ProductCargoContext v6;
  v6.mode = CargoAuthorityMode::V6_AUTHORITY;

  const auto selected = selectProductCargoContext(
      CargoAuthorityMode::V6_AUTHORITY, legacy, v6, true, true);

  EXPECT_TRUE(selected.legacy_positive_hazard_retained);
  EXPECT_TRUE(selected.legacy_clear_rejected);
  EXPECT_FALSE(selected.product.clear_authorized);
}

}  // namespace
}  // namespace ndt_slam
