#include "ndt_slam/static_height_field.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

std::vector<Eigen::Vector3f> layer(float x, float y, float z,
                                   std::size_t count = 12U) {
  std::vector<Eigen::Vector3f> points;
  for (std::size_t i = 0U; i < count; ++i) {
    points.emplace_back(
        x + 0.01F * static_cast<float>(i % 3U),
        y + 0.01F * static_cast<float>((i / 3U) % 3U),
        z + 0.005F * static_cast<float>(i % 5U));
  }
  return points;
}

TEST(StaticHeightFieldTest, KeepsSeparatedVerticalLayersAndUsesZ95) {
  StaticHeightField field;
  auto objects = layer(1.0F, 2.0F, 0.4F);
  const auto upper = layer(1.0F, 2.0F, 1.8F);
  objects.insert(objects.end(), upper.begin(), upper.end());
  const auto result = field.build(
      objects, {}, StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE);

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.occupied_cells, 1U);
  ASSERT_EQ(result.layer_count, 2U);
  const auto* cell = field.cell(packStaticEvidenceCell(4, 8));
  ASSERT_NE(cell, nullptr);
  ASSERT_EQ(cell->layers.size(), 2U);
  EXPECT_LT(cell->layers[0].z95, cell->layers[1].z05);
  EXPECT_LT(cell->layers[1].z95, cell->layers[1].z_high + 0.01F);
}

TEST(StaticHeightFieldTest, RejectsIsolatedVerticalTail) {
  StaticHeightFieldConfig config;
  config.minimum_points_per_layer = 6U;
  StaticHeightField field(config);
  auto objects = layer(0.0F, 0.0F, 1.0F);
  objects.emplace_back(0.01F, 0.01F, 5.0F);
  const auto result = field.build(
      objects, {}, StaticEvidenceAuthority::RUNTIME_MATURE);
  ASSERT_TRUE(result.valid);
  const auto* cell = field.cell(packStaticEvidenceCell(0, 0));
  ASSERT_NE(cell, nullptr);
  ASSERT_EQ(cell->layers.size(), 1U);
  EXPECT_LT(cell->layers.front().z95, 2.0F);
}

TEST(StaticHeightFieldTest, GroundUsesLowStableSupportAndInterpolation) {
  std::vector<Eigen::Vector3f> ground;
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      ground.emplace_back(
          0.25F * x, 0.25F * y,
          0.002F * x - 0.001F * y + 0.01F);
    }
  }
  ground.emplace_back(0.0F, 0.0F, 1.5F);  // misclassified high ground
  StaticHeightField field;
  const auto build = field.build(
      layer(3.0F, 3.0F, 1.0F), ground,
      StaticEvidenceAuthority::RUNTIME_MATURE);
  ASSERT_TRUE(build.valid);
  EXPECT_GT(build.elevated_ground_points, 0U);
  // x=1.2 is still inside the populated [1.0, 1.25) cell. Query the
  // adjacent empty cell so this assertion exercises interpolation rather
  // than a direct lookup.
  const auto support = field.supportAt(Eigen::Vector2f(1.3F, 0.0F));
  ASSERT_TRUE(support.valid);
  EXPECT_TRUE(support.interpolated);
  EXPECT_NEAR(support.z, 0.01F, 0.15F);
  EXPECT_GE(support.uncertainty_m,
            field.config().minimum_support_uncertainty_m);
}

TEST(StaticHeightFieldTest, QueryIsBoundedAndAuthorityIsPreserved) {
  StaticHeightField field;
  const auto objects = layer(1.0F, 0.0F, 1.2F);
  ASSERT_TRUE(field.build(
      objects, {},
      StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE).valid);
  StaticHeightQuery query;
  query.center_map = Eigen::Vector2f::Zero();
  query.length_m = 4.0F;
  query.width_m = 1.6F;
  query.shell_m = 5.0F;
  query.minimum_z = 0.0F;
  query.maximum_z = 3.0F;
  const auto result = field.query(query);
  ASSERT_TRUE(result.valid);
  ASSERT_GT(result.matched_cells, 0U);
  EXPECT_EQ(result.strongest_authority,
            StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE);

  query.maximum_cells = 10U;
  const auto rejected = field.query(query);
  EXPECT_FALSE(rejected.valid);
  EXPECT_FALSE(rejected.bounded);
}

TEST(StaticHeightFieldTest,
     ExcludedOriginCellCannotContributeFormalClearCoverage) {
  StaticHeightField field;
  auto objects = layer(0.05F, 0.05F, 1.0F);
  const auto external = layer(0.55F, 0.05F, 1.2F);
  objects.insert(objects.end(), external.begin(), external.end());
  ASSERT_TRUE(field.build(
      objects, {}, StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE,
      1U, 9U).valid);

  StaticHeightQuery query;
  query.center_map = Eigen::Vector2f(0.25F, 0.125F);
  query.length_m = 1.0F;
  query.width_m = 0.25F;
  query.shell_m = 0.25F;
  query.minimum_z = 0.0F;
  query.maximum_z = 3.0F;
  query.exclusion_authorized = true;
  query.excluded_component_id = 17U;
  query.excluded_component_generation = field.mapGeneration();
  query.excluded_members.insert(
      StaticHeightLayerNodeId{packStaticEvidenceCell(0, 0), 0U});

  const auto result = field.query(query);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.excluded_origin_cells, 1U);
  EXPECT_EQ(result.excluded_layer_count, 1U);
  EXPECT_GT(result.raw_covered_cells,
            result.effective_external_covered_cells);
  EXPECT_EQ(result.covered_cells,
            result.effective_external_covered_cells);
  EXPECT_EQ(result.clear_shell_covered_cells,
            result.effective_external_covered_cells);
  EXPECT_GT(result.matched_cells, 0U);
}

TEST(StaticHeightFieldTest,
     CurrentCargoSelfLayerCannotWarnOrContributeClearCoverage) {
  StaticHeightField field;
  auto objects = layer(0.05F, 0.05F, 1.0F);
  const auto external = layer(1.05F, 0.05F, 1.2F);
  objects.insert(objects.end(), external.begin(), external.end());
  ASSERT_TRUE(field.build(
      objects, {}, StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE,
      1U, 11U).valid);

  StaticHeightQuery query;
  query.center_map = Eigen::Vector2f(0.05F, 0.05F);
  query.length_m = 0.50F;
  query.width_m = 0.50F;
  query.shell_m = 1.25F;
  query.minimum_z = 0.0F;
  query.maximum_z = 3.0F;
  query.cargo_self_exclusion_authorized = true;
  query.cargo_self_length_m = 0.50F;
  query.cargo_self_width_m = 0.50F;
  query.cargo_self_minimum_z = 0.50F;
  query.cargo_self_maximum_z = 1.50F;

  const auto result = field.query(query);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.excluded_cargo_self_cells, 1U);
  EXPECT_EQ(result.excluded_cargo_self_layer_count, 1U);
  EXPECT_GT(result.raw_covered_cells,
            result.effective_external_covered_cells);
  EXPECT_EQ(result.matched_cells, 1U);
  EXPECT_NEAR(result.nearest_horizontal_distance_m, 0.75F, 0.26F);
}

}  // namespace
}  // namespace ndt_slam
