#include "ndt_slam/static_height_component_extractor.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

void addCellLayer(int x, int y, float z,
                  std::vector<Eigen::Vector3f>* points) {
  for (int i = 0; i < 12; ++i) {
    points->emplace_back(
        (static_cast<float>(x) + 0.25F) * 0.25F,
        (static_cast<float>(y) + 0.25F) * 0.25F,
        z + 0.002F * static_cast<float>(i));
  }
}

TEST(StaticHeightComponentExtractorTest, KeepsSeparatedComponentsStable) {
  std::vector<Eigen::Vector3f> objects;
  std::vector<Eigen::Vector3f> ground;
  for (const int base_x : {0, 8}) {
    for (int dx = 0; dx < 2; ++dx) {
      for (int dy = 0; dy < 2; ++dy) {
        addCellLayer(base_x + dx, dy, 1.0F, &objects);
        ground.emplace_back(
            (static_cast<float>(base_x + dx) + 0.25F) * 0.25F,
            (static_cast<float>(dy) + 0.25F) * 0.25F, 0.0F);
      }
    }
  }
  StaticHeightField field;
  ASSERT_TRUE(field.build(
      objects, ground, StaticEvidenceAuthority::RUNTIME_MATURE,
      1U, 7U).valid);
  StaticHeightComponentExtractorConfig config;
  config.minimum_component_cells = 4U;
  config.maximum_anchor_distance_m = 4.0F;
  StaticHeightComponentExtractor extractor(config);
  StaticHeightComponentQuery query;
  query.hook_anchor_map = Eigen::Vector2f(1.0F, 0.25F);
  query.map_generation = 7U;
  const auto first = extractor.extract(field, query);
  const auto second = extractor.extract(field, query);
  ASSERT_EQ(first.size(), 2U);
  ASSERT_EQ(second.size(), 2U);
  EXPECT_NE(first[0].component_id, first[1].component_id);
  EXPECT_EQ(first[0].component_id, second[0].component_id);
  EXPECT_EQ(first[1].component_id, second[1].component_id);
}

}  // namespace
}  // namespace ndt_slam
