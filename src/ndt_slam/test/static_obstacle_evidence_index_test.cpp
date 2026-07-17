#include "ndt_slam/static_obstacle_evidence_index.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

StaticObstacleEvidenceConfig testConfig() {
  StaticObstacleEvidenceConfig config;
  config.minimum_observations = 2U;
  config.minimum_stable_duration_sec = 1.0;
  config.minimum_cell_overlap = 0.50F;
  config.minimum_iou = 0.50F;
  config.minimum_height_overlap = 0.50F;
  config.minimum_matched_cells = 2U;
  config.pre_cargo_minimum_sequence_gap = 1U;
  return config;
}

StaticEvidenceCellGeometryMap twoCells() {
  return {
      {packStaticEvidenceCell(4, 8), {0.20F, 1.20F}},
      {packStaticEvidenceCell(5, 8), {0.25F, 1.15F}}};
}

StaticProvenanceQuery queryFor(std::uint64_t generation) {
  StaticProvenanceQuery query;
  query.occupied_map_cells = {
      packStaticEvidenceCell(4, 8), packStaticEvidenceCell(5, 8)};
  query.min_z_map = 0.30F;
  query.max_z_map = 1.10F;
  query.expected_map_generation = generation;
  return query;
}

TEST(StaticObstacleEvidenceIndexTest, CleanStableCellsAuthorizeMapMatch) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(0U);
  index.observeFilteredCells(twoCells(), 1.0, 0U);
  index.observeFilteredCells(twoCells(), 2.0, 0U);
  index.confirmCleanCells(twoCells(), {}, 2.0, 0U);

  const auto decision = index.query(queryFor(0U));
  EXPECT_TRUE(decision.authorized);
  EXPECT_EQ(decision.provenance, ExternalProvenance::STATIC_MAP_MATCH);
  EXPECT_FLOAT_EQ(decision.matched_cell_ratio, 1.0F);
  EXPECT_GE(decision.stable_observation_count, 2U);
}

TEST(StaticObstacleEvidenceIndexTest, PreCargoSequenceIsIndependentEvidence) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(11U);
  index.observeFilteredCells(twoCells(), 1.0, 11U);
  index.observeFilteredCells(twoCells(), 2.0, 11U);
  index.confirmCleanCells(twoCells(), {}, 2.0, 11U);

  auto query = queryFor(11U);
  query.cargo_track_start_sequence =
      index.latestObservationSequence();
  const auto decision = index.query(query);
  EXPECT_TRUE(decision.authorized);
  EXPECT_EQ(decision.provenance, ExternalProvenance::PRE_CARGO_OCCUPANCY);
}

TEST(StaticObstacleEvidenceIndexTest, ThinTopSurfacesUseBoundedHeightTolerance) {
  StaticObstacleEvidenceIndex index(testConfig());
  StaticEvidenceCellGeometryMap planar = {
      {packStaticEvidenceCell(4, 8), {1.00F, 1.00F}},
      {packStaticEvidenceCell(5, 8), {1.01F, 1.01F}}};
  index.reset(5U);
  index.observeFilteredCells(planar, 1.0, 5U);
  index.observeFilteredCells(planar, 2.0, 5U);
  index.confirmCleanCells(planar, {}, 2.0, 5U);

  auto query = queryFor(5U);
  query.min_z_map = 1.02F;
  query.max_z_map = 1.02F;
  EXPECT_TRUE(index.query(query).authorized);

  query.min_z_map = 1.30F;
  query.max_z_map = 1.30F;
  EXPECT_FALSE(index.query(query).authorized);
}

TEST(StaticObstacleEvidenceIndexTest, DenyAndGenerationRemainFailSafe) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(3U);
  index.observeFilteredCells(twoCells(), 1.0, 3U);
  index.observeFilteredCells(twoCells(), 2.0, 3U);
  index.confirmCleanCells(
      twoCells(), {packStaticEvidenceCell(4, 8)}, 2.0, 3U);

  EXPECT_FALSE(index.query(queryFor(3U)).authorized);
  EXPECT_FALSE(index.query(queryFor(4U)).authorized);

  index.reset(4U);
  index.observeFilteredCells(twoCells(), 3.0, 3U);
  index.confirmCleanCells(twoCells(), {}, 3.0, 3U);
  EXPECT_FALSE(index.query(queryFor(4U)).authorized);
  EXPECT_EQ(index.snapshot()->map_generation, 4U);
}

}  // namespace
}  // namespace ndt_slam
