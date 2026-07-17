#include "ndt_slam/static_obstacle_evidence_index.hpp"

#include <gtest/gtest.h>

#include <cstdio>

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
  config.maximum_observation_gap_sec = 2.0;
  config.maximum_observation_sequence_gap = 1U;
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

TEST(StaticObstacleEvidenceIndexTest,
     InvalidatedCellCannotBeReauthorizedInSameBuild) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(7U);
  index.observeFilteredCells(twoCells(), 1.0, 7U, 1U);
  index.observeFilteredCells(twoCells(), 2.0, 7U, 2U);
  index.confirmCleanCells(
      twoCells(), {packStaticEvidenceCell(4, 8)}, 2.0, 7U, 3U);

  EXPECT_FALSE(index.query(queryFor(7U)).authorized);
  EXPECT_EQ(index.snapshot()->cells.count(packStaticEvidenceCell(4, 8)),
            0U);
}

TEST(StaticObstacleEvidenceIndexTest,
     ObservationGapResetsAuthorizationStreak) {
  auto config = testConfig();
  config.maximum_observation_gap_sec = 0.5;
  StaticObstacleEvidenceIndex index(config);
  index.reset(8U);
  index.observeFilteredCells(twoCells(), 1.0, 8U, 1U);
  index.observeFilteredCells(twoCells(), 2.0, 8U, 2U);
  index.confirmCleanCells(twoCells(), {}, 2.0, 8U, 1U);

  const auto decision = index.query(queryFor(8U));
  EXPECT_FALSE(decision.authorized);
  EXPECT_EQ(decision.reason, "static_map_insufficient_matched_cells");
}

TEST(StaticObstacleEvidenceIndexTest,
     MissingSequenceResetsStableDuration) {
  auto config = testConfig();
  config.minimum_matched_cells = 1U;
  StaticObstacleEvidenceIndex index(config);
  index.reset(9U);
  StaticEvidenceCellGeometryMap one = {
      {packStaticEvidenceCell(4, 8), {0.20F, 1.20F}}};
  StaticEvidenceCellGeometryMap other = {
      {packStaticEvidenceCell(99, 99), {0.20F, 1.20F}}};
  index.observeFilteredCells(one, 1.0, 9U, 1U);
  index.observeFilteredCells(other, 1.2, 9U, 2U);
  index.observeFilteredCells(one, 1.4, 9U, 3U);
  index.confirmCleanCells(one, {}, 1.4, 9U, 1U);

  auto query = queryFor(9U);
  query.occupied_map_cells.resize(1U);
  EXPECT_FALSE(index.query(query).authorized);
}

TEST(StaticObstacleEvidenceIndexTest,
     StaleCleanResultCannotUndoNewerInvalidation) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(10U);
  index.observeFilteredCells(twoCells(), 1.0, 10U, 1U);
  index.observeFilteredCells(twoCells(), 2.0, 10U, 2U);
  index.invalidateCells(
      {packStaticEvidenceCell(4, 8)}, 5U, 2.1, 10U);
  index.confirmCleanCells(twoCells(), {}, 2.0, 10U, 4U);

  EXPECT_FALSE(index.query(queryFor(10U)).authorized);
  EXPECT_EQ(index.snapshot()->cells.count(packStaticEvidenceCell(4, 8)),
            0U);
}

TEST(StaticObstacleEvidenceIndexTest,
     ContinuousCommitsDoNotStarveStaticConfirmation) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(12U);
  index.observeFilteredCells(twoCells(), 1.0, 12U, 100U);
  index.observeFilteredCells(twoCells(), 2.0, 12U, 101U);
  const auto mutation =
      index.confirmCleanCells(twoCells(), {}, 1.5, 12U, 1U);

  EXPECT_EQ(mutation.confirmed_cells, 2U);
  EXPECT_TRUE(index.query(queryFor(12U)).authorized);
}

TEST(StaticObstacleEvidenceIndexTest,
     SparseCurrentObservationMatchesDenseStaticMap) {
  StaticObstacleEvidenceIndex index(testConfig());
  StaticEvidenceCellGeometryMap dense = twoCells();
  for (int x = 6; x <= 9; ++x) {
    dense.emplace(packStaticEvidenceCell(x, 8),
                  StaticEvidenceCellGeometry{0.20F, 1.20F});
  }
  index.reset(13U);
  index.observeFilteredCells(dense, 1.0, 13U, 1U);
  index.observeFilteredCells(dense, 2.0, 13U, 2U);
  index.confirmCleanCells(dense, {}, 2.0, 13U, 1U);

  auto sparse_query = queryFor(13U);
  sparse_query.occupied_map_cells = {
      packStaticEvidenceCell(4, 8), packStaticEvidenceCell(9, 8)};
  const auto decision = index.query(sparse_query);
  EXPECT_TRUE(decision.authorized);
  EXPECT_LT(decision.matched_iou, testConfig().minimum_iou);
  EXPECT_FLOAT_EQ(decision.matched_cell_ratio, 1.0F);
}

TEST(StaticObstacleEvidenceIndexTest,
     PersistReloadPreservesStaticAuthorization) {
  StaticObstacleEvidenceIndex source(testConfig());
  source.reset(14U);
  source.observeFilteredCells(twoCells(), 1.0, 14U, 1U);
  source.observeFilteredCells(twoCells(), 2.0, 14U, 2U);
  source.confirmCleanCells(twoCells(), {}, 2.0, 14U, 1U);
  const auto snapshot = source.snapshot();
  const std::string path = "static_evidence_index_test.csv";
  std::remove(path.c_str());
  std::remove((path + ".tmp").c_str());
  std::string reason;
  ASSERT_TRUE(source.saveSnapshot(snapshot, path, &reason)) << reason;

  StaticObstacleEvidenceIndex loaded(testConfig());
  loaded.reset(15U);
  ASSERT_TRUE(loaded.loadSnapshot(
      path, 15U, snapshot->map_generation, snapshot->revision, &reason))
      << reason;
  EXPECT_TRUE(loaded.query(queryFor(15U)).authorized);
  std::remove(path.c_str());
}

}  // namespace
}  // namespace ndt_slam
