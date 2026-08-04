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
  config.immature_max_observation_gap_sec = 2.0;
  config.immature_gap_retention_ratio = 0.50;
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
  config.immature_max_observation_gap_sec = 0.5;
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
  ASSERT_TRUE(loaded.snapshot());
  EXPECT_EQ(loaded.snapshot()->revision, snapshot->revision);
  EXPECT_TRUE(loaded.query(queryFor(15U)).authorized);
  std::remove(path.c_str());
}

TEST(StaticObstacleEvidenceIndexTest,
     ExpiredImmatureHistoryCannotAuthorizeAfterOneNewObservation) {
  auto config = testConfig();
  config.minimum_observations = 3U;
  config.minimum_stable_duration_sec = 1.0;
  config.immature_max_observation_gap_sec = 0.5;
  StaticObstacleEvidenceIndex index(config);
  index.reset(21U);
  index.observeFilteredCells(twoCells(), 1.0, 21U, 1U);
  index.confirmCleanCells(twoCells(), {}, 1.0, 21U, 1U);
  ASSERT_EQ(index.matureCellCount(), 0U);

  index.observeFilteredCells(twoCells(), 2.0, 21U, 2U);
  index.observeFilteredCells(twoCells(), 2.4, 21U, 3U);
  index.confirmCleanCells(twoCells(), {}, 2.4, 21U, 2U);

  EXPECT_EQ(index.matureCellCount(), 0U);
  EXPECT_FALSE(index.query(queryFor(21U)).authorized);
  const auto snapshot = index.snapshot();
  ASSERT_TRUE(snapshot);
  EXPECT_FALSE(snapshot->cells.begin()->second.temporally_mature);
  EXPECT_EQ(snapshot->cells.begin()->second.consecutive_observation_count,
            2U);
}

TEST(StaticObstacleEvidenceIndexTest,
     WarehouseRevisitsWithinImmatureWindowCanMature) {
  auto config = testConfig();
  config.minimum_observations = 4U;
  config.minimum_stable_duration_sec = 1.0;
  config.immature_max_observation_gap_sec = 300.0;
  StaticObstacleEvidenceIndex index(config);
  index.reset(28U);

  index.observeFilteredCells(twoCells(), 1.0, 28U, 1U);
  index.observeFilteredCells(twoCells(), 61.0, 28U, 2U);
  index.observeFilteredCells(twoCells(), 121.0, 28U, 3U);
  index.observeFilteredCells(twoCells(), 181.0, 28U, 4U);
  index.confirmCleanCells(twoCells(), {}, 181.0, 28U, 1U);

  EXPECT_EQ(index.matureCellCount(), 2U);
  EXPECT_EQ(index.diagnostics().reset_by_time_gap, 0U);
  EXPECT_TRUE(index.query(queryFor(28U)).authorized);
}

TEST(StaticObstacleEvidenceIndexTest,
     ExpiredImmatureHistoryDecaysInsteadOfBeingDiscarded) {
  auto config = testConfig();
  config.minimum_observations = 4U;
  config.minimum_stable_duration_sec = 1.0;
  config.immature_max_observation_gap_sec = 0.5;
  config.immature_gap_retention_ratio = 0.50;
  StaticObstacleEvidenceIndex index(config);
  index.reset(32U);

  index.observeFilteredCells(twoCells(), 1.0, 32U, 1U);
  index.observeFilteredCells(twoCells(), 1.4, 32U, 2U);
  index.observeFilteredCells(twoCells(), 1.8, 32U, 3U);
  index.observeFilteredCells(twoCells(), 2.8, 32U, 4U);
  index.confirmCleanCells(twoCells(), {}, 2.8, 32U, 1U);

  const auto snapshot = index.snapshot();
  ASSERT_TRUE(snapshot);
  ASSERT_FALSE(snapshot->cells.empty());
  EXPECT_EQ(snapshot->cells.begin()->second.consecutive_observation_count,
            2U);
  EXPECT_EQ(index.diagnostics().decayed_by_time_gap, 2U);
  EXPECT_EQ(index.matureCellCount(), 0U);
  EXPECT_FALSE(index.query(queryFor(32U)).authorized);
}

TEST(StaticObstacleEvidenceIndexTest,
     AdjacentAsynchronousCleanBuildsMatureWithinConfiguredGap) {
  auto config = testConfig();
  config.minimum_observations = 4U;
  config.minimum_stable_duration_sec = 1.0;
  config.immature_max_observation_gap_sec = 5.0;
  StaticObstacleEvidenceIndex index(config);
  index.reset(29U);
  index.observeFilteredCells(twoCells(), 1.0, 29U, 1U);
  index.observeFilteredCells(twoCells(), 2.5, 29U, 2U);
  index.observeFilteredCells(twoCells(), 4.0, 29U, 3U);
  index.observeFilteredCells(twoCells(), 5.5, 29U, 4U);
  index.confirmCleanCells(twoCells(), {}, 5.5, 29U, 1U);

  EXPECT_EQ(index.matureCellCount(), 2U);
  EXPECT_EQ(index.diagnostics().reset_by_time_gap, 0U);
  EXPECT_TRUE(index.query(queryFor(29U)).authorized);
}

TEST(StaticObstacleEvidenceIndexTest,
     EmptyMapCommitBreaksConsecutiveObservation) {
  auto config = testConfig();
  config.minimum_stable_duration_sec = 0.3;
  StaticObstacleEvidenceIndex index(config);
  index.reset(22U);
  index.observeFilteredCells(twoCells(), 1.0, 22U, 1U);
  index.observeFilteredCells({}, 1.2, 22U, 2U);
  index.observeFilteredCells(twoCells(), 1.4, 22U, 3U);
  index.confirmCleanCells(twoCells(), {}, 1.4, 22U, 1U);

  EXPECT_EQ(index.latestObservationSequence(), 3U);
  EXPECT_EQ(index.matureCellCount(), 0U);
  EXPECT_FALSE(index.query(queryFor(22U)).authorized);
}

TEST(StaticObstacleEvidenceIndexTest,
     MatureCellRemainsAuthorizedUntilInvalidated) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(23U);
  index.observeFilteredCells(twoCells(), 1.0, 23U, 1U);
  index.observeFilteredCells(twoCells(), 2.0, 23U, 2U);
  index.confirmCleanCells(twoCells(), {}, 2.0, 23U, 1U);
  ASSERT_EQ(index.matureCellCount(), 2U);
  ASSERT_TRUE(index.query(queryFor(23U)).authorized);

  index.observeFilteredCells({}, 3.0, 23U, 3U);
  index.observeFilteredCells(twoCells(), 10.0, 23U, 4U);
  index.confirmCleanCells(twoCells(), {}, 10.0, 23U, 2U);
  EXPECT_EQ(index.matureCellCount(), 2U);
  EXPECT_TRUE(index.query(queryFor(23U)).authorized);

  index.invalidateCells(
      {packStaticEvidenceCell(4, 8)}, 3U, 10.1, 23U);
  EXPECT_FALSE(index.query(queryFor(23U)).authorized);
}

TEST(StaticObstacleEvidenceIndexTest,
     NotInViewPausesStreakWithoutAddingInvisibleTime) {
  auto config = testConfig();
  config.minimum_observations = 3U;
  config.minimum_stable_duration_sec = 0.3;
  StaticObstacleEvidenceIndex index(config);
  index.reset(24U);
  index.observeFilteredCells(twoCells(), 1.0, 24U, 1U);
  index.observeFilteredCells(twoCells(), 1.2, 24U, 2U);
  index.observeFilteredCells({}, 1.3, 24U, 3U);
  index.observeFilteredCells(twoCells(), 1.5, 24U, 4U);
  index.confirmCleanCells(twoCells(), {}, 1.5, 24U, 1U);

  EXPECT_EQ(index.matureCellCount(), 0U);
  EXPECT_GT(index.diagnostics().not_in_view, 0U);
  index.observeFilteredCells(twoCells(), 1.7, 24U, 5U);
  EXPECT_EQ(index.matureCellCount(), 2U);
}

TEST(StaticObstacleEvidenceIndexTest,
     ObservedFreeInvalidatesButNotInViewDoesNot) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(25U);
  index.observeFilteredCells(twoCells(), 1.0, 25U, 1U);
  index.observeFilteredCells(twoCells(), 2.0, 25U, 2U);
  index.confirmCleanCells(twoCells(), {}, 2.0, 25U, 1U);
  ASSERT_TRUE(index.query(queryFor(25U)).authorized);

  const auto free_key = packStaticEvidenceCell(4, 8);
  index.observeCleanBuildCells(
      {}, {free_key}, {free_key}, 3.0, 25U, 2U);
  EXPECT_FALSE(index.query(queryFor(25U)).authorized);
  EXPECT_EQ(index.diagnostics().observed_free, 1U);
}

TEST(StaticObstacleEvidenceIndexTest,
     ObservedFreeBlocksStaleCleanReconfirmation) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(27U);
  index.observeFilteredCells(twoCells(), 1.0, 27U, 1U);
  index.observeFilteredCells(twoCells(), 2.0, 27U, 2U);
  index.confirmCleanCells(twoCells(), {}, 2.0, 27U, 1U);
  ASSERT_TRUE(index.query(queryFor(27U)).authorized);

  StaticEvidenceCellKeySet free_cells;
  for (const auto& item : twoCells()) free_cells.insert(item.first);
  index.observeCleanBuildCells(
      {}, free_cells, free_cells, 3.0, 27U, 2U);
  index.confirmCleanCells(twoCells(), {}, 3.0, 27U, 2U);
  EXPECT_EQ(index.matureCellCount(), 0U);
  EXPECT_TRUE(index.snapshot()->cells.empty());
}

TEST(StaticObstacleEvidenceIndexTest,
     OperatorApprovedBaselineDoesNotPretendToBeRuntimeMature) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(26U);
  index.confirmCleanCells(twoCells(), {}, 1.0, 26U, 1U);
  ASSERT_EQ(index.matureCellCount(), 0U);
  EXPECT_FALSE(index.query(queryFor(26U)).authorized);

  index.setSnapshotAuthority(
      StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE);
  const auto decision = index.query(queryFor(26U));
  EXPECT_TRUE(decision.authorized);
  EXPECT_EQ(decision.authority,
            StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE);
  EXPECT_EQ(index.matureCellCount(), 0U);
}

TEST(StaticObstacleEvidenceIndexTest,
     PreparedInstallFailureLeavesOldIdentity) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(30U);
  const auto before = index.snapshot();
  ASSERT_TRUE(before);
  StaticEvidenceSnapshot invalid = *before;
  invalid.revision = 0U;
  PreparedStaticEvidenceInstall prepared;
  std::string reason;
  EXPECT_FALSE(index.prepareSnapshotInstall(
      invalid, 31U, &prepared, &reason));
  const auto after = index.snapshot();
  ASSERT_TRUE(after);
  EXPECT_EQ(after->map_generation, before->map_generation);
  EXPECT_EQ(after->revision, before->revision);
  EXPECT_EQ(after->cells.size(), before->cells.size());
}

TEST(StaticObstacleEvidenceIndexTest,
     StaticRestoreFailureLeavesOldMap) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(40U);
  const auto before = index.snapshot();
  ASSERT_TRUE(before);
  StaticEvidenceSnapshot wrong_schema = *before;
  wrong_schema.schema_version = 0U;
  PreparedStaticEvidenceInstall prepared;
  EXPECT_FALSE(index.prepareSnapshotInstall(
      wrong_schema, 41U, &prepared, nullptr));
  EXPECT_EQ(index.snapshot(), before);
}

TEST(StaticObstacleEvidenceIndexTest,
     SuccessfulInstallChangesAllIdentityFieldsTogether) {
  StaticObstacleEvidenceIndex index(testConfig());
  index.reset(50U);
  StaticEvidenceSnapshot candidate = *index.snapshot();
  candidate.revision = 77U;
  candidate.latest_observation_sequence = 12U;
  candidate.authority =
      StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE;
  PreparedStaticEvidenceInstall prepared;
  std::string reason;
  ASSERT_TRUE(index.prepareSnapshotInstall(
      candidate, 51U, &prepared, &reason)) << reason;
  EXPECT_NE(index.snapshot()->revision, 77U);
  index.installPreparedSnapshot(std::move(prepared));
  const auto installed = index.snapshot();
  ASSERT_TRUE(installed);
  EXPECT_EQ(installed->map_generation, 51U);
  EXPECT_EQ(installed->revision, 77U);
  EXPECT_EQ(installed->latest_observation_sequence, 12U);
  EXPECT_EQ(installed->authority,
            StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE);
}

}  // namespace
}  // namespace ndt_slam
