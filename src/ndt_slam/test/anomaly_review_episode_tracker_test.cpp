#include "ndt_slam/anomaly_review_episode_tracker.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

AnomalyReviewEpisodeInput review(double stamp) {
  AnomalyReviewEpisodeInput input;
  input.stamp_sec = stamp;
  input.candidate_code = 29;
  input.key.cargo_lifecycle_id = 4U;
  input.key.cargo_track_id = 7U;
  input.key.obstacle_track_id = 11U;
  input.key.pose_generation = 2U;
  input.key.map_generation = 9U;
  return input;
}

TEST(AnomalyReviewEpisodeTrackerTest, RequiresTwoAdvancingFramesToEnter) {
  AnomalyReviewEpisodeTracker tracker;
  EXPECT_EQ(tracker.update(review(1.0)).output_code, 34);
  const auto repeated = tracker.update(review(1.0));
  EXPECT_EQ(repeated.output_code, 34);
  EXPECT_EQ(repeated.event, "REPEATED_STAMP_IGNORED");
  const auto entered = tracker.update(review(1.1));
  EXPECT_EQ(entered.output_code, 29);
  EXPECT_TRUE(entered.active);
  EXPECT_TRUE(entered.emit_event);
  EXPECT_EQ(entered.event, "ENTER");
}

TEST(AnomalyReviewEpisodeTrackerTest, TimesOutAndSuppressesSameEpisodeReentry) {
  AnomalyReviewEpisodeTracker tracker;
  tracker.update(review(1.0));
  tracker.update(review(1.1));
  const auto timeout = tracker.update(review(2.6));
  EXPECT_EQ(timeout.output_code, 34);
  EXPECT_EQ(timeout.event, "TIMEOUT");
  EXPECT_EQ(tracker.update(review(2.7)).output_code, 34);
  EXPECT_EQ(tracker.update(review(4.7)).output_code, 34);
  EXPECT_EQ(tracker.update(review(4.8)).output_code, 29);
}

TEST(AnomalyReviewEpisodeTrackerTest, StandardWarningAlwaysPromotesImmediately) {
  AnomalyReviewEpisodeTracker tracker;
  tracker.update(review(1.0));
  tracker.update(review(1.1));
  AnomalyReviewEpisodeInput warning = review(1.2);
  warning.candidate_code = 17;
  const auto promoted = tracker.update(warning);
  EXPECT_EQ(promoted.output_code, 17);
  EXPECT_EQ(promoted.event, "PROMOTE");
}

TEST(AnomalyReviewEpisodeTrackerTest, GenerationChangeCannotInheritEpisode) {
  AnomalyReviewEpisodeTracker tracker;
  tracker.update(review(1.0));
  tracker.update(review(1.1));
  auto next_generation = review(1.2);
  ++next_generation.key.pose_generation;
  EXPECT_EQ(tracker.update(next_generation).output_code, 34);
  EXPECT_EQ(tracker.update(review(1.3)).output_code, 34);
}

}  // namespace
}  // namespace ndt_slam
