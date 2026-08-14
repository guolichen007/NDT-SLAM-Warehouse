#pragma once

#include <cstdint>
#include <string>

namespace ndt_slam {

struct AnomalyReviewEpisodeConfig {
  int enter_confirm_frames = 2;
  double maximum_active_sec = 1.5;
  double reentry_cooldown_sec = 2.0;
};

struct AnomalyReviewEpisodeKey {
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_track_id = 0U;
  std::uint64_t obstacle_track_id = 0U;
  std::uint64_t pose_generation = 0U;
  std::uint64_t map_generation = 0U;

  bool operator==(const AnomalyReviewEpisodeKey& other) const noexcept;
};

struct AnomalyReviewEpisodeInput {
  double stamp_sec = 0.0;
  std::int32_t candidate_code = 34;
  AnomalyReviewEpisodeKey key;
};

struct AnomalyReviewEpisodeDecision {
  std::int32_t output_code = 34;
  bool active = false;
  bool emit_event = false;
  std::string event = "NONE";
  int confirmation_count = 0;
};

class AnomalyReviewEpisodeTracker {
 public:
  explicit AnomalyReviewEpisodeTracker(
      const AnomalyReviewEpisodeConfig& config = {});

  void setConfig(const AnomalyReviewEpisodeConfig& config);
  void reset();
  AnomalyReviewEpisodeDecision update(
      const AnomalyReviewEpisodeInput& input);

 private:
  AnomalyReviewEpisodeConfig config_;
  bool has_stamp_ = false;
  double last_stamp_sec_ = 0.0;
  std::int32_t last_output_code_ = 34;
  bool pending_ = false;
  AnomalyReviewEpisodeKey pending_key_;
  int pending_confirmations_ = 0;
  double pending_since_sec_ = 0.0;
  bool active_ = false;
  AnomalyReviewEpisodeKey active_key_;
  double active_since_sec_ = 0.0;
  bool cooldown_ = false;
  AnomalyReviewEpisodeKey cooldown_key_;
  double cooldown_until_sec_ = 0.0;
};

}  // namespace ndt_slam
