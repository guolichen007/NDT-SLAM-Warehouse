#include "ndt_slam/anomaly_review_episode_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {
namespace {

constexpr std::int32_t kLevel1 = 17;
constexpr std::int32_t kLevel2 = 18;
constexpr std::int32_t kReview = 29;
constexpr std::int32_t kObstacleInvalid = 34;
constexpr double kStampEpsilonSec = 1.0e-6;

bool standardWarning(std::int32_t code) {
  return code == kLevel1 || code == kLevel2;
}

bool validConfig(const AnomalyReviewEpisodeConfig& config) {
  return config.enter_confirm_frames >= 1 &&
      std::isfinite(config.maximum_active_sec) &&
      config.maximum_active_sec > 0.0 &&
      std::isfinite(config.reentry_cooldown_sec) &&
      config.reentry_cooldown_sec >= 0.0;
}

}  // namespace

bool AnomalyReviewEpisodeKey::operator==(
    const AnomalyReviewEpisodeKey& other) const noexcept {
  return cargo_lifecycle_id == other.cargo_lifecycle_id &&
      cargo_track_id == other.cargo_track_id &&
      obstacle_track_id == other.obstacle_track_id &&
      pose_generation == other.pose_generation &&
      map_generation == other.map_generation;
}

AnomalyReviewEpisodeTracker::AnomalyReviewEpisodeTracker(
    const AnomalyReviewEpisodeConfig& config) {
  setConfig(config);
}

void AnomalyReviewEpisodeTracker::setConfig(
    const AnomalyReviewEpisodeConfig& config) {
  config_ = validConfig(config) ? config : AnomalyReviewEpisodeConfig{};
  reset();
}

void AnomalyReviewEpisodeTracker::reset() {
  has_stamp_ = false;
  last_stamp_sec_ = 0.0;
  last_output_code_ = kObstacleInvalid;
  pending_ = false;
  pending_confirmations_ = 0;
  pending_since_sec_ = 0.0;
  active_ = false;
  active_since_sec_ = 0.0;
  cooldown_ = false;
  cooldown_until_sec_ = 0.0;
}

AnomalyReviewEpisodeDecision AnomalyReviewEpisodeTracker::update(
    const AnomalyReviewEpisodeInput& input) {
  AnomalyReviewEpisodeDecision decision;
  decision.output_code = input.candidate_code;
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec < 0.0) {
    reset();
    decision.output_code = kObstacleInvalid;
    decision.emit_event = true;
    decision.event = "INVALID_STAMP";
    return decision;
  }
  if (has_stamp_ && input.stamp_sec + kStampEpsilonSec < last_stamp_sec_) {
    reset();
    decision.output_code = kObstacleInvalid;
    decision.emit_event = true;
    decision.event = "TIME_ROLLBACK";
    has_stamp_ = true;
    last_stamp_sec_ = input.stamp_sec;
    last_output_code_ = decision.output_code;
    return decision;
  }
  if (has_stamp_ &&
      std::abs(input.stamp_sec - last_stamp_sec_) <= kStampEpsilonSec) {
    if (standardWarning(input.candidate_code)) {
      decision.output_code = input.candidate_code;
      decision.emit_event = active_ || pending_;
      decision.event = decision.emit_event ? "PROMOTE" : "NONE";
      pending_ = false;
      pending_confirmations_ = 0;
      active_ = false;
      cooldown_ = false;
      last_output_code_ = decision.output_code;
      return decision;
    }
    if (input.candidate_code != kReview) {
      decision.output_code = input.candidate_code;
      decision.emit_event = active_ || pending_;
      decision.event = decision.emit_event ? "DISAPPEAR" : "NONE";
      pending_ = false;
      pending_confirmations_ = 0;
      active_ = false;
      last_output_code_ = decision.output_code;
      return decision;
    }
    decision.output_code = last_output_code_;
    decision.active = active_;
    decision.confirmation_count = pending_confirmations_;
    decision.event = "REPEATED_STAMP_IGNORED";
    return decision;
  }
  has_stamp_ = true;
  last_stamp_sec_ = input.stamp_sec;

  if (standardWarning(input.candidate_code)) {
    decision.emit_event = active_ || pending_;
    decision.event = decision.emit_event ? "PROMOTE" : "NONE";
    pending_ = false;
    pending_confirmations_ = 0;
    active_ = false;
    cooldown_ = false;
    last_output_code_ = input.candidate_code;
    return decision;
  }

  if (input.candidate_code != kReview) {
    decision.emit_event = active_ || pending_;
    decision.event = decision.emit_event ? "DISAPPEAR" : "NONE";
    pending_ = false;
    pending_confirmations_ = 0;
    active_ = false;
    if (cooldown_ && input.stamp_sec >= cooldown_until_sec_) {
      cooldown_ = false;
    }
    last_output_code_ = input.candidate_code;
    return decision;
  }

  if (cooldown_ && input.key == cooldown_key_ &&
      input.stamp_sec < cooldown_until_sec_) {
    decision.output_code = kObstacleInvalid;
    decision.event = "REENTRY_SUPPRESSED";
    last_output_code_ = decision.output_code;
    return decision;
  }
  if (cooldown_ && input.stamp_sec >= cooldown_until_sec_) {
    cooldown_ = false;
  }

  if (active_) {
    if (!(input.key == active_key_)) {
      active_ = false;
      pending_ = false;
      pending_confirmations_ = 0;
    } else if (input.stamp_sec - active_since_sec_ <=
               config_.maximum_active_sec) {
      decision.output_code = kReview;
      decision.active = true;
      decision.confirmation_count = pending_confirmations_;
      decision.event = "ACTIVE";
      last_output_code_ = decision.output_code;
      return decision;
    } else {
      active_ = false;
      pending_ = false;
      pending_confirmations_ = 0;
      cooldown_ = true;
      cooldown_key_ = input.key;
      cooldown_until_sec_ =
          input.stamp_sec + config_.reentry_cooldown_sec;
      decision.output_code = kObstacleInvalid;
      decision.emit_event = true;
      decision.event = "TIMEOUT";
      last_output_code_ = decision.output_code;
      return decision;
    }
  }

  if (!pending_ || !(input.key == pending_key_)) {
    pending_ = true;
    pending_key_ = input.key;
    pending_confirmations_ = 1;
    pending_since_sec_ = input.stamp_sec;
  } else {
    ++pending_confirmations_;
  }
  decision.confirmation_count = pending_confirmations_;
  if (pending_confirmations_ < config_.enter_confirm_frames) {
    decision.output_code = kObstacleInvalid;
    decision.event = "ENTER_PENDING";
  } else {
    active_ = true;
    active_key_ = pending_key_;
    active_since_sec_ = pending_since_sec_;
    pending_ = false;
    decision.output_code = kReview;
    decision.active = true;
    decision.emit_event = true;
    decision.event = "ENTER";
  }
  last_output_code_ = decision.output_code;
  return decision;
}

}  // namespace ndt_slam
