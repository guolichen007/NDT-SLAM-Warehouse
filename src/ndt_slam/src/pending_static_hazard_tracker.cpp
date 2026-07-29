#include "ndt_slam/pending_static_hazard_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ndt_slam {
namespace {

constexpr std::int32_t kNear3m = 17;
constexpr std::int32_t kNear5m = 18;
constexpr double kStampEpsilonSec = 1.0e-4;

bool warningCode(std::int32_t code) {
  return code == kNear3m || code == kNear5m;
}

bool validConfig(const PendingStaticHazardTrackerConfig& config) {
  return config.minimum_confirmations >= 2 &&
      std::isfinite(config.maximum_observation_gap_sec) &&
      config.maximum_observation_gap_sec > 0.0 &&
      std::isfinite(config.minimum_cell_overlap) &&
      config.minimum_cell_overlap > 0.0F &&
      config.minimum_cell_overlap <= 1.0F;
}

void normalizeCells(std::vector<std::int64_t>* cells) {
  std::sort(cells->begin(), cells->end());
  cells->erase(std::unique(cells->begin(), cells->end()), cells->end());
}

std::size_t intersectionCount(
    const std::vector<std::int64_t>& left,
    const std::vector<std::int64_t>& right) {
  std::size_t count = 0U;
  auto left_it = left.begin();
  auto right_it = right.begin();
  while (left_it != left.end() && right_it != right.end()) {
    if (*left_it < *right_it) {
      ++left_it;
    } else if (*right_it < *left_it) {
      ++right_it;
    } else {
      ++count;
      ++left_it;
      ++right_it;
    }
  }
  return count;
}

float cellOverlap(
    const std::vector<std::int64_t>& left,
    const std::vector<std::int64_t>& right) {
  const std::size_t denominator = std::min(left.size(), right.size());
  if (denominator == 0U) return 0.0F;
  return static_cast<float>(intersectionCount(left, right)) /
      static_cast<float>(denominator);
}

std::uint32_t makeStaticObstacleId(
    std::uint64_t map_generation,
    const std::vector<std::int64_t>& cells) {
  // FNV-1a with the high bit reserved for the static-map namespace. Live
  // tracker IDs start at one, so downstream diagnostics cannot confuse them.
  std::uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](std::uint64_t value) {
    for (int index = 0; index < 8; ++index) {
      hash ^= static_cast<std::uint8_t>(value & 0xFFU);
      hash *= 1099511628211ULL;
      value >>= 8U;
    }
  };
  mix(map_generation);
  for (const std::int64_t cell : cells) {
    mix(static_cast<std::uint64_t>(cell));
  }
  return 0x80000000U |
      static_cast<std::uint32_t>((hash ^ (hash >> 32U)) & 0x7FFFFFFFU);
}

}  // namespace

PendingStaticHazardTracker::PendingStaticHazardTracker(
    const PendingStaticHazardTrackerConfig& config) {
  setConfig(config);
}

void PendingStaticHazardTracker::setConfig(
    const PendingStaticHazardTrackerConfig& config) {
  config_ = validConfig(config)
      ? config : PendingStaticHazardTrackerConfig{};
  reset();
}

void PendingStaticHazardTracker::reset() {
  active_ = false;
  last_stamp_sec_ = 0.0;
  cargo_lifecycle_id_ = 0U;
  map_generation_ = 0U;
  obstacle_id_ = 0U;
  confirmations_ = 0;
  matched_cell_keys_.clear();
}

PendingStaticHazardDecision PendingStaticHazardTracker::update(
    PendingStaticHazardObservation observation) {
  PendingStaticHazardDecision decision;
  if (!validConfig(config_) || !std::isfinite(observation.stamp_sec) ||
      observation.stamp_sec <= 0.0) {
    reset();
    decision.reason = "invalid_static_hazard_tracker_input";
    return decision;
  }
  normalizeCells(&observation.matched_cell_keys);
  decision.valid = true;
  decision.hazard_observed = observation.hazard &&
      warningCode(observation.warning_code);

  const bool evidence_valid = observation.cargo_lifecycle_id != 0U &&
      observation.map_generation != 0U &&
      observation.authority_valid && observation.query_valid &&
      observation.query_bounded && decision.hazard_observed &&
      !observation.matched_cell_keys.empty();
  if (!evidence_valid) {
    reset();
    decision.reason = !observation.authority_valid
        ? "static_authority_not_valid"
        : (!observation.query_valid || !observation.query_bounded
               ? "static_query_not_reliable"
               : (!decision.hazard_observed
                      ? "no_static_positive_hazard"
                      : "static_hazard_identity_missing"));
    return decision;
  }

  if (active_ &&
      observation.stamp_sec + kStampEpsilonSec < last_stamp_sec_) {
    reset();
  }

  const float overlap = active_
      ? cellOverlap(
            observation.matched_cell_keys, matched_cell_keys_)
      : 0.0F;
  decision.cell_overlap = overlap;
  const bool same_context = active_ &&
      observation.cargo_lifecycle_id == cargo_lifecycle_id_ &&
      observation.map_generation == map_generation_ &&
      observation.stamp_sec - last_stamp_sec_ <=
          config_.maximum_observation_gap_sec + kStampEpsilonSec &&
      overlap >= config_.minimum_cell_overlap;

  if (same_context &&
      observation.stamp_sec <= last_stamp_sec_ + kStampEpsilonSec) {
    decision.warning_code = observation.warning_code;
    decision.obstacle_id = obstacle_id_;
    decision.confirmations = confirmations_;
    decision.authorized =
        confirmations_ >= config_.minimum_confirmations;
    decision.reason = decision.authorized
        ? "static_pending_hazard_authorized"
        : "repeated_static_hazard_stamp";
    return decision;
  }

  if (!same_context) {
    active_ = true;
    cargo_lifecycle_id_ = observation.cargo_lifecycle_id;
    map_generation_ = observation.map_generation;
    obstacle_id_ = makeStaticObstacleId(
        observation.map_generation, observation.matched_cell_keys);
    confirmations_ = 1;
  } else {
    ++confirmations_;
  }
  last_stamp_sec_ = observation.stamp_sec;
  matched_cell_keys_ = std::move(observation.matched_cell_keys);

  decision.warning_code = observation.warning_code;
  decision.obstacle_id = obstacle_id_;
  decision.confirmations = confirmations_;
  decision.authorized =
      confirmations_ >= config_.minimum_confirmations;
  decision.reason = decision.authorized
      ? "static_pending_hazard_authorized"
      : "static_pending_hazard_confirmation_pending";
  return decision;
}

}  // namespace ndt_slam
