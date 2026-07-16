#include "ndt_slam/cargo_obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ndt_slam {
namespace {

constexpr std::uint16_t kLevel1 = 17U;
constexpr std::uint16_t kLevel2 = 18U;
constexpr double kStampEpsilonSec = 1.0e-4;

bool validConfig(const CargoObstacleTrackerConfig& config) {
  return config.confirm_frames >= 2 && config.minimum_points > 0U &&
      std::isfinite(config.maximum_observation_gap_sec) &&
      config.maximum_observation_gap_sec > 0.0 &&
      std::isfinite(config.stale_track_sec) &&
      config.stale_track_sec >= config.maximum_observation_gap_sec &&
      std::isfinite(config.association_max_centroid_distance_m) &&
      config.association_max_centroid_distance_m > 0.0F &&
      std::isfinite(config.association_max_top_step_m) &&
      config.association_max_top_step_m > 0.0F &&
      std::isfinite(config.static_velocity_threshold_mps) &&
      config.static_velocity_threshold_mps >= 0.0F;
}

bool validObservation(const CargoObstacleObservation& observation,
                      std::size_t minimum_points) {
  return observation.centroid_map.allFinite() &&
      std::isfinite(observation.top_z95_map) &&
      std::isfinite(observation.footprint_distance_m) &&
      observation.footprint_distance_m >= 0.0F &&
      std::isfinite(observation.conservative_clearance_m) &&
      observation.point_count >= minimum_points &&
      (observation.warning_code == kLevel1 ||
       observation.warning_code == kLevel2);
}

int warningPriority(std::uint16_t code) {
  if (code == kLevel1) return 2;
  if (code == kLevel2) return 1;
  return 0;
}

bool moreDangerous(const CargoObstacleTrack& candidate,
                   const CargoObstacleTrack& current) {
  const int candidate_priority = warningPriority(candidate.warning_code);
  const int current_priority = warningPriority(current.warning_code);
  if (candidate_priority != current_priority) {
    return candidate_priority > current_priority;
  }
  if (candidate.conservative_clearance_m !=
      current.conservative_clearance_m) {
    return candidate.conservative_clearance_m <
        current.conservative_clearance_m;
  }
  if (candidate.footprint_distance_m != current.footprint_distance_m) {
    return candidate.footprint_distance_m < current.footprint_distance_m;
  }
  return candidate.track_id < current.track_id;
}

}  // namespace

CargoObstacleTracker::CargoObstacleTracker(
    const CargoObstacleTrackerConfig& config) {
  setConfig(config);
}

void CargoObstacleTracker::setConfig(
    const CargoObstacleTrackerConfig& config) {
  config_ = validConfig(config) ? config : CargoObstacleTrackerConfig{};
  reset();
}

void CargoObstacleTracker::reset() {
  tracks_.clear();
  next_track_id_ = 1U;
  cycle_ = 0U;
  has_stamp_ = false;
  last_stamp_sec_ = 0.0;
}

CargoObstacleTrackerDecision CargoObstacleTracker::update(
    double stamp_sec,
    const std::vector<CargoObstacleObservation>& observations) {
  CargoObstacleTrackerDecision decision;
  if (!validConfig(config_) || !std::isfinite(stamp_sec) ||
      stamp_sec <= 0.0) {
    decision.reason = "invalid_obstacle_track_input";
    return decision;
  }
  if (has_stamp_ && stamp_sec + kStampEpsilonSec < last_stamp_sec_) {
    reset();
  } else if (has_stamp_ &&
             stamp_sec <= last_stamp_sec_ + kStampEpsilonSec) {
    decision.valid = true;
    decision.reason = "repeated_obstacle_track_stamp";
    return decision;
  }
  has_stamp_ = true;
  last_stamp_sec_ = stamp_sec;
  ++cycle_;

  tracks_.erase(
      std::remove_if(
          tracks_.begin(), tracks_.end(), [&](const CargoObstacleTrack& track) {
            return stamp_sec - track.last_stamp_sec >
                config_.stale_track_sec + kStampEpsilonSec;
          }),
      tracks_.end());
  for (CargoObstacleTrack& track : tracks_) {
    track.observed_this_cycle = false;
  }

  std::vector<bool> track_assigned(tracks_.size(), false);
  for (const CargoObstacleObservation& observation : observations) {
    if (!validObservation(observation, config_.minimum_points)) continue;
    decision.hazard_observed = true;

    std::size_t best_index = tracks_.size();
    float best_distance = std::numeric_limits<float>::infinity();
    for (std::size_t index = 0; index < tracks_.size(); ++index) {
      if (track_assigned[index]) continue;
      const CargoObstacleTrack& track = tracks_[index];
      const double gap_sec = stamp_sec - track.last_stamp_sec;
      if (gap_sec < -kStampEpsilonSec ||
          gap_sec > config_.maximum_observation_gap_sec +
              kStampEpsilonSec) {
        continue;
      }
      const float centroid_distance =
          (observation.centroid_map - track.centroid_map).norm();
      if (centroid_distance >
              config_.association_max_centroid_distance_m ||
          std::abs(observation.top_z95_map - track.top_z95_map) >
              config_.association_max_top_step_m) {
        continue;
      }
      if (centroid_distance < best_distance) {
        best_distance = centroid_distance;
        best_index = index;
      }
    }

    if (best_index == tracks_.size()) {
      CargoObstacleTrack track;
      track.track_id = next_track_id_++;
      if (next_track_id_ == 0U) next_track_id_ = 1U;
      track.centroid_map = observation.centroid_map;
      track.top_z95_map = observation.top_z95_map;
      track.footprint_distance_m = observation.footprint_distance_m;
      track.conservative_clearance_m =
          observation.conservative_clearance_m;
      track.point_count = observation.point_count;
      track.warning_code = observation.warning_code;
      track.consecutive_observations = 1;
      track.first_stamp_sec = stamp_sec;
      track.last_stamp_sec = stamp_sec;
      track.last_observation_cycle = cycle_;
      track.observed_this_cycle = true;
      track.current_source_index = observation.source_index;
      tracks_.push_back(track);
      track_assigned.push_back(true);
      continue;
    }

    CargoObstacleTrack& track = tracks_[best_index];
    const double dt_sec = stamp_sec - track.last_stamp_sec;
    const Eigen::Vector3f previous_centroid = track.centroid_map;
    const bool consecutive =
        track.last_observation_cycle + 1U == cycle_ &&
        dt_sec <= config_.maximum_observation_gap_sec + kStampEpsilonSec;
    track.consecutive_observations = consecutive
        ? track.consecutive_observations + 1 : 1;
    if (dt_sec > kStampEpsilonSec) {
      track.velocity_map =
          (observation.centroid_map - previous_centroid) /
          static_cast<float>(dt_sec);
    }
    track.centroid_map = observation.centroid_map;
    track.top_z95_map = observation.top_z95_map;
    track.footprint_distance_m = observation.footprint_distance_m;
    track.conservative_clearance_m = observation.conservative_clearance_m;
    track.point_count = observation.point_count;
    track.warning_code = observation.warning_code;
    track.last_stamp_sec = stamp_sec;
    track.last_observation_cycle = cycle_;
    track.observed_this_cycle = true;
    track.confirmed =
        track.consecutive_observations >= config_.confirm_frames;
    track.static_obstacle = track.confirmed &&
        track.velocity_map.head<2>().norm() <=
            config_.static_velocity_threshold_mps;
    track.current_source_index = observation.source_index;
    track_assigned[best_index] = true;
  }

  decision.valid = true;
  const CargoObstacleTrack* selected = nullptr;
  const CargoObstacleTrack* candidate = nullptr;
  for (const CargoObstacleTrack& track : tracks_) {
    if (!track.observed_this_cycle) continue;
    if (candidate == nullptr || moreDangerous(track, *candidate)) {
      candidate = &track;
    }
    if (track.confirmed &&
        (selected == nullptr || moreDangerous(track, *selected))) {
      selected = &track;
    }
  }
  const CargoObstacleTrack* diagnostic = selected != nullptr
      ? selected : candidate;
  if (diagnostic != nullptr) {
    decision.selected_track_id = diagnostic->track_id;
    decision.selected_source_index = diagnostic->current_source_index;
    decision.selected_confirm_count =
        diagnostic->consecutive_observations;
    decision.selected_track_age_sec =
        stamp_sec - diagnostic->first_stamp_sec;
    decision.selected_track_static = diagnostic->static_obstacle;
    decision.selected_track_velocity = diagnostic->velocity_map;
  }
  if (selected != nullptr) {
    decision.confirmed_hazard = true;
    decision.warning_code = selected->warning_code;
    decision.reason = "persistent_obstacle_track_confirmed";
  } else if (decision.hazard_observed) {
    decision.reason = "persistent_obstacle_track_pending";
  } else {
    decision.reason = "no_hazard_observation";
  }
  return decision;
}

}  // namespace ndt_slam
