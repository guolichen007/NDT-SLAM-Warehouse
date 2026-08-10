#include "ndt_slam/cargo_obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace ndt_slam {
namespace {

constexpr std::uint16_t kLevel1 = 17U;
constexpr std::uint16_t kLevel2 = 18U;
constexpr std::uint16_t kClear = 14U;
constexpr double kStampEpsilonSec = 1.0e-4;

bool warningCode(std::uint16_t code) {
  return code == kLevel1 || code == kLevel2;
}

std::size_t cellIntersectionCount(
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

float cellOverlap(const std::vector<std::int64_t>& left,
                  const std::vector<std::int64_t>& right) {
  const std::size_t denominator = std::min(left.size(), right.size());
  if (denominator == 0U) return 0.0F;
  return static_cast<float>(cellIntersectionCount(left, right)) /
      static_cast<float>(denominator);
}

float cellIou(const std::vector<std::int64_t>& left,
              const std::vector<std::int64_t>& right) {
  const std::size_t intersection = cellIntersectionCount(left, right);
  const std::size_t union_size = left.size() + right.size() - intersection;
  if (union_size == 0U) return 0.0F;
  return static_cast<float>(intersection) /
      static_cast<float>(union_size);
}

std::pair<std::int32_t, std::int32_t> unpackCell(
    std::int64_t key) {
  const std::uint64_t packed = static_cast<std::uint64_t>(key);
  return {
      static_cast<std::int32_t>(
          static_cast<std::uint32_t>(packed >> 32U)),
      static_cast<std::int32_t>(static_cast<std::uint32_t>(packed))};
}

std::int64_t packCell(std::int32_t x, std::int32_t y) {
  const std::uint64_t packed =
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
      static_cast<std::uint32_t>(y);
  return static_cast<std::int64_t>(packed);
}

float directionalNeighborCellOverlap(
    const std::vector<std::int64_t>& source,
    const std::vector<std::int64_t>& target,
    int radius) {
  if (source.empty() || target.empty() || radius <= 0) return 0.0F;
  std::size_t matched = 0U;
  for (const std::int64_t key : source) {
    const auto xy = unpackCell(key);
    bool found = false;
    for (int dx = -radius; dx <= radius && !found; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        if (std::binary_search(
                target.begin(), target.end(),
                packCell(xy.first + dx, xy.second + dy))) {
          found = true;
          break;
        }
      }
    }
    if (found) ++matched;
  }
  return static_cast<float>(matched) /
      static_cast<float>(source.size());
}

float neighborCellOverlap(
    const std::vector<std::int64_t>& left,
    const std::vector<std::int64_t>& right,
    int radius) {
  return std::max(
      directionalNeighborCellOverlap(left, right, radius),
      directionalNeighborCellOverlap(right, left, radius));
}

bool validConfig(const CargoObstacleTrackerConfig& config) {
  return config.confirm_frames >= 2 && config.minimum_points > 0U &&
      config.far_history_confirm_frames >= 2 &&
      std::isfinite(config.far_history_confirm_duration_sec) &&
      config.far_history_confirm_duration_sec >= 0.0 &&
      std::isfinite(config.level1_warning_distance_m) &&
      config.level1_warning_distance_m > 0.0F &&
      std::isfinite(config.level2_warning_distance_m) &&
      config.level2_warning_distance_m >
          config.level1_warning_distance_m &&
      std::isfinite(config.acquisition_distance_m) &&
      config.acquisition_distance_m > config.level2_warning_distance_m &&
      std::isfinite(config.embedded_distance_threshold_m) &&
      config.embedded_distance_threshold_m >= 0.0F &&
      config.embedded_distance_threshold_m <
          config.level1_warning_distance_m &&
      std::isfinite(config.maximum_observation_gap_sec) &&
      config.maximum_observation_gap_sec > 0.0 &&
      std::isfinite(config.stale_track_sec) &&
      config.stale_track_sec >= config.maximum_observation_gap_sec &&
      std::isfinite(config.association_max_centroid_distance_m) &&
      config.association_max_centroid_distance_m > 0.0F &&
      std::isfinite(config.association_max_top_step_m) &&
      config.association_max_top_step_m > 0.0F &&
      config.association_neighbor_cell_radius >= 0 &&
      config.association_neighbor_cell_radius <= 2 &&
      std::isfinite(config.static_track_cell_overlap_min) &&
      config.static_track_cell_overlap_min > 0.0F &&
      config.static_track_cell_overlap_min <= 1.0F &&
      std::isfinite(config.static_track_iou_min) &&
      config.static_track_iou_min > 0.0F &&
      config.static_track_iou_min <= 1.0F &&
      std::isfinite(config.static_provenance_min_cargo_motion_m) &&
      config.static_provenance_min_cargo_motion_m >= 0.0F &&
      config.static_cargo_min_voxel_points >= config.minimum_points &&
      std::isfinite(config.static_cargo_min_xy_area_m2) &&
      config.static_cargo_min_xy_area_m2 > 0.0F &&
      std::isfinite(config.static_cargo_min_long_side_m) &&
      config.static_cargo_min_long_side_m > 0.0F &&
      std::isfinite(config.static_cargo_min_height_span_m) &&
      config.static_cargo_min_height_span_m > 0.0F &&
      config.static_cargo_min_occupied_cells > 0U &&
      config.known_static_confirm_frames >= config.confirm_frames &&
      config.static_cargo_confirm_frames >= config.confirm_frames &&
      std::isfinite(config.static_cargo_confirm_sec) &&
      config.static_cargo_confirm_sec >= 0.0 &&
      std::isfinite(config.static_velocity_threshold_mps) &&
      config.static_velocity_threshold_mps >= 0.0F;
}

bool qualifiesForFarHistory(
    const CargoObstacleObservation& observation,
    const CargoObstacleTrackerConfig& config) {
  if (!observation.source_validated ||
      !std::isfinite(observation.footprint_distance_m) ||
      !std::isfinite(observation.horizontal_uncertainty_m) ||
      observation.horizontal_uncertainty_m < 0.0F) {
    return false;
  }
  const float safe_distance = observation.footprint_distance_m -
      observation.horizontal_uncertainty_m;
  return safe_distance > config.level2_warning_distance_m &&
      observation.footprint_distance_m <= config.acquisition_distance_m;
}

bool validObservation(const CargoObstacleObservation& observation,
                      const CargoObstacleTrackerConfig& config) {
  const bool warning_distance_valid =
      observation.warning_code == kLevel1
      ? observation.footprint_distance_m <=
            config.level1_warning_distance_m
      : observation.warning_code == kLevel2 &&
            observation.footprint_distance_m >
                config.level1_warning_distance_m &&
            observation.footprint_distance_m <=
                config.level2_warning_distance_m;
  const bool decision_code_valid = observation.warning_eligible
      ? warningCode(observation.warning_code) && warning_distance_valid
      : observation.warning_code == kClear &&
            observation.footprint_distance_m >
                config.level2_warning_distance_m &&
            observation.footprint_distance_m <=
                config.acquisition_distance_m;
  return observation.centroid_map.allFinite() &&
      std::isfinite(observation.top_z95_map) &&
      std::isfinite(observation.bottom_z05_map) &&
      observation.top_z95_map >= observation.bottom_z05_map &&
      std::isfinite(observation.vertical_continuity_ratio) &&
      observation.vertical_continuity_ratio >= 0.0F &&
      observation.vertical_continuity_ratio <= 1.0F &&
      (!observation.warning_eligible || !observation.entirely_above_cargo) &&
      std::isfinite(observation.footprint_distance_m) &&
      observation.footprint_distance_m >= 0.0F &&
      std::isfinite(observation.conservative_clearance_m) &&
      std::isfinite(observation.horizontal_uncertainty_m) &&
      observation.horizontal_uncertainty_m >= 0.0F &&
      observation.point_count >= config.minimum_points &&
      decision_code_valid;
}

bool hasLargeStaticCargoGeometry(
    const CargoObstacleObservation& observation,
    const CargoObstacleTrackerConfig& config) {
  const bool raw_support_valid =
      config.static_cargo_min_raw_equivalent_points == 0U ||
      observation.raw_equivalent_point_count >=
          config.static_cargo_min_raw_equivalent_points;
  return observation.point_count >= config.static_cargo_min_voxel_points &&
      raw_support_valid && std::isfinite(observation.xy_area_m2) &&
      observation.xy_area_m2 >= config.static_cargo_min_xy_area_m2 &&
      std::isfinite(observation.long_side_m) &&
      observation.long_side_m >= config.static_cargo_min_long_side_m &&
      std::isfinite(observation.height_span_m) &&
      observation.height_span_m >= config.static_cargo_min_height_span_m &&
      observation.occupied_cells >= config.static_cargo_min_occupied_cells;
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

const char* externalProvenanceName(
    ExternalProvenance provenance) noexcept {
  switch (provenance) {
    case ExternalProvenance::OUTSIDE_CARGO_SHELL_ONLY:
      return "OUTSIDE_CARGO_SHELL_ONLY";
    case ExternalProvenance::PRE_CARGO_OCCUPANCY:
      return "PRE_CARGO_OCCUPANCY";
    case ExternalProvenance::STATIC_MAP_MATCH:
      return "STATIC_MAP_MATCH";
    case ExternalProvenance::CARGO_MOVED_AWAY_PERSISTENCE:
      return "CARGO_MOVED_AWAY_PERSISTENCE";
    case ExternalProvenance::DUAL_LIDAR_CONSENSUS:
      return "DUAL_LIDAR_CONSENSUS";
    case ExternalProvenance::NONE:
    default:
      return "NONE";
  }
}

bool authorizesStaticObstacle(ExternalProvenance provenance) noexcept {
  return provenance == ExternalProvenance::PRE_CARGO_OCCUPANCY ||
      provenance == ExternalProvenance::STATIC_MAP_MATCH ||
      provenance == ExternalProvenance::CARGO_MOVED_AWAY_PERSISTENCE ||
      provenance == ExternalProvenance::DUAL_LIDAR_CONSENSUS;
}

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
  decision.created_track_count = created_track_count_;
  decision.association_reset_count = association_reset_count_;
  if (!validConfig(config_) || !std::isfinite(stamp_sec) ||
      stamp_sec <= 0.0) {
    decision.reason = "invalid_obstacle_track_input";
    return decision;
  }
  if (has_stamp_ && stamp_sec + kStampEpsilonSec < last_stamp_sec_) {
    reset();
    decision.created_track_count = created_track_count_;
    decision.association_reset_count = association_reset_count_;
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
  for (CargoObstacleObservation observation : observations) {
    if (!validObservation(observation, config_)) continue;
    std::sort(observation.occupied_map_cells.begin(),
              observation.occupied_map_cells.end());
    observation.occupied_map_cells.erase(
        std::unique(observation.occupied_map_cells.begin(),
                    observation.occupied_map_cells.end()),
        observation.occupied_map_cells.end());
    decision.hazard_observed =
        decision.hazard_observed ||
        (observation.warning_eligible &&
         warningCode(observation.warning_code));

    std::size_t best_index = tracks_.size();
    float best_cost = std::numeric_limits<float>::infinity();
    float best_overlap = 0.0F;
    float best_iou = 0.0F;
    float best_neighbor_overlap = 0.0F;
    for (std::size_t index = 0; index < tracks_.size(); ++index) {
      if (track_assigned[index]) continue;
      const CargoObstacleTrack& track = tracks_[index];
      const double gap_sec = stamp_sec - track.last_stamp_sec;
      if (gap_sec < -kStampEpsilonSec ||
          gap_sec > config_.maximum_observation_gap_sec +
              kStampEpsilonSec) {
        continue;
      }
      const Eigen::Vector3f predicted_centroid = track.centroid_map +
          track.velocity_map * static_cast<float>(std::max(0.0, gap_sec));
      const float centroid_distance =
          (observation.centroid_map - predicted_centroid).norm();
      const float overlap = cellOverlap(
          observation.occupied_map_cells, track.occupied_map_cells);
      const float iou = cellIou(
          observation.occupied_map_cells, track.occupied_map_cells);
      const float neighbor_overlap = neighborCellOverlap(
          observation.occupied_map_cells, track.occupied_map_cells,
          config_.association_neighbor_cell_radius);
      const bool neighboring_fragment_match =
          neighbor_overlap >= config_.static_track_cell_overlap_min &&
          centroid_distance <=
              2.0F * config_.association_max_centroid_distance_m;
      const bool spatial_match = centroid_distance <=
              config_.association_max_centroid_distance_m ||
          overlap >= config_.static_track_cell_overlap_min ||
          iou >= config_.static_track_iou_min ||
          neighboring_fragment_match;
      if (!spatial_match ||
          std::abs(observation.top_z95_map - track.top_z95_map) >
              config_.association_max_top_step_m) {
        continue;
      }
      const float normalized_distance = centroid_distance /
          config_.association_max_centroid_distance_m;
      const float cost = normalized_distance +
          (observation.occupied_map_cells.empty() ||
                   track.occupied_map_cells.empty()
               ? 1.0F
               : 1.0F - std::max(
                     std::max(overlap, iou),
                     0.80F * neighbor_overlap));
      if (cost < best_cost) {
        best_cost = cost;
        best_overlap = overlap;
        best_iou = iou;
        best_neighbor_overlap = neighbor_overlap;
        best_index = index;
      }
    }

    if (best_index == tracks_.size()) {
      CargoObstacleTrack track;
      track.track_id = next_track_id_++;
      if (next_track_id_ == 0U) next_track_id_ = 1U;
      track.centroid_map = observation.centroid_map;
      track.top_z95_map = observation.top_z95_map;
      track.bottom_z05_map = observation.bottom_z05_map;
      track.vertical_continuity_ratio =
          observation.vertical_continuity_ratio;
      track.entirely_above_cargo = observation.entirely_above_cargo;
      track.footprint_distance_m = observation.footprint_distance_m;
      track.conservative_clearance_m =
          observation.conservative_clearance_m;
      track.point_count = observation.point_count;
      track.warning_code = observation.warning_code;
      track.total_consecutive_observations = 1;
      track.validated_consecutive_observations =
          observation.source_validated ? 1 : 0;
      track.consecutive_observations =
          track.total_consecutive_observations;
      track.large_cluster_geometry_valid =
          hasLargeStaticCargoGeometry(observation, config_);
      track.geometry_validated_consecutive_observations =
          observation.source_validated &&
                  track.large_cluster_geometry_valid
              ? 1 : 0;
      track.provenance = observation.provenance;
      track.provenance_valid =
          authorizesStaticObstacle(observation.provenance);
      track.certified_static_provenance =
          observation.certified_static_provenance;
      track.current_embedded =
          observation.footprint_distance_m <
          config_.embedded_distance_threshold_m;
      track.current_near_field =
          observation.footprint_distance_m <=
          config_.level1_warning_distance_m;
      track.separated_validated_observations =
          observation.source_validated && !track.current_embedded ? 1 : 0;
      track.separated_obstacle_history_valid =
          track.separated_validated_observations >= config_.confirm_frames;
      const bool qualifies_far =
          qualifiesForFarHistory(observation, config_);
      track.far_field_validated_observations = qualifies_far ? 1 : 0;
      track.far_field_first_stamp_sec = qualifies_far ? stamp_sec : 0.0;
      track.far_field_duration_sec = 0.0;
      track.far_field_history_valid =
          track.far_field_validated_observations >=
              config_.far_history_confirm_frames &&
          config_.far_history_confirm_duration_sec <= kStampEpsilonSec;
      track.occupied_map_cells = observation.occupied_map_cells;
      track.identity_anchor_map_cells = observation.occupied_map_cells;
      track.first_cargo_center_valid = observation.cargo_center_valid &&
          observation.cargo_center_map.allFinite();
      if (track.first_cargo_center_valid) {
        track.first_cargo_center_map = observation.cargo_center_map;
      }
      track.association_reset_reason = tracks_.empty()
          ? "new_obstacle_track"
          : "static_track_association_reset";
      ++created_track_count_;
      if (!tracks_.empty()) ++association_reset_count_;
      decision.created_track_count = created_track_count_;
      decision.association_reset_count = association_reset_count_;
      track.static_provenance_consecutive_observations =
          observation.source_validated &&
                  track.large_cluster_geometry_valid &&
                  track.provenance_valid
              ? 1 : 0;
      if (track.static_provenance_consecutive_observations > 0) {
        track.static_provenance_first_stamp_sec = stamp_sec;
      }
      track.first_stamp_sec = stamp_sec;
      track.last_stamp_sec = stamp_sec;
      track.last_observation_cycle = cycle_;
      track.observed_this_cycle = true;
      track.current_source_index = observation.source_index;
      track.current_source_validated = observation.source_validated;
      track.current_warning_eligible =
          observation.warning_eligible &&
          warningCode(observation.warning_code);
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
    track.total_consecutive_observations = consecutive
        ? track.total_consecutive_observations + 1 : 1;
    track.consecutive_observations =
        track.total_consecutive_observations;
    track.validated_consecutive_observations =
        observation.source_validated
            ? (consecutive
                   ? track.validated_consecutive_observations + 1
                   : 1)
            : 0;
    if (dt_sec > kStampEpsilonSec) {
      track.velocity_map =
          (observation.centroid_map - previous_centroid) /
          static_cast<float>(dt_sec);
    }
    track.centroid_map = observation.centroid_map;
    track.top_z95_map = observation.top_z95_map;
    track.bottom_z05_map = observation.bottom_z05_map;
    track.vertical_continuity_ratio =
        observation.vertical_continuity_ratio;
    track.entirely_above_cargo = observation.entirely_above_cargo;
    track.footprint_distance_m = observation.footprint_distance_m;
    track.conservative_clearance_m = observation.conservative_clearance_m;
    track.point_count = observation.point_count;
    track.warning_code = observation.warning_code;
    track.large_cluster_geometry_valid =
        hasLargeStaticCargoGeometry(observation, config_);
    track.geometry_validated_consecutive_observations =
        observation.source_validated &&
                track.large_cluster_geometry_valid
            ? (consecutive
                   ? track.geometry_validated_consecutive_observations + 1
                   : 1)
            : 0;
    track.last_cell_overlap = best_overlap;
    track.last_cell_iou = best_iou;
    track.last_neighbor_cell_overlap = best_neighbor_overlap;
    track.last_anchor_cell_overlap = cellOverlap(
        observation.occupied_map_cells,
        track.identity_anchor_map_cells);
    track.last_association_cost = best_cost;
    track.association_reset_reason.clear();
    if (observation.cargo_center_valid &&
        observation.cargo_center_map.allFinite()) {
      if (!track.first_cargo_center_valid) {
        track.first_cargo_center_map = observation.cargo_center_map;
        track.first_cargo_center_valid = true;
      }
      track.maximum_cargo_displacement_m = std::max(
          track.maximum_cargo_displacement_m,
          (observation.cargo_center_map -
           track.first_cargo_center_map).norm());
    }
    const bool map_identity_stable =
        track.last_anchor_cell_overlap >=
            config_.static_track_cell_overlap_min;
    const bool static_motion_valid =
        track.velocity_map.head<2>().norm() <=
            config_.static_velocity_threshold_mps ||
        track.last_anchor_cell_overlap >= 0.50F;
    if (authorizesStaticObstacle(observation.provenance)) {
      track.provenance = observation.provenance;
    } else if (!track.provenance_valid &&
               observation.source_validated &&
               track.large_cluster_geometry_valid &&
               map_identity_stable &&
               track.maximum_cargo_displacement_m >=
                   config_.static_provenance_min_cargo_motion_m &&
               static_motion_valid) {
      // This is independent physical evidence: the cargo has moved in map,
      // while the obstacle remains on the same occupied map cells.
      track.provenance =
          ExternalProvenance::CARGO_MOVED_AWAY_PERSISTENCE;
    } else if (!track.provenance_valid) {
      track.provenance = observation.provenance;
    }
    track.provenance_valid = authorizesStaticObstacle(track.provenance);
    track.certified_static_provenance =
        track.certified_static_provenance ||
        observation.certified_static_provenance;
    track.current_embedded =
        observation.footprint_distance_m <
        config_.embedded_distance_threshold_m;
    if (observation.source_validated && !track.current_embedded) {
      track.separated_validated_observations = consecutive
          ? track.separated_validated_observations + 1 : 1;
      if (track.separated_validated_observations >=
          config_.confirm_frames) {
        track.separated_obstacle_history_valid = true;
      }
    } else if (!track.current_embedded) {
      track.separated_validated_observations = 0;
    }
    track.current_near_field =
        observation.footprint_distance_m <=
        config_.level1_warning_distance_m;
    if (!track.far_field_history_valid) {
      if (qualifiesForFarHistory(observation, config_)) {
        if (!consecutive ||
            track.far_field_validated_observations == 0) {
          track.far_field_validated_observations = 1;
          track.far_field_first_stamp_sec = stamp_sec;
        } else {
          ++track.far_field_validated_observations;
        }
        track.far_field_duration_sec = std::max(
            0.0, stamp_sec - track.far_field_first_stamp_sec);
        if (track.far_field_validated_observations >=
                config_.far_history_confirm_frames &&
            track.far_field_duration_sec + kStampEpsilonSec >=
                config_.far_history_confirm_duration_sec) {
          track.far_field_history_valid = true;
        }
      } else {
        // Entering <=5 m before both evidence gates complete remains an
        // anomalous near-field appearance; one earlier sample cannot promote
        // the track to normal warning authority.
        track.far_field_validated_observations = 0;
        track.far_field_first_stamp_sec = 0.0;
        track.far_field_duration_sec = 0.0;
      }
    }
    track.occupied_map_cells = observation.occupied_map_cells;
    const bool static_provenance_frame = observation.source_validated &&
        track.large_cluster_geometry_valid &&
        track.provenance_valid &&
        static_motion_valid;
    if (static_provenance_frame) {
      if (!consecutive ||
          track.static_provenance_consecutive_observations == 0) {
        track.static_provenance_consecutive_observations = 1;
        track.static_provenance_first_stamp_sec = stamp_sec;
      } else {
        ++track.static_provenance_consecutive_observations;
      }
    } else {
      track.static_provenance_consecutive_observations = 0;
      track.static_provenance_first_stamp_sec = 0.0;
    }
    track.last_stamp_sec = stamp_sec;
    track.last_observation_cycle = cycle_;
    track.observed_this_cycle = true;
    track.confirmed = track.validated_consecutive_observations >=
        config_.confirm_frames;
    const bool known_static_source =
        track.provenance == ExternalProvenance::STATIC_MAP_MATCH ||
        track.provenance == ExternalProvenance::PRE_CARGO_OCCUPANCY;
    const int required_static_frames = known_static_source
        ? config_.known_static_confirm_frames
        : config_.static_cargo_confirm_frames;
    const double required_static_duration_sec = known_static_source
        ? 0.0 : config_.static_cargo_confirm_sec;
    track.static_obstacle = track.confirmed &&
        track.static_provenance_consecutive_observations >=
            required_static_frames &&
        track.static_provenance_first_stamp_sec > 0.0 &&
        stamp_sec - track.static_provenance_first_stamp_sec +
                kStampEpsilonSec >=
            required_static_duration_sec;
    track.current_source_index = observation.source_index;
    track.current_source_validated = observation.source_validated;
    track.current_warning_eligible =
        observation.warning_eligible &&
        warningCode(observation.warning_code);
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
    const bool warning_authorized = track.current_warning_eligible &&
        warningCode(track.warning_code) && track.confirmed &&
        (!config_.require_far_field_history_for_warnings ||
         track.far_field_history_valid ||
         track.certified_static_provenance) &&
        (!config_.require_large_geometry_for_warning ||
         track.geometry_validated_consecutive_observations >=
             config_.confirm_frames) &&
        track.current_source_validated &&
        (!track.current_embedded ||
         track.separated_obstacle_history_valid ||
         track.provenance_valid) &&
        (!config_.require_static_cargo_for_warning || track.static_obstacle);
    if (warning_authorized &&
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
        diagnostic->validated_consecutive_observations;
    decision.selected_geometry_confirm_count =
        diagnostic->geometry_validated_consecutive_observations;
    decision.selected_track_age_sec =
        stamp_sec - diagnostic->first_stamp_sec;
    decision.selected_track_static = diagnostic->static_obstacle;
    decision.selected_large_geometry_valid =
        diagnostic->large_cluster_geometry_valid;
    decision.selected_provenance = diagnostic->provenance;
    decision.selected_provenance_valid = diagnostic->provenance_valid;
    decision.selected_embedded = diagnostic->current_embedded;
    decision.selected_embedded_authorized =
        !diagnostic->current_embedded ||
        diagnostic->separated_obstacle_history_valid ||
        diagnostic->provenance_valid;
    decision.selected_separated_observations =
        diagnostic->separated_validated_observations;
    decision.selected_near_field = diagnostic->current_near_field;
    decision.selected_near_field_authorized =
        !diagnostic->current_near_field ||
        diagnostic->far_field_history_valid ||
        diagnostic->certified_static_provenance;
    decision.selected_far_field_history_valid =
        diagnostic->far_field_history_valid;
    decision.selected_far_field_observations =
        diagnostic->far_field_validated_observations;
    decision.selected_far_field_duration_sec =
        diagnostic->far_field_duration_sec;
    decision.selected_certified_static_provenance =
        diagnostic->certified_static_provenance;
    decision.selected_static_provenance_streak =
        diagnostic->static_provenance_consecutive_observations;
    decision.selected_static_age_sec =
        diagnostic->static_provenance_first_stamp_sec > 0.0
            ? std::max(
                  0.0, stamp_sec -
                      diagnostic->static_provenance_first_stamp_sec)
            : 0.0;
    decision.selected_track_cell_overlap = diagnostic->last_cell_overlap;
    decision.selected_track_iou = diagnostic->last_cell_iou;
    decision.selected_track_neighbor_cell_overlap =
        diagnostic->last_neighbor_cell_overlap;
    decision.selected_association_cost = diagnostic->last_association_cost;
    decision.selected_association_reset_reason =
        diagnostic->association_reset_reason;
    decision.selected_track_velocity = diagnostic->velocity_map;
  }
  if (selected != nullptr) {
    decision.confirmed_hazard = true;
    decision.warning_code = selected->warning_code;
    decision.reason = "persistent_obstacle_track_confirmed";
  } else if (decision.hazard_observed) {
    if (diagnostic == nullptr) {
      decision.reason = "static_track_association_reset";
    } else if (!diagnostic->current_source_validated) {
      decision.reason = "current_source_unvalidated";
    } else if (diagnostic->current_embedded &&
               !diagnostic->separated_obstacle_history_valid &&
               !diagnostic->provenance_valid) {
      decision.reason = "embedded_obstacle_origin_unresolved";
    } else if (config_.require_far_field_history_for_warnings &&
               warningCode(diagnostic->warning_code) &&
               !diagnostic->far_field_history_valid &&
               !diagnostic->certified_static_provenance) {
      decision.reason = "warning_track_missing_true_far_history";
    } else if (!config_.require_static_cargo_for_warning) {
      decision.reason = "persistent_obstacle_track_pending";
    } else if (config_.require_large_geometry_for_warning &&
               (!diagnostic->large_cluster_geometry_valid ||
                diagnostic->geometry_validated_consecutive_observations <
                    config_.confirm_frames)) {
      decision.reason = "static_geometry_below_threshold";
    } else if (!diagnostic->provenance_valid) {
      decision.reason = "static_provenance_unavailable";
    } else if (diagnostic->velocity_map.head<2>().norm() >
                   config_.static_velocity_threshold_mps &&
               diagnostic->last_anchor_cell_overlap < 0.50F) {
      decision.reason = "static_velocity_not_stable";
    } else if (diagnostic->static_provenance_consecutive_observations <
               ((diagnostic->provenance ==
                     ExternalProvenance::STATIC_MAP_MATCH ||
                 diagnostic->provenance ==
                     ExternalProvenance::PRE_CARGO_OCCUPANCY)
                    ? config_.known_static_confirm_frames
                    : config_.static_cargo_confirm_frames)) {
      decision.reason = "static_frames_pending";
    } else if (diagnostic->static_provenance_first_stamp_sec <= 0.0 ||
               stamp_sec - diagnostic->static_provenance_first_stamp_sec +
                       kStampEpsilonSec <
                   ((diagnostic->provenance ==
                         ExternalProvenance::STATIC_MAP_MATCH ||
                     diagnostic->provenance ==
                         ExternalProvenance::PRE_CARGO_OCCUPANCY)
                        ? 0.0
                        : config_.static_cargo_confirm_sec)) {
      decision.reason = "static_duration_pending";
    } else if (!diagnostic->association_reset_reason.empty()) {
      decision.reason = "static_track_association_reset";
    } else {
      decision.reason = "static_authorization_pending";
    }
  } else if (candidate != nullptr) {
    decision.reason = "directional_collision_track_acquiring";
  } else {
    decision.reason = "clear_no_hazard_cluster";
  }
  return decision;
}

}  // namespace ndt_slam
