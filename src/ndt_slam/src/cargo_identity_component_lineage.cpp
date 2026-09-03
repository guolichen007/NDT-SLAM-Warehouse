#include "ndt_slam/cargo_identity_component_lineage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ndt_slam {
namespace {

constexpr double kEpsilon = 1.0e-9;

bool finiteComponent(const CargoIdentityComponentDescriptor& component,
                     double source_stamp_sec) {
  return component.source_stamp_sec > 0.0 &&
      std::abs(component.source_stamp_sec - source_stamp_sec) <= kEpsilon &&
      component.center_base.allFinite() &&
      component.robust_xy_extent.allFinite() &&
      (component.robust_xy_extent.array() > 0.0).all() &&
      std::isfinite(component.robust_x05) &&
      std::isfinite(component.robust_x95) &&
      std::isfinite(component.robust_y05) &&
      std::isfinite(component.robust_y95) &&
      component.robust_x95 > component.robust_x05 &&
      component.robust_y95 > component.robust_y05 &&
      component.exact_seed_frame_group_id != 0U;
}

bool finiteFrame(const CargoIdentityComponentLineageFrame& frame) {
  return std::isfinite(frame.source_stamp_sec) &&
      frame.source_stamp_sec > 0.0 && frame.lifecycle_id != 0U &&
      frame.source_frame_identity.valid() &&
      std::abs(frame.source_frame_identity.sensor_source_stamp_sec -
               frame.source_stamp_sec) <= 1.0e-6 &&
      frame.pose_map_base.allFinite();
}

Eigen::Vector2d mapCenter(
    const CargoIdentityComponentDescriptor& component,
    const Eigen::Matrix4d& pose_map_base) {
  const Eigen::Vector4d point(
      component.center_base.x(), component.center_base.y(), 0.0, 1.0);
  return (pose_map_base * point).head<2>();
}

double extentStep(const CargoIdentityComponentDescriptor& previous,
                  const CargoIdentityComponentDescriptor& current) {
  double maximum = 0.0;
  for (int axis = 0; axis < 2; ++axis) {
    const double denominator = std::max(
        std::max(std::abs(previous.robust_xy_extent[axis]),
                 std::abs(current.robust_xy_extent[axis])), kEpsilon);
    maximum = std::max(maximum, std::abs(
        previous.robust_xy_extent[axis] -
        current.robust_xy_extent[axis]) / denominator);
  }
  return maximum;
}

double egoXyStep(const CargoIdentityComponentLineageFrame& previous,
                 const CargoIdentityComponentLineageFrame& current) {
  return (current.pose_map_base.block<2, 1>(0, 3) -
          previous.pose_map_base.block<2, 1>(0, 3)).norm();
}

CargoIdentityMotionObservabilityState motionObservability(
    double ego_xy_step_m, double maximum_xy_step_m,
    bool gravity_valid, HookLoadState gravity_state) {
  if (ego_xy_step_m > maximum_xy_step_m + kEpsilon) {
    return CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE;
  }
  if (!gravity_valid) {
    return CargoIdentityMotionObservabilityState::UNKNOWN_FAIL_CLOSED;
  }
  if (gravity_state == HookLoadState::LOADED) {
    return CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE;
  }
  if (gravity_state == HookLoadState::EMPTY) {
    return CargoIdentityMotionObservabilityState::IDLE_ZERO_LOAD;
  }
  return CargoIdentityMotionObservabilityState::UNKNOWN_FAIL_CLOSED;
}

}  // namespace

const char* cargoIdentityMotionObservabilityStateName(
    CargoIdentityMotionObservabilityState state) noexcept {
  switch (state) {
    case CargoIdentityMotionObservabilityState::UNKNOWN_FAIL_CLOSED:
      return "UNKNOWN_FAIL_CLOSED";
    case CargoIdentityMotionObservabilityState::IDLE_ZERO_LOAD:
      return "IDLE_ZERO_LOAD";
    case CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE:
      return "LOAD_PRESENT_UNOBSERVABLE";
    case CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE:
      return "EGO_MOTION_OBSERVABLE";
    case CargoIdentityMotionObservabilityState::WORLD_STATIC_PROVEN:
      return "WORLD_STATIC_PROVEN";
  }
  return "UNKNOWN_FAIL_CLOSED";
}

CargoIdentityComponentLineage::CargoIdentityComponentLineage(
    const CargoIdentityComponentLineageConfig& config) {
  setConfig(config);
  reset_reason_ = "constructed";
}

void CargoIdentityComponentLineage::setConfig(
    const CargoIdentityComponentLineageConfig& config) {
  config_ = config;
  if (!(config_.maximum_xy_step_m > 0.0)) {
    config_.maximum_xy_step_m = 0.30;
  }
  if (!(config_.maximum_size_relative_step >= 0.0)) {
    config_.maximum_size_relative_step = 0.60;
  }
  if (!(config_.maximum_observation_gap_sec > 0.0)) {
    config_.maximum_observation_gap_sec = 0.50;
  }
  if (!(config_.ambiguity_cost_margin >= 0.0)) {
    config_.ambiguity_cost_margin = 0.08;
  }
  reset("config_changed");
}

void CargoIdentityComponentLineage::reset(const std::string& reason) {
  recent_frames_.clear();
  reset_reason_ = reason;
}

CargoIdentityComponentLineageResult CargoIdentityComponentLineage::update(
    const CargoIdentityComponentLineageFrame& frame) {
  CargoIdentityComponentLineageResult result;
  result.gravity_valid = frame.gravity_valid;
  result.gravity_state = frame.gravity_state;
  if (!finiteFrame(frame)) {
    reset("invalid_current_frame");
    result.reset_reason = reset_reason_;
    return result;
  }

  CargoIdentityComponentLineageFrame current = frame;
  current.components.erase(
      std::remove_if(current.components.begin(), current.components.end(),
                     [&](const CargoIdentityComponentDescriptor& component) {
                       return !finiteComponent(component,
                                               current.source_stamp_sec);
                     }),
      current.components.end());

  if (!recent_frames_.empty()) {
    const auto& newest = recent_frames_.back();
    const double gap = current.source_stamp_sec - newest.source_stamp_sec;
    std::string reason;
    if (!(gap > 0.0)) {
      reason = "source_stamp_rollback";
    } else if (gap > config_.maximum_observation_gap_sec) {
      reason = "source_gap";
    } else if (current.source_frame_identity.processing_frame_index <=
               newest.source_frame_identity.processing_frame_index) {
      reason = "source_frame_sequence_changed";
    } else if (current.source_frame_identity.time_epoch_id !=
               newest.source_frame_identity.time_epoch_id) {
      reason = "source_frame_epoch_changed";
    } else if (!samePoseAuthorityIdentity(
                   current.pose_identity, newest.pose_identity)) {
      reason = "pose_authority_changed";
    } else if (current.lifecycle_id != newest.lifecycle_id) {
      reason = "lifecycle_changed";
    }
    if (!reason.empty()) {
      reset(reason);
      result.reset_reason = reason;
    }
  }

  if (recent_frames_.empty()) {
    recent_frames_.push_back(std::move(current));
    if (result.reset_reason == "none") result.reset_reason = reset_reason_;
    return result;
  }

  const auto& newest = recent_frames_.back();
  result.ego_xy_step_m = egoXyStep(newest, current);
  result.motion_observability_state = motionObservability(
      result.ego_xy_step_m, config_.maximum_xy_step_m,
      current.gravity_valid, current.gravity_state);
  result.load_present_unobservable =
      result.motion_observability_state ==
      CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE;

  struct Pair {
    std::size_t previous = 0U;
    std::size_t current = 0U;
    double base_step = std::numeric_limits<double>::infinity();
    double map_step = std::numeric_limits<double>::infinity();
    double extent_step = std::numeric_limits<double>::infinity();
    double cost = std::numeric_limits<double>::infinity();
    bool base_feasible = false;
    bool world_static_feasible = false;
    bool extent_feasible = false;
  };

  // Evaluate newest-to-oldest.  Multiple observations may be emitted for one
  // current component because only CargoPhysicalIdentityAuthority knows which
  // prior component stamp is actually bound to an existing Cargo history.
  // An ambiguity at a newer source frame blocks older fallback for that
  // current component; fallback must never bypass reciprocal competition.
  std::vector<std::uint64_t> ambiguity_blocked_current_ids;
  std::uint64_t source_frame_offset = 0U;
  for (auto previous_frame = recent_frames_.rbegin();
       previous_frame != recent_frames_.rend(); ++previous_frame) {
    ++source_frame_offset;
    const double source_age_sec =
        current.source_stamp_sec - previous_frame->source_stamp_sec;
    if (!(source_age_sec > 0.0) ||
        source_age_sec > config_.maximum_observation_gap_sec) {
      continue;
    }
    const double pair_ego_xy_step_m = egoXyStep(*previous_frame, current);
    const auto pair_motion_observability = motionObservability(
        pair_ego_xy_step_m, config_.maximum_xy_step_m,
        current.gravity_valid, current.gravity_state);

    std::vector<Pair> pairs;
    pairs.reserve(previous_frame->components.size() *
                  current.components.size());
    for (std::size_t pi = 0U; pi < previous_frame->components.size(); ++pi) {
      for (std::size_t ci = 0U; ci < current.components.size(); ++ci) {
        const auto& previous = previous_frame->components[pi];
        const auto& component = current.components[ci];
        Pair pair;
        pair.previous = pi;
        pair.current = ci;
        pair.base_step = (component.center_base - previous.center_base).norm();
        pair.map_step = (mapCenter(component, current.pose_map_base) -
                         mapCenter(previous,
                                   previous_frame->pose_map_base)).norm();
        pair.extent_step = extentStep(previous, component);
        pair.base_feasible = pair.base_step <=
            config_.maximum_xy_step_m + kEpsilon;
        pair.world_static_feasible = pair.map_step <=
            config_.maximum_xy_step_m + kEpsilon;
        pair.extent_feasible = pair.extent_step <=
            config_.maximum_size_relative_step + kEpsilon;
        pair.cost = pair.base_step / config_.maximum_xy_step_m;
        pairs.push_back(pair);
      }
    }
    result.pair_count += pairs.size();

    std::vector<int> current_best(current.components.size(), -1);
    std::vector<int> previous_best(previous_frame->components.size(), -1);
    std::vector<bool> current_ambiguous(current.components.size(), false);
    std::vector<bool> previous_ambiguous(
        previous_frame->components.size(), false);

    const auto eligible = [&](const Pair& pair) {
      if (!pair.base_feasible || !pair.extent_feasible) return false;
      if (pair_motion_observability ==
          CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE) {
        return !pair.world_static_feasible;
      }
      return pair_motion_observability ==
          CargoIdentityMotionObservabilityState::LOAD_PRESENT_UNOBSERVABLE;
    };
    for (std::size_t ci = 0U; ci < current.components.size(); ++ci) {
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0U; index < pairs.size(); ++index) {
        if (pairs[index].current == ci && eligible(pairs[index]) &&
            pairs[index].cost < best) {
          best = pairs[index].cost;
          current_best[ci] = static_cast<int>(index);
        }
      }
      if (current_best[ci] >= 0) {
        const int competitive = static_cast<int>(std::count_if(
            pairs.begin(), pairs.end(), [&](const Pair& pair) {
              return pair.current == ci && eligible(pair) &&
                  pair.cost <= best + config_.ambiguity_cost_margin +
                      kEpsilon;
            }));
        current_ambiguous[ci] = competitive != 1;
        if (current_ambiguous[ci]) {
          ambiguity_blocked_current_ids.push_back(
              current.components[ci].component_id);
        }
      }
    }
    for (std::size_t pi = 0U; pi < previous_frame->components.size(); ++pi) {
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0U; index < pairs.size(); ++index) {
        if (pairs[index].previous == pi && eligible(pairs[index]) &&
            pairs[index].cost < best) {
          best = pairs[index].cost;
          previous_best[pi] = static_cast<int>(index);
        }
      }
      if (previous_best[pi] >= 0) {
        const int competitive = static_cast<int>(std::count_if(
            pairs.begin(), pairs.end(), [&](const Pair& pair) {
              return pair.previous == pi && eligible(pair) &&
                  pair.cost <= best + config_.ambiguity_cost_margin +
                      kEpsilon;
            }));
        previous_ambiguous[pi] = competitive != 1;
      }
    }

    for (const Pair& pair : pairs) {
      if (!pair.base_feasible || !pair.extent_feasible) continue;
      if (pair_motion_observability ==
              CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE &&
          pair.world_static_feasible) {
        ++result.world_static_veto_count;
        continue;
      }
      const int current_index = current_best[pair.current];
      const int previous_index = previous_best[pair.previous];
      if (current_ambiguous[pair.current] ||
          previous_ambiguous[pair.previous]) {
        ++result.ambiguous_count;
        const auto current_id = current.components[pair.current].component_id;
        if (std::find(ambiguity_blocked_current_ids.begin(),
                      ambiguity_blocked_current_ids.end(), current_id) ==
            ambiguity_blocked_current_ids.end()) {
          ambiguity_blocked_current_ids.push_back(current_id);
        }
        continue;
      }
      if (current_index < 0 || previous_index < 0 ||
          &pairs[static_cast<std::size_t>(current_index)] != &pair ||
          &pairs[static_cast<std::size_t>(previous_index)] != &pair) {
        continue;
      }
      const auto& previous = previous_frame->components[pair.previous];
      const auto& component = current.components[pair.current];
      if (std::find(ambiguity_blocked_current_ids.begin(),
                    ambiguity_blocked_current_ids.end(),
                    component.component_id) !=
          ambiguity_blocked_current_ids.end()) {
        continue;
      }
      CargoIdentitySupportLineageObservation observation;
      observation.valid = true;
      observation.state = CargoIdentityLineageState::MATCHED;
      observation.previous_source_stamp_sec =
          previous_frame->source_stamp_sec;
      observation.source_stamp_sec = current.source_stamp_sec;
      observation.source_age_sec = source_age_sec;
      observation.source_frame_offset = source_frame_offset;
      observation.previous_component_id = previous.component_id;
      observation.current_component_id = component.component_id;
      observation.exact_seed_frame_group_id =
          component.exact_seed_frame_group_id;
      observation.robust_xy_center = component.center_base;
      observation.robust_xy_extent = component.robust_xy_extent;
      observation.robust_x05 = component.robust_x05;
      observation.robust_x95 = component.robust_x95;
      observation.robust_y05 = component.robust_y05;
      observation.robust_y95 = component.robust_y95;
      observation.base_step_m = pair.base_step;
      observation.map_step_m = pair.map_step;
      observation.ego_xy_step_m = pair_ego_xy_step_m;
      observation.extent_step = pair.extent_step;
      observation.motion_observability_state = pair_motion_observability;
      result.observations.push_back(std::move(observation));
    }
  }
  result.match_count = result.observations.size();
  recent_frames_.push_back(std::move(current));
  while (recent_frames_.size() > kMaximumRecentSourceFrames) {
    recent_frames_.pop_front();
  }
  reset_reason_ = "none";
  return result;
}

}  // namespace ndt_slam
