#include "ndt_slam/cargo_physical_identity_authority.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <iterator>
#include <set>
#include <utility>

namespace ndt_slam {
namespace {

constexpr double kEpsilon = 1.0e-9;

bool finiteCandidate(const CargoPhysicalCandidateObservation& candidate) {
  return candidate.stamp_sec > 0.0 && std::isfinite(candidate.stamp_sec) &&
      candidate.center.allFinite() && candidate.size.allFinite() &&
      (candidate.size.array() > 0.0).all() &&
      std::isfinite(candidate.yaw_rad) && std::isfinite(candidate.z95) &&
      std::isfinite(candidate.vertical_uncertainty_m) &&
      candidate.vertical_uncertainty_m >= 0.0 &&
      !candidate.member_component_ids.empty() && candidate.point_support > 0U;
}

std::vector<std::uint64_t> canonicalMembers(
    std::vector<std::uint64_t> members) {
  std::sort(members.begin(), members.end());
  members.erase(std::unique(members.begin(), members.end()), members.end());
  return members;
}

bool overlaps(const std::vector<std::uint64_t>& lhs,
              const std::vector<std::uint64_t>& rhs) {
  std::vector<std::uint64_t> intersection;
  std::set_intersection(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(),
                        std::back_inserter(intersection));
  return !intersection.empty();
}

bool equivalentGeometry(const CargoPhysicalCandidateObservation& lhs,
                        const CargoPhysicalCandidateObservation& rhs,
                        double center_tolerance,
                        double size_relative_tolerance) {
  if ((lhs.center - rhs.center).norm() > center_tolerance) return false;
  for (int axis = 0; axis < 3; ++axis) {
    const double denominator = std::max(
        std::max(std::abs(lhs.size[axis]), std::abs(rhs.size[axis])), kEpsilon);
    if (std::abs(lhs.size[axis] - rhs.size[axis]) / denominator >
        size_relative_tolerance) {
      return false;
    }
  }
  return true;
}

int requiredFrames(HookLoadSignalRole role, bool gravity_valid,
                   HookLoadState gravity_state, int base) {
  if (role != HookLoadSignalRole::AUXILIARY) return std::max(1, base);
  if (gravity_valid && gravity_state == HookLoadState::EMPTY) {
    return std::max(1, base) + 2;
  }
  if (!gravity_valid || gravity_state == HookLoadState::UNKNOWN) {
    return std::max(1, base) + 1;
  }
  return std::max(1, base);
}

}  // namespace

const char* cargoCandidateAssociationStateName(
    CargoCandidateAssociationState state) noexcept {
  switch (state) {
    case CargoCandidateAssociationState::MATCHED: return "MATCHED";
    case CargoCandidateAssociationState::AMBIGUOUS: return "AMBIGUOUS";
    case CargoCandidateAssociationState::NEW_HISTORY: return "NEW_HISTORY";
  }
  return "INVALID";
}

const char* cargoPhysicalIdentityStateName(
    CargoPhysicalIdentityState state) noexcept {
  switch (state) {
    case CargoPhysicalIdentityState::UNKNOWN: return "UNKNOWN";
    case CargoPhysicalIdentityState::AMBIGUOUS: return "AMBIGUOUS";
    case CargoPhysicalIdentityState::VALIDATED: return "VALIDATED";
  }
  return "INVALID";
}

const char* cargoLiftBaselineSourceName(
    CargoLiftBaselineSource source) noexcept {
  switch (source) {
    case CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE:
      return "PRE_LOAD_FROZEN_BASELINE";
    case CargoLiftBaselineSource::POST_LOAD_FIRST_FRESH_OBSERVATION:
      return "POST_LOAD_FIRST_FRESH_OBSERVATION";
    case CargoLiftBaselineSource::UNAVAILABLE_STARTED_LOADED:
      return "UNAVAILABLE_STARTED_LOADED";
  }
  return "INVALID";
}

const char* cargoExistenceSourceName(CargoExistenceSource source) noexcept {
  switch (source) {
    case CargoExistenceSource::NONE: return "NONE";
    case CargoExistenceSource::GRAVITY_LOADED: return "GRAVITY_LOADED";
    case CargoExistenceSource::STRICT_LIDAR: return "STRICT_LIDAR";
  }
  return "INVALID";
}

std::vector<CargoPhysicalGroupObservation> groupCargoPhysicalCandidates(
    const std::vector<CargoPhysicalCandidateObservation>& candidates,
    double equivalent_center_tolerance_m,
    double equivalent_size_relative_tolerance) {
  std::map<std::vector<std::uint64_t>, CargoPhysicalGroupObservation> exact;
  for (CargoPhysicalCandidateObservation candidate : candidates) {
    candidate.member_component_ids =
        canonicalMembers(std::move(candidate.member_component_ids));
    if (!finiteCandidate(candidate)) continue;
    auto& group = exact[candidate.member_component_ids];
    group.member_component_ids = candidate.member_component_ids;
    group.hypotheses.push_back(candidate);
  }

  std::vector<CargoPhysicalGroupObservation> groups;
  groups.reserve(exact.size());
  std::uint64_t next_group_id = 1U;
  for (auto& entry : exact) {
    auto& group = entry.second;
    group.frame_group_id = next_group_id++;
    group.representative = group.hypotheses.front();
    group.geometry_resolved = true;
    for (std::size_t i = 1; i < group.hypotheses.size(); ++i) {
      if (!equivalentGeometry(group.representative, group.hypotheses[i],
                              equivalent_center_tolerance_m,
                              equivalent_size_relative_tolerance)) {
        group.geometry_resolved = false;
      }
    }
    groups.push_back(group);
  }

  for (std::size_t i = 0; i < groups.size(); ++i) {
    for (std::size_t j = i + 1; j < groups.size(); ++j) {
      if (overlaps(groups[i].member_component_ids,
                   groups[j].member_component_ids)) {
        groups[i].group_ambiguous = true;
        groups[j].group_ambiguous = true;
        groups[i].geometry_resolved = false;
        groups[j].geometry_resolved = false;
      }
    }
  }
  return groups;
}

bool matchesResolvedPhysicalHypothesis(
    const CargoPhysicalCandidateObservation& candidate,
    const CargoPhysicalIdentityDecision& decision) {
  return decision.geometry_resolved && decision.resolved_candidate_id != 0U &&
      candidate.candidate_id == decision.resolved_candidate_id &&
      canonicalMembers(candidate.member_component_ids) ==
          canonicalMembers(decision.resolved_member_component_ids);
}

CargoPhysicalIdentityAuthority::CargoPhysicalIdentityAuthority(
    const CargoPhysicalIdentityConfig& config) {
  setConfig(config);
}

void CargoPhysicalIdentityAuthority::setConfig(
    const CargoPhysicalIdentityConfig& config) {
  config_ = config;
  if (!(config_.maximum_xy_step_m > 0.0)) config_.maximum_xy_step_m = 0.30;
  if (!(config_.maximum_z_speed_mps > 0.0)) config_.maximum_z_speed_mps = 1.50;
  if (!(config_.z_step_margin_m >= 0.0)) config_.z_step_margin_m = 0.05;
  if (!(config_.maximum_size_relative_step >= 0.0)) {
    config_.maximum_size_relative_step = 0.60;
  }
  if (!(config_.ambiguity_cost_margin >= 0.0)) {
    config_.ambiguity_cost_margin = 0.08;
  }
  if (!(config_.minimum_significant_change_m > 0.0)) {
    config_.minimum_significant_change_m = 0.15;
  }
  if (!(config_.significance_sigma > 0.0)) config_.significance_sigma = 3.0;
  if (!(config_.maximum_observation_gap_sec > 0.0)) {
    config_.maximum_observation_gap_sec = 0.50;
  }
  if (!(config_.maximum_source_age_sec >= 0.0)) {
    config_.maximum_source_age_sec = 0.50;
  }
  config_.lift_confirm_frames = std::max(1, config_.lift_confirm_frames);
  reset("config_changed");
}

void CargoPhysicalIdentityAuthority::reset(const std::string& reason) {
  histories_.clear();
  decision_ = CargoPhysicalIdentityDecision{};
  next_history_id_ = 1U;
  load_epoch_ = 0U;
  lifecycle_id_ = 0U;
  validated_history_id_ = 0U;
  initialized_ = false;
  previous_existence_phase_ = false;
  started_loaded_without_baseline_ = false;
  last_pipeline_stamp_sec_ = 0.0;
  reset_reason_ = reason;
}

CargoPhysicalIdentityDecision CargoPhysicalIdentityAuthority::update(
    const CargoPhysicalIdentityInput& input) {
  decision_ = CargoPhysicalIdentityDecision{};
  decision_.load_epoch = load_epoch_;
  const bool stamp_valid = std::isfinite(input.pipeline_stamp_sec) &&
      input.pipeline_stamp_sec > 0.0 &&
      (!initialized_ || input.pipeline_stamp_sec > last_pipeline_stamp_sec_);
  if (!stamp_valid) {
    decision_.reason = "source_time_invalid_or_not_advanced";
    return decision_;
  }
  decision_.valid_input = true;

  if (input.rearm || (initialized_ && input.lifecycle_id != lifecycle_id_)) {
    histories_.clear();
    validated_history_id_ = 0U;
    previous_existence_phase_ = false;
    started_loaded_without_baseline_ = false;
    ++load_epoch_;
  }
  lifecycle_id_ = input.lifecycle_id;
  last_pipeline_stamp_sec_ = input.pipeline_stamp_sec;
  initialized_ = true;

  const bool gravity_loaded = input.gravity_valid &&
      input.gravity_state == HookLoadState::LOADED;
  const bool gravity_empty = input.gravity_valid &&
      input.gravity_state == HookLoadState::EMPTY;
  const bool pre_load_phase = gravity_empty;
  const bool gravity_load_edge = gravity_loaded && !previous_existence_phase_;
  if (gravity_load_edge) {
    ++load_epoch_;
    started_loaded_without_baseline_ = input.node_started_loaded &&
        std::none_of(histories_.begin(), histories_.end(),
                     [](const History& history) { return history.has_preload; });
    for (History& history : histories_) {
      // Candidate-specific lift proof belongs to exactly one load epoch.
      // Preserve only the current EMPTY-phase preload observation; never let
      // confirmation, validation, or a prior frozen baseline cross the edge.
      history.lift_confirm_count = 0;
      history.lift_confirmed = false;
      history.validation_stamp_sec = 0.0;
      history.last_consumed_evidence_stamp_sec = 0.0;
      history.baseline_frozen = false;
      history.baseline_z95 = std::numeric_limits<double>::quiet_NaN();
      history.baseline_stamp_sec = 0.0;
      if (history.has_preload) {
        history.baseline_frozen = true;
        history.baseline_source =
            CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE;
        history.baseline_z95 = history.preload_z95;
        history.baseline_uncertainty_m = history.preload_uncertainty_m;
        history.baseline_stamp_sec = history.preload_stamp_sec;
        // Consume the EMPTY-phase baseline exactly once. A later load edge
        // without a new EMPTY observation must not reuse this Cargo's origin.
        history.has_preload = false;
      }
    }
  }

  struct Pair {
    std::size_t group = 0U;
    std::size_t history = 0U;
    double cost = 0.0;
  };
  std::vector<Pair> feasible;
  for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
    const auto& observation = input.groups[gi].representative;
    if (!finiteCandidate(observation)) continue;
    for (std::size_t hi = 0; hi < histories_.size(); ++hi) {
      const History& history = histories_[hi];
      const double dt = input.pipeline_stamp_sec - history.last_stamp_sec;
      if (!(dt > 0.0) || dt > config_.maximum_observation_gap_sec) continue;
      const auto& previous = history.last_group.representative;
      const double xy = (observation.center.head<2>() -
                         previous.center.head<2>()).norm();
      const double z_limit = config_.maximum_z_speed_mps * dt +
          config_.z_step_margin_m + observation.vertical_uncertainty_m +
          previous.vertical_uncertainty_m;
      const double dz = std::abs(observation.z95 - previous.z95);
      double size_cost = 0.0;
      bool size_ok = true;
      for (int axis = 0; axis < 3; ++axis) {
        const double denominator = std::max(
            std::max(std::abs(observation.size[axis]),
                     std::abs(previous.size[axis])), kEpsilon);
        const double relative =
            std::abs(observation.size[axis] - previous.size[axis]) /
            denominator;
        size_cost += relative;
        size_ok = size_ok && relative <= config_.maximum_size_relative_step;
      }
      if (xy <= config_.maximum_xy_step_m && dz <= z_limit && size_ok) {
        feasible.push_back({gi, hi, xy / config_.maximum_xy_step_m +
            dz / std::max(z_limit, kEpsilon) + size_cost});
      }
    }
  }

  std::vector<int> group_match(input.groups.size(), -1);
  std::vector<bool> group_ambiguous(input.groups.size(), false);
  for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
    double best = std::numeric_limits<double>::infinity();
    int best_hi = -1;
    for (const Pair& pair : feasible) {
      if (pair.group != gi) continue;
      if (pair.cost + kEpsilon < best) {
        best = pair.cost;
        best_hi = static_cast<int>(pair.history);
      }
    }
    const int competitive_history_count = static_cast<int>(std::count_if(
        feasible.begin(), feasible.end(), [&](const Pair& pair) {
          return pair.group == gi && pair.cost <=
              best + config_.ambiguity_cost_margin;
        }));
    if (competitive_history_count > 1) {
      group_ambiguous[gi] = true;
      continue;
    }
    if (best_hi < 0) continue;
    int history_best_group = -1;
    double history_best = std::numeric_limits<double>::infinity();
    for (const Pair& pair : feasible) {
      if (pair.history != static_cast<std::size_t>(best_hi)) continue;
      if (pair.cost + kEpsilon < history_best) {
        history_best = pair.cost;
        history_best_group = static_cast<int>(pair.group);
      }
    }
    const int competitive_group_count = static_cast<int>(std::count_if(
        feasible.begin(), feasible.end(), [&](const Pair& pair) {
          return pair.history == static_cast<std::size_t>(best_hi) &&
              pair.cost <= history_best + config_.ambiguity_cost_margin;
        }));
    if (competitive_group_count == 1 && history_best_group ==
        static_cast<int>(gi)) {
      group_match[gi] = best_hi;
    } else {
      group_ambiguous[gi] = true;
    }
  }

  CargoCandidateAssociationState frame_association =
      CargoCandidateAssociationState::NEW_HISTORY;
  for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
    if (input.groups[gi].group_ambiguous || group_ambiguous[gi]) {
      frame_association = CargoCandidateAssociationState::AMBIGUOUS;
      if (group_match[gi] >= 0) {
        histories_[static_cast<std::size_t>(group_match[gi])]
            .association_ambiguous = true;
      }
      continue;
    }

    History* history = nullptr;
    if (group_match[gi] >= 0) {
      history = &histories_[static_cast<std::size_t>(group_match[gi])];
      frame_association = CargoCandidateAssociationState::MATCHED;
    } else {
      histories_.push_back(History{});
      history = &histories_.back();
      history->id = next_history_id_++;
      if (gravity_loaded && !started_loaded_without_baseline_) {
        history->baseline_frozen = true;
        history->baseline_source =
            CargoLiftBaselineSource::POST_LOAD_FIRST_FRESH_OBSERVATION;
        history->baseline_z95 = input.groups[gi].representative.z95;
        history->baseline_uncertainty_m =
            input.groups[gi].representative.vertical_uncertainty_m;
        history->baseline_stamp_sec =
            input.groups[gi].representative.stamp_sec;
      } else if (input.hook_role != HookLoadSignalRole::REQUIRED &&
                 !started_loaded_without_baseline_) {
        // AUXILIARY gravity-conflict and DISABLED lidar-only operation must
        // retain the existing strict-lidar existence path. Freeze the first
        // independently associated observation; do not keep rewriting it.
        history->baseline_frozen = true;
        history->baseline_source =
            CargoLiftBaselineSource::POST_LOAD_FIRST_FRESH_OBSERVATION;
        history->baseline_z95 = input.groups[gi].representative.z95;
        history->baseline_uncertainty_m =
            input.groups[gi].representative.vertical_uncertainty_m;
        history->baseline_stamp_sec =
            input.groups[gi].representative.stamp_sec;
      }
    }
    history->association_ambiguous = false;
    history->last_group = input.groups[gi];
    history->last_stamp_sec = input.groups[gi].representative.stamp_sec;
    if (pre_load_phase) {
      history->has_preload = true;
      history->preload_z95 = input.groups[gi].representative.z95;
      history->preload_uncertainty_m =
          input.groups[gi].representative.vertical_uncertainty_m;
      history->preload_stamp_sec = input.groups[gi].representative.stamp_sec;
    }
    if (!history->baseline_frozen && !pre_load_phase &&
        !started_loaded_without_baseline_) {
      history->baseline_frozen = true;
      history->baseline_source = history->has_preload
          ? CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE
          : CargoLiftBaselineSource::POST_LOAD_FIRST_FRESH_OBSERVATION;
      history->baseline_z95 = history->has_preload
          ? history->preload_z95 : input.groups[gi].representative.z95;
      history->baseline_uncertainty_m = history->has_preload
          ? history->preload_uncertainty_m
          : input.groups[gi].representative.vertical_uncertainty_m;
      history->baseline_stamp_sec = history->has_preload
          ? history->preload_stamp_sec
          : input.groups[gi].representative.stamp_sec;
    }

    if (history->baseline_frozen) {
      const double gap = input.groups[gi].representative.stamp_sec -
          history->last_consumed_evidence_stamp_sec;
      if (!history->lift_confirmed &&
          history->last_consumed_evidence_stamp_sec > 0.0 &&
          gap > config_.maximum_observation_gap_sec) {
        history->lift_confirm_count = 0;
      }
      const double threshold = std::max(
          config_.minimum_significant_change_m,
          config_.significance_sigma * std::hypot(
              history->baseline_uncertainty_m,
              input.groups[gi].representative.vertical_uncertainty_m));
      const double delta = input.groups[gi].representative.z95 -
          history->baseline_z95;
      const bool evidence_advanced =
          input.groups[gi].representative.stamp_sec >
              history->last_consumed_evidence_stamp_sec + kEpsilon;
      if (evidence_advanced) {
        history->last_consumed_evidence_stamp_sec =
            input.groups[gi].representative.stamp_sec;
        if (delta >= threshold) {
          ++history->lift_confirm_count;
          const int required = requiredFrames(
              input.hook_role, input.gravity_valid, input.gravity_state,
              config_.lift_confirm_frames);
          if (history->lift_confirm_count >= required &&
              !history->lift_confirmed) {
            history->lift_confirmed = true;
            history->validation_stamp_sec = input.pipeline_stamp_sec;
          }
        } else if (!history->lift_confirmed) {
          // Pending confirmation may tolerate uncertainty-sized jitter near
          // the significant-change threshold, but a clear return toward the
          // frozen baseline breaks the consecutive evidence sequence.
          const double retention_margin = config_.significance_sigma *
              input.groups[gi].representative.vertical_uncertainty_m;
          const double retention_floor =
              std::max(0.0, threshold - retention_margin);
          if (delta < retention_floor) {
            history->lift_confirm_count = 0;
          }
        }
        if (delta < -threshold) {
          history->lift_confirm_count = 0;
          history->lift_confirmed = false;
          history->validation_stamp_sec = 0.0;
        }
      }
    }
  }

  std::vector<History*> fresh_confirmed;
  for (History& history : histories_) {
    const double age = input.pipeline_stamp_sec - history.last_stamp_sec;
    const bool fresh = age >= -kEpsilon &&
        age <= config_.maximum_source_age_sec;
    if (history.lift_confirmed && fresh && !history.association_ambiguous) {
      fresh_confirmed.push_back(&history);
    }
  }

  if (input.hook_role == HookLoadSignalRole::REQUIRED) {
    decision_.cargo_exists = gravity_loaded;
    decision_.existence_source = gravity_loaded
        ? CargoExistenceSource::GRAVITY_LOADED : CargoExistenceSource::NONE;
  } else if (gravity_loaded) {
    decision_.cargo_exists = true;
    decision_.existence_source = CargoExistenceSource::GRAVITY_LOADED;
  } else {
    decision_.cargo_exists = !fresh_confirmed.empty();
    decision_.existence_source = decision_.cargo_exists
        ? CargoExistenceSource::STRICT_LIDAR : CargoExistenceSource::NONE;
  }

  if (gravity_empty && input.hook_role == HookLoadSignalRole::REQUIRED) {
    validated_history_id_ = 0U;
  }
  if (frame_association == CargoCandidateAssociationState::AMBIGUOUS) {
    validated_history_id_ = 0U;
  }
  if (decision_.cargo_exists && fresh_confirmed.size() == 1U) {
    validated_history_id_ = fresh_confirmed.front()->id;
  } else if (fresh_confirmed.size() != 1U) {
    validated_history_id_ = 0U;
  }

  decision_.association = frame_association;
  decision_.load_epoch = load_epoch_;
  if (started_loaded_without_baseline_) {
    decision_.baseline_source =
        CargoLiftBaselineSource::UNAVAILABLE_STARTED_LOADED;
    decision_.identity = CargoPhysicalIdentityState::UNKNOWN;
    decision_.reason = "identity_evidence_unavailable_started_loaded";
  } else if (fresh_confirmed.size() > 1U ||
             frame_association == CargoCandidateAssociationState::AMBIGUOUS) {
    decision_.identity = CargoPhysicalIdentityState::AMBIGUOUS;
    decision_.reason = "multiple_or_ambiguous_physical_groups";
  } else if (validated_history_id_ != 0U && decision_.cargo_exists) {
    History* selected = nullptr;
    for (History& history : histories_) {
      if (history.id == validated_history_id_) selected = &history;
    }
    if (selected) {
      const auto& group = selected->last_group;
      decision_.identity = CargoPhysicalIdentityState::VALIDATED;
      decision_.physical_history_id = selected->id;
      decision_.frame_group_id = group.frame_group_id;
      decision_.geometry_resolved = group.geometry_resolved &&
          !group.group_ambiguous;
      decision_.resolved_candidate_id = decision_.geometry_resolved
          ? group.representative.candidate_id : 0U;
      decision_.resolved_member_component_ids = decision_.geometry_resolved
          ? group.member_component_ids : std::vector<std::uint64_t>{};
      decision_.baseline_source = selected->baseline_source;
      decision_.current_candidate_fresh = true;
      decision_.lift_confirmed = selected->lift_confirmed;
      decision_.lift_confirm_count = selected->lift_confirm_count;
      decision_.required_lift_confirm_frames = requiredFrames(
          input.hook_role, input.gravity_valid, input.gravity_state,
          config_.lift_confirm_frames);
      decision_.baseline_z95 = selected->baseline_z95;
      decision_.current_z95 = group.representative.z95;
      decision_.lift_delta_m = decision_.current_z95 - decision_.baseline_z95;
      decision_.lift_threshold_m = std::max(
          config_.minimum_significant_change_m,
          config_.significance_sigma * std::hypot(
              selected->baseline_uncertainty_m,
              group.representative.vertical_uncertainty_m));
      decision_.evidence_age_sec =
          input.pipeline_stamp_sec - selected->last_stamp_sec;
      decision_.identity_validation_stamp_sec =
          selected->validation_stamp_sec;
      decision_.reason = decision_.geometry_resolved
          ? "candidate_specific_lift_validated"
          : "physical_identity_validated_geometry_ambiguous";
    }
  } else {
    decision_.identity = CargoPhysicalIdentityState::UNKNOWN;
    decision_.reason = decision_.cargo_exists
        ? "existence_without_candidate_identity"
        : "cargo_existence_not_proven";
  }

  previous_existence_phase_ = gravity_loaded || decision_.cargo_exists;
  return decision_;
}

}  // namespace ndt_slam
