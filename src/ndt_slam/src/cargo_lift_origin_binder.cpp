#include "ndt_slam/cargo_lift_origin_binder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

bool validConfig(const CargoLiftOriginConfig& config) {
  return config.maximum_anchor_distance_m > 0.0F &&
      config.minimum_significant_change_m > 0.0F &&
      config.significance_sigma > 0.0F &&
      config.minimum_source_coverage >= 0.0F &&
      config.minimum_source_coverage <= 1.0F &&
      config.minimum_revealed_support_coverage >= 0.0F &&
      config.minimum_revealed_support_coverage <= 1.0F &&
      config.lift_confirm_frames > 0 &&
      config.thickness_confirm_frames > 0 &&
      config.maximum_height_m > config.minimum_height_m;
}

int sourcePriority(CargoOriginCandidateSource source) {
  switch (source) {
    case CargoOriginCandidateSource::RETIRED_FORMAL_SHAPE: return 4;
    case CargoOriginCandidateSource::OPERATOR_APPROVED_BASELINE: return 3;
    case CargoOriginCandidateSource::RUNTIME_MATURE_STATIC: return 2;
    case CargoOriginCandidateSource::CONFIGURED_ENVELOPE: return 1;
  }
  return 0;
}

bool usable(const CargoOriginCandidate& candidate,
            const CargoLiftOriginConfig& config) {
  const float height = candidate.top_z95_map - candidate.support_z_map;
  const bool authority_valid =
      candidate.source == CargoOriginCandidateSource::RETIRED_FORMAL_SHAPE ||
      candidate.source == CargoOriginCandidateSource::CONFIGURED_ENVELOPE ||
      candidate.authority == StaticEvidenceAuthority::RUNTIME_MATURE ||
      candidate.authority ==
          StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE;
  return candidate.center_map.allFinite() &&
      std::isfinite(candidate.length_m) && candidate.length_m > 0.0F &&
      std::isfinite(candidate.width_m) && candidate.width_m > 0.0F &&
      std::isfinite(candidate.yaw_map_rad) &&
      std::isfinite(candidate.top_z95_map) &&
      std::isfinite(candidate.support_z_map) &&
      std::isfinite(candidate.uncertainty_m) &&
      candidate.uncertainty_m >= 0.0F && authority_valid &&
      std::isfinite(height) && height >= config.minimum_height_m &&
      height <= config.maximum_height_m;
}

}  // namespace

const char* cargoLiftEventStateName(CargoLiftEventState state) noexcept {
  switch (state) {
    case CargoLiftEventState::IDLE: return "IDLE";
    case CargoLiftEventState::PRELOAD_BASELINE_READY:
      return "PRELOAD_BASELINE_READY";
    case CargoLiftEventState::LOAD_DETECTED: return "LOAD_DETECTED";
    case CargoLiftEventState::ORIGIN_BINDING: return "ORIGIN_BINDING";
    case CargoLiftEventState::LIFT_CONFIRMING: return "LIFT_CONFIRMING";
    case CargoLiftEventState::THICKNESS_CONFIRMING:
      return "THICKNESS_CONFIRMING";
    case CargoLiftEventState::GEOMETRY_FROZEN: return "GEOMETRY_FROZEN";
    case CargoLiftEventState::TRANSPORT: return "TRANSPORT";
    case CargoLiftEventState::PLACEMENT_CONFIRMING:
      return "PLACEMENT_CONFIRMING";
    case CargoLiftEventState::LOADED_REACQUIRE: return "LOADED_REACQUIRE";
    case CargoLiftEventState::INVALID: return "INVALID";
  }
  return "INVALID";
}

CargoLiftOriginBinder::CargoLiftOriginBinder(
    const CargoLiftOriginConfig& config) {
  setConfig(config);
}

void CargoLiftOriginBinder::setConfig(const CargoLiftOriginConfig& config) {
  config_ = validConfig(config) ? config : CargoLiftOriginConfig{};
  reset();
}

void CargoLiftOriginBinder::reset() {
  result_ = CargoLiftOriginResult{};
  previous_loaded_ = false;
  last_stamp_sec_ = 0.0;
}

CargoLiftOriginResult CargoLiftOriginBinder::update(
    const CargoLiftOriginInput& input) {
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0 ||
      (last_stamp_sec_ > 0.0 && input.stamp_sec <= last_stamp_sec_)) {
    result_.state = CargoLiftEventState::INVALID;
    result_.valid = false;
    result_.reason = "source_time_invalid_or_rollback";
    previous_loaded_ = input.hook_loaded;
    last_stamp_sec_ = input.stamp_sec;
    return result_;
  }
  last_stamp_sec_ = input.stamp_sec;

  if (!input.hook_signal_valid) {
    result_.state = CargoLiftEventState::INVALID;
    result_.valid = false;
    result_.reason = "hook_signal_invalid";
    previous_loaded_ = input.hook_loaded;
    return result_;
  }
  if (!input.hook_loaded) {
    result_ = CargoLiftOriginResult{};
    result_.state = input.anchor_valid && !input.candidates.empty()
        ? CargoLiftEventState::PRELOAD_BASELINE_READY
        : CargoLiftEventState::IDLE;
    result_.reason = result_.state == CargoLiftEventState::IDLE
        ? "waiting_for_preload_baseline" : "preload_baseline_ready";
    previous_loaded_ = false;
    return result_;
  }

  const bool load_edge = !previous_loaded_ || input.hook_was_empty;
  if (!result_.valid &&
      (load_edge || input.node_started_loaded ||
       result_.state == CargoLiftEventState::INVALID)) {
    result_ = CargoLiftOriginResult{};
    result_.state = input.node_started_loaded && !input.hook_was_empty
        ? CargoLiftEventState::LOADED_REACQUIRE
        : CargoLiftEventState::LOAD_DETECTED;
    if (!input.anchor_valid || !input.hook_anchor_map.allFinite()) {
      result_.state = CargoLiftEventState::INVALID;
      result_.reason = "hook_anchor_invalid";
      previous_loaded_ = true;
      return result_;
    }
    const CargoOriginCandidate* selected = nullptr;
    float selected_score = -std::numeric_limits<float>::infinity();
    for (const CargoOriginCandidate& candidate : input.candidates) {
      if (!usable(candidate, config_)) continue;
      const float distance =
          (candidate.center_map - input.hook_anchor_map).norm();
      if (distance > config_.maximum_anchor_distance_m) continue;
      const float score = 10.0F * sourcePriority(candidate.source) -
          distance + 0.001F * static_cast<float>(candidate.point_count);
      if (!selected || score > selected_score) {
        selected = &candidate;
        selected_score = score;
      }
    }
    if (!selected) {
      result_.state = CargoLiftEventState::INVALID;
      result_.reason = "no_local_authorized_origin_candidate";
      previous_loaded_ = true;
      return result_;
    }
    result_.origin = *selected;
    result_.valid = true;
    result_.static_thickness_m =
        selected->top_z95_map - selected->support_z_map;
    result_.state = CargoLiftEventState::ORIGIN_BINDING;
    result_.reason = input.node_started_loaded
        ? "loaded_origin_reacquired" : "lift_origin_bound";
  }

  if (!result_.valid) {
    previous_loaded_ = true;
    return result_;
  }
  result_.change_threshold_m = std::max(
      config_.minimum_significant_change_m,
      config_.significance_sigma * std::sqrt(
          result_.origin.uncertainty_m * result_.origin.uncertainty_m +
          input.current_top_uncertainty_m * input.current_top_uncertainty_m));
  const bool top_valid = input.current_top_valid &&
      std::isfinite(input.current_top_z_map) &&
      input.source_coverage >= config_.minimum_source_coverage;
  if (top_valid) {
    result_.lift_delta_m =
        input.current_top_z_map - result_.origin.top_z95_map;
  }
  const bool revealed = input.revealed_support_valid &&
      std::isfinite(input.revealed_support_z_map) &&
      input.revealed_support_coverage >=
          config_.minimum_revealed_support_coverage &&
      result_.origin.top_z95_map - input.revealed_support_z_map >=
          result_.change_threshold_m;
  const bool lifted = top_valid &&
      result_.lift_delta_m >= result_.change_threshold_m && revealed;
  if (lifted) {
    ++result_.lift_confirm_count;
  } else if (top_valid && input.source_coverage >=
                         config_.minimum_source_coverage) {
    result_.lift_confirm_count = 0;
  }
  result_.state = CargoLiftEventState::LIFT_CONFIRMING;
  if (result_.lift_confirm_count >= config_.lift_confirm_frames) {
    result_.lift_confirmed = true;
    result_.revealed_thickness_m =
        result_.origin.top_z95_map - input.revealed_support_z_map;
    const bool thickness_valid =
        result_.revealed_thickness_m >= config_.minimum_height_m &&
        result_.revealed_thickness_m <= config_.maximum_height_m;
    if (thickness_valid) ++result_.thickness_confirm_count;
    result_.state = CargoLiftEventState::THICKNESS_CONFIRMING;
    result_.reason = thickness_valid
        ? "lift_confirmed_thickness_pending"
        : "lift_confirmed_revealed_thickness_invalid";
    if (result_.thickness_confirm_count >=
        config_.thickness_confirm_frames) {
      result_.thickness_ready = true;
      result_.state = CargoLiftEventState::GEOMETRY_FROZEN;
      result_.reason = "origin_and_revealed_thickness_confirmed";
    }
  } else {
    result_.reason = "lift_confirmation_pending";
  }
  previous_loaded_ = true;
  return result_;
}

}  // namespace ndt_slam
