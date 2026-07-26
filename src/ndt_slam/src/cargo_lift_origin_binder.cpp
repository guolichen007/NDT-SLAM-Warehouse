#include "ndt_slam/cargo_lift_origin_binder.hpp"
#include "ndt_slam/static_evidence_authorization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

constexpr double kEvidenceStampEpsilonSec = 1.0e-6;

bool validConfig(const CargoLiftOriginConfig& config) {
  return config.maximum_anchor_distance_m > 0.0F &&
      config.minimum_significant_change_m > 0.0F &&
      config.significance_sigma > 0.0F &&
      config.minimum_source_coverage >= 0.0F &&
      config.minimum_source_coverage <= 1.0F &&
      config.minimum_top_coverage >= 0.0F &&
      config.minimum_top_coverage <= 1.0F &&
      config.minimum_revealed_support_coverage >= 0.0F &&
      config.minimum_revealed_support_coverage <= 1.0F &&
      config.lift_confirm_frames > 0 &&
      config.thickness_confirm_frames > 0 &&
      config.maximum_observation_gap_sec > 0.0 &&
      config.maximum_source_age_sec >= 0.0 &&
      config.maximum_height_m > config.minimum_height_m;
}

int sourcePriority(CargoOriginCandidateSource source) {
  switch (source) {
    case CargoOriginCandidateSource::OPERATOR_APPROVED_BASELINE: return 4;
    case CargoOriginCandidateSource::RUNTIME_MATURE_STATIC: return 3;
    case CargoOriginCandidateSource::RETIRED_FORMAL_SHAPE: return 2;
    case CargoOriginCandidateSource::CONFIGURED_ENVELOPE: return 1;
  }
  return 0;
}

bool formalStaticOrigin(const CargoOriginCandidate& candidate) {
  if (!authorizeStaticEvidence(candidate.authority)
           .formal_origin_authorized) {
    return false;
  }
  return candidate.source ==
             CargoOriginCandidateSource::OPERATOR_APPROVED_BASELINE ||
      (candidate.source ==
           CargoOriginCandidateSource::RUNTIME_MATURE_STATIC &&
       candidate.predates_cargo_lifecycle);
}

bool usable(const CargoOriginCandidate& candidate,
            const CargoLiftOriginConfig& config) {
  const float height = candidate.top_z95_map - candidate.support_z_map;
  const bool provisional_source =
      candidate.source == CargoOriginCandidateSource::RETIRED_FORMAL_SHAPE ||
      candidate.source == CargoOriginCandidateSource::CONFIGURED_ENVELOPE;
  const bool authority_valid = provisional_source ||
      formalStaticOrigin(candidate);
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

float candidateScore(const CargoOriginCandidate& candidate,
                     const CargoLiftOriginInput& input) {
  const float distance =
      (candidate.center_map - input.hook_anchor_map).norm();
  const float top_error = input.current_top_valid
      ? std::abs(candidate.top_z95_map - input.current_top_z_map)
      : 0.0F;
  // Formal static origin evidence always outranks provisional retired/config
  // geometry. Overlap and distance choose among candidates of the same
  // authority class.
  return (formalStaticOrigin(candidate) ? 1000.0F : 0.0F) +
      100.0F * candidate.candidate_overlap +
      20.0F * candidate.anchor_overlap - 5.0F * distance - top_error +
      2.0F * static_cast<float>(sourcePriority(candidate.source)) +
      0.001F * static_cast<float>(candidate.point_count);
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
  last_valid_lift_stamp_sec_ = 0.0;
  last_valid_thickness_stamp_sec_ = 0.0;
  last_consumed_support_stamp_sec_ = 0.0;
  last_consumed_top_stamp_sec_ = 0.0;
  bound_component_id_ = 0U;
}

CargoLiftOriginResult CargoLiftOriginBinder::update(
    const CargoLiftOriginInput& input) {
  const double previous_stamp_sec = last_stamp_sec_;
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0 ||
      (last_stamp_sec_ > 0.0 && input.stamp_sec <= last_stamp_sec_)) {
    result_.state = CargoLiftEventState::INVALID;
    result_.valid = false;
    result_.reason = "source_time_invalid_or_rollback";
    previous_loaded_ = input.hook_loaded;
    last_stamp_sec_ = input.stamp_sec;
    last_consumed_support_stamp_sec_ = 0.0;
    last_consumed_top_stamp_sec_ = 0.0;
    return result_;
  }
  last_stamp_sec_ = input.stamp_sec;
  if (previous_stamp_sec > 0.0 &&
      input.stamp_sec - previous_stamp_sec >
          config_.maximum_observation_gap_sec) {
    // Gaps break a confirmation window, but an already confirmed physical
    // event remains latched for the loaded lifecycle. Occlusion must not
    // "un-lift" cargo or discard an already frozen thickness.
    if (!result_.lift_confirmed) {
      result_.lift_confirm_count = 0;
      last_valid_lift_stamp_sec_ = 0.0;
      last_consumed_top_stamp_sec_ = 0.0;
    }
    if (!result_.thickness_ready) {
      result_.thickness_confirm_count = 0;
      last_valid_thickness_stamp_sec_ = 0.0;
      last_consumed_support_stamp_sec_ = 0.0;
    }
  }

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
    last_consumed_support_stamp_sec_ = 0.0;
    last_consumed_top_stamp_sec_ = 0.0;
    return result_;
  }

  const bool load_edge = !previous_loaded_ || input.hook_was_empty;
  if (result_.valid) {
    const bool bound_identity_present = std::any_of(
        input.candidates.begin(), input.candidates.end(),
        [&](const CargoOriginCandidate& candidate) {
          return candidate.component_id == result_.origin.component_id &&
              candidate.source == result_.origin.source &&
              usable(candidate, config_);
        });
    if (!bound_identity_present) {
      result_ = CargoLiftOriginResult{};
      result_.state = CargoLiftEventState::INVALID;
      result_.reason = "origin_identity_changed";
      bound_component_id_ = 0U;
      last_valid_lift_stamp_sec_ = 0.0;
      last_valid_thickness_stamp_sec_ = 0.0;
      last_consumed_support_stamp_sec_ = 0.0;
      last_consumed_top_stamp_sec_ = 0.0;
    } else if (!formalStaticOrigin(result_.origin)) {
      const CargoOriginCandidate* upgrade = nullptr;
      float upgrade_score = -std::numeric_limits<float>::infinity();
      for (const CargoOriginCandidate& candidate : input.candidates) {
        if (!usable(candidate, config_) ||
            !formalStaticOrigin(candidate)) {
          continue;
        }
        const float distance =
            (candidate.center_map - input.hook_anchor_map).norm();
        if (distance > config_.maximum_anchor_distance_m) continue;
        const float score = candidateScore(candidate, input);
        if (!upgrade || score > upgrade_score) {
          upgrade = &candidate;
          upgrade_score = score;
        }
      }
      if (upgrade) {
        result_ = CargoLiftOriginResult{};
        result_.origin = *upgrade;
        result_.valid = true;
        result_.state = CargoLiftEventState::ORIGIN_BINDING;
        result_.static_thickness_m =
            upgrade->top_z95_map - upgrade->support_z_map;
        result_.reason = "formal_static_origin_upgraded";
        bound_component_id_ = upgrade->component_id;
        last_valid_lift_stamp_sec_ = 0.0;
        last_valid_thickness_stamp_sec_ = 0.0;
        last_consumed_support_stamp_sec_ = 0.0;
        last_consumed_top_stamp_sec_ = 0.0;
      }
    }
  }
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
      const float score = candidateScore(candidate, input) +
          (candidate.component_id == bound_component_id_ ? 10.0F : 0.0F);
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
    if (bound_component_id_ != 0U &&
        bound_component_id_ != selected->component_id) {
      result_.lift_confirm_count = 0;
      result_.thickness_confirm_count = 0;
    }
    bound_component_id_ = selected->component_id;
    last_consumed_support_stamp_sec_ = 0.0;
    last_consumed_top_stamp_sec_ = 0.0;
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
  const bool top_fresh = std::isfinite(input.current_top_stamp_sec) &&
      input.current_top_stamp_sec > 0.0 &&
      std::abs(input.stamp_sec - input.current_top_stamp_sec) <=
          config_.maximum_source_age_sec;
  const bool support_fresh =
      std::isfinite(input.revealed_support_stamp_sec) &&
      input.revealed_support_stamp_sec > 0.0 &&
      std::abs(input.stamp_sec - input.revealed_support_stamp_sec) <=
          config_.maximum_source_age_sec;
  const bool top_valid = input.current_top_valid && top_fresh &&
      std::isfinite(input.current_top_z_map) &&
      input.source_coverage >= config_.minimum_source_coverage &&
      input.top_coverage >= config_.minimum_top_coverage;
  if (top_valid) {
    result_.lift_delta_m =
        input.current_top_z_map - result_.origin.top_z95_map;
  }
  const bool revealed = input.revealed_support_valid && support_fresh &&
      std::isfinite(input.revealed_support_z_map) &&
      input.revealed_support_coverage >=
          config_.minimum_revealed_support_coverage &&
      result_.origin.top_z95_map - input.revealed_support_z_map >=
          result_.change_threshold_m;
  const bool lifted = top_valid &&
      result_.lift_delta_m >= result_.change_threshold_m;
  const bool top_evidence_advanced = lifted &&
      input.current_top_stamp_sec >
          last_consumed_top_stamp_sec_ + kEvidenceStampEpsilonSec;
  if (!result_.lift_confirmed) {
    if (top_evidence_advanced) {
      if (last_valid_lift_stamp_sec_ > 0.0 &&
          input.stamp_sec - last_valid_lift_stamp_sec_ >
              config_.maximum_observation_gap_sec) {
        result_.lift_confirm_count = 0;
      }
      ++result_.lift_confirm_count;
      last_valid_lift_stamp_sec_ = input.stamp_sec;
      last_consumed_top_stamp_sec_ = input.current_top_stamp_sec;
    } else if (!lifted) {
      result_.lift_confirm_count = 0;
      last_valid_lift_stamp_sec_ = 0.0;
    }
    if (result_.lift_confirm_count >= config_.lift_confirm_frames) {
      result_.lift_confirmed = true;
    }
  }

  if (!result_.lift_confirmed) {
    result_.thickness_confirm_count = 0;
    last_valid_thickness_stamp_sec_ = 0.0;
    result_.state = CargoLiftEventState::LIFT_CONFIRMING;
    result_.reason = lifted && !top_evidence_advanced
        ? "waiting_for_new_top_evidence"
        : "lift_confirmation_pending";
    previous_loaded_ = true;
    return result_;
  }

  const float revealed_thickness_m = revealed
      ? result_.origin.top_z95_map - input.revealed_support_z_map
      : std::numeric_limits<float>::quiet_NaN();
  const bool thickness_valid = revealed &&
      revealed_thickness_m >= config_.minimum_height_m &&
      revealed_thickness_m <= config_.maximum_height_m;
  const bool support_evidence_advanced = thickness_valid &&
      input.revealed_support_stamp_sec >
          last_consumed_support_stamp_sec_ + kEvidenceStampEpsilonSec;
  if (!result_.thickness_ready) {
    if (support_evidence_advanced) {
      if (last_valid_thickness_stamp_sec_ > 0.0 &&
          input.stamp_sec - last_valid_thickness_stamp_sec_ >
              config_.maximum_observation_gap_sec) {
        result_.thickness_confirm_count = 0;
      }
      ++result_.thickness_confirm_count;
      last_valid_thickness_stamp_sec_ = input.stamp_sec;
      last_consumed_support_stamp_sec_ =
          input.revealed_support_stamp_sec;
      result_.revealed_thickness_m = revealed_thickness_m;
    } else if (!revealed || !thickness_valid) {
      result_.thickness_confirm_count = 0;
      last_valid_thickness_stamp_sec_ = 0.0;
    }
    result_.state = CargoLiftEventState::THICKNESS_CONFIRMING;
    result_.reason = !thickness_valid
        ? "lift_confirmed_revealed_thickness_invalid"
        : (support_evidence_advanced
               ? "lift_confirmed_thickness_pending"
               : "waiting_for_new_support_evidence");
    if (result_.thickness_confirm_count >=
        config_.thickness_confirm_frames) {
      result_.thickness_ready = true;
    }
  }
  if (result_.thickness_ready) {
    result_.state = CargoLiftEventState::GEOMETRY_FROZEN;
    result_.reason = "origin_and_revealed_thickness_confirmed";
  }
  previous_loaded_ = true;
  return result_;
}

}  // namespace ndt_slam
