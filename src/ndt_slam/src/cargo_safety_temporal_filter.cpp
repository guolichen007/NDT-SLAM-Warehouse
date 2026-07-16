#include "ndt_slam/cargo_safety_temporal_filter.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {
namespace {

constexpr std::uint16_t kClear = 14U;
constexpr std::uint16_t kLevel1 = 17U;
constexpr std::uint16_t kLevel2 = 18U;
constexpr double kStampEpsilonSec = 1.0e-4;

bool isProtocolCode(std::uint16_t code) {
  return code == kClear || code == kLevel1 || code == kLevel2;
}

bool validConfig(const CargoSafetyTemporalConfig& config) {
  return config.hazard_confirm_frames >= 2 &&
      config.clear_confirm_frames >= 2 &&
      config.minimum_hazard_cluster_points > 0U &&
      std::isfinite(config.maximum_evidence_gap_sec) &&
      config.maximum_evidence_gap_sec > 0.0 &&
      std::isfinite(config.maximum_centroid_step_m) &&
      config.maximum_centroid_step_m > 0.0F &&
      std::isfinite(config.maximum_distance_step_m) &&
      config.maximum_distance_step_m > 0.0F &&
      std::isfinite(config.maximum_clearance_step_m) &&
      config.maximum_clearance_step_m > 0.0F;
}

}  // namespace

CargoSafetyTemporalFilter::CargoSafetyTemporalFilter(
    const CargoSafetyTemporalConfig& config) {
  setConfig(config);
}

void CargoSafetyTemporalFilter::setConfig(
    const CargoSafetyTemporalConfig& config) {
  config_ = validConfig(config) ? config : CargoSafetyTemporalConfig{};
  reset();
}

void CargoSafetyTemporalFilter::reset() {
  has_source_stamp_ = false;
  last_source_stamp_sec_ = 0.0;
  candidate_valid_ = false;
  candidate_code_ = 0U;
  candidate_count_ = 0;
  candidate_stamp_sec_ = 0.0;
  candidate_centroid_.setZero();
  candidate_distance_m_ = 0.0F;
  candidate_clearance_m_ = 0.0F;
  confirmed_valid_ = false;
  confirmed_code_ = 0U;
}

CargoSafetyTemporalDecision CargoSafetyTemporalFilter::currentDecision(
    const std::string& reason) const {
  CargoSafetyTemporalDecision decision;
  decision.stable = confirmed_valid_;
  decision.pending = !confirmed_valid_;
  decision.code = confirmed_valid_ ? confirmed_code_ : 0U;
  decision.candidate_code = candidate_valid_ ? candidate_code_ : 0U;
  decision.confirmed_code = confirmed_valid_ ? confirmed_code_ : 0U;
  decision.evidence_count = candidate_count_;
  decision.reason = reason;
  return decision;
}

CargoSafetyTemporalDecision CargoSafetyTemporalFilter::pendingDecision(
    const std::string& reason) const {
  CargoSafetyTemporalDecision decision;
  decision.stable = false;
  decision.pending = true;
  decision.code = 0U;
  decision.candidate_code = candidate_valid_ ? candidate_code_ : 0U;
  decision.confirmed_code = confirmed_valid_ ? confirmed_code_ : 0U;
  decision.evidence_count = candidate_count_;
  decision.reason = reason;
  return decision;
}

CargoSafetyTemporalDecision CargoSafetyTemporalFilter::update(
    const CargoSafetyTemporalInput& input) {
  if (!validConfig(config_) || !input.raw_valid ||
      !std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0 ||
      !isProtocolCode(input.raw_code)) {
    candidate_valid_ = false;
    candidate_count_ = 0;
    return pendingDecision("invalid_raw_evidence");
  }

  if (has_source_stamp_ &&
      input.stamp_sec + kStampEpsilonSec < last_source_stamp_sec_) {
    // A bag replay or upstream restart creates a new source-time epoch. Old
    // confirmations must not leak across it, and the filter must recover on
    // the immediately following increasing stamps.
    reset();
  } else if (has_source_stamp_ &&
             input.stamp_sec <=
                 last_source_stamp_sec_ + kStampEpsilonSec) {
    // A repeated callback is not current physical evidence. In particular it
    // must not keep a previously confirmed 17/18 alive indefinitely.
    return pendingDecision("repeated_source_stamp_ignored");
  }
  has_source_stamp_ = true;
  last_source_stamp_sec_ = input.stamp_sec;

  const bool hazard = input.raw_code == kLevel1 || input.raw_code == kLevel2;
  const bool robust_hazard = hazard &&
      input.cluster_points >= config_.minimum_hazard_cluster_points &&
      input.cluster_centroid.allFinite() &&
      std::isfinite(input.footprint_distance_m) &&
      input.footprint_distance_m >= 0.0F &&
      std::isfinite(input.conservative_clearance_m);
  if (hazard && !robust_hazard) {
    candidate_valid_ = false;
    candidate_count_ = 0;
    return pendingDecision("hazard_cluster_too_sparse");
  }

  const bool same_candidate = candidate_valid_ &&
      candidate_code_ == input.raw_code;
  const bool gap_continuous = same_candidate &&
      input.stamp_sec - candidate_stamp_sec_ <=
          config_.maximum_evidence_gap_sec + kStampEpsilonSec;
  bool spatially_continuous = true;
  bool continuous = gap_continuous;
  if (continuous && hazard) {
    spatially_continuous =
        (input.cluster_centroid - candidate_centroid_).norm() <=
            config_.maximum_centroid_step_m &&
        std::abs(input.footprint_distance_m - candidate_distance_m_) <=
            config_.maximum_distance_step_m &&
        std::abs(input.conservative_clearance_m -
                 candidate_clearance_m_) <=
            config_.maximum_clearance_step_m;
    continuous = spatially_continuous;
  }

  std::string pending_reason;
  if (hazard && same_candidate && !gap_continuous) {
    pending_reason = "hazard_evidence_gap";
  } else if (hazard && gap_continuous && !spatially_continuous) {
    pending_reason = "hazard_spatial_discontinuity";
  } else if (hazard && candidate_valid_ &&
             candidate_code_ != input.raw_code) {
    pending_reason = "hazard_level_transition_pending";
  }

  if (continuous) {
    ++candidate_count_;
  } else {
    candidate_valid_ = true;
    candidate_code_ = input.raw_code;
    candidate_count_ = 1;
  }
  candidate_stamp_sec_ = input.stamp_sec;
  if (hazard) {
    candidate_centroid_ = input.cluster_centroid;
    candidate_distance_m_ = input.footprint_distance_m;
    candidate_clearance_m_ = input.conservative_clearance_m;
  }

  const int required_frames = hazard
      ? config_.hazard_confirm_frames : config_.clear_confirm_frames;
  if (candidate_count_ >= required_frames) {
    const bool changed = !confirmed_valid_ ||
        confirmed_code_ != input.raw_code;
    confirmed_valid_ = true;
    confirmed_code_ = input.raw_code;
    CargoSafetyTemporalDecision decision =
        currentDecision(hazard
            ? "hazard_spatiotemporally_confirmed"
            : "clear_temporally_confirmed");
    decision.pending = false;
    decision.newly_confirmed = changed;
    decision.use_current_evidence = true;
    return decision;
  }

  if (pending_reason.empty()) {
    pending_reason = hazard
        ? "hazard_spatiotemporal_confirmation_pending"
        : "clear_confirmation_pending";
  }
  return pendingDecision(pending_reason);
}

}  // namespace ndt_slam
