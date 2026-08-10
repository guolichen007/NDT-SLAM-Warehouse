#include "ndt_slam/cargo_geometry_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

namespace ndt_slam {
namespace {

bool validConfig(const CargoGeometryFusionConfig& config) {
  return config.minimum_confirm_frames > 0 &&
      config.shape_confirmation_window_frames >=
          config.minimum_confirm_frames &&
      config.shape_confirmation_window_frames >=
          config.positive_only_confirm_frames &&
      config.maximum_initial_dimension_mad_m > 0.0F &&
      std::isfinite(config.positive_only_uncertainty_floor_m) &&
      config.positive_only_uncertainty_floor_m > 0.0F &&
      config.positive_only_confirm_frames > 0 &&
      config.maximum_observation_gap_sec > 0.0 &&
      config.maximum_source_disagreement_m > 0.0F &&
      config.maximum_fused_uncertainty_m > 0.0F &&
      config.maximum_height_m > config.minimum_height_m &&
      config.huber_delta_m > 0.0F &&
      config.configured_bottom_margin_m >= 0.0F &&
      config.conservative_shrink_confirm_frames > 0 &&
      config.maximum_shrink_per_frame_m > 0.0F &&
      config.conservative_expand_confirm_frames > 0 &&
      config.minimum_live_shape_confidence_for_expand >= 0.0F &&
      config.minimum_live_shape_confidence_for_expand <= 1.0F &&
      config.minimum_live_dimension_support > 0U &&
      config.minimum_initial_shape_confidence >= 0.0F &&
      config.minimum_initial_shape_confidence <= 1.0F &&
      config.minimum_live_shape_confidence_for_shrink >= 0.0F &&
      config.minimum_live_shape_confidence_for_shrink <= 1.0F &&
      config.minimum_physical_length_m > 0.0F &&
      config.minimum_physical_width_m > 0.0F &&
      std::isfinite(config.formal_transition_start_length_m) &&
      config.formal_transition_start_length_m >=
          config.minimum_physical_length_m &&
      std::isfinite(config.formal_transition_start_width_m) &&
      config.formal_transition_start_width_m >=
          config.minimum_physical_width_m &&
      !config.configured_fallback_is_formal_floor;
}

bool validDimensionEvidence(const CargoGeometryFrame& frame,
                            const CargoGeometryFusionConfig& config,
                            float minimum_shape_confidence) {
  return frame.footprint_valid &&
      std::isfinite(frame.length_m) && frame.length_m > 0.0F &&
      std::isfinite(frame.width_m) && frame.width_m > 0.0F &&
      frame.dimension_observation_complete &&
      frame.dimension_support_points >=
          config.minimum_live_dimension_support &&
      std::isfinite(frame.dimension_shape_confidence) &&
      frame.dimension_shape_confidence >=
          minimum_shape_confidence;
}

struct WeightedHeight {
  CargoThicknessSource source;
  CargoThicknessConstraint constraint;
  float height;
  float uncertainty;
  float weight;
};

float weightedMedian(std::vector<WeightedHeight> values) {
  std::sort(values.begin(), values.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.height < rhs.height;
            });
  float total = 0.0F;
  for (const auto& value : values) total += value.weight;
  float cumulative = 0.0F;
  for (const auto& value : values) {
    cumulative += value.weight;
    if (cumulative >= 0.5F * total) return value.height;
  }
  return values.empty() ? 0.0F : values.back().height;
}

float finiteMedian(std::vector<float> values) {
  if (values.empty()) return 0.0F;
  const auto middle = values.begin() +
      static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

float finiteMad(const std::vector<float>& values, float median) {
  std::vector<float> residuals;
  residuals.reserve(values.size());
  for (const float value : values) {
    if (std::isfinite(value)) residuals.push_back(std::abs(value - median));
  }
  return finiteMedian(std::move(residuals));
}

}  // namespace

const char* cargoThicknessSourceName(CargoThicknessSource source) noexcept {
  switch (source) {
    case CargoThicknessSource::STATIC_ORIGIN_TOP_SUPPORT:
      return "STATIC_ORIGIN_TOP_SUPPORT";
    case CargoThicknessSource::MAP_DIFF_REVEALED_SUPPORT:
      return "MAP_DIFF_REVEALED_SUPPORT";
    case CargoThicknessSource::LIVE_VISIBLE_EXTENT:
      return "LIVE_VISIBLE_EXTENT";
    case CargoThicknessSource::RETIRED_LOCKED_SHAPE:
      return "RETIRED_LOCKED_SHAPE";
    case CargoThicknessSource::CONFIGURED_FALLBACK:
      return "CONFIGURED_FALLBACK";
  }
  return "CONFIGURED_FALLBACK";
}

const char* cargoThicknessConstraintName(
    CargoThicknessConstraint constraint) noexcept {
  switch (constraint) {
    case CargoThicknessConstraint::FULL_MEASUREMENT:
      return "FULL_MEASUREMENT";
    case CargoThicknessConstraint::LOWER_BOUND:
      return "LOWER_BOUND";
    case CargoThicknessConstraint::PRIOR_ONLY:
      return "PRIOR_ONLY";
  }
  return "PRIOR_ONLY";
}

const char* cargoGeometryAuthorizationName(
    CargoGeometryAuthorization authorization) noexcept {
  switch (authorization) {
    case CargoGeometryAuthorization::PENDING:
      return "PENDING";
    case CargoGeometryAuthorization::POSITIVE_ONLY:
      return "POSITIVE_ONLY";
    case CargoGeometryAuthorization::FORMAL:
      return "FORMAL";
  }
  return "PENDING";
}

CargoGeometryFusion::CargoGeometryFusion(
    const CargoGeometryFusionConfig& config) {
  setConfig(config);
}

void CargoGeometryFusion::setConfig(
    const CargoGeometryFusionConfig& config) {
  config_ = validConfig(config) ? config : CargoGeometryFusionConfig{};
  reset();
}

void CargoGeometryFusion::reset() {
  result_ = CargoFrozenGeometry{};
  thickness_confirm_track_segment_id_ = 0U;
  thickness_candidate_window_.clear();
  thickness_confirmation_mode_initialized_ = false;
  thickness_confirmation_formal_mode_ = false;
  last_stamp_sec_ = 0.0;
  shrink_confirm_count_ = 0;
  shrink_track_segment_id_ = 0U;
  shrink_quality_window_.clear();
  shrink_length_window_.clear();
  shrink_width_window_.clear();
  expand_confirm_count_ = 0;
  expand_track_segment_id_ = 0U;
  pending_expand_length_m_ = 0.0F;
  pending_expand_width_m_ = 0.0F;
  shape_confirm_count_ = 0;
  shape_confirm_track_segment_id_ = 0U;
  shape_quality_window_.clear();
  initial_length_window_.clear();
  initial_width_window_.clear();
  formal_promotion_confirm_count_ = 0;
  formal_promotion_track_segment_id_ = 0U;
}

CargoFrozenGeometry CargoGeometryFusion::update(
    const CargoGeometryFrame& frame) {
  const double previous_stamp_sec = last_stamp_sec_;
  if (!std::isfinite(frame.stamp_sec) || frame.stamp_sec <= 0.0 ||
      (last_stamp_sec_ > 0.0 && frame.stamp_sec <= last_stamp_sec_)) {
    const auto config = config_;
    reset();
    config_ = config;
    result_.reason = "source_time_invalid_or_rollback";
    return result_;
  }
  last_stamp_sec_ = frame.stamp_sec;
  if (!result_.frozen && previous_stamp_sec > 0.0 &&
      frame.stamp_sec - previous_stamp_sec >
          config_.maximum_observation_gap_sec) {
    result_.confirm_frames = 0;
    thickness_candidate_window_.clear();
    shape_confirm_count_ = 0;
    shape_quality_window_.clear();
    initial_length_window_.clear();
    initial_width_window_.clear();
  }
  if (frame.cargo_lifecycle_id == 0U) {
    result_.valid = false;
    result_.reason = "cargo_lifecycle_invalid";
    return result_;
  }
  if (result_.cargo_lifecycle_id != 0U &&
      result_.cargo_lifecycle_id != frame.cargo_lifecycle_id) {
    const auto config = config_;
    reset();
    config_ = config;
    last_stamp_sec_ = frame.stamp_sec;
  }

  if (result_.frozen) {
    const bool observation_continuous =
        previous_stamp_sec <= 0.0 ||
        frame.stamp_sec - previous_stamp_sec <=
            config_.maximum_observation_gap_sec;
    if (frame.track_segment_id !=
            formal_promotion_track_segment_id_ ||
        !observation_continuous) {
      formal_promotion_confirm_count_ = 0;
      formal_promotion_track_segment_id_ = frame.track_segment_id;
    }
    if (!result_.formal_authorized) {
      const CargoThicknessObservation* static_origin = nullptr;
      const CargoThicknessObservation* revealed_support = nullptr;
      const CargoThicknessObservation* best_live = nullptr;
      const auto observation_weight =
          [](const CargoThicknessObservation& observation) {
            return observation.confidence /
                (observation.uncertainty_m *
                 observation.uncertainty_m);
          };
      for (const CargoThicknessObservation& observation :
           frame.thickness) {
        const bool usable = observation.valid &&
            std::isfinite(observation.height_m) &&
            std::isfinite(observation.uncertainty_m) &&
            std::isfinite(observation.confidence) &&
            observation.height_m >= config_.minimum_height_m &&
            observation.height_m <= config_.maximum_height_m &&
            observation.uncertainty_m > 0.0F &&
            observation.confidence > 0.0F;
        if (!usable) continue;
        if (observation.source ==
                CargoThicknessSource::LIVE_VISIBLE_EXTENT &&
            observation.constraint ==
                CargoThicknessConstraint::FULL_MEASUREMENT) {
          if (!best_live ||
              observation_weight(observation) >
                  observation_weight(*best_live)) {
            best_live = &observation;
          }
        } else if (observation.source ==
                   CargoThicknessSource::STATIC_ORIGIN_TOP_SUPPORT) {
          if (!static_origin || observation_weight(observation) >
                  observation_weight(*static_origin)) {
            static_origin = &observation;
          }
        } else if (observation.source ==
                   CargoThicknessSource::MAP_DIFF_REVEALED_SUPPORT) {
          if (!revealed_support || observation_weight(observation) >
                  observation_weight(*revealed_support)) {
            revealed_support = &observation;
          }
        }
      }
      const CargoThicknessObservation* best_static =
          revealed_support ? revealed_support : static_origin;
      const bool static_revealed_consistent = static_origin &&
          revealed_support &&
          static_origin->constraint ==
              CargoThicknessConstraint::FULL_MEASUREMENT &&
          revealed_support->constraint ==
              CargoThicknessConstraint::FULL_MEASUREMENT &&
          std::abs(static_origin->height_m - revealed_support->height_m) <=
              config_.maximum_source_disagreement_m;
      const bool static_live_consistent = best_static && best_live &&
          best_static->constraint ==
              CargoThicknessConstraint::FULL_MEASUREMENT &&
          std::abs(best_static->height_m - best_live->height_m) <=
              config_.maximum_source_disagreement_m;
      const bool promotion_consistent =
          frame.formal_track_locked &&
          frame.center_valid && frame.center.allFinite() &&
          validDimensionEvidence(
              frame, config_,
              config_.minimum_live_shape_confidence_for_shrink) &&
          (static_revealed_consistent || static_live_consistent);
      formal_promotion_confirm_count_ = promotion_consistent
          ? formal_promotion_confirm_count_ + 1 : 0;
      if (formal_promotion_confirm_count_ >=
          config_.minimum_confirm_frames) {
        result_.formal_authorized = true;
        result_.degraded_live_only = false;
        result_.authorization = CargoGeometryAuthorization::FORMAL;
        result_.source_conflict = false;
        result_.independent_sources = 2U;
        if (static_revealed_consistent) {
          result_.accepted_sources = {
              CargoThicknessSource::STATIC_ORIGIN_TOP_SUPPORT,
              CargoThicknessSource::MAP_DIFF_REVEALED_SUPPORT};
          result_.height_m = std::max(
              static_origin->height_m, revealed_support->height_m);
          result_.height_uncertainty_m = std::max(
              static_origin->uncertainty_m,
              revealed_support->uncertainty_m);
        } else {
          result_.accepted_sources = {
              best_static->source,
              CargoThicknessSource::LIVE_VISIBLE_EXTENT};
          result_.height_m = std::max(
              best_static->height_m, best_live->height_m);
          result_.height_uncertainty_m = std::max(
              best_static->uncertainty_m, best_live->uncertainty_m);
        }
        result_.thickness_lower_bound_m = std::max(
            config_.minimum_height_m,
            result_.height_m - result_.height_uncertainty_m);
        result_.thickness_upper_bound_m = std::min(
            config_.maximum_height_m,
            result_.height_m + result_.height_uncertainty_m);
      }
    }
    if (frame.track_segment_id != shrink_track_segment_id_) {
      shrink_confirm_count_ = 0;
      shrink_track_segment_id_ = frame.track_segment_id;
      shrink_quality_window_.clear();
      shrink_length_window_.clear();
      shrink_width_window_.clear();
    }
    if (frame.track_segment_id != expand_track_segment_id_ ||
        !observation_continuous) {
      expand_confirm_count_ = 0;
      expand_track_segment_id_ = frame.track_segment_id;
      pending_expand_length_m_ = 0.0F;
      pending_expand_width_m_ = 0.0F;
    }
    if (!observation_continuous) {
      shrink_confirm_count_ = 0;
      shrink_quality_window_.clear();
      shrink_length_window_.clear();
      shrink_width_window_.clear();
    }
    result_.track_segment_id = frame.track_segment_id;
    if (frame.center_valid && frame.center.allFinite()) {
      result_.center = frame.center;
    }
    if (frame.observed_top_valid && std::isfinite(frame.observed_top_m)) {
      result_.bottom_m = frame.observed_top_m - result_.height_m;
    } else if (frame.center_valid && frame.center.allFinite()) {
      result_.bottom_m = frame.center.z() - 0.5F * result_.height_m;
    }
    if (std::isfinite(result_.bottom_m)) {
      result_.conservative_top_reference_m =
          result_.bottom_m + result_.height_m;
      result_.conservative_tracking_allowance_m =
          std::max(0.0F, frame.tracking_uncertainty_m);
      result_.conservative_baseline_allowance_m =
          result_.formal_authorized
              ? std::max(0.0F, frame.top_uncertainty_m)
              : std::max(0.15F, 3.0F *
                    std::max(0.0F, frame.top_uncertainty_m));
      result_.conservative_safety_margin_m =
          config_.configured_bottom_margin_m;
      result_.conservative_bottom_m =
          result_.conservative_top_reference_m -
          (result_.formal_authorized
               ? result_.height_m + result_.height_uncertainty_m
               : std::max(result_.height_m,
                          result_.thickness_upper_bound_m)) -
          result_.conservative_tracking_allowance_m -
          result_.conservative_baseline_allowance_m -
          result_.conservative_safety_margin_m;
    }
    const bool dimension_quality_valid =
        validDimensionEvidence(
            frame, config_,
            config_.minimum_live_shape_confidence_for_shrink);
    if (dimension_quality_valid) {
      const float length_floor = std::max(
          config_.minimum_physical_length_m,
          std::max(0.0F, frame.static_length_lower_bound_m));
      const float width_floor = std::max(
          config_.minimum_physical_width_m,
          std::max(0.0F, frame.static_width_lower_bound_m));
      const bool expands = frame.length_m > result_.length_m ||
          frame.width_m > result_.width_m;
      if (config_.immediate_expand_enabled && expands) {
        result_.length_m = std::max(result_.length_m, frame.length_m);
        result_.width_m = std::max(result_.width_m, frame.width_m);
        shrink_confirm_count_ = 0;
        shrink_quality_window_.clear();
        shrink_length_window_.clear();
        shrink_width_window_.clear();
        expand_confirm_count_ = 0;
        pending_expand_length_m_ = 0.0F;
        pending_expand_width_m_ = 0.0F;
      } else if (expands &&
                 frame.dimension_shape_confidence >=
                     config_.minimum_live_shape_confidence_for_expand) {
        shrink_confirm_count_ = 0;
        shrink_quality_window_.clear();
        shrink_length_window_.clear();
        shrink_width_window_.clear();
        const float supported_length =
            std::max(result_.length_m, frame.length_m);
        const float supported_width =
            std::max(result_.width_m, frame.width_m);
        if (expand_confirm_count_ == 0) {
          pending_expand_length_m_ = supported_length;
          pending_expand_width_m_ = supported_width;
        } else {
          // Use the lower bound supported by every frame in the streak. One
          // contaminated large frame cannot dictate the frozen dimensions.
          pending_expand_length_m_ =
              std::min(pending_expand_length_m_, supported_length);
          pending_expand_width_m_ =
              std::min(pending_expand_width_m_, supported_width);
        }
        ++expand_confirm_count_;
        if (expand_confirm_count_ >=
            config_.conservative_expand_confirm_frames) {
          result_.length_m =
              std::max(result_.length_m, pending_expand_length_m_);
          result_.width_m =
              std::max(result_.width_m, pending_expand_width_m_);
          expand_confirm_count_ = 0;
          pending_expand_length_m_ = 0.0F;
          pending_expand_width_m_ = 0.0F;
        }
      } else if (expands) {
        // A mixed larger/smaller weak observation cannot exploit the shrink
        // branch to alter either frozen dimension.
        shrink_confirm_count_ = 0;
        shrink_quality_window_.clear();
        shrink_length_window_.clear();
        shrink_width_window_.clear();
        expand_confirm_count_ = 0;
        pending_expand_length_m_ = 0.0F;
        pending_expand_width_m_ = 0.0F;
      } else if (frame.length_m < result_.length_m ||
                 frame.width_m < result_.width_m) {
        expand_confirm_count_ = 0;
        pending_expand_length_m_ = 0.0F;
        pending_expand_width_m_ = 0.0F;
        shrink_quality_window_.push_back(true);
        shrink_length_window_.push_back(frame.length_m);
        shrink_width_window_.push_back(frame.width_m);
        while (shrink_quality_window_.size() >
               static_cast<std::size_t>(
                   config_.shape_confirmation_window_frames)) {
          shrink_quality_window_.pop_front();
          shrink_length_window_.pop_front();
          shrink_width_window_.pop_front();
        }
        shrink_confirm_count_ = static_cast<int>(std::count(
            shrink_quality_window_.begin(),
            shrink_quality_window_.end(), true));
        if (shrink_confirm_count_ >=
            config_.conservative_shrink_confirm_frames) {
          std::vector<float> shrink_lengths;
          std::vector<float> shrink_widths;
          shrink_lengths.reserve(shrink_length_window_.size());
          shrink_widths.reserve(shrink_width_window_.size());
          for (const float value : shrink_length_window_) {
            if (std::isfinite(value)) shrink_lengths.push_back(value);
          }
          for (const float value : shrink_width_window_) {
            if (std::isfinite(value)) shrink_widths.push_back(value);
          }
          const float supported_length = finiteMedian(shrink_lengths);
          const float supported_width = finiteMedian(shrink_widths);
          const bool stable_window =
              finiteMad(shrink_lengths, supported_length) <=
                  config_.maximum_initial_dimension_mad_m &&
              finiteMad(shrink_widths, supported_width) <=
                  config_.maximum_initial_dimension_mad_m;
          if (stable_window) {
            result_.length_m = std::max(
                length_floor,
                std::max(supported_length,
                         result_.length_m -
                             config_.maximum_shrink_per_frame_m));
            result_.width_m = std::max(
                width_floor,
                std::max(supported_width,
                         result_.width_m -
                             config_.maximum_shrink_per_frame_m));
            shrink_quality_window_.clear();
            shrink_length_window_.clear();
            shrink_width_window_.clear();
            shrink_confirm_count_ = 0;
          }
        }
      } else {
        shrink_confirm_count_ = 0;
        shrink_quality_window_.clear();
        shrink_length_window_.clear();
        shrink_width_window_.clear();
        expand_confirm_count_ = 0;
        pending_expand_length_m_ = 0.0F;
        pending_expand_width_m_ = 0.0F;
      }
    } else {
      // Partial sides, tiny clusters and weak identities may only retain the
      // conservative frozen dimensions; they never authorize shrinkage.
      shrink_quality_window_.push_back(false);
      shrink_length_window_.push_back(
          std::numeric_limits<float>::quiet_NaN());
      shrink_width_window_.push_back(
          std::numeric_limits<float>::quiet_NaN());
      while (shrink_quality_window_.size() >
             static_cast<std::size_t>(
                 config_.shape_confirmation_window_frames)) {
        shrink_quality_window_.pop_front();
        shrink_length_window_.pop_front();
        shrink_width_window_.pop_front();
      }
      shrink_confirm_count_ = static_cast<int>(std::count(
          shrink_quality_window_.begin(),
          shrink_quality_window_.end(), true));
      expand_confirm_count_ = 0;
      pending_expand_length_m_ = 0.0F;
      pending_expand_width_m_ = 0.0F;
    }
    result_.valid = frame.center_valid && frame.center.allFinite() &&
        std::isfinite(result_.bottom_m);
    result_.reason = result_.valid
        ? (result_.authorization == CargoGeometryAuthorization::FORMAL
               ? "frozen_formal_geometry_pose_updated"
               : "frozen_positive_only_geometry_pose_updated")
        : "frozen_geometry_pose_or_top_invalid";
    return result_;
  }

  if (shape_confirm_track_segment_id_ != frame.track_segment_id) {
    shape_confirm_track_segment_id_ = frame.track_segment_id;
    shape_confirm_count_ = 0;
    shape_quality_window_.clear();
    initial_length_window_.clear();
    initial_width_window_.clear();
  }
  if (thickness_confirm_track_segment_id_ != frame.track_segment_id) {
    thickness_confirm_track_segment_id_ = frame.track_segment_id;
    thickness_candidate_window_.clear();
  }
  thickness_candidate_window_.push_back(
      std::numeric_limits<float>::quiet_NaN());
  while (thickness_candidate_window_.size() >
         static_cast<std::size_t>(
             config_.shape_confirmation_window_frames)) {
    thickness_candidate_window_.pop_front();
  }
  const bool initial_dimension_valid = validDimensionEvidence(
      frame, config_, config_.minimum_initial_shape_confidence);
  shape_quality_window_.push_back(initial_dimension_valid);
  initial_length_window_.push_back(initial_dimension_valid
      ? frame.length_m : std::numeric_limits<float>::quiet_NaN());
  initial_width_window_.push_back(initial_dimension_valid
      ? frame.width_m : std::numeric_limits<float>::quiet_NaN());
  while (shape_quality_window_.size() >
         static_cast<std::size_t>(
             config_.shape_confirmation_window_frames)) {
    shape_quality_window_.pop_front();
    initial_length_window_.pop_front();
    initial_width_window_.pop_front();
  }
  shape_confirm_count_ = static_cast<int>(std::count(
      shape_quality_window_.begin(), shape_quality_window_.end(), true));
  result_.shape_confirm_frames = shape_confirm_count_;
  std::vector<float> initial_lengths;
  std::vector<float> initial_widths;
  initial_lengths.reserve(initial_length_window_.size());
  initial_widths.reserve(initial_width_window_.size());
  for (const float value : initial_length_window_) {
    if (std::isfinite(value)) initial_lengths.push_back(value);
  }
  for (const float value : initial_width_window_) {
    if (std::isfinite(value)) initial_widths.push_back(value);
  }
  const float robust_initial_length = finiteMedian(initial_lengths);
  const float robust_initial_width = finiteMedian(initial_widths);
  const bool initial_dimensions_stable =
      !initial_lengths.empty() && !initial_widths.empty() &&
      finiteMad(initial_lengths, robust_initial_length) <=
          config_.maximum_initial_dimension_mad_m &&
      finiteMad(initial_widths, robust_initial_width) <=
          config_.maximum_initial_dimension_mad_m;

  if (!frame.center_valid || !frame.center.allFinite() ||
      !frame.footprint_valid || !std::isfinite(frame.length_m) ||
      !std::isfinite(frame.width_m) || !std::isfinite(frame.yaw_rad) ||
      frame.length_m <= 0.0F || frame.width_m <= 0.0F) {
    result_.valid = false;
    result_.reason = "footprint_or_center_invalid";
    return result_;
  }

  std::map<CargoThicknessSource, WeightedHeight> best_by_source;
  for (const CargoThicknessObservation& observation : frame.thickness) {
    if (!observation.valid || !std::isfinite(observation.height_m) ||
        !std::isfinite(observation.uncertainty_m) ||
        !std::isfinite(observation.confidence) ||
        observation.height_m < config_.minimum_height_m ||
        observation.height_m > config_.maximum_height_m ||
        observation.uncertainty_m <= 0.0F ||
        observation.confidence <= 0.0F) {
      continue;
    }
    const float weight = observation.confidence /
        (observation.uncertainty_m * observation.uncertainty_m);
    const WeightedHeight candidate{
        observation.source, observation.constraint, observation.height_m,
        observation.uncertainty_m, weight};
    const auto found = best_by_source.find(observation.source);
    if (found == best_by_source.end() ||
        candidate.weight > found->second.weight) {
      best_by_source[observation.source] = candidate;
    }
  }
  std::size_t independent_sources = 0U;
  bool has_static_origin = false;
  bool has_revealed_support = false;
  for (const auto& item : best_by_source) {
    if (item.first != CargoThicknessSource::CONFIGURED_FALLBACK &&
        item.first != CargoThicknessSource::RETIRED_LOCKED_SHAPE &&
        item.second.constraint != CargoThicknessConstraint::PRIOR_ONLY) {
      ++independent_sources;
    }
    has_static_origin =
        has_static_origin ||
        item.first ==
            CargoThicknessSource::STATIC_ORIGIN_TOP_SUPPORT;
    has_revealed_support =
        has_revealed_support ||
        item.first ==
            CargoThicknessSource::MAP_DIFF_REVEALED_SUPPORT;
  }
  result_.cargo_lifecycle_id = frame.cargo_lifecycle_id;
  result_.track_segment_id = frame.track_segment_id;
  result_.independent_sources = independent_sources;
  const bool has_authoritative_static_height =
      has_static_origin || has_revealed_support;
  const auto best = [&](CargoThicknessSource source)
      -> const WeightedHeight* {
    const auto found = best_by_source.find(source);
    return found == best_by_source.end() ? nullptr : &found->second;
  };
  const WeightedHeight* static_origin = best(
      CargoThicknessSource::STATIC_ORIGIN_TOP_SUPPORT);
  const WeightedHeight* revealed = best(
      CargoThicknessSource::MAP_DIFF_REVEALED_SUPPORT);
  const WeightedHeight* live = best(
      CargoThicknessSource::LIVE_VISIBLE_EXTENT);
  const bool static_origin_is_full = static_origin &&
      static_origin->constraint ==
          CargoThicknessConstraint::FULL_MEASUREMENT;
  const bool revealed_is_full = revealed &&
      revealed->constraint ==
          CargoThicknessConstraint::FULL_MEASUREMENT;
  const auto mutually_consistent = [&](const WeightedHeight* lhs,
                                       const WeightedHeight* rhs) {
    return lhs && rhs &&
        std::abs(lhs->height - rhs->height) <=
            config_.maximum_source_disagreement_m;
  };
  const bool static_revealed_formal = static_origin_is_full &&
      revealed_is_full && mutually_consistent(static_origin, revealed);
  const WeightedHeight* best_static = revealed ? revealed : static_origin;
  const bool live_is_full = live && live->constraint ==
      CargoThicknessConstraint::FULL_MEASUREMENT;
  const bool static_live_formal = best_static &&
      best_static->constraint ==
          CargoThicknessConstraint::FULL_MEASUREMENT && live_is_full &&
      mutually_consistent(best_static, live);
  const bool has_formal_pair = frame.formal_track_locked &&
      (static_revealed_formal || static_live_formal);
  const bool source_conflict = best_static && live &&
      live->height > best_static->height +
          config_.maximum_source_disagreement_m;
  const bool lower_bound_compatible = best_static && live &&
      best_static->constraint ==
          CargoThicknessConstraint::FULL_MEASUREMENT &&
      live->constraint == CargoThicknessConstraint::LOWER_BOUND &&
      live->height <= best_static->height +
          config_.maximum_source_disagreement_m;
  const bool full_measurement_compatible = best_static && live &&
      best_static->constraint ==
          CargoThicknessConstraint::FULL_MEASUREMENT &&
      live->constraint == CargoThicknessConstraint::FULL_MEASUREMENT &&
      std::abs(live->height - best_static->height) <=
          config_.maximum_source_disagreement_m;
  const bool positive_only_candidate = frame.warning_track_stable && live &&
      ((!best_static &&
        config_.allow_positive_only_without_static_baseline) ||
       (best_static && lower_bound_compatible) ||
       full_measurement_compatible ||
       (source_conflict &&
        config_.allow_positive_only_on_source_conflict));

  if (!thickness_confirmation_mode_initialized_ ||
      thickness_confirmation_formal_mode_ != has_formal_pair) {
    thickness_confirmation_mode_initialized_ = true;
    thickness_confirmation_formal_mode_ = has_formal_pair;
    thickness_candidate_window_.clear();
    thickness_candidate_window_.push_back(
        std::numeric_limits<float>::quiet_NaN());
  }

  if (!has_formal_pair && !positive_only_candidate) {
    result_.valid = false;
    result_.authorization = CargoGeometryAuthorization::PENDING;
    result_.reason = has_authoritative_static_height && live
        ? "thickness_full_measurement_confirmation_pending"
        : "independent_thickness_sources_insufficient";
    result_.confirm_frames = 0;
    return result_;
  }

  std::vector<WeightedHeight> selected_values;
  if (has_formal_pair) {
    if (static_revealed_formal) {
      selected_values = {*static_origin, *revealed};
    } else {
      selected_values = {*best_static, *live};
    }
  } else {
    // Positive-only geometry uses the union of credible physical bounds. A
    // visible LiDAR extent is a lower bound on thickness, not an independent
    // point estimate that may invalidate the pre-lift baseline forever.
    if (best_static) selected_values.push_back(*best_static);
    selected_values.push_back(*live);
  }

  float fused = 0.0F;
  float uncertainty = 0.0F;
  float lower_bound = 0.0F;
  float upper_bound = 0.0F;
  for (const auto& value : selected_values) {
    lower_bound = std::max(lower_bound, value.height);
    upper_bound = std::max(
        upper_bound, value.height + value.uncertainty);
  }
  if (!has_formal_pair) {
    fused = std::clamp(lower_bound, config_.minimum_height_m,
                       config_.maximum_height_m);
    upper_bound = std::clamp(upper_bound, fused,
                             config_.maximum_height_m);
    uncertainty = std::max(
        upper_bound - fused,
        config_.positive_only_uncertainty_floor_m);
    upper_bound = std::min(
        config_.maximum_height_m,
        std::max(upper_bound, fused + uncertainty));
  } else {
    const float median = weightedMedian(selected_values);
    float weighted_sum = 0.0F;
    float weight_sum = 0.0F;
    for (const auto& value : selected_values) {
      const float residual = std::abs(value.height - median);
      const float huber = residual <= config_.huber_delta_m
          ? 1.0F : config_.huber_delta_m / residual;
      const float robust_weight = value.weight * huber;
      weighted_sum += robust_weight * value.height;
      weight_sum += robust_weight;
    }
    fused = weighted_sum / std::max(weight_sum, 1.0e-6F);
    float residual_variance = 0.0F;
    for (const auto& value : selected_values) {
      const float residual = value.height - fused;
      residual_variance += value.weight * residual * residual;
    }
    residual_variance /= std::max(weight_sum, 1.0e-6F);
    uncertainty = std::sqrt(
        1.0F / std::max(weight_sum, 1.0e-6F) + residual_variance);
    lower_bound = std::max(
        config_.minimum_height_m, fused - uncertainty);
    upper_bound = std::min(
        config_.maximum_height_m, fused + uncertainty);
  }
  if (!std::isfinite(fused) || !std::isfinite(uncertainty) ||
      (has_formal_pair &&
       uncertainty > config_.maximum_fused_uncertainty_m)) {
    result_.valid = false;
    result_.reason = "fused_thickness_uncertainty_exceeded";
    result_.confirm_frames = 0;
    return result_;
  }
  thickness_candidate_window_.back() = fused;
  std::vector<float> finite_candidates;
  finite_candidates.reserve(thickness_candidate_window_.size());
  for (const float candidate : thickness_candidate_window_) {
    if (std::isfinite(candidate)) finite_candidates.push_back(candidate);
  }
  const float robust_fused = finiteMedian(finite_candidates);
  const float stability_tolerance =
      0.5F * config_.maximum_source_disagreement_m;
  const bool current_stable =
      std::abs(fused - robust_fused) <= stability_tolerance;
  result_.confirm_frames = current_stable
      ? static_cast<int>(std::count_if(
            finite_candidates.begin(), finite_candidates.end(),
            [&](float candidate) {
              return std::abs(candidate - robust_fused) <=
                  stability_tolerance;
            }))
      : 0;
  if (current_stable) {
    fused = robust_fused;
    if (has_formal_pair) {
      lower_bound = std::max(
          config_.minimum_height_m, fused - uncertainty);
      upper_bound = std::min(
          config_.maximum_height_m, fused + uncertainty);
    } else {
      lower_bound = fused;
      upper_bound = std::min(
          config_.maximum_height_m,
          std::max(upper_bound, fused + uncertainty));
    }
  }
  result_.accepted_sources.clear();
  for (const auto& value : selected_values) {
    result_.accepted_sources.push_back(value.source);
  }
  result_.source_conflict = source_conflict;
  result_.thickness_lower_bound_m = lower_bound;
  result_.thickness_upper_bound_m = upper_bound;
  // A numerically usable candidate is not frozen geometry. Even after a
  // live-only freeze, runtime callers must check formal_authorized before
  // granting CLEAR/removal authority.
  result_.valid = false;
  const int required_confirm_frames = has_formal_pair
      ? config_.minimum_confirm_frames
      : config_.positive_only_confirm_frames;
  result_.reason = has_formal_pair
      ? "formal_thickness_confirmation_pending"
      : (source_conflict
             ? "positive_only_source_conflict_confirmation_pending"
             : (lower_bound_compatible
                    ? "positive_only_lower_bound_confirmation_pending"
                    : "positive_only_live_confirmation_pending"));
  if (result_.confirm_frames >= required_confirm_frames &&
      shape_confirm_count_ >= required_confirm_frames &&
      initial_dimensions_stable) {
    result_.frozen = true;
    result_.valid = true;
    result_.formal_authorized = has_formal_pair;
    result_.degraded_live_only = !has_formal_pair;
    result_.authorization = has_formal_pair
        ? CargoGeometryAuthorization::FORMAL
        : CargoGeometryAuthorization::POSITIVE_ONLY;
    result_.center = frame.center;
    result_.length_m = std::max(
        std::max(robust_initial_length,
                 config_.formal_transition_start_length_m),
        std::max(config_.minimum_physical_length_m,
                 frame.static_length_lower_bound_m));
    result_.width_m = std::max(
        std::max(robust_initial_width,
                 config_.formal_transition_start_width_m),
        std::max(config_.minimum_physical_width_m,
                 frame.static_width_lower_bound_m));
    result_.height_m = fused;
    result_.yaw_rad = frame.yaw_rad;
    result_.height_uncertainty_m = uncertainty;
    result_.bottom_m = frame.observed_top_valid &&
            std::isfinite(frame.observed_top_m)
        ? frame.observed_top_m - fused
        : frame.center.z() - 0.5F * fused;
    result_.conservative_top_reference_m =
        result_.bottom_m + result_.height_m;
    result_.conservative_tracking_allowance_m =
        std::max(0.0F, frame.tracking_uncertainty_m);
    result_.conservative_baseline_allowance_m = has_formal_pair
        ? std::max(0.0F, frame.top_uncertainty_m)
        : std::max(0.15F, 3.0F *
              std::max(0.0F, frame.top_uncertainty_m));
    result_.conservative_safety_margin_m =
        config_.configured_bottom_margin_m;
    result_.conservative_bottom_m =
        result_.conservative_top_reference_m -
        (has_formal_pair
             ? result_.height_m + result_.height_uncertainty_m
             : result_.thickness_upper_bound_m) -
        result_.conservative_tracking_allowance_m -
        result_.conservative_baseline_allowance_m -
        result_.conservative_safety_margin_m;
    result_.reason = result_.formal_authorized
        ? "geometry_frozen_formal_constraints"
        : (source_conflict
               ? "geometry_frozen_positive_only_source_conflict"
               : "geometry_frozen_positive_only_live_bound");
    shrink_track_segment_id_ = frame.track_segment_id;
    expand_track_segment_id_ = frame.track_segment_id;
  } else if (result_.confirm_frames >= required_confirm_frames) {
    if (shape_confirm_count_ >= required_confirm_frames &&
        !initial_dimensions_stable) {
      result_.reason = "initial_dimension_mad_exceeded";
    } else {
      result_.reason = has_formal_pair
          ? "formal_shape_confirmation_pending"
          : "positive_only_shape_confirmation_pending";
    }
  }
  return result_;
}

}  // namespace ndt_slam
