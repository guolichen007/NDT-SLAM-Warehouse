#include "ndt_slam/cargo_geometry_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

namespace ndt_slam {
namespace {

bool validConfig(const CargoGeometryFusionConfig& config) {
  return config.minimum_independent_sources > 0U &&
      config.minimum_confirm_frames > 0 &&
      config.maximum_observation_gap_sec > 0.0 &&
      config.maximum_source_disagreement_m > 0.0F &&
      config.maximum_fused_uncertainty_m > 0.0F &&
      config.maximum_height_m > config.minimum_height_m &&
      config.huber_delta_m > 0.0F &&
      config.configured_bottom_margin_m >= 0.0F &&
      config.conservative_shrink_confirm_frames > 0 &&
      config.maximum_shrink_per_frame_m > 0.0F &&
      config.minimum_live_dimension_support > 0U &&
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
                            const CargoGeometryFusionConfig& config) {
  return frame.footprint_valid &&
      std::isfinite(frame.length_m) && frame.length_m > 0.0F &&
      std::isfinite(frame.width_m) && frame.width_m > 0.0F &&
      frame.dimension_observation_complete &&
      frame.dimension_support_points >=
          config.minimum_live_dimension_support &&
      std::isfinite(frame.dimension_shape_confidence) &&
      frame.dimension_shape_confidence >=
          config.minimum_live_shape_confidence_for_shrink;
}

struct WeightedHeight {
  CargoThicknessSource source;
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
  pending_height_m_ = 0.0F;
  pending_uncertainty_m_ = 1.0F;
  pending_valid_ = false;
  last_stamp_sec_ = 0.0;
  shrink_confirm_count_ = 0;
  shrink_track_segment_id_ = 0U;
  shape_confirm_count_ = 0;
  shape_confirm_track_segment_id_ = 0U;
}

CargoFrozenGeometry CargoGeometryFusion::update(
    const CargoGeometryFrame& frame) {
  const double previous_stamp_sec = last_stamp_sec_;
  if (!std::isfinite(frame.stamp_sec) || frame.stamp_sec <= 0.0 ||
      (last_stamp_sec_ > 0.0 && frame.stamp_sec <= last_stamp_sec_)) {
    result_.valid = false;
    result_.reason = "source_time_invalid_or_rollback";
    return result_;
  }
  last_stamp_sec_ = frame.stamp_sec;
  if (!result_.frozen && previous_stamp_sec > 0.0 &&
      frame.stamp_sec - previous_stamp_sec >
          config_.maximum_observation_gap_sec) {
    result_.confirm_frames = 0;
    pending_valid_ = false;
    shape_confirm_count_ = 0;
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
    if (frame.track_segment_id != shrink_track_segment_id_) {
      shrink_confirm_count_ = 0;
      shrink_track_segment_id_ = frame.track_segment_id;
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
      result_.conservative_bottom_m = result_.bottom_m -
          std::max(0.0F, frame.top_uncertainty_m) -
          result_.height_uncertainty_m -
          std::max(0.0F, frame.tracking_uncertainty_m) -
          config_.configured_bottom_margin_m;
    }
    const bool dimension_quality_valid =
        validDimensionEvidence(frame, config_);
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
      } else if (frame.length_m < result_.length_m ||
                 frame.width_m < result_.width_m) {
        ++shrink_confirm_count_;
        if (shrink_confirm_count_ >=
            config_.conservative_shrink_confirm_frames) {
          result_.length_m = std::max(
              length_floor,
              std::max(frame.length_m,
                       result_.length_m -
                           config_.maximum_shrink_per_frame_m));
          result_.width_m = std::max(
              width_floor,
              std::max(frame.width_m,
                       result_.width_m -
                           config_.maximum_shrink_per_frame_m));
        }
      } else {
        shrink_confirm_count_ = 0;
      }
    } else {
      // Partial sides, tiny clusters and weak identities may only retain the
      // conservative frozen dimensions; they never authorize shrinkage.
      shrink_confirm_count_ = 0;
    }
    result_.valid = frame.center_valid && frame.center.allFinite() &&
        std::isfinite(result_.bottom_m);
    result_.reason = result_.valid
        ? "frozen_geometry_pose_and_asymmetric_dimensions_updated"
        : "frozen_geometry_pose_or_top_invalid";
    return result_;
  }

  if (!frame.center_valid || !frame.center.allFinite() ||
      !frame.footprint_valid || !std::isfinite(frame.length_m) ||
      !std::isfinite(frame.width_m) || !std::isfinite(frame.yaw_rad) ||
      frame.length_m <= 0.0F || frame.width_m <= 0.0F) {
    result_.valid = false;
    result_.reason = "footprint_or_center_invalid";
    return result_;
  }

  if (shape_confirm_track_segment_id_ != frame.track_segment_id) {
    shape_confirm_track_segment_id_ = frame.track_segment_id;
    shape_confirm_count_ = 0;
  }
  if (validDimensionEvidence(frame, config_)) {
    ++shape_confirm_count_;
  } else {
    shape_confirm_count_ = 0;
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
        observation.source, observation.height_m,
        observation.uncertainty_m, weight};
    const auto found = best_by_source.find(observation.source);
    if (found == best_by_source.end() ||
        candidate.weight > found->second.weight) {
      best_by_source[observation.source] = candidate;
    }
  }
  std::vector<WeightedHeight> values;
  std::size_t independent_sources = 0U;
  for (const auto& item : best_by_source) {
    values.push_back(item.second);
    if (item.first != CargoThicknessSource::CONFIGURED_FALLBACK) {
      ++independent_sources;
    }
  }
  result_.cargo_lifecycle_id = frame.cargo_lifecycle_id;
  result_.track_segment_id = frame.track_segment_id;
  result_.independent_sources = independent_sources;
  if (independent_sources < config_.minimum_independent_sources) {
    result_.valid = false;
    result_.reason = "independent_thickness_sources_insufficient";
    result_.confirm_frames = 0;
    pending_valid_ = false;
    return result_;
  }
  float minimum = values.front().height;
  float maximum = values.front().height;
  for (const auto& value : values) {
    minimum = std::min(minimum, value.height);
    maximum = std::max(maximum, value.height);
  }
  if (maximum - minimum > config_.maximum_source_disagreement_m) {
    result_.valid = false;
    result_.reason = "thickness_source_disagreement";
    result_.confirm_frames = 0;
    pending_valid_ = false;
    return result_;
  }

  const float median = weightedMedian(values);
  float weighted_sum = 0.0F;
  float weight_sum = 0.0F;
  for (const auto& value : values) {
    const float residual = std::abs(value.height - median);
    const float huber = residual <= config_.huber_delta_m
        ? 1.0F : config_.huber_delta_m / residual;
    const float robust_weight = value.weight * huber;
    weighted_sum += robust_weight * value.height;
    weight_sum += robust_weight;
  }
  const float fused = weighted_sum / std::max(weight_sum, 1.0e-6F);
  float residual_variance = 0.0F;
  for (const auto& value : values) {
    const float residual = value.height - fused;
    residual_variance += value.weight * residual * residual;
  }
  residual_variance /= std::max(weight_sum, 1.0e-6F);
  const float uncertainty = std::sqrt(
      1.0F / std::max(weight_sum, 1.0e-6F) + residual_variance);
  if (!std::isfinite(fused) || !std::isfinite(uncertainty) ||
      uncertainty > config_.maximum_fused_uncertainty_m) {
    result_.valid = false;
    result_.reason = "fused_thickness_uncertainty_exceeded";
    result_.confirm_frames = 0;
    pending_valid_ = false;
    return result_;
  }

  const bool consistent = pending_valid_ &&
      std::abs(fused - pending_height_m_) <=
          0.5F * config_.maximum_source_disagreement_m;
  result_.confirm_frames = consistent ? result_.confirm_frames + 1 : 1;
  pending_height_m_ = fused;
  pending_uncertainty_m_ = uncertainty;
  pending_valid_ = true;
  result_.accepted_sources.clear();
  for (const auto& value : values) {
    result_.accepted_sources.push_back(value.source);
  }
  // A numerically usable candidate is not formal geometry. Runtime callers
  // must not gain safety/removal authority until the shape is frozen.
  result_.valid = false;
  result_.reason = "thickness_confirmation_pending";
  if (result_.confirm_frames >= config_.minimum_confirm_frames &&
      shape_confirm_count_ >= config_.minimum_confirm_frames) {
    result_.frozen = true;
    result_.valid = true;
    result_.center = frame.center;
    result_.length_m = std::max(
        std::max(frame.length_m,
                 config_.formal_transition_start_length_m),
        std::max(config_.minimum_physical_length_m,
                 frame.static_length_lower_bound_m));
    result_.width_m = std::max(
        std::max(frame.width_m,
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
    result_.conservative_bottom_m = result_.bottom_m -
        std::max(0.0F, frame.top_uncertainty_m) - uncertainty -
        std::max(0.0F, frame.tracking_uncertainty_m) -
        config_.configured_bottom_margin_m;
    result_.reason = "geometry_frozen";
    shrink_track_segment_id_ = frame.track_segment_id;
  } else if (result_.confirm_frames >= config_.minimum_confirm_frames) {
    result_.reason = "formal_shape_confirmation_pending";
  }
  return result_;
}

}  // namespace ndt_slam
