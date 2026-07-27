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
      config.shape_confirmation_window_frames >=
          config.minimum_confirm_frames &&
      std::isfinite(config.degraded_live_only_uncertainty_floor_m) &&
      config.degraded_live_only_uncertainty_floor_m > 0.0F &&
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
  expand_confirm_count_ = 0;
  expand_track_segment_id_ = 0U;
  pending_expand_length_m_ = 0.0F;
  pending_expand_width_m_ = 0.0F;
  shape_confirm_count_ = 0;
  shape_confirm_track_segment_id_ = 0U;
  shape_quality_window_.clear();
  formal_promotion_confirm_count_ = 0;
  formal_promotion_track_segment_id_ = 0U;
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
    shape_quality_window_.clear();
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
      const CargoThicknessObservation* best_static = nullptr;
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
                CargoThicknessSource::LIVE_VISIBLE_EXTENT) {
          if (!best_live ||
              observation_weight(observation) >
                  observation_weight(*best_live)) {
            best_live = &observation;
          }
        } else if (
            observation.source ==
                CargoThicknessSource::STATIC_ORIGIN_TOP_SUPPORT ||
            observation.source ==
                CargoThicknessSource::MAP_DIFF_REVEALED_SUPPORT) {
          if (!best_static ||
              observation_weight(observation) >
                  observation_weight(*best_static)) {
            best_static = &observation;
          }
        }
      }
      const bool promotion_consistent =
          frame.formal_track_locked &&
          frame.center_valid && frame.center.allFinite() &&
          validDimensionEvidence(frame, config_) &&
          best_static && best_live &&
          std::abs(best_static->height_m - best_live->height_m) <=
              config_.maximum_source_disagreement_m &&
          std::abs(best_static->height_m - result_.height_m) <=
              config_.maximum_source_disagreement_m &&
          std::abs(best_live->height_m - result_.height_m) <=
              config_.maximum_source_disagreement_m;
      formal_promotion_confirm_count_ = promotion_consistent
          ? formal_promotion_confirm_count_ + 1 : 0;
      if (formal_promotion_confirm_count_ >=
          config_.minimum_confirm_frames) {
        result_.formal_authorized = true;
        result_.degraded_live_only = false;
        result_.independent_sources = 2U;
        result_.accepted_sources = {
            best_static->source,
            CargoThicknessSource::LIVE_VISIBLE_EXTENT};
      }
    }
    if (frame.track_segment_id != shrink_track_segment_id_) {
      shrink_confirm_count_ = 0;
      shrink_track_segment_id_ = frame.track_segment_id;
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
        expand_confirm_count_ = 0;
        pending_expand_length_m_ = 0.0F;
        pending_expand_width_m_ = 0.0F;
      } else if (expands &&
                 frame.dimension_shape_confidence >=
                     config_.minimum_live_shape_confidence_for_expand) {
        shrink_confirm_count_ = 0;
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
        expand_confirm_count_ = 0;
        pending_expand_length_m_ = 0.0F;
        pending_expand_width_m_ = 0.0F;
      } else if (frame.length_m < result_.length_m ||
                 frame.width_m < result_.width_m) {
        expand_confirm_count_ = 0;
        pending_expand_length_m_ = 0.0F;
        pending_expand_width_m_ = 0.0F;
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
        expand_confirm_count_ = 0;
        pending_expand_length_m_ = 0.0F;
        pending_expand_width_m_ = 0.0F;
      }
    } else {
      // Partial sides, tiny clusters and weak identities may only retain the
      // conservative frozen dimensions; they never authorize shrinkage.
      shrink_confirm_count_ = 0;
      expand_confirm_count_ = 0;
      pending_expand_length_m_ = 0.0F;
      pending_expand_width_m_ = 0.0F;
    }
    result_.valid = frame.center_valid && frame.center.allFinite() &&
        std::isfinite(result_.bottom_m);
    result_.reason = result_.valid
        ? (result_.formal_authorized
               ? "frozen_formal_geometry_pose_updated"
               : "frozen_degraded_geometry_pose_updated")
        : "frozen_geometry_pose_or_top_invalid";
    return result_;
  }

  if (shape_confirm_track_segment_id_ != frame.track_segment_id) {
    shape_confirm_track_segment_id_ = frame.track_segment_id;
    shape_confirm_count_ = 0;
    shape_quality_window_.clear();
  }
  shape_quality_window_.push_back(validDimensionEvidence(frame, config_));
  while (shape_quality_window_.size() >
         static_cast<std::size_t>(
             config_.shape_confirmation_window_frames)) {
    shape_quality_window_.pop_front();
  }
  shape_confirm_count_ = static_cast<int>(std::count(
      shape_quality_window_.begin(), shape_quality_window_.end(), true));
  result_.shape_confirm_frames = shape_confirm_count_;

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
  bool has_static_origin = false;
  bool has_revealed_support = false;
  bool has_live_visible_extent = false;
  for (const auto& item : best_by_source) {
    values.push_back(item.second);
    if (item.first != CargoThicknessSource::CONFIGURED_FALLBACK) {
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
    has_live_visible_extent =
        has_live_visible_extent ||
        item.first == CargoThicknessSource::LIVE_VISIBLE_EXTENT;
  }
  result_.cargo_lifecycle_id = frame.cargo_lifecycle_id;
  result_.track_segment_id = frame.track_segment_id;
  result_.independent_sources = independent_sources;
  const bool has_authoritative_static_height =
      has_static_origin || has_revealed_support;
  const bool has_formal_static_live_pair =
      has_authoritative_static_height && has_live_visible_extent;
  const bool degraded_live_only_candidate =
      config_.allow_degraded_live_only_freeze &&
      frame.formal_track_locked &&
      has_live_visible_extent && !has_authoritative_static_height;
  if (independent_sources < config_.minimum_independent_sources &&
      !degraded_live_only_candidate) {
    result_.valid = false;
    result_.reason = "independent_thickness_sources_insufficient";
    result_.confirm_frames = 0;
    pending_valid_ = false;
    return result_;
  }
  if (config_.require_authoritative_static_and_live_thickness &&
      !has_formal_static_live_pair &&
      !degraded_live_only_candidate) {
    result_.valid = false;
    result_.reason =
        "authoritative_static_and_live_thickness_required";
    result_.confirm_frames = 0;
    pending_valid_ = false;
    return result_;
  }
  if (degraded_live_only_candidate) {
    // Never let configured or retired estimates make a live-only candidate
    // look like independent physical corroboration. Its height is derived
    // exclusively from the multi-frame LiDAR extent and receives an explicit
    // conservative uncertainty floor below.
    values.erase(
        std::remove_if(
            values.begin(), values.end(),
            [](const WeightedHeight& value) {
              return value.source !=
                  CargoThicknessSource::LIVE_VISIBLE_EXTENT;
            }),
        values.end());
    result_.independent_sources = 1U;
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
  // A numerically usable candidate is not frozen geometry. Even after a
  // live-only freeze, runtime callers must check formal_authorized before
  // granting CLEAR/removal authority.
  result_.valid = false;
  const int required_confirm_frames = degraded_live_only_candidate
      ? std::max(
            config_.minimum_confirm_frames,
            config_.shape_confirmation_window_frames)
      : config_.minimum_confirm_frames;
  result_.reason = degraded_live_only_candidate
      ? "degraded_live_thickness_confirmation_pending"
      : "thickness_confirmation_pending";
  if (result_.confirm_frames >= required_confirm_frames &&
      shape_confirm_count_ >= required_confirm_frames) {
    result_.frozen = true;
    result_.valid = true;
    result_.formal_authorized = has_formal_static_live_pair;
    result_.degraded_live_only = degraded_live_only_candidate;
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
    result_.height_uncertainty_m = degraded_live_only_candidate
        ? std::max(
              uncertainty,
              config_.degraded_live_only_uncertainty_floor_m)
        : uncertainty;
    result_.bottom_m = frame.observed_top_valid &&
            std::isfinite(frame.observed_top_m)
        ? frame.observed_top_m - fused
        : frame.center.z() - 0.5F * fused;
    result_.conservative_bottom_m = result_.bottom_m -
        std::max(0.0F, frame.top_uncertainty_m) -
        result_.height_uncertainty_m -
        std::max(0.0F, frame.tracking_uncertainty_m) -
        config_.configured_bottom_margin_m;
    result_.reason = result_.formal_authorized
        ? "geometry_frozen_formal_static_live"
        : "geometry_frozen_degraded_live_only";
    shrink_track_segment_id_ = frame.track_segment_id;
    expand_track_segment_id_ = frame.track_segment_id;
  } else if (result_.confirm_frames >= required_confirm_frames) {
    result_.reason = degraded_live_only_candidate
        ? "degraded_live_shape_confirmation_pending"
        : "formal_shape_confirmation_pending";
  }
  return result_;
}

}  // namespace ndt_slam
