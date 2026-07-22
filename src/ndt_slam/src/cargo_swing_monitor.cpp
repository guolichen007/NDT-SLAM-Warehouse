#include "ndt_slam/cargo_swing_monitor.hpp"

#include "ndt_slam/cargo_oriented_footprint.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ndt_slam {
namespace {

constexpr float kRadToDeg = 57.2957795130823208768F;

bool validConfig(const CargoSwingConfig& config) {
  return std::isfinite(config.configured_sling_length_m) &&
      config.configured_sling_length_m > 0.0F &&
      std::isfinite(config.minimum_rope_length_m) &&
      config.minimum_rope_length_m > 0.0F &&
      config.configured_sling_length_m >= config.minimum_rope_length_m &&
      std::isfinite(config.minimum_valid_observation_sec) &&
      config.minimum_valid_observation_sec >= 0.0 &&
      std::isfinite(config.maximum_observation_gap_sec) &&
      config.maximum_observation_gap_sec > 0.0 &&
      std::isfinite(config.history_window_sec) &&
      config.history_window_sec >= config.maximum_observation_gap_sec &&
      std::isfinite(config.stationary_settle_delay_sec) &&
      config.stationary_settle_delay_sec >= 0.0 &&
      std::isfinite(config.measurement_filter_alpha) &&
      config.measurement_filter_alpha > 0.0F &&
      config.measurement_filter_alpha <= 1.0F &&
      std::isfinite(config.normal_angle_deg) &&
      std::isfinite(config.warning_angle_deg) &&
      std::isfinite(config.alarm_angle_deg) &&
      std::isfinite(config.immediate_alarm_angle_deg) &&
      config.normal_angle_deg >= 0.0F &&
      config.warning_angle_deg >= config.normal_angle_deg &&
      config.alarm_angle_deg >= config.warning_angle_deg &&
      config.immediate_alarm_angle_deg >= config.alarm_angle_deg &&
      std::isfinite(config.warning_offset_m) &&
      std::isfinite(config.alarm_offset_m) &&
      std::isfinite(config.immediate_alarm_offset_m) &&
      config.warning_offset_m >= 0.0F &&
      config.alarm_offset_m >= config.warning_offset_m &&
      config.immediate_alarm_offset_m >= config.alarm_offset_m &&
      std::isfinite(config.normal_speed_mps) &&
      std::isfinite(config.warning_speed_mps) &&
      config.normal_speed_mps >= 0.0F &&
      config.warning_speed_mps >= config.normal_speed_mps &&
      std::isfinite(config.sway_end_angle_deg) &&
      std::isfinite(config.sway_end_speed_mps) &&
      config.sway_end_angle_deg >= 0.0F &&
      config.sway_end_speed_mps >= 0.0F &&
      std::isfinite(config.sway_end_confirm_sec) &&
      config.sway_end_confirm_sec >= 0.0 &&
      std::isfinite(config.minimum_alarm_hold_sec) &&
      config.minimum_alarm_hold_sec >= 0.0 &&
      std::isfinite(config.maximum_alarm_evidence_hold_sec) &&
      config.maximum_alarm_evidence_hold_sec >= 0.0 &&
      std::isfinite(config.sway_end_offset_m) &&
      config.sway_end_offset_m >= 0.0F &&
      std::isfinite(config.skew_suspect_angle_deg) &&
      std::isfinite(config.skew_alarm_angle_deg) &&
      config.skew_suspect_angle_deg >= 0.0F &&
      config.skew_alarm_angle_deg >= config.skew_suspect_angle_deg &&
      std::isfinite(config.skew_suspect_offset_m) &&
      std::isfinite(config.skew_alarm_offset_m) &&
      std::isfinite(config.skew_immediate_offset_m) &&
      config.skew_suspect_offset_m >= 0.0F &&
      config.skew_alarm_offset_m >= config.skew_suspect_offset_m &&
      config.skew_immediate_offset_m >= config.skew_alarm_offset_m &&
      std::isfinite(config.skew_direction_consistency) &&
      config.skew_direction_consistency >= 0.0F &&
      config.skew_direction_consistency <= 1.0F &&
      std::isfinite(config.skew_min_dc_to_ac_ratio) &&
      config.skew_min_dc_to_ac_ratio >= 0.0F &&
      std::isfinite(config.skew_min_duration_sec) &&
      config.skew_min_duration_sec >= 0.0 &&
      std::isfinite(config.skew_alarm_confirm_sec) &&
      config.skew_alarm_confirm_sec >= 0.0 &&
      config.skew_max_zero_crossings >= 0 &&
      std::isfinite(config.crossing_deadband_m) &&
      config.crossing_deadband_m > 0.0F &&
      std::isfinite(config.torsion_detect_deg) &&
      std::isfinite(config.torsion_warning_deg) &&
      std::isfinite(config.torsion_alarm_deg) &&
      config.torsion_detect_deg >= 0.0F &&
      config.torsion_warning_deg >= config.torsion_detect_deg &&
      config.torsion_alarm_deg >= config.torsion_warning_deg &&
      std::isfinite(config.minimum_identity_confidence) &&
      config.minimum_identity_confidence >= 0.0F &&
      config.minimum_identity_confidence <= 1.0F &&
      std::isfinite(config.minimum_shape_confidence) &&
      config.minimum_shape_confidence >= 0.0F &&
      config.minimum_shape_confidence <= 1.0F &&
      std::isfinite(config.maximum_tracking_residual_m) &&
      config.maximum_tracking_residual_m >= 0.0F &&
      std::isfinite(config.minimum_orientation_confidence) &&
      config.minimum_orientation_confidence >= 0.0F &&
      config.minimum_orientation_confidence <= 1.0F &&
      std::isfinite(config.minimum_torsion_aspect_ratio) &&
      config.minimum_torsion_aspect_ratio > 1.0F;
}

float percentile(std::vector<float> values, float q) {
  if (values.empty()) return 0.0F;
  std::sort(values.begin(), values.end());
  const float position = std::clamp(q, 0.0F, 1.0F) *
      static_cast<float>(values.size() - 1U);
  const std::size_t low = static_cast<std::size_t>(std::floor(position));
  const std::size_t high = static_cast<std::size_t>(std::ceil(position));
  const float fraction = position - static_cast<float>(low);
  return values[low] * (1.0F - fraction) + values[high] * fraction;
}

bool severeSway(CargoSwayState state) {
  return state == CargoSwayState::SWAY_WARNING ||
      state == CargoSwayState::SWAY_ALARM ||
      state == CargoSwayState::SETTLING;
}

}  // namespace

float shortestAxialAngle(float lhs_rad, float rhs_rad) {
  return normalizeAxialYaw(lhs_rad - rhs_rad);
}

CargoSwingMonitor::CargoSwingMonitor(const CargoSwingConfig& config) {
  setConfig(config);
}

void CargoSwingMonitor::setConfig(const CargoSwingConfig& config) {
  config_ = validConfig(config) ? config : CargoSwingConfig{};
  reset();
}

void CargoSwingMonitor::reset() {
  result_ = CargoSwingResult{};
  history_.clear();
  cargo_lifecycle_id_ = 0U;
  track_segment_id_ = 0U;
  hook_anchor_source_.clear();
  last_input_stamp_sec_ = 0.0;
  last_measurement_stamp_sec_ = 0.0;
  sway_state_change_stamp_sec_ = 0.0;
  skew_state_change_stamp_sec_ = 0.0;
  torsion_state_change_stamp_sec_ = 0.0;
  stationary_enter_stamp_sec_ = 0.0;
  sway_below_end_stamp_sec_ = 0.0;
  sway_alarm_enter_stamp_sec_ = 0.0;
  last_severe_measurement_stamp_sec_ = 0.0;
  settling_enter_stamp_sec_ = 0.0;
  skew_alarm_candidate_stamp_sec_ = 0.0;
  immediate_alarm_latched_ = false;
  settling_active_ = false;
  filtered_offset_valid_ = false;
  filtered_offset_.setZero();
  previous_crane_motion_state_ = CargoPhysicalMotionState::UNKNOWN;
}

CargoSwingResult CargoSwingMonitor::update(const CargoSwingInput& input) {
  if (!config_.enabled) {
    reset();
    result_.reason = "disabled";
    return result_;
  }
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0) {
    result_.valid = false;
    result_.observation_state = CargoSwingObservationState::INVALID;
    result_.reason = "invalid_timestamp";
    return result_;
  }
  if (last_input_stamp_sec_ > 0.0 &&
      input.stamp_sec <= last_input_stamp_sec_) {
    reset();
    last_input_stamp_sec_ = input.stamp_sec;
    result_.observation_state =
        CargoSwingObservationState::TIMESTAMP_ROLLBACK;
    result_.reason = "timestamp_rollback_reset";
    return result_;
  }
  last_input_stamp_sec_ = input.stamp_sec;
  if (!input.localization_valid || !input.hook_loaded ||
      !input.hook_anchor_valid || !input.hook_anchor_base.allFinite() ||
      input.cargo_lifecycle_id == 0U) {
    reset();
    last_input_stamp_sec_ = input.stamp_sec;
    result_.reason = !input.localization_valid
        ? "localization_invalid"
        : (!input.hook_loaded ? "hook_not_loaded"
                              : "hook_anchor_or_lifecycle_invalid");
    return result_;
  }

  const bool identity_changed = cargo_lifecycle_id_ != 0U &&
      (cargo_lifecycle_id_ != input.cargo_lifecycle_id ||
       track_segment_id_ != input.track_segment_id ||
       hook_anchor_source_ != input.hook_anchor_source);
  if (identity_changed) {
    reset();
    cargo_lifecycle_id_ = input.cargo_lifecycle_id;
    track_segment_id_ = input.track_segment_id;
    hook_anchor_source_ = input.hook_anchor_source;
    last_input_stamp_sec_ = input.stamp_sec;
    result_.observation_state = CargoSwingObservationState::TRACK_CHANGED;
    result_.reason = "track_lifecycle_or_anchor_changed_reset";
    return result_;
  }
  cargo_lifecycle_id_ = input.cargo_lifecycle_id;
  track_segment_id_ = input.track_segment_id;
  hook_anchor_source_ = input.hook_anchor_source;

  if (input.crane_motion_state != previous_crane_motion_state_) {
    if (input.crane_motion_state == CargoPhysicalMotionState::STATIONARY) {
      stationary_enter_stamp_sec_ = input.stamp_sec;
      // Stationary oscillation evidence must start after the transition,
      // never inherit travel-phase samples or velocity.
      history_.clear();
      last_measurement_stamp_sec_ = 0.0;
      filtered_offset_valid_ = false;
      filtered_offset_.setZero();
    } else if (input.crane_motion_state ==
               CargoPhysicalMotionState::MOVING) {
      stationary_enter_stamp_sec_ = 0.0;
    }
    previous_crane_motion_state_ = input.crane_motion_state;
  }

  const bool current_measurement = input.track_retained &&
      input.track_locked && input.observation_associated_current &&
      input.measured_center_valid && input.measured_center_base.allFinite();
  if (!current_measurement) {
    result_.observation_age_sec = last_measurement_stamp_sec_ > 0.0
        ? static_cast<float>(input.stamp_sec - last_measurement_stamp_sec_)
        : std::numeric_limits<float>::infinity();
    const double hold_limit =
        (immediate_alarm_latched_ || settling_active_ ||
         severeSway(result_.sway_state))
            ? config_.maximum_alarm_evidence_hold_sec
            : config_.maximum_observation_gap_sec;
    result_.valid = last_measurement_stamp_sec_ > 0.0 &&
        result_.observation_age_sec <= hold_limit;
    result_.observation_state = result_.valid
        ? CargoSwingObservationState::SHORT_GAP_HOLD
        : CargoSwingObservationState::STALE;
    result_.reason = result_.valid
        ? "short_gap_hold_no_evidence_accumulation"
        : "swing_evidence_stale";
    // LOST_HOLD and any unassociated observation deliberately leave history,
    // peaks, zero crossings and durations untouched.
    return result_;
  }

  const bool tracking_quality_valid =
      std::isfinite(input.identity_confidence) &&
      input.identity_confidence >= config_.minimum_identity_confidence &&
      std::isfinite(input.shape_confidence) &&
      input.shape_confidence >= config_.minimum_shape_confidence &&
      std::isfinite(input.horizontal_tracking_residual_m) &&
      input.horizontal_tracking_residual_m >= 0.0F &&
      input.horizontal_tracking_residual_m <=
          config_.maximum_tracking_residual_m;
  if (!tracking_quality_valid) {
    result_.valid = false;
    result_.observation_state = CargoSwingObservationState::INVALID;
    result_.observation_age_sec = last_measurement_stamp_sec_ > 0.0
        ? static_cast<float>(input.stamp_sec - last_measurement_stamp_sec_)
        : std::numeric_limits<float>::infinity();
    result_.reason = "tracking_quality_insufficient";
    return result_;
  }

  const bool measurement_gap_reset = last_measurement_stamp_sec_ > 0.0 &&
      input.stamp_sec - last_measurement_stamp_sec_ >
          config_.maximum_observation_gap_sec;
  if (measurement_gap_reset) {
    history_.clear();
    last_measurement_stamp_sec_ = 0.0;
    sway_state_change_stamp_sec_ = input.stamp_sec;
    skew_state_change_stamp_sec_ = input.stamp_sec;
    torsion_state_change_stamp_sec_ = input.stamp_sec;
    sway_below_end_stamp_sec_ = 0.0;
    skew_alarm_candidate_stamp_sec_ = 0.0;
    filtered_offset_valid_ = false;
    filtered_offset_.setZero();
    // A long observation gap invalidates kinematic history, but it must not
    // silently clear a previously latched safety alarm.
  }

  const Eigen::Vector2f raw_offset =
      input.measured_center_base.head<2>() -
      input.hook_anchor_base.head<2>();
  if (!filtered_offset_valid_) {
    filtered_offset_ = raw_offset;
    filtered_offset_valid_ = true;
  } else {
    filtered_offset_ = (1.0F - config_.measurement_filter_alpha) *
        filtered_offset_ + config_.measurement_filter_alpha * raw_offset;
  }
  const double dt = last_measurement_stamp_sec_ > 0.0
      ? input.stamp_sec - last_measurement_stamp_sec_ : 0.0;
  const Eigen::Vector2f previous_offset = history_.empty()
      ? filtered_offset_ : history_.back().offset;
  last_measurement_stamp_sec_ = input.stamp_sec;

  const float measured_rope_length =
      input.hook_anchor_base.z() - input.measured_center_base.z();
  CargoRopeLengthSource rope_source = CargoRopeLengthSource::MEASURED;
  float rope_length = measured_rope_length;
  if (!std::isfinite(rope_length) ||
      rope_length < config_.minimum_rope_length_m) {
    rope_length = config_.configured_sling_length_m;
    rope_source = CargoRopeLengthSource::CONFIG_FALLBACK;
  }
  const bool angle_authoritative =
      rope_source == CargoRopeLengthSource::MEASURED;
  const float offset_m = raw_offset.norm();
  const float angle_deg = std::atan2(offset_m, rope_length) * kRadToDeg;
  float horizontal_speed = 0.0F;
  float radial_speed = 0.0F;
  if (dt > 1.0e-4) {
    const Eigen::Vector2f velocity =
        (filtered_offset_ - previous_offset) / static_cast<float>(dt);
    horizontal_speed = velocity.norm();
    const float filtered_norm = filtered_offset_.norm();
    if (filtered_norm > 1.0e-5F) {
      radial_speed = velocity.dot(filtered_offset_ / filtered_norm);
    }
  }
  float yaw_error_deg = 0.0F;
  const bool yaw_evidence_valid =
      input.locked_yaw_valid && input.measured_yaw_valid &&
      std::isfinite(input.locked_yaw_base_rad) &&
      std::isfinite(input.measured_yaw_base_rad) &&
      std::isfinite(input.orientation_confidence) &&
      input.orientation_confidence >=
          config_.minimum_orientation_confidence &&
      input.shape_confidence >= config_.minimum_shape_confidence &&
      std::isfinite(input.cargo_length_m) && input.cargo_length_m > 0.0F &&
      std::isfinite(input.cargo_width_m) && input.cargo_width_m > 0.0F &&
      std::max(input.cargo_length_m, input.cargo_width_m) /
              std::min(input.cargo_length_m, input.cargo_width_m) >=
          config_.minimum_torsion_aspect_ratio;
  if (yaw_evidence_valid) {
    yaw_error_deg = std::abs(shortestAxialAngle(
        input.measured_yaw_base_rad,
        input.locked_yaw_base_rad)) * kRadToDeg;
  }
  history_.push_back(Sample{
      input.stamp_sec, filtered_offset_, angle_deg, yaw_error_deg});
  while (!history_.empty() &&
         input.stamp_sec - history_.front().stamp_sec >
             config_.history_window_sec) {
    history_.pop_front();
  }

  Eigen::Vector2f mean_offset = Eigen::Vector2f::Zero();
  float mean_norm = 0.0F;
  for (const Sample& sample : history_) {
    mean_offset += sample.offset;
    mean_norm += sample.offset.norm();
  }
  mean_offset /= static_cast<float>(history_.size());
  mean_norm /= static_cast<float>(history_.size());
  Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
  for (const Sample& sample : history_) {
    const Eigen::Vector2f detrended = sample.offset - mean_offset;
    covariance.noalias() += detrended * detrended.transpose();
  }
  covariance /= static_cast<float>(history_.size());
  Eigen::Vector2f principal_axis = Eigen::Vector2f::UnitX();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
  if (solver.info() == Eigen::Success &&
      solver.eigenvectors().allFinite()) {
    principal_axis = solver.eigenvectors().col(1).normalized();
  }
  std::vector<float> projections;
  projections.reserve(history_.size());
  int oscillation_zero_crossings = 0;
  int previous_sign = 0;
  for (const Sample& sample : history_) {
    const float projection =
        (sample.offset - mean_offset).dot(principal_axis);
    projections.push_back(projection);
    const int sign = projection > config_.crossing_deadband_m
        ? 1 : (projection < -config_.crossing_deadband_m ? -1 : 0);
    if (sign != 0) {
      if (previous_sign != 0 && sign != previous_sign) {
        ++oscillation_zero_crossings;
      }
      previous_sign = sign;
    }
  }
  const float oscillation_amplitude =
      percentile(projections, 0.95F) - percentile(projections, 0.05F);
  const float direction_consistency = mean_norm > 1.0e-5F
      ? std::clamp(mean_offset.norm() / mean_norm, 0.0F, 1.0F)
      : 0.0F;
  float ac_square_sum = 0.0F;
  for (const Sample& sample : history_) {
    ac_square_sum += (sample.offset - mean_offset).squaredNorm();
  }
  const float dc_offset_m = mean_offset.norm();
  const float ac_rms_m = std::sqrt(
      ac_square_sum / static_cast<float>(history_.size()));
  const float dc_to_ac_ratio = dc_offset_m /
      std::max(ac_rms_m, 0.01F);
  const float oscillation_confidence = std::clamp(
      0.5F * ac_rms_m / std::max(config_.warning_offset_m, 0.01F) +
          0.25F * static_cast<float>(oscillation_zero_crossings),
      0.0F, 1.0F);
  const float skew_confidence = std::clamp(
      direction_consistency *
          (dc_to_ac_ratio / std::max(config_.skew_min_dc_to_ac_ratio, 0.1F)),
      0.0F, 1.0F);
  const double history_duration = history_.size() > 1U
      ? history_.back().stamp_sec - history_.front().stamp_sec : 0.0;
  const Eigen::Vector2f skew_axis = mean_offset.norm() > 1.0e-5F
      ? mean_offset.normalized() : principal_axis;
  int zero_crossings = 0;
  previous_sign = 0;
  for (const Sample& sample : history_) {
    const float projection = sample.offset.dot(skew_axis);
    const int sign = projection > config_.crossing_deadband_m
        ? 1 : (projection < -config_.crossing_deadband_m ? -1 : 0);
    if (sign != 0) {
      if (previous_sign != 0 && sign != previous_sign) ++zero_crossings;
      previous_sign = sign;
    }
  }
  const double stationary_duration =
      input.crane_motion_state == CargoPhysicalMotionState::STATIONARY &&
          stationary_enter_stamp_sec_ > 0.0
      ? std::max(0.0, input.stamp_sec - stationary_enter_stamp_sec_)
      : 0.0;

  CargoSwayState sway = CargoSwayState::NOT_EVALUATED;
  CargoSkewPullState skew = CargoSkewPullState::NOT_EVALUATED;
  CargoTorsionState torsion = CargoTorsionState::NOT_EVALUATED;
  float mean_angle_deg = 0.0F;
  bool skew_alarm_authority_missing = false;
  const bool hoist_up = input.hoist_state_fresh &&
      input.hoist_motion_state == HoistMotionState::UP;
  const bool hoist_alarm_authorized = hoist_up ||
      config_.allow_skew_alarm_without_hoist_up;
  const bool immediate_skew_offset =
      offset_m >= config_.skew_immediate_offset_m;
  if (immediate_skew_offset) {
    skew = hoist_alarm_authorized
        ? CargoSkewPullState::SKEW_PULL_ALARM
        : CargoSkewPullState::SKEW_PULL_SUSPECTED;
    skew_alarm_authority_missing = !hoist_alarm_authorized;
  }
  const bool immediate_sway_alarm =
      offset_m >= config_.immediate_alarm_offset_m ||
      (angle_authoritative &&
       angle_deg >= config_.immediate_alarm_angle_deg);
  if (immediate_sway_alarm) {
    immediate_alarm_latched_ = true;
    settling_active_ = false;
    sway_alarm_enter_stamp_sec_ = input.stamp_sec;
    last_severe_measurement_stamp_sec_ = input.stamp_sec;
    sway_below_end_stamp_sec_ = 0.0;
  }
  const bool formal_alarm_measurement =
      offset_m >= config_.alarm_offset_m ||
      (angle_authoritative && angle_deg >= config_.alarm_angle_deg);
  if (formal_alarm_measurement) {
    last_severe_measurement_stamp_sec_ = input.stamp_sec;
  }
  const bool angle_below_end = !angle_authoritative ||
      angle_deg <= config_.sway_end_angle_deg;
  const bool below_end = angle_below_end &&
      offset_m <= config_.sway_end_offset_m &&
      horizontal_speed <= config_.sway_end_speed_mps;
  if (immediate_alarm_latched_) {
    const bool minimum_hold_complete = sway_alarm_enter_stamp_sec_ > 0.0 &&
        input.stamp_sec - sway_alarm_enter_stamp_sec_ >=
            config_.minimum_alarm_hold_sec;
    if (minimum_hold_complete && below_end) {
      if (sway_below_end_stamp_sec_ <= 0.0) {
        sway_below_end_stamp_sec_ = input.stamp_sec;
      }
      if (input.stamp_sec - sway_below_end_stamp_sec_ >=
          config_.sway_end_confirm_sec) {
        immediate_alarm_latched_ = false;
        settling_active_ = true;
        settling_enter_stamp_sec_ = input.stamp_sec;
        sway = CargoSwayState::SETTLING;
      } else {
        sway = CargoSwayState::SWAY_ALARM;
      }
    } else {
      sway = CargoSwayState::SWAY_ALARM;
      sway_below_end_stamp_sec_ = 0.0;
    }
  } else if (settling_active_) {
    if (formal_alarm_measurement) {
      immediate_alarm_latched_ = true;
      settling_active_ = false;
      sway_alarm_enter_stamp_sec_ = input.stamp_sec;
      sway = CargoSwayState::SWAY_ALARM;
    } else if (!below_end) {
      sway = CargoSwayState::SETTLING;
      settling_enter_stamp_sec_ = input.stamp_sec;
    } else if (input.stamp_sec - settling_enter_stamp_sec_ >=
               config_.sway_end_confirm_sec) {
      settling_active_ = false;
      sway = CargoSwayState::NORMAL;
    } else {
      sway = CargoSwayState::SETTLING;
    }
  } else if (history_duration >= config_.minimum_valid_observation_sec) {
    if (formal_alarm_measurement) {
      sway = CargoSwayState::SWAY_ALARM;
    } else if ((angle_authoritative &&
                angle_deg >= config_.warning_angle_deg) ||
               offset_m >= config_.warning_offset_m ||
               horizontal_speed >= config_.warning_speed_mps ||
               (input.crane_motion_state ==
                    CargoPhysicalMotionState::STATIONARY &&
                stationary_duration >= config_.stationary_settle_delay_sec &&
                oscillation_zero_crossings >= 1 &&
                oscillation_amplitude >= config_.warning_offset_m)) {
      sway = CargoSwayState::SWAY_WARNING;
    } else if ((angle_authoritative &&
                angle_deg >= config_.normal_angle_deg) ||
               horizontal_speed >= config_.normal_speed_mps ||
               oscillation_amplitude >= 0.5F * config_.warning_offset_m) {
      sway = CargoSwayState::SWAY_DETECTED;
    } else {
      sway = CargoSwayState::NORMAL;
    }

    mean_angle_deg = std::atan2(
        mean_offset.norm(), rope_length) * kRadToDeg;
    const bool skew_angle_suspect = angle_authoritative &&
        mean_angle_deg >= config_.skew_suspect_angle_deg;
    const bool skew_offset_suspect =
        dc_offset_m >= config_.skew_suspect_offset_m;
    const bool skew_suspected =
        (skew_angle_suspect || skew_offset_suspect) &&
        direction_consistency >= config_.skew_direction_consistency &&
        dc_to_ac_ratio >= config_.skew_min_dc_to_ac_ratio &&
        zero_crossings <= config_.skew_max_zero_crossings &&
        history_duration >= config_.skew_min_duration_sec;
    skew = skew_suspected ? CargoSkewPullState::SKEW_PULL_SUSPECTED
                          : CargoSkewPullState::NO_SKEW_PULL;
    // Configured rope length remains diagnostic-only for every formal angle
    // threshold, including skew pull. Absolute LiDAR offset is the independent
    // safety channel when measured rope length is unavailable.
    const bool skew_angle_alarm = angle_authoritative &&
        mean_angle_deg >= config_.skew_alarm_angle_deg;
    const bool skew_offset_alarm =
        dc_offset_m >= config_.skew_alarm_offset_m;
    const bool skew_immediate_alarm = immediate_skew_offset;
    const bool skew_alarm_evidence = skew_suspected &&
        (skew_angle_alarm || skew_offset_alarm);
    skew_alarm_authority_missing =
        (skew_alarm_evidence || skew_immediate_alarm) &&
        !hoist_alarm_authorized;
    if (skew_immediate_alarm && hoist_alarm_authorized) {
      skew = CargoSkewPullState::SKEW_PULL_ALARM;
      skew_alarm_candidate_stamp_sec_ = input.stamp_sec;
    } else if (skew_alarm_evidence && hoist_alarm_authorized) {
      if (skew_alarm_candidate_stamp_sec_ <= 0.0) {
        skew_alarm_candidate_stamp_sec_ = input.stamp_sec;
      }
      if (input.stamp_sec - skew_alarm_candidate_stamp_sec_ >=
          config_.skew_alarm_confirm_sec) {
        skew = CargoSkewPullState::SKEW_PULL_ALARM;
      }
    } else {
      skew_alarm_candidate_stamp_sec_ = 0.0;
    }

    if (yaw_evidence_valid) {
      if (yaw_error_deg >= config_.torsion_alarm_deg) {
        torsion = CargoTorsionState::TORSION_ALARM;
      } else if (yaw_error_deg >= config_.torsion_warning_deg) {
        torsion = CargoTorsionState::TORSION_WARNING;
      } else if (yaw_error_deg >= config_.torsion_detect_deg) {
        torsion = CargoTorsionState::TORSION_DETECTED;
      } else {
        torsion = CargoTorsionState::NORMAL;
      }
    }
  } else if (severeSway(result_.sway_state)) {
    // An immature replacement window is not evidence that a previously
    // established safety state disappeared.
    sway = result_.sway_state;
  }

  if (sway != result_.sway_state || sway_state_change_stamp_sec_ <= 0.0) {
    sway_state_change_stamp_sec_ = input.stamp_sec;
  }
  if (skew != result_.skew_pull_state || skew_state_change_stamp_sec_ <= 0.0) {
    skew_state_change_stamp_sec_ = input.stamp_sec;
  }
  if (torsion != result_.torsion_state ||
      torsion_state_change_stamp_sec_ <= 0.0) {
    torsion_state_change_stamp_sec_ = input.stamp_sec;
  }
  result_.valid = true;
  result_.observation_state =
      CargoSwingObservationState::CURRENT_MEASUREMENT;
  result_.sway_state = sway;
  result_.skew_pull_state = skew;
  result_.torsion_state = torsion;
  result_.offset_xy_m = raw_offset;
  result_.offset_m = offset_m;
  result_.rope_length_m = rope_length;
  result_.rope_length_source = rope_source;
  result_.angle_authoritative = angle_authoritative;
  result_.angle_deg = angle_deg;
  result_.horizontal_speed_mps = horizontal_speed;
  result_.radial_speed_mps = radial_speed;
  result_.oscillation_amplitude_m = oscillation_amplitude;
  result_.direction_consistency = direction_consistency;
  result_.dc_offset_m = dc_offset_m;
  result_.ac_rms_m = ac_rms_m;
  result_.dc_to_ac_ratio = dc_to_ac_ratio;
  result_.oscillation_confidence = oscillation_confidence;
  result_.skew_confidence = skew_confidence;
  result_.zero_crossings = zero_crossings;
  result_.yaw_error_deg = yaw_error_deg;
  result_.observation_age_sec = 0.0F;
  result_.sway_state_duration_sec = static_cast<float>(std::max(
      0.0, input.stamp_sec - sway_state_change_stamp_sec_));
  result_.skew_state_duration_sec = static_cast<float>(std::max(
      0.0, input.stamp_sec - skew_state_change_stamp_sec_));
  result_.torsion_state_duration_sec = static_cast<float>(std::max(
      0.0, input.stamp_sec - torsion_state_change_stamp_sec_));
  result_.state_duration_sec = std::max(
      result_.sway_state_duration_sec,
      std::max(result_.skew_state_duration_sec,
               result_.torsion_state_duration_sec));
  result_.hoist_up_confirmed = input.hoist_state_fresh &&
      input.hoist_motion_state == HoistMotionState::UP;
  result_.alarm_inhibited = skew_alarm_authority_missing;
  result_.recommended_action = CargoSwingRecommendedAction::NONE;
  if (sway == CargoSwayState::SWAY_ALARM ||
      torsion == CargoTorsionState::TORSION_ALARM) {
    result_.recommended_action = CargoSwingRecommendedAction::STOP_AND_SETTLE;
  } else if (skew == CargoSkewPullState::SKEW_PULL_ALARM) {
    result_.recommended_action = CargoSwingRecommendedAction::STOP_TRAVEL;
  } else if (result_.alarm_inhibited) {
    result_.recommended_action =
        CargoSwingRecommendedAction::INHIBIT_HOIST_UP;
  } else if (sway == CargoSwayState::SWAY_WARNING ||
             torsion == CargoTorsionState::TORSION_WARNING) {
    result_.recommended_action =
        CargoSwingRecommendedAction::REDUCE_TRAVEL_SPEED;
  } else if (sway == CargoSwayState::SWAY_DETECTED ||
             skew == CargoSkewPullState::SKEW_PULL_SUSPECTED ||
             torsion == CargoTorsionState::TORSION_DETECTED) {
    result_.recommended_action = CargoSwingRecommendedAction::WATCH;
  }
  result_.reason = result_.alarm_inhibited
      ? "hoist_up_evidence_unavailable"
      : (measurement_gap_reset
             ? "current_measurement_after_gap_reset"
             : (rope_source == CargoRopeLengthSource::CONFIG_FALLBACK
             ? "current_measurement_config_rope_fallback"
             : "current_measurement"));
  return result_;
}

}  // namespace ndt_slam
