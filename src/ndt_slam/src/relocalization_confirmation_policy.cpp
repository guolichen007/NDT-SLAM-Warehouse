#include "ndt_slam/relocalization_confirmation_policy.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {
namespace {

bool poseFinite(const Sophus::SE3d& pose) {
  return pose.translation().allFinite() &&
      pose.so3().matrix().allFinite();
}

double yawOf(const Sophus::SE3d& pose) {
  const Eigen::Matrix3d rotation = pose.so3().matrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

bool validConfig(const RelocalizationConfirmationConfig& config) {
  return config.required_confirmations >= 2 &&
      config.maximum_age_frames > 0U &&
      std::isfinite(config.maximum_age_sec) &&
      config.maximum_age_sec > 0.0 &&
      std::isfinite(config.maximum_translation_delta_m) &&
      config.maximum_translation_delta_m > 0.0 &&
      std::isfinite(config.maximum_yaw_delta_deg) &&
      config.maximum_yaw_delta_deg > 0.0;
}

}  // namespace

RelocalizationConfirmationDecision evaluateRelocalizationConfirmation(
    const RelocalizationConfirmationInput& input,
    const RelocalizationConfirmationConfig& config) {
  RelocalizationConfirmationDecision decision;
  if (!validConfig(config) ||
      !std::isfinite(input.current_stamp_sec) ||
      input.current_stamp_sec <= 0.0) {
    decision.reason = "invalid_confirmation_policy_input";
    return decision;
  }

  const RelocalizationResult& result = input.result;
  if (result.map_generation != input.expected_map_generation ||
      result.pose_version != input.expected_pose_version) {
    decision.outcome =
        RelocalizationConfirmationOutcome::DISCARD_IDENTITY;
    decision.reason = "stale_map_or_pose_generation_discarded";
    return decision;
  }
  if (result.frame_index <= input.last_result_frame) {
    decision.outcome =
        RelocalizationConfirmationOutcome::DISCARD_DUPLICATE;
    decision.reason = "duplicate_or_out_of_order_result";
    return decision;
  }
  decision.update_last_result_frame = true;
  decision.last_result_frame = result.frame_index;

  const double result_age_sec =
      input.current_stamp_sec - result.stamp_sec;
  const bool future_frame = result.frame_index > input.current_frame_index;
  const bool frame_stale = !future_frame &&
      input.current_frame_index - result.frame_index >
          config.maximum_age_frames;
  if (future_frame || frame_stale ||
      !std::isfinite(result_age_sec) || result_age_sec < -0.05 ||
      result_age_sec > config.maximum_age_sec) {
    decision.outcome =
        RelocalizationConfirmationOutcome::DISCARD_STALE;
    decision.reason = "stale_result_discarded";
    return decision;
  }
  if (!result.valid) {
    decision.outcome =
        RelocalizationConfirmationOutcome::DISCARD_INVALID;
    decision.reason = result.reason.empty()
        ? "relocalization_candidate_invalid" : result.reason;
    return decision;
  }
  if (!std::isfinite(result.fitness) ||
      !std::isfinite(result.probability) ||
      !poseFinite(result.pose) || !poseFinite(result.reference_pose)) {
    decision.outcome =
        RelocalizationConfirmationOutcome::DISCARD_INVALID;
    decision.reason = "nonfinite_relocalization_candidate";
    return decision;
  }

  const Sophus::SE3d correction =
      result.pose * result.reference_pose.inverse();
  if (!poseFinite(correction)) {
    decision.outcome =
        RelocalizationConfirmationOutcome::DISCARD_INVALID;
    decision.reason = "nonfinite_relocalization_correction";
    return decision;
  }
  decision.correction = correction;

  bool consistent = false;
  if (input.previous_confirmation_count > 0 &&
      poseFinite(input.previous_correction)) {
    const double translation =
        (correction.translation().head<2>() -
         input.previous_correction.translation().head<2>()).norm();
    const double yaw_delta_deg = std::abs(std::atan2(
        std::sin(yawOf(correction) - yawOf(input.previous_correction)),
        std::cos(yawOf(correction) - yawOf(input.previous_correction)))) *
        180.0 / M_PI;
    consistent =
        translation <= config.maximum_translation_delta_m &&
        yaw_delta_deg <= config.maximum_yaw_delta_deg;
  }
  decision.confirmation_count = consistent
      ? input.previous_confirmation_count + 1 : 1;
  decision.outcome =
      decision.confirmation_count >= config.required_confirmations
      ? RelocalizationConfirmationOutcome::CONFIRMED
      : RelocalizationConfirmationOutcome::CONFIRMING;
  decision.reason = decision.outcome ==
          RelocalizationConfirmationOutcome::CONFIRMED
      ? "relocalization_candidate_confirmed"
      : (consistent
             ? "relocalization_candidate_consistent"
             : "relocalization_confirmation_restarted");
  return decision;
}

}  // namespace ndt_slam
