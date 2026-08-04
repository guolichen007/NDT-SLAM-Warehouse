#include "ndt_slam/localization_health_policy.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {

LocalizationHealthPolicy::LocalizationHealthPolicy(
    const LocalizationHealthConfig& config) {
    configure(config);
}

void LocalizationHealthPolicy::configure(
    const LocalizationHealthConfig& config) {
    config_ = config;
    config_.required_consecutive_frames =
        std::max(1, config_.required_consecutive_frames);
    config_.maximum_fitness = std::max(0.0, config_.maximum_fitness);
    config_.maximum_innovation_m =
        std::max(0.0, config_.maximum_innovation_m);
    config_.maximum_frame_gap_sec =
        std::max(0.01, config_.maximum_frame_gap_sec);
    reset("configured");
}

void LocalizationHealthPolicy::reset(const std::string& reason) {
    decision_ = LocalizationHealthDecision{};
    decision_.reason = reason;
    previous_stamp_sec_ = 0.0;
    have_previous_stamp_ = false;
}

LocalizationHealthDecision LocalizationHealthPolicy::update(
    const LocalizationHealthEvidence& evidence) {
    auto reject = [this](const char* reason) {
        decision_.frame_qualified = false;
        decision_.localization_verified = false;
        decision_.consecutive_qualified_frames = 0;
        decision_.reason = reason;
    };

    if (!std::isfinite(evidence.stamp_sec) || evidence.stamp_sec <= 0.0) {
        reject("invalid_stamp");
    } else if (have_previous_stamp_ &&
               evidence.stamp_sec <= previous_stamp_sec_) {
        reject("time_not_monotonic");
    } else if (have_previous_stamp_ &&
               evidence.stamp_sec - previous_stamp_sec_ >
                   config_.maximum_frame_gap_sec) {
        reject("frame_gap");
    } else if (!evidence.ndt_converged) {
        reject("ndt_not_converged");
    } else if (!evidence.ndt_accepted) {
        reject("ndt_not_accepted");
    } else if (evidence.prediction_only) {
        reject("prediction_only");
    } else if (!evidence.observability_valid) {
        reject("observability_invalid");
    } else if (!evidence.pose_finite) {
        reject("pose_nonfinite");
    } else if (evidence.nonphysical_correction) {
        reject("nonphysical_correction");
    } else if (evidence.output_step_limited) {
        reject("output_step_limited");
    } else if (evidence.ekf_recovered) {
        reject("ekf_covariance_recovered");
    } else if (!std::isfinite(evidence.fitness) ||
               evidence.fitness > config_.maximum_fitness) {
        reject("fitness_above_strict_limit");
    } else if (!std::isfinite(evidence.raw_step_m) ||
               !std::isfinite(evidence.maximum_allowed_step_m) ||
               evidence.maximum_allowed_step_m <= 0.0 ||
               evidence.raw_step_m > evidence.maximum_allowed_step_m) {
        reject("raw_step_invalid");
    } else if (!std::isfinite(evidence.innovation_m) ||
               evidence.innovation_m > config_.maximum_innovation_m) {
        reject("innovation_above_strict_limit");
    } else {
        decision_.frame_qualified = true;
        decision_.consecutive_qualified_frames = std::min(
            config_.required_consecutive_frames,
            decision_.consecutive_qualified_frames + 1);
        decision_.localization_verified =
            decision_.consecutive_qualified_frames >=
            config_.required_consecutive_frames;
        decision_.reason = decision_.localization_verified
            ? "strict_verification_complete"
            : "strict_frame_qualified";
    }

    previous_stamp_sec_ = evidence.stamp_sec;
    have_previous_stamp_ =
        std::isfinite(evidence.stamp_sec) && evidence.stamp_sec > 0.0;
    return decision_;
}

LocalizationHealthDecision LocalizationHealthPolicy::decision() const {
    return decision_;
}

}  // namespace ndt_slam
