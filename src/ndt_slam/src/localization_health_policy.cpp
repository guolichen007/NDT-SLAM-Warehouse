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
    config_.minimum_qualified_frames = std::clamp(
        config_.minimum_qualified_frames, 1,
        config_.required_consecutive_frames);
    config_.maximum_consecutive_failures = std::clamp(
        config_.maximum_consecutive_failures, 0,
        config_.required_consecutive_frames - 1);
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
    verified_latched_ = false;
    qualification_window_.clear();
}

LocalizationHealthDecision LocalizationHealthPolicy::update(
    const LocalizationHealthEvidence& evidence) {
    bool temporal_hard_failure = false;
    auto hard_reject = [this, &temporal_hard_failure](const char* reason) {
        temporal_hard_failure = true;
        decision_.frame_qualified = false;
        decision_.localization_verified = false;
        decision_.consecutive_qualified_frames = 0;
        decision_.evaluated_window_frames = 0;
        decision_.consecutive_failed_frames = 0;
        decision_.reason = reason;
        verified_latched_ = false;
        qualification_window_.clear();
    };

    const char* rejection_reason = nullptr;

    if (!std::isfinite(evidence.stamp_sec) || evidence.stamp_sec <= 0.0) {
        hard_reject("invalid_stamp");
    } else if (have_previous_stamp_ &&
               evidence.stamp_sec <= previous_stamp_sec_) {
        hard_reject("time_not_monotonic");
    } else if (have_previous_stamp_ &&
               evidence.stamp_sec - previous_stamp_sec_ >
                   config_.maximum_frame_gap_sec) {
        hard_reject("frame_gap");
    } else if (!evidence.ndt_converged) {
        rejection_reason = "ndt_not_converged";
    } else if (!evidence.ndt_accepted) {
        rejection_reason = "ndt_not_accepted";
    } else if (evidence.prediction_only) {
        rejection_reason = "prediction_only";
    } else if (!evidence.observability_valid) {
        rejection_reason = "observability_invalid";
    } else if (!evidence.pose_finite) {
        rejection_reason = "pose_nonfinite";
    } else if (!evidence.fixed_yaw_contract) {
        rejection_reason = "fixed_yaw_contract_failed";
    } else if (!evidence.candidate_basin_continuity) {
        rejection_reason = "candidate_basin_changed";
    } else if (evidence.nonphysical_correction) {
        rejection_reason = "nonphysical_correction";
    } else if (evidence.output_step_limited) {
        rejection_reason = "output_step_limited";
    } else if (evidence.ekf_recovered) {
        rejection_reason = "ekf_covariance_recovered";
    } else if (!std::isfinite(evidence.fitness) ||
               evidence.fitness > config_.maximum_fitness) {
        rejection_reason = "fitness_above_strict_limit";
    } else if (!std::isfinite(evidence.raw_step_m) ||
               !std::isfinite(evidence.maximum_allowed_step_m) ||
               evidence.maximum_allowed_step_m <= 0.0 ||
               evidence.raw_step_m > evidence.maximum_allowed_step_m) {
        rejection_reason = "raw_step_invalid";
    } else if (!std::isfinite(evidence.innovation_m) ||
               evidence.innovation_m > config_.maximum_innovation_m) {
        rejection_reason = "innovation_above_strict_limit";
    } else {
        decision_.frame_qualified = true;
    }

    if (!temporal_hard_failure) {
        decision_.frame_qualified = rejection_reason == nullptr;
        qualification_window_.push_back(decision_.frame_qualified);
        while (qualification_window_.size() >
               static_cast<std::size_t>(
                   config_.required_consecutive_frames)) {
            qualification_window_.pop_front();
        }
        const int qualified = static_cast<int>(std::count(
            qualification_window_.begin(), qualification_window_.end(),
            true));
        if (decision_.frame_qualified) {
            decision_.consecutive_failed_frames = 0;
        } else {
            ++decision_.consecutive_failed_frames;
        }
        decision_.consecutive_qualified_frames = qualified;
        decision_.evaluated_window_frames = static_cast<int>(
            qualification_window_.size());
        const bool window_complete = decision_.evaluated_window_frames >=
            config_.required_consecutive_frames;
        const bool window_healthy = window_complete &&
            qualified >= config_.minimum_qualified_frames &&
            decision_.consecutive_failed_frames <=
                config_.maximum_consecutive_failures;
        decision_.localization_verified = window_healthy &&
            (verified_latched_ || decision_.frame_qualified);
        verified_latched_ = decision_.localization_verified;
        if (decision_.localization_verified) {
            decision_.reason = decision_.frame_qualified
                ? "strict_window_verification_complete"
                : "strict_window_tolerating_transient_failure";
        } else if (rejection_reason != nullptr) {
            decision_.reason = rejection_reason;
        } else if (window_complete &&
                   qualified < config_.minimum_qualified_frames) {
            decision_.reason =
                "strict_window_qualified_frames_insufficient";
        } else {
            decision_.reason = "strict_window_collecting";
        }
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
