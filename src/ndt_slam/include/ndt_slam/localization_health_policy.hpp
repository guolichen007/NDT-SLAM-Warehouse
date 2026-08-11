#pragma once

#include <cstdint>
#include <deque>
#include <string>

namespace ndt_slam {

struct LocalizationHealthConfig {
    int required_consecutive_frames = 8;
    int minimum_qualified_frames = 6;
    int maximum_consecutive_failures = 2;
    double maximum_fitness = 0.35;
    double maximum_innovation_m = 0.75;
    double maximum_frame_gap_sec = 0.50;
};

struct LocalizationHealthEvidence {
    double stamp_sec = 0.0;
    bool ndt_converged = false;
    bool ndt_accepted = false;
    bool prediction_only = true;
    bool observability_valid = false;
    bool pose_finite = false;
    bool nonphysical_correction = false;
    bool output_step_limited = false;
    bool ekf_recovered = false;
    bool fixed_yaw_contract = true;
    bool candidate_basin_continuity = true;
    double fitness = 0.0;
    double raw_step_m = 0.0;
    double maximum_allowed_step_m = 0.0;
    double innovation_m = 0.0;
};

struct LocalizationHealthDecision {
    bool frame_qualified = false;
    bool localization_verified = false;
    int consecutive_qualified_frames = 0;
    int evaluated_window_frames = 0;
    int consecutive_failed_frames = 0;
    std::string reason = "not_evaluated";
};

class LocalizationHealthPolicy {
public:
    explicit LocalizationHealthPolicy(
        const LocalizationHealthConfig& config = LocalizationHealthConfig{});

    void configure(const LocalizationHealthConfig& config);
    void reset(const std::string& reason = "reset");
    LocalizationHealthDecision update(
        const LocalizationHealthEvidence& evidence);
    LocalizationHealthDecision decision() const;

private:
    LocalizationHealthConfig config_;
    LocalizationHealthDecision decision_;
    double previous_stamp_sec_ = 0.0;
    bool have_previous_stamp_ = false;
    bool verified_latched_ = false;
    std::deque<bool> qualification_window_;
};

}  // namespace ndt_slam
