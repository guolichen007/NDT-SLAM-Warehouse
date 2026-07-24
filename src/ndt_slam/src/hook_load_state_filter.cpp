#include "ndt_slam/hook_load_state_filter.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {

HookLoadStateFilter::HookLoadStateFilter(const HookLoadStateConfig& config)
    : config_(config) {}

void HookLoadStateFilter::setConfig(const HookLoadStateConfig& config) {
    config_ = config;
    reset("config_changed");
}

bool HookLoadStateFilter::configValid() const {
    return std::isfinite(config_.low_threshold_v) &&
           std::isfinite(config_.high_threshold_v) &&
           config_.low_threshold_v < config_.high_threshold_v &&
           std::isfinite(config_.hysteresis_v) && config_.hysteresis_v >= 0.0 &&
           2.0 * config_.hysteresis_v <
               config_.high_threshold_v - config_.low_threshold_v &&
           config_.confirm_samples > 0U &&
           std::isfinite(config_.minimum_transition_duration_sec) &&
           config_.minimum_transition_duration_sec >= 0.0 &&
           std::isfinite(config_.stale_timeout_sec) &&
           config_.stale_timeout_sec > 0.0 &&
           std::isfinite(config_.valid_voltage_min_v) &&
           std::isfinite(config_.valid_voltage_max_v) &&
           config_.valid_voltage_min_v < config_.valid_voltage_max_v;
}

HookLoadState HookLoadStateFilter::classify(double voltage) const {
    const double low_enter = config_.low_threshold_v - config_.hysteresis_v;
    const double low_exit = config_.low_threshold_v + config_.hysteresis_v;
    const double high_exit = config_.high_threshold_v - config_.hysteresis_v;
    const double high_enter = config_.high_threshold_v + config_.hysteresis_v;

    if (stable_state_ == HookLoadState::UNKNOWN) {
        if (voltage < config_.low_threshold_v) return HookLoadState::INHIBIT;
        if (voltage <= config_.high_threshold_v) return HookLoadState::EMPTY;
        return HookLoadState::LOADED;
    }
    if (stable_state_ == HookLoadState::INHIBIT) {
        if (voltage > high_enter) return HookLoadState::LOADED;
        if (voltage >= low_exit && voltage <= high_exit) {
            return HookLoadState::EMPTY;
        }
        return HookLoadState::INHIBIT;
    }
    if (stable_state_ == HookLoadState::EMPTY) {
        if (voltage > high_enter) return HookLoadState::LOADED;
        if (voltage < low_enter) return HookLoadState::INHIBIT;
        return HookLoadState::EMPTY;
    }

    if (voltage < low_enter) return HookLoadState::INHIBIT;
    if (voltage >= low_exit && voltage < high_exit) {
        return HookLoadState::EMPTY;
    }
    return HookLoadState::LOADED;
}

HookLoadStateResult HookLoadStateFilter::fail(
    const std::string& reason, double voltage) {
    stable_state_ = HookLoadState::UNKNOWN;
    pending_state_ = HookLoadState::UNKNOWN;
    pending_samples_ = 0;
    pending_since_valid_ = false;
    pending_since_source_time_sec_ = 0.0;
    stable_samples_ = 0;
    last_voltage_ = static_cast<float>(voltage);
    invalid_reason_ = reason;
    return result(reason);
}

HookLoadStateResult HookLoadStateFilter::result(
    const std::string& reason) const {
    HookLoadStateResult output;
    output.valid = stable_state_ != HookLoadState::UNKNOWN;
    output.fresh = has_sample_ && output.valid;
    output.state = stable_state_;
    output.voltage = last_voltage_;
    output.stable_samples = stable_samples_;
    output.reason = reason;
    return output;
}

HookLoadStateResult HookLoadStateFilter::ingest(
    double voltage, double wall_time_sec) {
    return ingest(voltage, wall_time_sec, wall_time_sec);
}

HookLoadStateResult HookLoadStateFilter::ingest(
    double voltage, double source_time_sec, double wall_time_sec) {
    if (!configValid()) return fail("invalid_config", voltage);
    if (!std::isfinite(source_time_sec) || source_time_sec < 0.0) {
        return fail("invalid_source_time", voltage);
    }
    if (!std::isfinite(wall_time_sec) || wall_time_sec < 0.0) {
        return fail("invalid_wall_time", voltage);
    }
    if (has_sample_ && wall_time_sec + 1.0e-6 < last_wall_time_sec_) {
        last_wall_time_sec_ = wall_time_sec;
        last_sample_wall_time_sec_ = wall_time_sec;
        has_sample_ = true;
        return fail("wall_time_rollback", voltage);
    }
    if (has_seen_source_time_ &&
        source_time_sec + 1.0e-6 < last_seen_source_time_sec_) {
        last_seen_source_time_sec_ = source_time_sec;
        has_seen_source_time_ = true;
        last_wall_time_sec_ = wall_time_sec;
        last_sample_wall_time_sec_ = wall_time_sec;
        has_sample_ = true;
        return fail("source_time_rollback", voltage);
    }
    if (has_seen_source_time_ &&
        std::abs(source_time_sec - last_seen_source_time_sec_) <= 1.0e-6) {
        if (wall_time_sec - last_sample_wall_time_sec_ + 1.0e-6 >=
            config_.stale_timeout_sec) {
            has_sample_ = false;
            return fail("signal_stale", voltage);
        }
        return result(invalid_reason_.empty()
            ? "duplicate_sample_ignored" : invalid_reason_);
    }
    last_wall_time_sec_ = wall_time_sec;
    last_seen_source_time_sec_ = source_time_sec;
    has_seen_source_time_ = true;
    last_sample_wall_time_sec_ = wall_time_sec;
    has_sample_ = true;

    if (!std::isfinite(voltage)) return fail("non_finite_voltage", voltage);
    if (voltage < config_.valid_voltage_min_v ||
        voltage > config_.valid_voltage_max_v) {
        return fail("voltage_out_of_range", voltage);
    }
    last_voltage_ = static_cast<float>(voltage);
    invalid_reason_.clear();

    const HookLoadState candidate = classify(voltage);
    if (candidate == stable_state_ && stable_state_ != HookLoadState::UNKNOWN) {
        pending_state_ = stable_state_;
        pending_samples_ = 0;
        pending_since_valid_ = false;
        pending_since_source_time_sec_ = 0.0;
        stable_samples_ = std::min<std::uint32_t>(
            stable_samples_ + 1U, std::numeric_limits<std::uint32_t>::max());
        return result("stable");
    }

    if (candidate == pending_state_) {
        ++pending_samples_;
    } else {
        pending_state_ = candidate;
        pending_samples_ = 1U;
        pending_since_valid_ = true;
        pending_since_source_time_sec_ = source_time_sec;
    }
    const double pending_duration_sec = pending_since_valid_
        ? source_time_sec - pending_since_source_time_sec_ : 0.0;
    if (pending_samples_ >= config_.confirm_samples &&
        std::isfinite(pending_duration_sec) &&
        pending_duration_sec + 1.0e-6 >=
            config_.minimum_transition_duration_sec) {
        stable_state_ = pending_state_;
        stable_samples_ = pending_samples_;
        pending_samples_ = 0;
        pending_since_valid_ = false;
        pending_since_source_time_sec_ = 0.0;
        return result("transition_confirmed");
    }
    return result("transition_pending");
}

HookLoadStateResult HookLoadStateFilter::tick(double wall_time_sec) {
    if (!configValid()) return fail("invalid_config", last_voltage_);
    if (!std::isfinite(wall_time_sec) || wall_time_sec < 0.0) {
        return fail("invalid_wall_time", last_voltage_);
    }
    if (has_sample_ && wall_time_sec + 1.0e-6 < last_wall_time_sec_) {
        last_wall_time_sec_ = wall_time_sec;
        return fail("wall_time_rollback", last_voltage_);
    }
    last_wall_time_sec_ = wall_time_sec;
    if (!has_sample_) return result("no_sample");
    if (wall_time_sec - last_sample_wall_time_sec_ + 1.0e-6 >=
        config_.stale_timeout_sec) {
        has_sample_ = false;
        return fail("signal_stale", last_voltage_);
    }
    return result(invalid_reason_.empty() ? "fresh" : invalid_reason_);
}

void HookLoadStateFilter::reset(const std::string& reason) {
    stable_state_ = HookLoadState::UNKNOWN;
    pending_state_ = HookLoadState::UNKNOWN;
    pending_samples_ = 0;
    pending_since_valid_ = false;
    pending_since_source_time_sec_ = 0.0;
    stable_samples_ = 0;
    has_sample_ = false;
    has_seen_source_time_ = false;
    last_wall_time_sec_ = 0.0;
    last_seen_source_time_sec_ = 0.0;
    last_sample_wall_time_sec_ = 0.0;
    last_voltage_ = std::numeric_limits<float>::quiet_NaN();
    invalid_reason_ = reason;
}

}  // namespace ndt_slam
