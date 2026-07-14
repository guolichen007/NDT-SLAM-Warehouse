#pragma once

#include <cstdint>
#include <limits>
#include <string>

namespace ndt_slam {

enum class HookLoadState : std::uint8_t {
    UNKNOWN = 0,
    INHIBIT = 1,
    EMPTY = 2,
    LOADED = 3
};

struct HookLoadStateConfig {
    double low_threshold_v = 1.90;
    double high_threshold_v = 2.10;
    double hysteresis_v = 0.03;
    std::uint32_t confirm_samples = 2;
    double stale_timeout_sec = 2.50;
    double valid_voltage_min_v = 0.0;
    double valid_voltage_max_v = 6.0;
};

struct HookLoadStateResult {
    bool valid = false;
    bool fresh = false;
    HookLoadState state = HookLoadState::UNKNOWN;
    float voltage = std::numeric_limits<float>::quiet_NaN();
    std::uint32_t stable_samples = 0;
    std::string reason = "startup_unknown";
};

class HookLoadStateFilter {
public:
    explicit HookLoadStateFilter(
        const HookLoadStateConfig& config = HookLoadStateConfig());

    void setConfig(const HookLoadStateConfig& config);
    const HookLoadStateConfig& config() const { return config_; }
    HookLoadStateResult ingest(double voltage, double wall_time_sec);
    HookLoadStateResult ingest(double voltage,
                               double source_time_sec,
                               double wall_time_sec);
    HookLoadStateResult tick(double wall_time_sec);
    void reset(const std::string& reason = "reset");

private:
    bool configValid() const;
    HookLoadState classify(double voltage) const;
    HookLoadStateResult fail(const std::string& reason, double voltage);
    HookLoadStateResult result(const std::string& reason) const;

    HookLoadStateConfig config_;
    HookLoadState stable_state_ = HookLoadState::UNKNOWN;
    HookLoadState pending_state_ = HookLoadState::UNKNOWN;
    std::uint32_t pending_samples_ = 0;
    std::uint32_t stable_samples_ = 0;
    bool has_sample_ = false;
    double last_wall_time_sec_ = 0.0;
    double last_source_time_sec_ = 0.0;
    double last_sample_wall_time_sec_ = 0.0;
    float last_voltage_ = std::numeric_limits<float>::quiet_NaN();
    std::string invalid_reason_ = "startup_unknown";
};

}  // namespace ndt_slam
