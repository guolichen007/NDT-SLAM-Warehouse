#pragma once

#include <cstddef>
#include <deque>
#include <string>

namespace ndt_slam {

enum class NdtFitnessCircuitState {
  WARMING = 0,
  CLOSED,
  OPEN,
};

const char* ndtFitnessCircuitStateName(
    NdtFitnessCircuitState state) noexcept;

struct NdtFitnessCircuitBreakerConfig {
  bool enabled = true;
  std::size_t window_size = 120U;
  std::size_t warmup_samples = 30U;
  double minimum_adaptive_threshold = 0.35;
  double hard_reject_threshold = 2.0;
  double median_multiplier = 2.0;
  double mad_multiplier = 6.0;
  double minimum_mad = 0.005;
  double recovery_threshold_ratio = 0.70;
  int enter_confirmations = 3;
  int recovery_confirmations = 5;
};

struct NdtFitnessCircuitDecision {
  bool allow_measurement = true;
  bool circuit_open = false;
  bool transitioned_open = false;
  bool transitioned_closed = false;
  bool baseline_ready = false;
  double adaptive_threshold = 0.0;
  double recovery_threshold = 0.0;
  double median = 0.0;
  double mad = 0.0;
  int bad_streak = 0;
  int recovery_streak = 0;
  NdtFitnessCircuitState state = NdtFitnessCircuitState::WARMING;
  std::string reason = "warming";
};

// Rejects sustained target-relative fitness degradation without imposing a
// density-dependent fixed cliff. An absolute ceiling still rejects a single
// catastrophic match immediately.
class NdtFitnessCircuitBreaker {
 public:
  explicit NdtFitnessCircuitBreaker(
      const NdtFitnessCircuitBreakerConfig& config =
          NdtFitnessCircuitBreakerConfig{});

  void setConfig(const NdtFitnessCircuitBreakerConfig& config);
  const NdtFitnessCircuitBreakerConfig& config() const noexcept {
    return config_;
  }
  void reset();
  NdtFitnessCircuitDecision update(double fitness);
  bool open() const noexcept { return open_; }

 private:
  void appendBaseline(double fitness);
  void computeBaseline(double* median, double* mad) const;

  NdtFitnessCircuitBreakerConfig config_;
  std::deque<double> baseline_;
  bool open_ = false;
  int bad_streak_ = 0;
  int recovery_streak_ = 0;
};

}  // namespace ndt_slam
