#include "ndt_slam/ndt_fitness_circuit_breaker.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace ndt_slam {
namespace {

bool validConfig(const NdtFitnessCircuitBreakerConfig& config) {
  return config.window_size >= 10U &&
      config.warmup_samples >= 5U &&
      config.warmup_samples <= config.window_size &&
      std::isfinite(config.minimum_adaptive_threshold) &&
      config.minimum_adaptive_threshold > 0.0 &&
      std::isfinite(config.hard_reject_threshold) &&
      config.hard_reject_threshold >
          config.minimum_adaptive_threshold &&
      std::isfinite(config.median_multiplier) &&
      config.median_multiplier >= 1.0 &&
      std::isfinite(config.mad_multiplier) &&
      config.mad_multiplier > 0.0 &&
      std::isfinite(config.minimum_mad) &&
      config.minimum_mad > 0.0 &&
      std::isfinite(config.recovery_threshold_ratio) &&
      config.recovery_threshold_ratio > 0.0 &&
      config.recovery_threshold_ratio < 1.0 &&
      config.enter_confirmations >= 2 &&
      config.recovery_confirmations >= 2;
}

double medianOf(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const std::size_t middle = values.size() / 2U;
  std::nth_element(
      values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2U != 0U) return upper;
  std::nth_element(
      values.begin(), values.begin() + middle - 1U, values.end());
  return 0.5 * (values[middle - 1U] + upper);
}

}  // namespace

const char* ndtFitnessCircuitStateName(
    NdtFitnessCircuitState state) noexcept {
  switch (state) {
    case NdtFitnessCircuitState::WARMING:
      return "WARMING";
    case NdtFitnessCircuitState::CLOSED:
      return "CLOSED";
    case NdtFitnessCircuitState::OPEN:
      return "OPEN";
  }
  return "UNKNOWN";
}

NdtFitnessCircuitBreaker::NdtFitnessCircuitBreaker(
    const NdtFitnessCircuitBreakerConfig& config) {
  setConfig(config);
}

void NdtFitnessCircuitBreaker::setConfig(
    const NdtFitnessCircuitBreakerConfig& config) {
  config_ = validConfig(config)
      ? config : NdtFitnessCircuitBreakerConfig{};
  reset();
}

void NdtFitnessCircuitBreaker::reset() {
  baseline_.clear();
  open_ = false;
  bad_streak_ = 0;
  recovery_streak_ = 0;
}

void NdtFitnessCircuitBreaker::appendBaseline(double fitness) {
  baseline_.push_back(fitness);
  while (baseline_.size() > config_.window_size) {
    baseline_.pop_front();
  }
}

void NdtFitnessCircuitBreaker::computeBaseline(
    double* median, double* mad) const {
  const std::vector<double> samples(baseline_.begin(), baseline_.end());
  *median = medianOf(samples);
  std::vector<double> deviations;
  deviations.reserve(samples.size());
  for (const double sample : samples) {
    deviations.push_back(std::abs(sample - *median));
  }
  *mad = medianOf(std::move(deviations));
}

NdtFitnessCircuitDecision NdtFitnessCircuitBreaker::update(
    double fitness) {
  NdtFitnessCircuitDecision decision;
  if (!config_.enabled) {
    decision.allow_measurement = std::isfinite(fitness) && fitness >= 0.0;
    decision.state = NdtFitnessCircuitState::CLOSED;
    decision.reason = decision.allow_measurement
        ? "fitness_circuit_disabled"
        : "invalid_fitness";
    return decision;
  }

  computeBaseline(&decision.median, &decision.mad);
  decision.baseline_ready =
      baseline_.size() >= config_.warmup_samples;
  const double robust_threshold = std::max(
      decision.median * config_.median_multiplier,
      decision.median +
          config_.mad_multiplier *
              std::max(config_.minimum_mad, decision.mad));
  decision.adaptive_threshold = decision.baseline_ready
      ? std::clamp(
            std::max(
                config_.minimum_adaptive_threshold,
                robust_threshold),
            config_.minimum_adaptive_threshold,
            config_.hard_reject_threshold)
      : config_.hard_reject_threshold;
  decision.recovery_threshold = std::max(
      config_.minimum_mad,
      decision.adaptive_threshold *
          config_.recovery_threshold_ratio);

  const bool invalid = !std::isfinite(fitness) || fitness < 0.0;
  const bool hard_reject =
      !invalid && fitness >= config_.hard_reject_threshold;
  const bool adaptive_reject = !invalid && decision.baseline_ready &&
      fitness >= decision.adaptive_threshold;

  if (invalid || hard_reject) {
    decision.transitioned_open = !open_;
    open_ = true;
    bad_streak_ = config_.enter_confirmations;
    recovery_streak_ = 0;
    decision.allow_measurement = false;
    decision.circuit_open = true;
    decision.state = NdtFitnessCircuitState::OPEN;
    decision.bad_streak = bad_streak_;
    decision.reason = invalid
        ? "invalid_fitness_circuit_open"
        : "hard_fitness_limit_circuit_open";
    return decision;
  }

  if (open_) {
    if (fitness <= decision.recovery_threshold) {
      ++recovery_streak_;
    } else {
      recovery_streak_ = 0;
    }
    if (recovery_streak_ >= config_.recovery_confirmations) {
      open_ = false;
      bad_streak_ = 0;
      recovery_streak_ = 0;
      appendBaseline(fitness);
      decision.allow_measurement = true;
      decision.transitioned_closed = true;
      decision.state = NdtFitnessCircuitState::CLOSED;
      decision.reason = "fitness_circuit_recovered";
    } else {
      decision.allow_measurement = false;
      decision.circuit_open = true;
      decision.state = NdtFitnessCircuitState::OPEN;
      decision.reason = "fitness_circuit_recovery_pending";
    }
    decision.bad_streak = bad_streak_;
    decision.recovery_streak = recovery_streak_;
    return decision;
  }

  if (adaptive_reject) {
    ++bad_streak_;
    recovery_streak_ = 0;
    if (bad_streak_ >= config_.enter_confirmations) {
      open_ = true;
      decision.allow_measurement = false;
      decision.circuit_open = true;
      decision.transitioned_open = true;
      decision.state = NdtFitnessCircuitState::OPEN;
      decision.reason = "adaptive_fitness_circuit_open";
    } else {
      decision.state = NdtFitnessCircuitState::CLOSED;
      decision.reason = "adaptive_fitness_confirmation_pending";
    }
  } else {
    bad_streak_ = 0;
    recovery_streak_ = 0;
    appendBaseline(fitness);
    decision.state = baseline_.size() >= config_.warmup_samples
        ? NdtFitnessCircuitState::CLOSED
        : NdtFitnessCircuitState::WARMING;
    decision.reason = decision.state == NdtFitnessCircuitState::WARMING
        ? "fitness_baseline_warming"
        : "fitness_within_adaptive_limit";
  }
  decision.bad_streak = bad_streak_;
  decision.recovery_streak = recovery_streak_;
  decision.circuit_open = open_;
  return decision;
}

}  // namespace ndt_slam
