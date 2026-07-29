#include "ndt_slam/ndt_fitness_circuit_breaker.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace ndt_slam {
namespace {

NdtFitnessCircuitBreakerConfig testConfig() {
  NdtFitnessCircuitBreakerConfig config;
  config.window_size = 20U;
  config.warmup_samples = 5U;
  config.minimum_adaptive_threshold = 0.20;
  config.hard_reject_threshold = 1.0;
  config.enter_confirmations = 3;
  config.recovery_confirmations = 3;
  return config;
}

void warm(NdtFitnessCircuitBreaker* breaker) {
  for (int index = 0; index < 5; ++index) {
    ASSERT_TRUE(breaker->update(0.05).allow_measurement);
  }
}

TEST(NdtFitnessCircuitBreaker, SustainedAdaptiveSpikeOpensCircuit) {
  NdtFitnessCircuitBreaker breaker(testConfig());
  warm(&breaker);
  EXPECT_TRUE(breaker.update(0.40).allow_measurement);
  EXPECT_TRUE(breaker.update(0.40).allow_measurement);
  const auto opened = breaker.update(0.40);
  EXPECT_FALSE(opened.allow_measurement);
  EXPECT_TRUE(opened.circuit_open);
  EXPECT_TRUE(opened.transitioned_open);
}

TEST(NdtFitnessCircuitBreaker, HardLimitRejectsImmediately) {
  NdtFitnessCircuitBreaker breaker(testConfig());
  const auto opened = breaker.update(1.0);
  EXPECT_FALSE(opened.allow_measurement);
  EXPECT_TRUE(opened.transitioned_open);
  EXPECT_EQ(opened.reason, "hard_fitness_limit_circuit_open");
}

TEST(NdtFitnessCircuitBreaker, RecoveryUsesHysteresisAndConfirmations) {
  NdtFitnessCircuitBreaker breaker(testConfig());
  warm(&breaker);
  breaker.update(0.40);
  breaker.update(0.40);
  ASSERT_FALSE(breaker.update(0.40).allow_measurement);

  EXPECT_FALSE(breaker.update(0.05).allow_measurement);
  EXPECT_FALSE(breaker.update(0.05).allow_measurement);
  const auto recovered = breaker.update(0.05);
  EXPECT_TRUE(recovered.allow_measurement);
  EXPECT_TRUE(recovered.transitioned_closed);
}

TEST(NdtFitnessCircuitBreaker, InvalidFitnessNeverReachesEkf) {
  NdtFitnessCircuitBreaker breaker(testConfig());
  const auto decision = breaker.update(
      std::numeric_limits<double>::infinity());
  EXPECT_FALSE(decision.allow_measurement);
  EXPECT_TRUE(decision.circuit_open);
  EXPECT_EQ(decision.reason, "invalid_fitness_circuit_open");
}

TEST(NdtFitnessCircuitBreaker, ResetRequiresNewBaseline) {
  NdtFitnessCircuitBreaker breaker(testConfig());
  warm(&breaker);
  ASSERT_TRUE(breaker.update(0.05).baseline_ready);
  breaker.reset();
  const auto decision = breaker.update(0.05);
  EXPECT_FALSE(decision.baseline_ready);
  EXPECT_EQ(decision.state, NdtFitnessCircuitState::WARMING);
}

}  // namespace
}  // namespace ndt_slam
