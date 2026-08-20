#include "ndt_slam/se2_observability_proxy.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace ndt_slam {
namespace {

TEST(Se2ObservabilityProxyTest, LabelsSingleCorridorAsWeakEvidence) {
  std::vector<Eigen::Vector2d> points;
  for (int i = -100; i <= 100; ++i) {
    points.emplace_back(0.05 * i, 0.0);
  }
  Se2ObservabilityProxyConfig config;
  config.minimum_points = 30U;
  const auto result = estimateSe2ObservabilityProxy(points, config);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_FALSE(result.yaw_observability_strong);
  EXPECT_EQ(result.reason, "weak_structural_yaw_proxy_normal_operation");
}

TEST(Se2ObservabilityProxyTest, TwoDirectionsProvideCoverage) {
  std::vector<Eigen::Vector2d> points;
  for (int i = -60; i <= 60; ++i) {
    points.emplace_back(0.05 * i, -2.0);
    points.emplace_back(-2.0, 0.05 * i);
  }
  Se2ObservabilityProxyConfig config;
  config.minimum_points = 30U;
  config.minimum_direction_coverage = 0.05;
  const auto result = estimateSe2ObservabilityProxy(points, config);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_GT(result.direction_coverage, 0.05);
}

}  // namespace
}  // namespace ndt_slam
