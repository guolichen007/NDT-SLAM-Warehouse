#include "ndt_slam/crane_startup_relocalizer.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

class FakeBackend final : public IGlobalRegistrationBackend {
 public:
  explicit FakeBackend(bool ambiguous = false) : ambiguous_(ambiguous) {}

  CraneRegistrationCandidate coarseCandidate(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
      const RelocalizationSeed& seed,
      const CraneStartupRelocalizerConfig&) const override {
    CraneRegistrationCandidate result;
    result.valid = true;
    result.observability_valid = true;
    result.pose = seed.pose;
    result.seed_source = seed.source;
    result.fitness = std::abs(seed.pose.translation().x());
    return result;
  }

  CraneRegistrationCandidate refineCandidate(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
      const CraneRegistrationCandidate& coarse,
      const CraneStartupRelocalizerConfig&) const override {
    ++fine_calls_;
    auto result = coarse;
    result.fitness = ambiguous_ ? 0.2 : coarse.fitness * 0.5;
    return result;
  }

  mutable int fine_calls_ = 0;
  bool ambiguous_ = false;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() {
  auto value = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  value->push_back({0.0F, 0.0F, 0.0F});
  return value;
}

TEST(CraneStartupRelocalizerTest, CheckpointGridIsDeterministicAndConstrained) {
  CraneStartupRelocalizerConfig config;
  config.local_x_radius_m = 1.0;
  config.local_y_radius_m = 1.0;
  config.local_xy_step_m = 1.0;
  config.yaw_tolerance_deg = 0.0;
  config.fixed_z_m = 9.5;
  config.fixed_rail_yaw_rad = 0.01;
  CraneStartupRelocalizer relocalizer(config);
  RecoveryCheckpointData checkpoint;
  checkpoint.x = 18.0;
  checkpoint.y = 4.0;
  checkpoint.z = 100.0;
  checkpoint.yaw = 1.0;
  const auto seeds = relocalizer.checkpointSeeds(checkpoint);
  ASSERT_EQ(seeds.size(), 9U);
  for (const auto& seed : seeds) {
    EXPECT_DOUBLE_EQ(seed.pose.translation().z(), 9.5);
    const auto rotation = seed.pose.so3().matrix();
    EXPECT_NEAR(rotation(2, 0), 0.0, 1.0e-12);
    EXPECT_NEAR(rotation(2, 1), 0.0, 1.0e-12);
  }
}

TEST(CraneStartupRelocalizerTest, RefinesOnlyTopKAndRejectsAmbiguity) {
  CraneStartupRelocalizerConfig config;
  config.top_k_fine = 2U;
  config.maximum_fitness = 10.0;
  config.minimum_fitness_margin = 0.1;
  CraneStartupRelocalizer relocalizer(config);
  std::vector<RelocalizationSeed> seeds(3U);
  for (int index = 0; index < 3; ++index) {
    seeds[index].pose = Sophus::SE3d(Eigen::Matrix3d::Identity(),
                                     Eigen::Vector3d(index + 1.0, 0.0, 0.0));
  }
  FakeBackend backend(true);
  const auto result = relocalizer.recover(cloud(), cloud(), seeds, backend);
  EXPECT_EQ(backend.fine_calls_, 2);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.ambiguous);
  EXPECT_EQ(result.reason, "ambiguous_top_candidates");
}

}  // namespace
}  // namespace ndt_slam

// evaluateRecoveryObservability is a standalone pure function exported
// for direct unit testing. Its declaration is replicated here to avoid
// exposing internals in the public header. It lives in the named internal
// namespace at file scope inside ndt_slam.
namespace ndt_slam {
namespace crane_startup_relocalizer_internal {
bool evaluateRecoveryObservability(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const RelocalizationConfig& ndt_config);
}  // namespace crane_startup_relocalizer_internal

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr warehouseLikeCloud() {
  auto c = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 80; ++i) {
    c->push_back({-4.0F + static_cast<float>(i % 16) * 0.5F,
                  -3.0F + static_cast<float>(i / 16) * 0.5F,
                  0.1F});
  }
  return c;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr xAxisLineCloud() {
  auto c = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 50; ++i) {
    c->push_back({-10.0F + static_cast<float>(i) * 0.4F, 0.0F, 0.1F});
  }
  return c;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr yAxisLineCloud() {
  auto c = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 50; ++i) {
    c->push_back({0.0F, -10.0F + static_cast<float>(i) * 0.4F, 0.1F});
  }
  return c;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr diagonalLineCloud() {
  auto c = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 50; ++i) {
    const float t = -5.0F + static_cast<float>(i) * 0.2F;
    c->push_back({t, t, 0.1F});
  }
  return c;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr tooFewPointsCloud() {
  auto c = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  c->push_back({0.0F, 0.0F, 0.0F});
  c->push_back({1.0F, 0.0F, 0.0F});
  return c;
}

RelocalizationConfig testObservabilityConfig() {
  RelocalizationConfig cfg;
  cfg.min_source_points = 30;
  cfg.min_target_points = 30;
  return cfg;
}

TEST(RecoveryObservabilityTest, Warehouse2DIsObservable) {
  EXPECT_TRUE(crane_startup_relocalizer_internal::evaluateRecoveryObservability(
      warehouseLikeCloud(), warehouseLikeCloud(), testObservabilityConfig()));
}

TEST(RecoveryObservabilityTest, XAxisLineIsNotObservable) {
  EXPECT_FALSE(crane_startup_relocalizer_internal::evaluateRecoveryObservability(
      xAxisLineCloud(), xAxisLineCloud(), testObservabilityConfig()));
}

TEST(RecoveryObservabilityTest, YAxisLineIsNotObservable) {
  EXPECT_FALSE(crane_startup_relocalizer_internal::evaluateRecoveryObservability(
      yAxisLineCloud(), yAxisLineCloud(), testObservabilityConfig()));
}

TEST(RecoveryObservabilityTest, DiagonalLineIsNotObservable) {
  EXPECT_FALSE(crane_startup_relocalizer_internal::evaluateRecoveryObservability(
      diagonalLineCloud(), diagonalLineCloud(), testObservabilityConfig()));
}

TEST(RecoveryObservabilityTest, TooFewPointsIsNotObservable) {
  EXPECT_FALSE(crane_startup_relocalizer_internal::evaluateRecoveryObservability(
      tooFewPointsCloud(), tooFewPointsCloud(), testObservabilityConfig()));
}

TEST(RecoveryObservabilityTest, SourceDegenerateTargetNormalIsNotObservable) {
  EXPECT_FALSE(crane_startup_relocalizer_internal::evaluateRecoveryObservability(
      xAxisLineCloud(), warehouseLikeCloud(), testObservabilityConfig()));
}

TEST(RecoveryObservabilityTest, SourceNormalTargetDegenerateIsNotObservable) {
  EXPECT_FALSE(crane_startup_relocalizer_internal::evaluateRecoveryObservability(
      warehouseLikeCloud(), xAxisLineCloud(), testObservabilityConfig()));
}

}  // namespace
}  // namespace ndt_slam
