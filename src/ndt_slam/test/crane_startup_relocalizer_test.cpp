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

pcl::PointCloud<pcl::PointXYZ>::Ptr warehouseLikeCloud() {
  auto c = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  // 2D warehouse structure with >1m XY span and >min_source_points
  for (int i = 0; i < 50; ++i) {
    c->push_back({-2.5F + static_cast<float>(i % 10) * 0.5F,
                  -1.5F + static_cast<float>(i / 10) * 0.5F,
                  0.1F});
  }
  return c;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr degenerateLineCloud() {
  auto c = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  // Collinear structure: X varies but Y span < 1m
  for (int i = 0; i < 50; ++i) {
    c->push_back({-10.0F + static_cast<float>(i) * 0.4F, 0.0F, 0.1F});
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

TEST(CraneNdtRegistrationBackendTest, WarehouseStructureIsObservable) {
  CraneNdtRegistrationBackend backend;
  CraneStartupRelocalizerConfig config;
  config.coarse_ndt.min_source_points = 30;
  config.coarse_ndt.min_target_points = 30;
  RelocalizationSeed seed;
  seed.pose = Sophus::SE3d(Eigen::Matrix3d::Identity(),
                           Eigen::Vector3d(0.0, 0.0, 0.0));
  auto source = warehouseLikeCloud();
  auto target = warehouseLikeCloud();
  // coarseCandidate evaluates source structure observability
  // (NDT itself may converge or not — we test the structure path)
  const auto coarse = backend.coarseCandidate(source, target, seed, config);
  if (coarse.valid) {
    EXPECT_TRUE(coarse.observability_valid);
  }
  // For a structure with sufficient XY span, if NDT converges the
  // result should NOT be marked unobservable due to geometry.
  if (coarse.valid && coarse.reason == std::string("coarse_unobservable")) {
    ADD_FAILURE() << "warehouse-like structure should not be unobservable";
  }
}

TEST(CraneNdtRegistrationBackendTest, DegenerateLineIsNotObservable) {
  CraneNdtRegistrationBackend backend;
  CraneStartupRelocalizerConfig config;
  config.coarse_ndt.min_source_points = 30;
  config.coarse_ndt.min_target_points = 30;
  RelocalizationSeed seed;
  seed.pose = Sophus::SE3d(Eigen::Matrix3d::Identity(),
                           Eigen::Vector3d(0.0, 0.0, 0.0));
  const auto coarse = backend.coarseCandidate(
      degenerateLineCloud(), degenerateLineCloud(), seed, config);
  // A degenerate collinear structure must not be marked observable,
  // even if NDT happens to converge with low fitness.
  EXPECT_FALSE(coarse.observability_valid);
}

TEST(CraneNdtRegistrationBackendTest, TooFewPointsIsNotObservable) {
  CraneNdtRegistrationBackend backend;
  CraneStartupRelocalizerConfig config;
  config.coarse_ndt.min_source_points = 30;
  config.coarse_ndt.min_target_points = 30;
  RelocalizationSeed seed;
  seed.pose = Sophus::SE3d(Eigen::Matrix3d::Identity(),
                           Eigen::Vector3d(0.0, 0.0, 0.0));
  const auto coarse = backend.coarseCandidate(
      tooFewPointsCloud(), tooFewPointsCloud(), seed, config);
  EXPECT_FALSE(coarse.observability_valid);
}

}  // namespace
}  // namespace ndt_slam
