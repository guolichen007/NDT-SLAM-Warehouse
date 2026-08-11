#include "ndt_slam/crane_place_descriptor.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace ndt_slam {
namespace {

pcl::PointCloud<pcl::PointXYZ> placeCloud(float scale = 1.0F) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int ring = 1; ring <= 8; ++ring) {
    for (int sector = 0; sector < 18; ++sector) {
      const float angle = static_cast<float>(sector * 0.27);
      const float radius = scale * static_cast<float>(ring * 0.8 + sector * 0.03);
      cloud.push_back({radius * std::cos(angle), radius * std::sin(angle),
                       0.2F + static_cast<float>((ring + 2 * sector) % 7)});
    }
  }
  return cloud;
}

TEST(CranePlaceDescriptorTest, ReturnsOnlyRankedPriors) {
  CranePlaceDescriptor descriptor;
  const Sophus::SE3d pose(Eigen::Matrix3d::Identity(),
                          Eigen::Vector3d(18.0, 4.0, 9.0));
  ASSERT_TRUE(descriptor.addPlace(7U, pose, placeCloud()));
  const auto candidates = descriptor.query(placeCloud(), 3U);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().id, 7U);
  EXPECT_GT(candidates.front().similarity, 0.99);
  EXPECT_DOUBLE_EQ(candidates.front().prior_pose.translation().x(), 18.0);
}

pcl::PointCloud<pcl::PointXYZ> negativeZPlaceCloud() {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int ring = 1; ring <= 8; ++ring) {
    for (int sector = 0; sector < 18; ++sector) {
      const float angle = static_cast<float>(sector * 0.27);
      const float radius = static_cast<float>(ring * 0.8 + sector * 0.03);
      // All points at negative Z (structure below sensor)
      cloud.push_back({radius * std::cos(angle), radius * std::sin(angle),
                       -2.0F - static_cast<float>((ring + 2 * sector) % 7)});
    }
  }
  return cloud;
}

TEST(CranePlaceDescriptorTest, NegativeZPlaceIsNotAllZero) {
  CranePlaceDescriptor descriptor;
  const Sophus::SE3d pose(Eigen::Matrix3d::Identity(),
                          Eigen::Vector3d(1.0, 2.0, -3.0));
  ASSERT_TRUE(descriptor.addPlace(1U, pose, negativeZPlaceCloud()));
  EXPECT_GT(descriptor.size(), 0U);
}

TEST(CranePlaceDescriptorTest, NegativeZPlaceMatchesSameStructure) {
  CranePlaceDescriptor descriptor;
  const Sophus::SE3d pose(Eigen::Matrix3d::Identity(),
                          Eigen::Vector3d(1.0, 2.0, -3.0));
  ASSERT_TRUE(descriptor.addPlace(1U, pose, negativeZPlaceCloud()));
  const auto candidates = descriptor.query(negativeZPlaceCloud(), 3U);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().id, 1U);
  EXPECT_GT(candidates.front().similarity, 0.95);
}

TEST(CranePlaceDescriptorTest, DifferentNegativeZStructuresAreNotIdentical) {
  CranePlaceDescriptor descriptor;
  const Sophus::SE3d pose_a(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d(0.0, 0.0, -5.0));
  const Sophus::SE3d pose_b(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d(0.0, 0.0, -5.0));
  auto cloud_a = negativeZPlaceCloud();
  // Perturb offsets so the two structures have different Z distributions.
  auto cloud_b = negativeZPlaceCloud();
  for (auto& pt : cloud_b.points) pt.z -= 3.0F;
  ASSERT_TRUE(descriptor.addPlace(1U, pose_a, cloud_a));
  ASSERT_TRUE(descriptor.addPlace(2U, pose_b, cloud_b));
  const auto candidates_a = descriptor.query(cloud_a, 3U);
  ASSERT_GE(candidates_a.size(), 1U);
  // The best match for cloud_a should be place 1, not place 2.
  EXPECT_EQ(candidates_a.front().id, 1U);
}

TEST(CranePlaceDescriptorTest, NanAndInfAreIgnored) {
  CranePlaceDescriptor descriptor;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({1.0F, 0.0F, -1.0F});
  cloud.push_back({2.0F, 0.0F, std::numeric_limits<float>::quiet_NaN()});
  cloud.push_back({3.0F, 0.0F, std::numeric_limits<float>::infinity()});
  cloud.push_back({4.0F, 0.0F, -std::numeric_limits<float>::infinity()});
  const Sophus::SE3d pose(Eigen::Matrix3d::Identity(),
                          Eigen::Vector3d(0.0, 0.0, 0.0));
  ASSERT_TRUE(descriptor.addPlace(1U, pose, cloud));
}

}  // namespace
}  // namespace ndt_slam
