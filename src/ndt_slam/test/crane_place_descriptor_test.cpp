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

}  // namespace
}  // namespace ndt_slam
