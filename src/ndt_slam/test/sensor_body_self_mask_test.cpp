#include "ndt_slam/sensor_body_self_mask.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

pcl::PointCloud<pcl::PointXYZ> sampleCloud() {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  cloud.push_back(pcl::PointXYZ(3.0F, 0.0F, 0.0F));
  return cloud;
}

TEST(SensorBodySelfMaskTest, PreviewDoesNotAuthorizeUncommissionedMask) {
  SensorBodySelfMaskConfig config;
  config.enabled = true;
  config.commissioned = false;
  config.frame_id = "sensor_body";
  SensorBodySelfMask mask;
  mask.configure(config);
  const auto result = mask.filter(sampleCloud(), "sensor_body");
  EXPECT_EQ(result.kept->size(), 2U);
  EXPECT_FALSE(result.mapping_ready);
  EXPECT_EQ(result.reason, "self_mask_not_commissioned");
}

TEST(SensorBodySelfMaskTest, CommissionedMaskRemovesOnlyRigidBodyBox) {
  SensorBodySelfMaskConfig config;
  config.enabled = true;
  config.commissioned = true;
  config.frame_id = "sensor_body";
  config.half_extent = Eigen::Vector3f(0.5F, 0.5F, 0.5F);
  config.maximum_removed_ratio = 0.75;
  SensorBodySelfMask mask;
  mask.configure(config);
  const auto result = mask.filter(sampleCloud(), "sensor_body");
  EXPECT_EQ(result.removed->size(), 1U);
  EXPECT_EQ(result.kept->size(), 1U);
  EXPECT_TRUE(result.mapping_ready);
}

TEST(SensorBodySelfMaskTest, ExcessiveRemovalBlocksFormalMapping) {
  SensorBodySelfMaskConfig config;
  config.enabled = true;
  config.commissioned = true;
  config.frame_id = "sensor_body";
  config.half_extent = Eigen::Vector3f(5.0F, 5.0F, 5.0F);
  config.maximum_removed_ratio = 0.25;
  SensorBodySelfMask mask;
  mask.configure(config);
  const auto result = mask.filter(sampleCloud(), "sensor_body");
  EXPECT_FALSE(result.mapping_ready);
  EXPECT_EQ(result.reason, "self_mask_removed_ratio_exceeded");
}

TEST(SensorBodySelfMaskTest, DisabledOrZeroSizedMaskNeverAuthorizesMapping) {
  SensorBodySelfMaskConfig config;
  config.enabled = false;
  config.commissioned = true;
  config.frame_id = "sensor_body";
  SensorBodySelfMask mask;
  mask.configure(config);
  EXPECT_FALSE(mask.filter(sampleCloud(), "sensor_body").mapping_ready);

  config.enabled = true;
  mask.configure(config);
  const auto invalid_geometry = mask.filter(sampleCloud(), "sensor_body");
  EXPECT_FALSE(invalid_geometry.mapping_ready);
  EXPECT_EQ(invalid_geometry.reason, "self_mask_geometry_invalid");
}

}  // namespace
}  // namespace ndt_slam
