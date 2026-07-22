#include "ndt_slam/revealed_support_observer.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

StaticHeightComponent origin() {
  StaticHeightComponent component;
  component.valid = true;
  component.component_id = 9U;
  component.map_generation = 3U;
  component.support_z_map = 0.0F;
  component.top_z95_map = 1.0F;
  component.members = {
      {packStaticEvidenceCell(0, 0), 0U},
      {packStaticEvidenceCell(1, 0), 0U}};
  return component;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr observation(bool show_support) {
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 3; ++i) {
    cloud->push_back(pcl::PointXYZ(0.05F, 0.05F,
                                  show_support ? 0.0F : 1.0F));
    cloud->push_back(pcl::PointXYZ(0.30F, 0.05F, 1.0F));
  }
  return cloud;
}

RevealedSupportObservationInput input(double stamp, bool show_support) {
  RevealedSupportObservationInput value;
  value.stamp_sec = stamp;
  value.cargo_lifecycle_id = 5U;
  value.map_generation = 3U;
  value.origin_component = origin();
  value.observation_cloud_base = observation(show_support);
  value.T_map_base = Sophus::SE3d();
  return value;
}

TEST(RevealedSupportObserverTest, UsesCurrentFrameCoverageOnly) {
  RevealedSupportObserver observer;
  const auto revealed = observer.update(input(1.0, true));
  ASSERT_TRUE(revealed.valid);
  EXPECT_EQ(revealed.origin_observable_cells, 2U);
  EXPECT_EQ(revealed.origin_revealed_cells, 1U);
  EXPECT_FLOAT_EQ(revealed.coverage, 0.5F);
  const auto covered_again = observer.update(input(1.1, false));
  EXPECT_FALSE(covered_again.valid);
  EXPECT_EQ(covered_again.origin_revealed_cells, 0U);
  EXPECT_FLOAT_EQ(covered_again.coverage, 0.0F);
  EXPECT_DOUBLE_EQ(covered_again.evidence_stamp_sec, 1.0);
}

}  // namespace
}  // namespace ndt_slam
