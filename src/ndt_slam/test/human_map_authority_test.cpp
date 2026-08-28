#include <gtest/gtest.h>

#include "ndt_slam/avoidance_map_mutation.hpp"
#include "ndt_slam/human_object_filter.hpp"

namespace ndt_slam {
namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr humanLikeCloud() {
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud->push_back({0.00F, 0.00F, 0.00F});
  cloud->push_back({0.10F, 0.00F, 0.30F});
  cloud->push_back({0.20F, 0.00F, 0.60F});
  cloud->push_back({0.00F, 0.20F, 0.80F});
  cloud->push_back({0.10F, 0.20F, 1.00F});
  cloud->push_back({0.20F, 0.20F, 1.20F});
  return cloud;
}

TEST(HumanMapAuthority, HumanPreRegistrationClassificationIsStateless) {
  HumanObjectDynamicFilter filter;
  HumanObjectFilterConfig classification;
  HumanTrackingConfig tracking;
  HumanEraserConfig eraser;
  filter.initialize(classification, tracking, eraser);
  auto safe = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  auto candidates = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);

  const HumanFrameClassification result = filter.classifyFrame(
      humanLikeCloud(), 1.0, 1U, 0.06F, safe, candidates);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(filter.getActiveTrackCount(), 0);
  EXPECT_FALSE(candidates->empty());
}

TEST(HumanMapAuthority, SamePhysicalFrameCannotAdvanceHumanTrackTwice) {
  HumanObjectDynamicFilter filter;
  HumanObjectFilterConfig classification;
  HumanTrackingConfig tracking;
  HumanEraserConfig eraser;
  filter.initialize(classification, tracking, eraser);
  auto safe = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  auto candidates = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  const auto frame = filter.classifyFrame(
      humanLikeCloud(), 1.0, 7U, 0.06F, safe, candidates);

  const HumanMapFilterSnapshot first = filter.updateMapTracks(
      frame, Eigen::Matrix4d::Identity(), 0.15F);
  const HumanMapFilterSnapshot duplicate = filter.updateMapTracks(
      frame, Eigen::Matrix4d::Identity(), 0.15F);

  ASSERT_TRUE(first.valid);
  EXPECT_FALSE(duplicate.valid);
  EXPECT_EQ(filter.getActiveTrackCount(), 1);
}

TEST(HumanMapAuthority, OlderMapCommitStampCannotRollbackHumanTrack) {
  HumanObjectDynamicFilter filter;
  HumanObjectFilterConfig classification;
  HumanTrackingConfig tracking;
  HumanEraserConfig eraser;
  filter.initialize(classification, tracking, eraser);
  auto safe = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  auto candidates = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  const auto newer = filter.classifyFrame(
      humanLikeCloud(), 2.0, 2U, 0.06F, safe, candidates);
  const auto older = filter.classifyFrame(
      humanLikeCloud(), 1.0, 1U, 0.06F, safe, candidates);

  ASSERT_TRUE(filter.updateMapTracks(
      newer, Eigen::Matrix4d::Identity(), 0.15F).valid);
  EXPECT_FALSE(filter.updateMapTracks(
      older, Eigen::Matrix4d::Identity(), 0.15F).valid);
  EXPECT_EQ(filter.getActiveTrackCount(), 1);
}

TEST(HumanMapAuthority, HumanMapTrackingUsesFinalFrameAuthorityPose) {
  HumanObjectDynamicFilter filter;
  HumanObjectFilterConfig classification;
  HumanTrackingConfig tracking;
  HumanEraserConfig eraser;
  filter.initialize(classification, tracking, eraser);
  auto safe = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  auto candidates = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  const auto frame = filter.classifyFrame(
      humanLikeCloud(), 3.0, 3U, 0.06F, safe, candidates);
  Eigen::Matrix4d final_pose = Eigen::Matrix4d::Identity();
  final_pose(0, 3) = 2.0;

  const auto snapshot = filter.updateMapTracks(frame, final_pose, 0.15F);

  ASSERT_TRUE(snapshot.valid);
  EXPECT_TRUE(snapshot.static_learning_blocks.human_cells.count({13, 0}) > 0U);
  EXPECT_EQ(snapshot.static_learning_blocks.human_cells.count({0, 0}), 0U);
}

TEST(HumanMapAuthority, HumanPointOwnershipPreservesBackgroundInBroadPrism) {
  CurrentFramePointOwnership ownership;
  ownership.valid = true;
  ownership.voxel_size_m = 0.05F;
  const pcl::PointXYZ human_point{0.10F, 0.10F, 1.00F};
  const pcl::PointXYZ background_point{0.18F, 0.10F, 1.00F};
  PointOwnershipVoxel voxel;
  ASSERT_TRUE(makePointOwnershipVoxel(
      human_point, ownership.voxel_size_m, &voxel));
  ownership.voxels.insert(voxel);

  EXPECT_TRUE(ownership.owns(human_point));
  EXPECT_FALSE(ownership.owns(background_point));
}

}  // namespace
}  // namespace ndt_slam
