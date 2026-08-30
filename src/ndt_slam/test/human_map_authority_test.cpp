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

SourceFrameIdentity frameIdentity(
    std::uint64_t index, double stamp,
    const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  return makeSourceFrameIdentity(index, stamp, 1U, cloud);
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

  const auto cloud = humanLikeCloud();
  const HumanFrameClassification result = filter.classifyFrame(
      cloud, frameIdentity(1U, 1.0, *cloud), 0.06F, safe, candidates);

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
  const auto cloud = humanLikeCloud();
  const auto frame = filter.classifyFrame(
      cloud, frameIdentity(7U, 1.0, *cloud), 0.06F, safe, candidates);

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
  const auto cloud = humanLikeCloud();
  const auto newer = filter.classifyFrame(
      cloud, frameIdentity(2U, 2.0, *cloud), 0.06F, safe, candidates);
  const auto older = filter.classifyFrame(
      cloud, frameIdentity(1U, 1.0, *cloud), 0.06F, safe, candidates);

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
  const auto cloud = humanLikeCloud();
  const auto frame = filter.classifyFrame(
      cloud, frameIdentity(3U, 3.0, *cloud), 0.06F, safe, candidates);
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
  const pcl::PointXYZ background_point{0.11F, 0.10F, 1.00F};
  pcl::PointCloud<pcl::PointXYZ> source;
  source.push_back(human_point);
  source.push_back(background_point);
  ownership.source_frame_identity = frameIdentity(1U, 1.0, source);
  SourcePointKey exact_key;
  ASSERT_TRUE(makeSourcePointKey(human_point, &exact_key));
  ownership.exact_points.insert(exact_key);
  PointOwnershipVoxel voxel;
  ASSERT_TRUE(makePointOwnershipVoxel(
      human_point, ownership.voxel_size_m, &voxel));
  ownership.voxels.insert(voxel);

  EXPECT_TRUE(ownership.owns(human_point));
  EXPECT_FALSE(ownership.owns(background_point));
}

TEST(HumanMapAuthority, ExactOwnershipCannotCrossSourceFrameIdentity) {
  const pcl::PointXYZ human_point{0.10F, 0.10F, 1.00F};
  pcl::PointCloud<pcl::PointXYZ> source;
  source.push_back(human_point);
  CurrentFramePointOwnership ownership;
  ownership.valid = true;
  ownership.source_frame_identity = frameIdentity(1U, 1.0, source);
  SourcePointKey key;
  ASSERT_TRUE(makeSourcePointKey(human_point, &key));
  ownership.exact_points.insert(key);

  const SourceFrameIdentity later = frameIdentity(2U, 2.0, source);
  EXPECT_TRUE(ownership.owns(ownership.source_frame_identity, human_point));
  EXPECT_FALSE(ownership.owns(later, human_point));
}

TEST(HumanMapAuthority, DuplicateAndOutOfOrderRejectionsAreObservable) {
  HumanObjectDynamicFilter filter;
  HumanObjectFilterConfig classification;
  HumanTrackingConfig tracking;
  HumanEraserConfig eraser;
  filter.initialize(classification, tracking, eraser);
  auto safe = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  auto candidates = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  const auto cloud = humanLikeCloud();
  const auto frame = filter.classifyFrame(
      cloud, frameIdentity(2U, 2.0, *cloud), 0.06F, safe, candidates);
  ASSERT_TRUE(filter.updateMapTracks(
      frame, Eigen::Matrix4d::Identity(), 0.15F).valid);
  EXPECT_FALSE(filter.updateMapTracks(
      frame, Eigen::Matrix4d::Identity(), 0.15F).valid);
  const auto older = filter.classifyFrame(
      cloud, frameIdentity(1U, 1.0, *cloud), 0.06F, safe, candidates);
  EXPECT_FALSE(filter.updateMapTracks(
      older, Eigen::Matrix4d::Identity(), 0.15F).valid);

  const HumanMapAuthorityDiagnostics diagnostics = filter.diagnostics();
  EXPECT_EQ(diagnostics.map_track_update_count, 1U);
  EXPECT_EQ(diagnostics.duplicate_update_reject_count, 1U);
  EXPECT_EQ(diagnostics.out_of_order_update_reject_count, 1U);
  EXPECT_EQ(diagnostics.track_high_water, 1U);
}

}  // namespace
}  // namespace ndt_slam
