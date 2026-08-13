#include <gtest/gtest.h>

#include "ndt_slam/keyframe_manager.hpp"

namespace ndt_slam {
namespace {

ros::Time stamp(double seconds) {
    ros::Time value;
    value.fromSec(seconds);
    return value;
}

TEST(KeyFrameManagerTimeEpoch, RebaseChangesOnlyTemporalEligibilityGate) {
    KeyFrameManager manager;
    manager.configure(10.0, 180.0, 1.5, 0);
    const Sophus::SE3d pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(1.0F, 2.0F, 3.0F));

    manager.addKeyFrame(pose, cloud, stamp(1779155914.0));
    const KeyFrame before = manager.getKeyFrames().back();

    manager.resetTemporalGateForSourceEpoch(stamp(1778217251.0));

    ASSERT_EQ(1U, manager.getKeyFrameCount());
    const KeyFrame& after = manager.getKeyFrames().back();
    EXPECT_EQ(before.id_, after.id_);
    EXPECT_EQ(before.stamp_, after.stamp_);
    EXPECT_EQ(before.cloud_.get(), after.cloud_.get());
    EXPECT_NEAR((before.pose_.inverse() * after.pose_).log().norm(), 0.0, 1e-12);

    // A stationary device is not forced to create a keyframe immediately.
    EXPECT_FALSE(manager.isKeyFrame(pose, stamp(1778217252.0)));
    // Normal time eligibility becomes available in the new source epoch.
    EXPECT_TRUE(manager.isKeyFrame(pose, stamp(1778217253.0)));
}

}  // namespace
}  // namespace ndt_slam
