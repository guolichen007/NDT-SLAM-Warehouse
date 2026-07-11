#include <gtest/gtest.h>
#include <cmath>
#include <utility>
#include <pcl/common/transforms.h>

#include "ndt_slam/loop_closure.hpp"
#include "ndt_slam/ndt_relocalizer.hpp"

namespace ndt_slam {
namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr makeAsymmetricCloud() {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (int ring = 1; ring <= 8; ++ring) {
        for (int sector = 0; sector < 18; ++sector) {
            const float angle = static_cast<float>(sector * 0.27);
            const float radius = static_cast<float>(ring * 0.8 + sector * 0.03);
            cloud->push_back(pcl::PointXYZ(
                radius * std::cos(angle), radius * std::sin(angle),
                0.2F + static_cast<float>((ring + 2 * sector) % 7) * 0.11F));
        }
    }
    return cloud;
}

TEST(RelocalizationScanContext, ReportsYawOffsetForGlobalSeed) {
    LoopClosureDetector detector;
    detector.configure(20, 60, 80.0, 8.0, 0.8, 1.0, 0.2);
    const auto keyframe_cloud = makeAsymmetricCloud();
    detector.addKeyFrame(Sophus::SE3d(), keyframe_cloud, ros::Time(1, 0));

    auto query = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    rotation.block<3, 3>(0, 0) = Eigen::AngleAxisf(
        -30.0F * static_cast<float>(M_PI) / 180.0F,
        Eigen::Vector3f::UnitZ()).toRotationMatrix();
    pcl::transformPointCloud(*keyframe_cloud, *query, rotation);

    const auto hints = detector.findRelocalizationHints(query, 1, 0.90);
    ASSERT_EQ(hints.size(), 1U);
    EXPECT_NEAR(hints.front().yaw_offset_rad * 180.0 / M_PI, 30.0, 6.1);
}

TEST(RelocalizationScanContext, BaseFrameDescriptorIsTranslationIndependent) {
    LoopClosureDetector detector;
    detector.configure(20, 60, 80.0, 8.0, 0.8, 1.0, 0.2);
    const auto cloud = makeAsymmetricCloud();
    const Sophus::SE3d map_pose(
        Eigen::Matrix3d::Identity(), Eigen::Vector3d(37.0, -12.0, 0.0));

    detector.addKeyFrame(map_pose, cloud, ros::Time(1, 0));
    const auto hints = detector.findRelocalizationHints(cloud, 1, 0.99);

    ASSERT_EQ(hints.size(), 1U);
    EXPECT_NEAR(hints.front().similarity, 1.0, 1.0e-9);
    EXPECT_NEAR(hints.front().pose.translation().x(), 37.0, 1.0e-9);
    EXPECT_NEAR(hints.front().pose.translation().y(), -12.0, 1.0e-9);
}

TEST(NdtRelocalizerLifecycle, RejectsIncompleteJobsWithoutBlocking) {
    NdtRelocalizer relocalizer;
    RelocalizationConfig config;
    relocalizer.configure(config);
    relocalizer.start();
    RelocalizationJob empty_job;
    EXPECT_FALSE(relocalizer.submit(std::move(empty_job)));
    relocalizer.stop();
    EXPECT_FALSE(relocalizer.busy());
}

}  // namespace
}  // namespace ndt_slam
