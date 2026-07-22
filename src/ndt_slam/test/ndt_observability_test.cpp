#include <gtest/gtest.h>

#include "ndt_slam/ndt_observability.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

namespace ndt_slam {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::vector<Eigen::Vector2d> wallPoints(double angle_rad,
                                        double offset,
                                        int count = 240) {
    const Eigen::Vector2d tangent(std::cos(angle_rad), std::sin(angle_rad));
    const Eigen::Vector2d normal(-tangent.y(), tangent.x());
    std::vector<Eigen::Vector2d> points;
    points.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
        const double along = -12.0 + 24.0 *
            static_cast<double>(index) / static_cast<double>(count - 1);
        points.push_back(along * tangent + offset * normal);
    }
    return points;
}

void append(std::vector<Eigen::Vector2d>& destination,
            const std::vector<Eigen::Vector2d>& source) {
    destination.insert(destination.end(), source.begin(), source.end());
}

NdtObservabilityConfig testConfig() {
    NdtObservabilityConfig config;
    config.min_structure_points = 100U;
    config.min_occupied_cells = 20U;
    config.min_local_normals = 40U;
    config.max_normal_samples = 500U;
    config.normal_search_radius_m = 0.80;
    return config;
}

double absoluteAlignment(const Eigen::Vector2d& lhs,
                         const Eigen::Vector2d& rhs) {
    return std::abs(lhs.normalized().dot(rhs.normalized()));
}

void expectWeakInnovationSuppressed(const NdtObservability& observability,
                                    const NdtObservabilityConfig& config) {
    const Eigen::Matrix2d covariance =
        buildObservabilityAwareMeasurementCovariance(
            0.02, observability, config);
    const Eigen::Matrix2d gain =
        (Eigen::Matrix2d::Identity() + covariance).inverse();
    const double strong_gain = observability.strong_direction.dot(
        gain * observability.strong_direction);
    const double weak_gain = observability.weak_direction.dot(
        gain * observability.weak_direction);
    EXPECT_GT(strong_gain, weak_gain);
}

TEST(NdtObservabilityTest, UniformPerpendicularStructureIsObservable) {
    std::vector<Eigen::Vector2d> points = wallPoints(0.0, -3.0);
    append(points, wallPoints(kPi * 0.5, 3.0));
    const auto result = estimateNdtObservabilityFromStructure(
        points, testConfig());

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_FALSE(result.degenerate);
    EXPECT_FALSE(result.severely_degenerate);
    EXPECT_GT(result.eigenvalue_ratio, 0.20);
}

TEST(NdtObservabilityTest, HorizontalWallsHaveWeakXDirection) {
    std::vector<Eigen::Vector2d> points = wallPoints(0.0, -2.0);
    append(points, wallPoints(0.0, 2.0));
    const auto config = testConfig();
    const auto result = estimateNdtObservabilityFromStructure(points, config);

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_TRUE(result.severely_degenerate);
    EXPECT_GT(absoluteAlignment(result.weak_direction,
                                Eigen::Vector2d::UnitX()), 0.95);
    expectWeakInnovationSuppressed(result, config);
}

TEST(NdtObservabilityTest, RotatedNinetyDegreesHasWeakYDirection) {
    std::vector<Eigen::Vector2d> points = wallPoints(kPi * 0.5, -2.0);
    append(points, wallPoints(kPi * 0.5, 2.0));
    const auto config = testConfig();
    const auto result = estimateNdtObservabilityFromStructure(points, config);

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_TRUE(result.severely_degenerate);
    EXPECT_GT(absoluteAlignment(result.weak_direction,
                                Eigen::Vector2d::UnitY()), 0.95);
    expectWeakInnovationSuppressed(result, config);
}

TEST(NdtObservabilityTest, FortyFiveDegreeWeakDirectionIsCoordinateFree) {
    const double angle = kPi / 4.0;
    std::vector<Eigen::Vector2d> points = wallPoints(angle, -2.0);
    append(points, wallPoints(angle, 2.0));
    const auto config = testConfig();
    const auto result = estimateNdtObservabilityFromStructure(points, config);

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_TRUE(result.severely_degenerate);
    const Eigen::Vector2d expected(std::cos(angle), std::sin(angle));
    EXPECT_GT(absoluteAlignment(result.weak_direction, expected), 0.95);
    expectWeakInnovationSuppressed(result, config);
}

TEST(NdtObservabilityTest, ModerateAndSevereInflationPreserveStrongDirection) {
    const auto config = testConfig();
    NdtObservability moderate;
    moderate.valid = true;
    moderate.degenerate = true;
    moderate.severely_degenerate = false;
    moderate.strong_direction = Eigen::Vector2d::UnitY();
    moderate.weak_direction = Eigen::Vector2d::UnitX();
    Eigen::Matrix2d covariance =
        buildObservabilityAwareMeasurementCovariance(0.02, moderate, config);
    EXPECT_NEAR(covariance(0, 0), 0.10, 1.0e-12);
    EXPECT_NEAR(covariance(1, 1), 0.02, 1.0e-12);

    moderate.severely_degenerate = true;
    covariance = buildObservabilityAwareMeasurementCovariance(
        0.02, moderate, config);
    EXPECT_NEAR(covariance(0, 0), 0.40, 1.0e-12);
    EXPECT_NEAR(covariance(1, 1), 0.02, 1.0e-12);
}

TEST(NdtObservabilityTest, SourceAxesRotateIntoEkfMeasurementFrame) {
    NdtObservability source;
    source.valid = true;
    source.degenerate = true;
    source.strong_direction = Eigen::Vector2d::UnitY();
    source.weak_direction = Eigen::Vector2d::UnitX();

    const NdtObservability yaw45 = rotateNdtObservability(source, kPi / 4.0);
    EXPECT_GT(absoluteAlignment(
                  yaw45.weak_direction,
                  Eigen::Vector2d(std::sqrt(0.5), std::sqrt(0.5))),
              0.999);

    const NdtObservability yaw90 = rotateNdtObservability(source, kPi / 2.0);
    EXPECT_GT(absoluteAlignment(yaw90.weak_direction,
                                Eigen::Vector2d::UnitY()), 0.999);
}

}  // namespace
}  // namespace ndt_slam
