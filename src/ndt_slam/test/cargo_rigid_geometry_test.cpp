#include <gtest/gtest.h>

#include "ndt_slam/cargo_rigid_geometry.hpp"

#include <cmath>

namespace ndt_slam {
namespace {

LockedCargoShape shape(float yaw = 0.0F) {
    LockedCargoShape value;
    value.valid = true;
    value.length_m = 2.0F;
    value.width_m = 0.8F;
    value.height_m = 1.2F;
    value.yaw_base_rad = yaw;
    value.orientation_confidence = 0.9F;
    return value;
}

LiveCargoPose pose(const Eigen::Vector3f& center) {
    LiveCargoPose value;
    value.valid = true;
    value.center_base = center;
    value.evidence_stamp_sec = 10.0;
    value.evaluation_stamp_sec = 10.0;
    value.source = CargoPoseSource::CURRENT_ASSOCIATED_LIDAR;
    return value;
}

TEST(CargoRigidGeometryTest, LostHoldExpiresFormalUseButKeepsDisplay) {
    const CargoFormalUseDecision short_hold = evaluateCargoFormalUse(
        true, true, 10.4, 10.0, 10.0, 0.5, 0.12F);
    EXPECT_TRUE(short_hold.display_valid);
    EXPECT_TRUE(short_hold.formal_safety_valid);
    EXPECT_TRUE(short_hold.formal_removal_valid);

    const CargoFormalUseDecision expired = evaluateCargoFormalUse(
        true, true, 10.6, 10.0, 10.0, 0.5, 0.18F);
    EXPECT_TRUE(expired.display_valid);
    EXPECT_FALSE(expired.formal_safety_valid);
    EXPECT_FALSE(expired.formal_removal_valid);
    EXPECT_EQ(expired.reason, "lost_hold_display_only_evidence_expired");
}

TEST(CargoRigidGeometryTest, PositionUncertaintyExpandsFormalFootprint) {
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(), pose(Eigen::Vector3f::Zero()),
        Eigen::Isometry3f::Identity(), 11U, 0.20F);
    ASSERT_TRUE(geometry.valid);
    const CargoObbFootprint expanded = toCargoObbFootprint(geometry, 0.20F);
    EXPECT_NEAR(expanded.length_m, 2.40F, 1.0e-5F);
    EXPECT_NEAR(expanded.width_m, 1.20F, 1.0e-5F);
    EXPECT_NEAR(pointToCargoObbDistance2D(
                    Eigen::Vector2f(1.30F, 0.0F), expanded),
                0.10F, 1.0e-5F);
}

TEST(CargoRigidGeometryTest, LivePoseRateLimitScalesWithSensorDt) {
    const Eigen::Vector3f residual(1.0F, 0.0F, 1.0F);
    const Eigen::Vector3f fast_rate = limitCargoPoseResidualByRate(
        residual, 0.10, 2.0F, 1.5F, 0.05F);
    EXPECT_NEAR(fast_rate.x(), 0.25F, 1.0e-5F);
    EXPECT_NEAR(fast_rate.z(), 0.20F, 1.0e-5F);

    const Eigen::Vector3f slower_sensor = limitCargoPoseResidualByRate(
        residual, 0.20, 2.0F, 1.5F, 0.05F);
    EXPECT_NEAR(slower_sensor.x(), 0.45F, 1.0e-5F);
    EXPECT_NEAR(slower_sensor.z(), 0.35F, 1.0e-5F);
}

TEST(CargoRigidGeometryTest, ShapeStaysFixedWhilePoseMovesAndHoists) {
    const LockedCargoShape locked = shape(0.4F);
    const RigidCargoGeometry first = buildCurrentRigidCargoGeometry(
        locked, pose(Eigen::Vector3f(0.0F, 0.0F, 1.0F)),
        Eigen::Isometry3f::Identity(), 7U, 0.05F);
    const RigidCargoGeometry moved = buildCurrentRigidCargoGeometry(
        locked, pose(Eigen::Vector3f(0.6F, -0.4F, 2.0F)),
        Eigen::Isometry3f::Identity(), 7U, 0.08F);
    ASSERT_TRUE(first.valid);
    ASSERT_TRUE(moved.valid);
    EXPECT_FLOAT_EQ(first.shape.length_m, moved.shape.length_m);
    EXPECT_FLOAT_EQ(first.shape.width_m, moved.shape.width_m);
    EXPECT_FLOAT_EQ(first.shape.height_m, moved.shape.height_m);
    EXPECT_FLOAT_EQ(first.shape.yaw_base_rad, moved.shape.yaw_base_rad);
    EXPECT_NEAR(moved.bottom_z_base - first.bottom_z_base, 1.0F, 1.0e-5F);
    EXPECT_NEAR(moved.top_z_base - first.top_z_base, 1.0F, 1.0e-5F);
}

TEST(CargoRigidGeometryTest, ContainsAndDistanceUseOrientedCoordinates) {
    constexpr float kQuarterTurn = 1.57079632679489661923F;
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(kQuarterTurn), pose(Eigen::Vector3f(0.0F, 0.0F, 1.0F)),
        Eigen::Isometry3f::Identity(), 9U, 0.05F);
    ASSERT_TRUE(geometry.valid);
    const CargoObbFootprint footprint = toCargoObbFootprint(geometry);
    EXPECT_TRUE(containsPointInCargoObbBase(
        Eigen::Vector3f(0.0F, 0.9F, 1.0F), footprint));
    EXPECT_FALSE(containsPointInCargoObbBase(
        Eigen::Vector3f(0.9F, 0.0F, 1.0F), footprint));
    EXPECT_NEAR(pointToCargoObbDistance2D(
                    Eigen::Vector2f(0.9F, 0.0F), footprint),
                0.5F, 1.0e-5F);
}

}  // namespace
}  // namespace ndt_slam
