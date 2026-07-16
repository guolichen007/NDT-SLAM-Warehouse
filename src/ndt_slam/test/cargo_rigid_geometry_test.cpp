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
    value.stamp_sec = 10.0;
    value.source = CargoPoseSource::CURRENT_ASSOCIATED_LIDAR;
    return value;
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
