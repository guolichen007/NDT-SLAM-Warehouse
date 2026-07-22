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

TEST(CargoRigidGeometryTest, FormalHeightUsesOneAuthorityDecision) {
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(), pose(Eigen::Vector3f(0.0F, 0.0F, 2.0F)),
        Eigen::Isometry3f::Identity(), 7U, 0.10F, 0.08F);
    ASSERT_TRUE(geometry.valid);
    EXPECT_DOUBLE_EQ(geometry.height_evidence_stamp_sec, 10.0);
    CargoBottomResult fusion;
    fusion.valid = true;
    fusion.source = CargoBottomSource::DIRECT_TOP_FROZEN_THICKNESS;
    fusion.reason = "fresh_top_with_track_frozen_thickness";
    fusion.uncertainty = 0.06F;
    fusion.confidence = 0.80F;
    const CargoFormalUseDecision lifetime = evaluateCargoFormalUse(
        true, false, 10.2, 10.0, 10.0, 0.5, 0.5, 0.10F);
    const CargoFormalHeightDecision decision =
        evaluateCargoFormalHeight(fusion, geometry, lifetime);
    ASSERT_TRUE(decision.valid) << decision.reason;
    EXPECT_EQ(decision.source,
              CargoBottomSource::DIRECT_TOP_FROZEN_THICKNESS);
    EXPECT_NEAR(decision.bottom_z_base, 1.4F, 1.0e-5F);
    EXPECT_NEAR(decision.top_z_base, 2.6F, 1.0e-5F);
}

TEST(CargoRigidGeometryTest, BuilderPreservesHeldPoseHeightEvidenceStamp) {
    LiveCargoPose held_pose = pose(Eigen::Vector3f(0.0F, 0.0F, 2.0F));
    held_pose.source = CargoPoseSource::HELD_LAST_RELIABLE_OFFSET;
    held_pose.evidence_stamp_sec = 8.5;
    held_pose.evaluation_stamp_sec = 10.0;
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(), held_pose, Eigen::Isometry3f::Identity(), 9U,
        0.15F, 0.10F);
    ASSERT_TRUE(geometry.valid);
    EXPECT_DOUBLE_EQ(geometry.pose_evidence_stamp_sec, 8.5);
    EXPECT_DOUBLE_EQ(geometry.height_evidence_stamp_sec, 8.5);
    EXPECT_DOUBLE_EQ(geometry.evaluation_stamp_sec, 10.0);
}

TEST(CargoRigidGeometryTest, ExpiredTopEvidenceCannotBeAuthorized) {
    RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(), pose(Eigen::Vector3f(0.0F, 0.0F, 2.0F)),
        Eigen::Isometry3f::Identity(), 8U, 0.10F, 0.08F);
    ASSERT_TRUE(geometry.valid);
    geometry.height_evidence_stamp_sec = 10.0;
    CargoBottomResult fusion;
    fusion.valid = true;
    fusion.source = CargoBottomSource::DIRECT_TOP_FROZEN_THICKNESS;
    const CargoFormalUseDecision lifetime = evaluateCargoFormalUse(
        true, false, 11.0, 10.8, 10.0, 0.5, 0.5, 0.10F);
    const CargoFormalHeightDecision decision =
        evaluateCargoFormalHeight(fusion, geometry, lifetime);
    EXPECT_FALSE(decision.valid);
    EXPECT_EQ(decision.reason, "formal_vertical_evidence_expired");
}

TEST(CargoRigidGeometryTest, LostHoldExpiresFormalUseButKeepsDisplay) {
    const CargoFormalUseDecision short_hold = evaluateCargoFormalUse(
        true, true, 10.4, 10.0, 10.0, 0.5, 0.5, 0.12F);
    EXPECT_TRUE(short_hold.display_valid);
    EXPECT_TRUE(short_hold.formal_safety_valid);
    EXPECT_TRUE(short_hold.formal_removal_valid);

    const CargoFormalUseDecision expired = evaluateCargoFormalUse(
        true, true, 10.6, 10.0, 10.0, 0.5, 0.5, 0.18F);
    EXPECT_TRUE(expired.display_valid);
    EXPECT_FALSE(expired.formal_safety_valid);
    EXPECT_FALSE(expired.formal_removal_valid);
    EXPECT_EQ(expired.reason, "formal_vertical_evidence_expired");
}

TEST(CargoRigidGeometryTest, LockedStateCannotBypassFormalEvidenceCutoff) {
    const CargoFormalUseDecision expired_while_locked =
        evaluateCargoFormalUse(
            true, false, 10.6, 10.0, 10.0, 0.5, 0.5, 0.18F);
    EXPECT_TRUE(expired_while_locked.display_valid);
    EXPECT_FALSE(expired_while_locked.formal_safety_valid);
    EXPECT_FALSE(expired_while_locked.formal_removal_valid);
    EXPECT_EQ(expired_while_locked.reason,
              "formal_vertical_evidence_expired");

    const CargoFormalUseDecision expired_after_state_transition =
        evaluateCargoFormalUse(
            true, true, 10.6, 10.0, 10.0, 0.5, 0.5, 0.18F);
    EXPECT_FALSE(expired_after_state_transition.formal_safety_valid);
    EXPECT_FALSE(expired_after_state_transition.formal_removal_valid);
}

TEST(CargoRigidGeometryTest, FrozenThicknessDoesNotRefreshVerticalPosition) {
    const CargoFormalUseDecision decision = evaluateCargoFormalUse(
        true, false, 20.0, 19.7, 10.0, 0.60, 0.60, 0.15F);
    EXPECT_FALSE(decision.formal_safety_valid);
    EXPECT_FALSE(decision.formal_removal_valid);
    EXPECT_GT(decision.height_age_sec, 9.0);
    EXPECT_EQ(decision.reason, "formal_vertical_evidence_expired");
}

TEST(CargoRigidGeometryTest, XyAndVerticalEvidenceExpireIndependently) {
    const CargoFormalUseDecision xy_expired = evaluateCargoFormalUse(
        true, false, 10.6, 10.0, 10.5, 0.50, 1.00, 0.15F);
    EXPECT_FALSE(xy_expired.formal_safety_valid);
    EXPECT_EQ(xy_expired.reason, "formal_xy_evidence_expired");

    const CargoFormalUseDecision vertical_expired = evaluateCargoFormalUse(
        true, false, 10.6, 10.5, 10.0, 1.00, 0.50, 0.15F);
    EXPECT_FALSE(vertical_expired.formal_safety_valid);
    EXPECT_EQ(vertical_expired.reason, "formal_vertical_evidence_expired");
}

TEST(CargoRigidGeometryTest, MissingObservationPredictsImmediatelyButPreservesEvidenceAge) {
    LiveCargoPose observed = pose(Eigen::Vector3f(1.0F, 2.0F, 3.0F));
    observed.position_uncertainty_m = 0.10F;
    const Eigen::Vector3f velocity(0.5F, 0.0F, 0.2F);

    const LiveCargoPose held_short = propagateHeldCargoPose(
        observed, velocity, 10.4, 0.5, 2.0F, 1.5F,
        0.30, 0.05F, 0.50F);
    ASSERT_TRUE(held_short.valid);
    EXPECT_EQ(held_short.source, CargoPoseSource::MOTION_PREDICTION);
    EXPECT_DOUBLE_EQ(held_short.evidence_stamp_sec, 10.0);
    EXPECT_DOUBLE_EQ(held_short.evaluation_stamp_sec, 10.4);
    const float short_prediction = 0.30F *
        (1.0F - std::exp(-0.40F / 0.30F));
    EXPECT_NEAR(held_short.center_base.x(),
                1.0F + 0.5F * short_prediction, 1.0e-5F);
    EXPECT_NEAR(held_short.center_base.z(),
                3.0F + 0.2F * short_prediction, 1.0e-5F);
    EXPECT_NEAR(held_short.position_uncertainty_m, 0.12F, 1.0e-5F);
    const CargoFormalUseDecision short_use = evaluateCargoFormalUse(
        true, false, 10.4, held_short.evidence_stamp_sec,
        held_short.evidence_stamp_sec, 0.5, 0.5,
        held_short.position_uncertainty_m);
    EXPECT_TRUE(short_use.formal_safety_valid);
    EXPECT_TRUE(short_use.formal_removal_valid);

    const LiveCargoPose held_expired = propagateHeldCargoPose(
        observed, velocity, 10.6, 0.5, 2.0F, 1.5F,
        0.30, 0.05F, 0.50F);
    ASSERT_TRUE(held_expired.valid);
    // Prediction is capped by the formal window, while evidence age continues.
    const float capped_prediction = 0.30F *
        (1.0F - std::exp(-0.50F / 0.30F));
    EXPECT_NEAR(held_expired.center_base.x(),
                1.0F + 0.5F * capped_prediction, 1.0e-5F);
    EXPECT_EQ(held_expired.vertical_source,
              CargoVerticalPoseSource::DISPLAY_FROZEN);
    EXPECT_DOUBLE_EQ(held_expired.evidence_stamp_sec, 10.0);
    const CargoFormalUseDecision expired_use = evaluateCargoFormalUse(
        true, false, 10.6, held_expired.evidence_stamp_sec,
        held_expired.evidence_stamp_sec, 0.5, 0.5,
        held_expired.position_uncertainty_m);
    EXPECT_TRUE(expired_use.display_valid);
    EXPECT_FALSE(expired_use.formal_safety_valid);
    EXPECT_FALSE(expired_use.formal_removal_valid);
}

TEST(CargoRigidGeometryTest, PositionUncertaintyExpandsFormalFootprint) {
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(), pose(Eigen::Vector3f::Zero()),
        Eigen::Isometry3f::Identity(), 11U, 0.20F, 0.10F);
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

TEST(CargoRigidGeometryTest, RepeatedAbnormalObservationsCannotExceedTotalSpeed) {
    CargoLivePoseStepInput input;
    input.previous_center = Eigen::Vector3f::Zero();
    input.previous_velocity = Eigen::Vector3f(20.0F, 0.0F, 20.0F);
    input.measured_center = Eigen::Vector3f(100.0F, 0.0F, 100.0F);
    input.sensor_dt_sec = 0.10;
    input.center_alpha = 1.0F;
    input.velocity_alpha = 1.0F;
    input.max_xy_speed_mps = 2.0F;
    input.max_z_speed_mps = 1.5F;
    input.step_margin_m = 0.05F;

    const CargoLivePoseStepResult first = updateCargoLivePoseStep(input);
    ASSERT_TRUE(first.valid);
    EXPECT_LE(first.filtered_center.head<2>().norm(), 0.25F + 1.0e-5F);
    EXPECT_LE(std::abs(first.filtered_center.z()), 0.20F + 1.0e-5F);
    EXPECT_LE(first.filtered_velocity.head<2>().norm(), 2.0F + 1.0e-5F);
    EXPECT_LE(std::abs(first.filtered_velocity.z()), 1.5F + 1.0e-5F);

    input.previous_center = first.filtered_center;
    input.previous_velocity = first.filtered_velocity;
    const CargoLivePoseStepResult second = updateCargoLivePoseStep(input);
    ASSERT_TRUE(second.valid);
    const Eigen::Vector3f second_step =
        second.filtered_center - first.filtered_center;
    EXPECT_LE(second_step.head<2>().norm(), 0.25F + 1.0e-5F);
    EXPECT_LE(std::abs(second_step.z()), 0.20F + 1.0e-5F);
    EXPECT_LE(second.filtered_velocity.head<2>().norm(), 2.0F + 1.0e-5F);
    EXPECT_LE(std::abs(second.filtered_velocity.z()), 1.5F + 1.0e-5F);
}

TEST(CargoRigidGeometryTest, TrackingResidualRemainsAfterBoundedCorrection) {
    CargoLivePoseStepInput input;
    input.previous_center = Eigen::Vector3f::Zero();
    input.measured_center = Eigen::Vector3f(1.0F, 0.0F, 0.8F);
    input.sensor_dt_sec = 0.10;
    input.center_alpha = 0.5F;
    input.velocity_alpha = 0.3F;
    input.max_xy_speed_mps = 2.0F;
    input.max_z_speed_mps = 1.5F;
    input.step_margin_m = 0.05F;
    const CargoLivePoseStepResult result = updateCargoLivePoseStep(input);
    ASSERT_TRUE(result.valid);
    EXPECT_GT(result.tracking_residual.head<2>().norm(), 0.70F);
    EXPECT_GT(std::abs(result.tracking_residual.z()), 0.60F);
    EXPECT_TRUE((result.tracking_residual -
                 (input.measured_center - result.filtered_center))
                    .isMuchSmallerThan(1.0F));
}

TEST(CargoRigidGeometryTest, ShapeStaysFixedWhilePoseMovesAndHoists) {
    const LockedCargoShape locked = shape(0.4F);
    const RigidCargoGeometry first = buildCurrentRigidCargoGeometry(
        locked, pose(Eigen::Vector3f(0.0F, 0.0F, 1.0F)),
        Eigen::Isometry3f::Identity(), 7U, 0.05F, 0.08F);
    const RigidCargoGeometry moved = buildCurrentRigidCargoGeometry(
        locked, pose(Eigen::Vector3f(0.6F, -0.4F, 2.0F)),
        Eigen::Isometry3f::Identity(), 7U, 0.08F, 0.12F);
    ASSERT_TRUE(first.valid);
    ASSERT_TRUE(moved.valid);
    EXPECT_FLOAT_EQ(first.shape.length_m, moved.shape.length_m);
    EXPECT_FLOAT_EQ(first.shape.width_m, moved.shape.width_m);
    EXPECT_FLOAT_EQ(first.shape.height_m, moved.shape.height_m);
    EXPECT_FLOAT_EQ(first.shape.yaw_base_rad, moved.shape.yaw_base_rad);
    EXPECT_NEAR(moved.bottom_z_base - first.bottom_z_base, 1.0F, 1.0e-5F);
    EXPECT_NEAR(moved.top_z_base - first.top_z_base, 1.0F, 1.0e-5F);
}

TEST(CargoRigidGeometryTest, NormalHoistUpdatesCenterWithoutChangingHeight) {
    const LockedCargoShape locked = shape(0.2F);
    const RigidCargoGeometry low = buildCurrentRigidCargoGeometry(
        locked, pose(Eigen::Vector3f(0.0F, 0.0F, 1.2F)),
        Eigen::Isometry3f::Identity(), 13U, 0.05F, 0.08F);
    const RigidCargoGeometry high = buildCurrentRigidCargoGeometry(
        locked, pose(Eigen::Vector3f(0.0F, 0.0F, 2.2F)),
        Eigen::Isometry3f::Identity(), 13U, 0.05F, 0.08F);
    ASSERT_TRUE(low.valid);
    ASSERT_TRUE(high.valid);
    EXPECT_FLOAT_EQ(low.shape.height_m, high.shape.height_m);
    EXPECT_NEAR(high.pose.center_base.z() - low.pose.center_base.z(),
                1.0F, 1.0e-5F);
}

TEST(CargoRigidGeometryTest, XyEvidenceCannotRefreshVerticalAuthority) {
    const CargoFormalUseDecision decision = evaluateCargoFormalUse(
        true, false, 30.0, 29.7, 20.0, 0.60, 0.60, 0.12F);
    EXPECT_FALSE(decision.formal_safety_valid);
    EXPECT_LT(decision.pose_age_sec, 0.60);
    EXPECT_GT(decision.height_age_sec, 9.0);
    EXPECT_EQ(decision.reason, "formal_vertical_evidence_expired");
}

TEST(CargoRigidGeometryTest, LostDisplayPredictionStopsAfterFormalHold) {
    LiveCargoPose observed = pose(Eigen::Vector3f::Zero());
    const Eigen::Vector3f velocity(1.0F, 0.0F, 0.0F);
    const LiveCargoPose first = propagateHeldCargoPose(
        observed, velocity, 10.5, 0.5, 2.0F, 1.5F,
        0.30, 0.05F, 0.50F);
    const LiveCargoPose later = propagateHeldCargoPose(
        observed, velocity, 17.0, 0.5, 2.0F, 1.5F,
        0.30, 0.05F, 0.50F);
    ASSERT_TRUE(first.valid);
    ASSERT_TRUE(later.valid);
    EXPECT_NEAR(first.center_base.x(), later.center_base.x(), 1.0e-5F);
    EXPECT_EQ(later.vertical_source,
              CargoVerticalPoseSource::DISPLAY_FROZEN);
}

TEST(CargoRigidGeometryTest, ContainsAndDistanceUseOrientedCoordinates) {
    constexpr float kQuarterTurn = 1.57079632679489661923F;
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(kQuarterTurn), pose(Eigen::Vector3f(0.0F, 0.0F, 1.0F)),
        Eigen::Isometry3f::Identity(), 9U, 0.05F, 0.08F);
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

TEST(CargoRigidGeometryTest, TrackingLagCargoPointsRemainSelfRemoved) {
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(), pose(Eigen::Vector3f(0.2F, 0.0F, 1.0F)),
        Eigen::Isometry3f::Identity(), 21U, 0.20F, 0.08F);
    ASSERT_TRUE(geometry.valid);
    const CargoObbFootprint footprint = toCargoObbFootprint(geometry);
    EXPECT_TRUE(containsPointInCargoObbBase(
        Eigen::Vector3f(-0.90F, 0.0F, 1.0F), footprint, 0.15F, 0.12F));
}

TEST(CargoRigidGeometryTest, SweptCargoPointsRemainSelfRemoved) {
    CargoObbFootprint start;
    start.valid = true;
    start.center_base = Eigen::Vector2f(0.0F, 0.0F);
    start.length_m = 1.0F;
    start.width_m = 0.5F;
    start.yaw_base_rad = 0.0F;
    start.min_z = 0.5F;
    start.max_z = 1.5F;
    CargoObbFootprint finish = start;
    finish.center_base.x() = 1.0F;
    EXPECT_TRUE(containsPointInSweptCargoObbBase(
        Eigen::Vector3f(0.5F, 0.0F, 1.0F), start, finish, 0.05F, 0.05F));
}

TEST(CargoRigidGeometryTest, RealObstacleNearCargoEdgeRemainsExternal) {
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(), pose(Eigen::Vector3f(0.0F, 0.0F, 1.0F)),
        Eigen::Isometry3f::Identity(), 22U, 0.10F, 0.08F);
    ASSERT_TRUE(geometry.valid);
    const CargoObbFootprint footprint = toCargoObbFootprint(geometry);
    EXPECT_FALSE(containsPointInCargoObbBase(
        Eigen::Vector3f(1.45F, 0.0F, 1.0F), footprint, 0.15F, 0.12F));
}

TEST(CargoRigidGeometryTest, VoxelScaleIdentityMatchRequiresCargoNeighborhood) {
    // A 12 cm centroid shift is normal for independently voxelized 15 cm
    // input frames and must remain the same physical cargo identity.
    EXPECT_TRUE(isCargoIdentityPointMatch(
        0.12F * 0.12F, 0.15F, true));
    EXPECT_FALSE(isCargoIdentityPointMatch(
        0.12F * 0.12F, 0.15F, false));
    EXPECT_FALSE(isCargoIdentityPointMatch(
        0.16F * 0.16F, 0.15F, true));
}

TEST(CargoRigidGeometryTest, IdentityShellRecoversSurfaceWithoutBroadMask) {
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(), pose(Eigen::Vector3f(0.0F, 0.0F, 1.0F)),
        Eigen::Isometry3f::Identity(), 25U, 0.05F, 0.08F);
    ASSERT_TRUE(geometry.valid);
    const CargoObbFootprint footprint = toCargoObbFootprint(geometry);
    const Eigen::Vector3f shifted_surface(
        0.0F, 0.0F, footprint.min_z - 0.20F);
    EXPECT_FALSE(containsPointInCargoObbBase(
        shifted_surface, footprint, 0.15F, 0.12F));
    EXPECT_TRUE(containsPointInCargoObbBase(
        shifted_surface, footprint, 0.30F, 0.27F));
    EXPECT_TRUE(isCargoIdentityPointMatch(
        0.12F * 0.12F, 0.15F, true));

    const Eigen::Vector3f real_obstacle(
        0.0F, 0.0F, footprint.min_z - 0.50F);
    EXPECT_FALSE(containsPointInCargoObbBase(
        real_obstacle, footprint, 0.30F, 0.27F));
    EXPECT_FALSE(isCargoIdentityPointMatch(
        0.12F * 0.12F, 0.15F, false));
}

TEST(CargoRigidGeometryTest, HorizontalResidualDoesNotInflateBottomUncertainty) {
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(), pose(Eigen::Vector3f(0.0F, 0.0F, 1.0F)),
        Eigen::Isometry3f::Identity(), 23U, 0.40F, 0.06F);
    ASSERT_TRUE(geometry.valid);
    EXPECT_FLOAT_EQ(geometry.horizontal_uncertainty_m, 0.40F);
    EXPECT_FLOAT_EQ(geometry.vertical_uncertainty_m, 0.06F);
}

TEST(CargoRigidGeometryTest, VerticalResidualDoesNotExpandHorizontalFootprint) {
    const RigidCargoGeometry geometry = buildCurrentRigidCargoGeometry(
        shape(), pose(Eigen::Vector3f(0.0F, 0.0F, 1.0F)),
        Eigen::Isometry3f::Identity(), 24U, 0.05F, 0.40F);
    ASSERT_TRUE(geometry.valid);
    const CargoObbFootprint footprint = toCargoObbFootprint(
        geometry, geometry.horizontal_uncertainty_m);
    EXPECT_NEAR(footprint.length_m, 2.10F, 1.0e-5F);
    EXPECT_NEAR(footprint.width_m, 0.90F, 1.0e-5F);
}

}  // namespace
}  // namespace ndt_slam
