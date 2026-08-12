#include <gtest/gtest.h>

#include "ndt_slam/cargo_subsystem.hpp"

namespace ndt_slam {
namespace {

CargoSubsystemFrameInput validFrame() {
  CargoSubsystemFrameInput input;
  input.source_stamp_sec = 10.0;
  input.evaluation_stamp_sec = 10.1;
  input.cargo_lifecycle_id = 7U;
  input.cargo_track_id = 11U;
  input.envelope.valid = true;
  input.envelope.cargo_lifecycle_id = 7U;
  input.envelope.pose_source =
      CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR;
  input.envelope.center_base = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
  input.envelope.length_m = 4.0F;
  input.envelope.width_m = 2.0F;
  input.envelope.height_m = 1.0F;
  input.envelope.yaw_base_rad = 0.2F;
  input.envelope.bottom_z_base = 2.5F;
  input.envelope.top_z_base = 3.5F;
  input.envelope.horizontal_uncertainty_m = 0.1F;
  input.envelope.vertical_uncertainty_m = 0.2F;
  input.identity_valid = true;
  input.lifecycle_valid = true;
  input.cloud_fresh = true;
  input.vertical_geometry_valid = true;
  input.vertical_authority = CargoVerticalAuthority::DIRECT_BOTTOM;
  input.positive_identity_authorized = true;
  return input;
}

TEST(CargoSubsystem, AtomicallyPublishesIdentityGeometryEnvelopeCapability) {
  CargoSubsystem subsystem;
  const CargoSubsystemSnapshot& snapshot = subsystem.update(validFrame());
  EXPECT_EQ(snapshot.track.cargo_lifecycle_id, 7U);
  EXPECT_EQ(snapshot.geometry.cargo_track_id, 11U);
  EXPECT_EQ(snapshot.envelope.cargo_track_id, 11U);
  EXPECT_TRUE(snapshot.track.valid);
  EXPECT_TRUE(snapshot.geometry.horizontal_valid);
  EXPECT_TRUE(snapshot.geometry.vertical_valid);
  EXPECT_EQ(snapshot.geometry.vertical_authority,
            CargoVerticalAuthority::DIRECT_BOTTOM);
  EXPECT_FLOAT_EQ(snapshot.envelope.conservative_length_m, 4.2F);
  EXPECT_TRUE(snapshot.capability.tracking);
  EXPECT_TRUE(snapshot.capability.positive_warning);
}

TEST(CargoSubsystem, VerticalInvalidDoesNotBlockHorizontalTracking) {
  CargoSubsystem subsystem;
  CargoSubsystemFrameInput input = validFrame();
  input.envelope.valid = false;
  input.envelope.bottom_z_base =
      std::numeric_limits<float>::quiet_NaN();
  input.vertical_geometry_valid = false;
  const CargoSubsystemSnapshot& snapshot = subsystem.update(input);
  EXPECT_TRUE(snapshot.geometry.horizontal_valid);
  EXPECT_FALSE(snapshot.geometry.vertical_valid);
  EXPECT_TRUE(snapshot.capability.perception);
  EXPECT_TRUE(snapshot.capability.tracking);
  EXPECT_FALSE(snapshot.capability.positive_warning);
}

TEST(CargoSubsystem, LifecycleMismatchClosesEveryCapability) {
  CargoSubsystem subsystem;
  CargoSubsystemFrameInput input = validFrame();
  input.lifecycle_valid = false;
  const CargoSubsystemSnapshot& snapshot = subsystem.update(input);
  EXPECT_FALSE(snapshot.track.valid);
  EXPECT_FALSE(snapshot.capability.perception);
  EXPECT_FALSE(snapshot.capability.tracking);
}

TEST(CargoSubsystem, MapsHeldPoseToPredictionWithoutInventingFreshLidar) {
  CargoSubsystem subsystem;
  CargoSubsystemFrameInput input = validFrame();
  input.envelope.pose_source =
      CargoEnvelopePoseSource::HOOK_PLUS_LAST_RELIABLE_OFFSET;
  const CargoSubsystemSnapshot& snapshot = subsystem.update(input);
  EXPECT_EQ(snapshot.track.source, CargoContractSource::MOTION_PREDICTION);
}

}  // namespace
}  // namespace ndt_slam
