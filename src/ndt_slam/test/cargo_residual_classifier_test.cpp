#include <gtest/gtest.h>

#include "ndt_slam/cargo_residual_classifier.hpp"

namespace ndt_slam {
namespace {

TEST(CargoResidualClassifier, NonzeroClusterIsExternal) {
  CargoResidualClassifierInput input;
  input.footprint_distance_m = 0.20F;
  const auto decision = classifyCargoResidual(
      CargoResidualClassifierConfig{}, input);
  EXPECT_TRUE(decision.source_validated);
  EXPECT_EQ(decision.classification, CargoResidualClass::EXTERNAL_OBSTACLE);
}

TEST(CargoResidualClassifier, IdentityAloneCannotHideObstacle) {
  CargoResidualClassifierInput input;
  input.footprint_distance_m = 0.0F;
  input.inside_xy_ratio = 1.0F;
  input.identity_match_ratio = 0.90F;
  input.surface_band_ratio = 0.90F;
  input.moves_with_cargo_score = 0.20F;
  const auto decision = classifyCargoResidual(
      CargoResidualClassifierConfig{}, input);
  EXPECT_FALSE(decision.source_validated);
  EXPECT_EQ(decision.classification, CargoResidualClass::UNKNOWN);
}

TEST(CargoResidualClassifier, IdentityAndMotionClassifyCargoSelf) {
  CargoResidualClassifierInput input;
  input.footprint_distance_m = 0.0F;
  input.inside_xy_ratio = 0.90F;
  input.identity_match_ratio = 0.80F;
  input.surface_band_ratio = 0.80F;
  input.moves_with_cargo_score = 0.90F;
  const auto decision = classifyCargoResidual(
      CargoResidualClassifierConfig{}, input);
  EXPECT_TRUE(decision.source_validated);
  EXPECT_EQ(decision.classification, CargoResidualClass::CARGO_SELF);
}

TEST(CargoResidualClassifier, StaticTrackRemainsExternalBelowCargo) {
  CargoResidualClassifierInput input;
  input.footprint_distance_m = 0.0F;
  input.inside_xy_ratio = 1.0F;
  input.identity_match_ratio = 0.80F;
  input.surface_band_ratio = 0.80F;
  input.moves_with_cargo_score = 0.10F;
  input.confirmed_static_track_match = true;
  const auto decision = classifyCargoResidual(
      CargoResidualClassifierConfig{}, input);
  EXPECT_TRUE(decision.source_validated);
  EXPECT_EQ(decision.classification, CargoResidualClass::EXTERNAL_OBSTACLE);
}

}  // namespace
}  // namespace ndt_slam
