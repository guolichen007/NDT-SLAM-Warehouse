#include <gtest/gtest.h>

#include <cmath>

#include "ndt_slam/cargo_vertical_evidence.hpp"

namespace ndt_slam {
namespace {

CargoVerticalEvidenceInput baseInput() {
  CargoVerticalEvidenceInput input;
  input.footprint_valid = true;
  input.footprint_center_base = Eigen::Vector2f::Zero();
  input.footprint_size_xy = Eigen::Vector2f(1.0F, 1.0F);
  return input;
}

CargoVerticalEvidenceConfig testConfig() {
  CargoVerticalEvidenceConfig config;
  config.surface_band_height_m = 0.10F;
  config.xy_cell_size_m = 0.20F;
  config.minimum_surface_points = 6U;
  config.minimum_surface_cells = 4U;
  config.minimum_surface_coverage_ratio = 0.10F;
  config.footprint_margin_m = 0.0F;
  config.thickness_slab_margin_m = 0.05F;
  return config;
}

void addSurface(std::vector<Eigen::Vector3f>* points, float z) {
  for (float x : {-0.30F, 0.0F, 0.30F}) {
    for (float y : {-0.30F, 0.0F, 0.30F}) {
      points->emplace_back(x, y, z + 0.005F * x);
    }
  }
}

TEST(CargoVerticalEvidence, LowGroundContaminationCannotPullTopDown) {
  auto input = baseInput();
  addSurface(&input.selected_points_base, 1.40F);
  for (int i = 0; i < 40; ++i) {
    input.selected_points_base.emplace_back(
        -0.45F + 0.02F * static_cast<float>(i % 10),
        -0.45F + 0.02F * static_cast<float>(i / 10), 0.05F);
  }
  input.frozen_thickness_valid = true;
  input.frozen_thickness_matches_lifecycle = true;
  input.frozen_thickness_m = 0.40F;

  const auto result = extractCargoVerticalEvidence(input, testConfig());
  ASSERT_TRUE(result.valid) << result.reject_reason;
  EXPECT_NEAR(result.top_z_base, 1.40F, 0.02F);
  EXPECT_EQ(result.removed_low_points, 40U);
  for (const auto& point : result.clean_vertical_points_base) {
    EXPECT_GT(point.z(), 0.90F);
  }
}

TEST(CargoVerticalEvidence, SupportedUpperSurfaceSelected) {
  auto input = baseInput();
  addSurface(&input.selected_points_base, 1.25F);
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  ASSERT_TRUE(result.valid) << result.reject_reason;
  EXPECT_GE(result.top_support_points, 9U);
  EXPECT_GE(result.top_surface_cells, 4U);
  EXPECT_NEAR(result.top_z_base, 1.25F, 0.02F);
}

TEST(CargoVerticalEvidence, SparseHighOutlierCannotBecomeTop) {
  auto input = baseInput();
  addSurface(&input.selected_points_base, 1.30F);
  input.selected_points_base.emplace_back(0.01F, 0.01F, 2.20F);
  input.selected_points_base.emplace_back(0.02F, 0.01F, 2.18F);
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  ASSERT_TRUE(result.valid) << result.reject_reason;
  EXPECT_NEAR(result.top_z_base, 1.30F, 0.02F);
}

TEST(CargoVerticalEvidence, HookLikeSparseHighPointsRejected) {
  auto input = baseInput();
  for (int i = 0; i < 12; ++i) {
    input.selected_points_base.emplace_back(
        0.01F * static_cast<float>(i % 2),
        0.01F * static_cast<float>(i / 2), 2.0F);
  }
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, "no_supported_upper_surface");
}

TEST(CargoVerticalEvidence, GroundReferenceOptionalWhenUpperSurfaceStrong) {
  auto input = baseInput();
  input.ground_reference_valid = false;
  input.ground_z_base = std::numeric_limits<float>::quiet_NaN();
  addSurface(&input.selected_points_base, 1.50F);
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  EXPECT_TRUE(result.valid) << result.reject_reason;
  EXPECT_FALSE(result.ground_filter_used);
}

TEST(CargoVerticalEvidence, InvalidGroundDoesNotDefaultToZero) {
  auto input = baseInput();
  input.ground_reference_valid = false;
  input.ground_z_base = 0.0F;
  addSurface(&input.selected_points_base, -0.20F);
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  ASSERT_TRUE(result.valid) << result.reject_reason;
  EXPECT_LT(result.top_z_base, 0.0F);
  EXPECT_FALSE(result.ground_filter_used);
}

TEST(CargoVerticalEvidence, FrozenThicknessBoundsVerticalSlab) {
  auto input = baseInput();
  addSurface(&input.selected_points_base, 1.40F);
  input.selected_points_base.emplace_back(0.20F, 0.20F, 1.05F);
  input.selected_points_base.emplace_back(-0.20F, -0.20F, 0.80F);
  input.frozen_thickness_valid = true;
  input.frozen_thickness_matches_lifecycle = true;
  input.frozen_thickness_m = 0.35F;
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  ASSERT_TRUE(result.valid) << result.reject_reason;
  EXPECT_TRUE(result.thickness_slab_used);
  EXPECT_EQ(result.removed_low_points, 1U);
}

TEST(CargoVerticalEvidence, FrozenThicknessCannotCreateTopWithoutSurface) {
  auto input = baseInput();
  input.selected_points_base.emplace_back(0.0F, 0.0F, 0.1F);
  input.frozen_thickness_valid = true;
  input.frozen_thickness_matches_lifecycle = true;
  input.frozen_thickness_m = 0.40F;
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(std::isfinite(result.top_z_base));
}

TEST(CargoVerticalEvidence, CurrentTopAndPointsUseSameVerticalEvidence) {
  auto input = baseInput();
  addSurface(&input.selected_points_base, 1.40F);
  input.selected_points_base.emplace_back(0.0F, 0.0F, 0.05F);
  input.frozen_thickness_valid = true;
  input.frozen_thickness_matches_lifecycle = true;
  input.frozen_thickness_m = 0.40F;
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  ASSERT_TRUE(result.valid) << result.reject_reason;
  ASSERT_FALSE(result.clean_vertical_points_base.empty());
  for (const auto& point : result.clean_vertical_points_base) {
    EXPECT_GE(point.z(), result.top_z_base - 0.45F - 1.0e-4F);
    EXPECT_LE(point.z(), result.top_z_base + 0.10F + 1.0e-4F);
  }
}

TEST(CargoVerticalEvidence, NoSupportedSurfaceReturnsInvalid) {
  auto input = baseInput();
  input.selected_points_base.emplace_back(-0.4F, -0.4F, 1.0F);
  input.selected_points_base.emplace_back(0.4F, 0.4F, 0.5F);
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, "no_supported_upper_surface");
  EXPECT_TRUE(result.clean_vertical_points_base.empty());
}

TEST(CargoVerticalEvidence, WrongLifecycleThicknessNotUsed) {
  auto input = baseInput();
  addSurface(&input.selected_points_base, 1.30F);
  input.selected_points_base.emplace_back(0.25F, 0.25F, 0.30F);
  input.frozen_thickness_valid = true;
  input.frozen_thickness_matches_lifecycle = false;
  input.frozen_thickness_m = 1.0F;
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  ASSERT_TRUE(result.valid) << result.reject_reason;
  EXPECT_FALSE(result.thickness_slab_used);
  EXPECT_EQ(result.removed_low_points, 1U);
}

TEST(CargoVerticalEvidence, FrameCloudUsesCanonicalExtractorSemantics) {
  auto vector_input = baseInput();
  addSurface(&vector_input.selected_points_base, 1.30F);
  const auto vector_result = extractCargoVerticalEvidence(
      vector_input, testConfig());
  ASSERT_TRUE(vector_result.valid) << vector_result.reject_reason;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (const Eigen::Vector3f& point : vector_input.selected_points_base) {
    pcl::PointXYZ cloud_point;
    cloud_point.x = point.x();
    cloud_point.y = point.y();
    cloud_point.z = point.z();
    cloud->push_back(cloud_point);
  }
  auto cloud_input = baseInput();
  cloud_input.selected_cloud_base = cloud;
  const auto cloud_result = extractCargoVerticalEvidence(
      cloud_input, testConfig());
  ASSERT_TRUE(cloud_result.valid) << cloud_result.reject_reason;
  EXPECT_NEAR(cloud_result.top_z_base, vector_result.top_z_base, 1.0e-6F);
  EXPECT_EQ(cloud_result.top_surface_cell_indices,
            vector_result.top_surface_cell_indices);
}

TEST(CargoVerticalEvidence, PointSourceMustBeUnique) {
  auto input = baseInput();
  addSurface(&input.selected_points_base, 1.30F);
  input.selected_cloud_base.reset(new pcl::PointCloud<pcl::PointXYZ>());
  const auto result = extractCargoVerticalEvidence(input, testConfig());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, "multiple_point_sources");
}

}  // namespace
}  // namespace ndt_slam
