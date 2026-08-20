#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace ndt_slam {

struct CargoVerticalEvidenceConfig {
  float surface_band_height_m = 0.10F;
  float xy_cell_size_m = 0.10F;
  std::size_t minimum_surface_points = 8U;
  std::size_t minimum_surface_cells = 2U;
  float minimum_surface_coverage_ratio = 0.02F;
  float footprint_margin_m = 0.12F;
  float thickness_slab_margin_m = 0.10F;
  float ground_hag_min_m = 0.15F;
  float ground_hag_max_m = 2.50F;
};

struct CargoVerticalEvidenceInput {
  std::vector<Eigen::Vector3f> selected_points_base;

  bool footprint_valid = false;
  Eigen::Vector2f footprint_center_base = Eigen::Vector2f::Zero();
  Eigen::Vector2f footprint_size_xy = Eigen::Vector2f::Zero();
  float footprint_yaw_base_rad = 0.0F;

  bool ground_reference_valid = false;
  float ground_z_base = std::numeric_limits<float>::quiet_NaN();

  bool frozen_thickness_valid = false;
  bool frozen_thickness_matches_lifecycle = false;
  float frozen_thickness_m = std::numeric_limits<float>::quiet_NaN();
};

struct CargoVerticalEvidence {
  bool valid = false;
  float top_z_base = std::numeric_limits<float>::quiet_NaN();
  std::vector<Eigen::Vector3f> clean_vertical_points_base;
  std::size_t footprint_points = 0U;
  std::size_t top_support_points = 0U;
  std::size_t top_surface_cells = 0U;
  float top_surface_coverage = 0.0F;
  std::size_t removed_low_points = 0U;
  bool ground_filter_used = false;
  bool thickness_slab_used = false;
  std::string reject_reason = "not_evaluated";
};

// Stateless SHADOW extractor. It does not select a cargo identity and never
// manufactures a ground height or a top surface from frozen thickness.
CargoVerticalEvidence extractCargoVerticalEvidence(
    const CargoVerticalEvidenceInput& input,
    const CargoVerticalEvidenceConfig& config);

}  // namespace ndt_slam
