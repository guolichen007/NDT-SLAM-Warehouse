#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ndt_slam {

enum class CargoThicknessSource : std::uint8_t {
  STATIC_ORIGIN_TOP_SUPPORT = 0,
  MAP_DIFF_REVEALED_SUPPORT = 1,
  LIVE_VISIBLE_EXTENT = 2,
  RETIRED_LOCKED_SHAPE = 3,
  CONFIGURED_FALLBACK = 4,
};

const char* cargoThicknessSourceName(CargoThicknessSource source) noexcept;

struct CargoGeometryFusionConfig {
  std::size_t minimum_independent_sources = 2U;
  int minimum_confirm_frames = 5;
  double maximum_observation_gap_sec = 0.50;
  float maximum_source_disagreement_m = 0.25F;
  float maximum_fused_uncertainty_m = 0.20F;
  float minimum_height_m = 0.30F;
  float maximum_height_m = 5.00F;
  float huber_delta_m = 0.20F;
  float configured_bottom_margin_m = 0.10F;
  int conservative_shrink_confirm_frames = 8;
  float maximum_shrink_per_frame_m = 0.03F;
  bool immediate_expand_enabled = true;
  std::size_t minimum_live_dimension_support = 30U;
  float minimum_live_shape_confidence_for_shrink = 0.75F;
  float minimum_conservative_length_m = 0.0F;
  float minimum_conservative_width_m = 0.0F;
};

struct CargoThicknessObservation {
  CargoThicknessSource source =
      CargoThicknessSource::CONFIGURED_FALLBACK;
  float height_m = std::numeric_limits<float>::quiet_NaN();
  float uncertainty_m = std::numeric_limits<float>::quiet_NaN();
  float confidence = 0.0F;
  bool valid = false;
};

struct CargoGeometryFrame {
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t track_segment_id = 0U;
  double stamp_sec = 0.0;
  bool center_valid = false;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  bool footprint_valid = false;
  float length_m = 0.0F;
  float width_m = 0.0F;
  float yaw_rad = 0.0F;
  bool observed_top_valid = false;
  float observed_top_m = std::numeric_limits<float>::quiet_NaN();
  float top_uncertainty_m = 0.0F;
  float tracking_uncertainty_m = 0.0F;
  std::size_t dimension_support_points = 0U;
  float dimension_shape_confidence = 0.0F;
  bool dimension_observation_complete = false;
  float static_length_lower_bound_m = 0.0F;
  float static_width_lower_bound_m = 0.0F;
  std::vector<CargoThicknessObservation> thickness;
};

struct CargoFrozenGeometry {
  bool valid = false;
  bool frozen = false;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t track_segment_id = 0U;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  float length_m = 0.0F;
  float width_m = 0.0F;
  float height_m = 0.0F;
  float yaw_rad = 0.0F;
  float height_uncertainty_m = 1.0F;
  float bottom_m = std::numeric_limits<float>::quiet_NaN();
  float conservative_bottom_m = std::numeric_limits<float>::quiet_NaN();
  int confirm_frames = 0;
  std::size_t independent_sources = 0U;
  std::vector<CargoThicknessSource> accepted_sources;
  std::string reason = "not_initialized";
};

class CargoGeometryFusion {
 public:
  explicit CargoGeometryFusion(
      const CargoGeometryFusionConfig& config = CargoGeometryFusionConfig{});
  void setConfig(const CargoGeometryFusionConfig& config);
  void reset();
  CargoFrozenGeometry update(const CargoGeometryFrame& frame);
  const CargoFrozenGeometry& result() const noexcept { return result_; }

 private:
  CargoGeometryFusionConfig config_;
  CargoFrozenGeometry result_;
  float pending_height_m_ = 0.0F;
  float pending_uncertainty_m_ = 1.0F;
  bool pending_valid_ = false;
  double last_stamp_sec_ = 0.0;
  int shrink_confirm_count_ = 0;
  std::uint64_t shrink_track_segment_id_ = 0U;
};

}  // namespace ndt_slam
