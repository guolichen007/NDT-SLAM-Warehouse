#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ndt_slam {

enum class CargoContractSource : std::uint8_t {
  UNKNOWN = 0,
  CURRENT_ASSOCIATED_LIDAR,
  MOTION_PREDICTION,
  FRESH_FORMAL_HOLD,
  CONFIGURED_PREVIEW,
};

enum class CargoVerticalAuthority : std::uint8_t {
  INVALID = 0,
  DIRECT_BOTTOM,
  SUPPORTED_TOP_MINUS_FROZEN_HEIGHT,
  FRESH_HELD_FORMAL,
};

struct CargoTrackSnapshot {
  double source_stamp_sec = 0.0;
  double evaluation_stamp_sec = 0.0;
  std::string frame_id = "base_link";
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_track_id = 0U;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f uncertainty = Eigen::Vector3f::Zero();
  CargoContractSource source = CargoContractSource::UNKNOWN;
  bool valid = false;
  bool fresh = false;
};

struct CargoGeometryEstimate {
  double source_stamp_sec = 0.0;
  double evaluation_stamp_sec = 0.0;
  std::string frame_id = "base_link";
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_track_id = 0U;
  float length_m = 0.0F;
  float width_m = 0.0F;
  float yaw_rad = 0.0F;
  float top_z_m = std::numeric_limits<float>::quiet_NaN();
  float bottom_z_m = std::numeric_limits<float>::quiet_NaN();
  float height_m = std::numeric_limits<float>::quiet_NaN();
  Eigen::Vector3f uncertainty = Eigen::Vector3f::Zero();
  CargoContractSource source = CargoContractSource::UNKNOWN;
  CargoVerticalAuthority vertical_authority =
      CargoVerticalAuthority::INVALID;
  bool horizontal_valid = false;
  bool vertical_valid = false;
  bool fresh = false;
};

struct CargoSafetyEnvelope {
  double source_stamp_sec = 0.0;
  double evaluation_stamp_sec = 0.0;
  std::string frame_id = "base_link";
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_track_id = 0U;
  Eigen::Vector2f center = Eigen::Vector2f::Zero();
  float nominal_length_m = 0.0F;
  float nominal_width_m = 0.0F;
  float conservative_length_m = 0.0F;
  float conservative_width_m = 0.0F;
  float yaw_rad = 0.0F;
  float safe_bottom_z_m = std::numeric_limits<float>::quiet_NaN();
  float horizontal_uncertainty_m = 0.0F;
  float vertical_uncertainty_m = 0.0F;
  CargoContractSource source = CargoContractSource::UNKNOWN;
  bool horizontal_valid = false;
  bool vertical_valid = false;
  bool formal = false;
  bool fresh = false;
};

struct ObstacleObservation {
  double source_stamp_sec = 0.0;
  std::uint64_t source_sequence = 0U;
  std::string frame_id = "base_link";
  std::uint64_t cargo_lifecycle_id = 0U;
  std::size_t source_index = 0U;
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  float footprint_distance_m = std::numeric_limits<float>::infinity();
  float top_z95_m = std::numeric_limits<float>::quiet_NaN();
  float bottom_z05_m = std::numeric_limits<float>::quiet_NaN();
  float horizontal_uncertainty_m = 0.0F;
  float vertical_uncertainty_m = 0.0F;
  bool physical_valid = false;
  bool hazard_geometry_valid = false;
  bool source_validated = false;
};

struct ObstacleTrackSnapshot {
  double source_stamp_sec = 0.0;
  std::string frame_id = "map";
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t physical_track_id = 0U;
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
  int confirmation_count = 0;
  int far_history_count = 0;
  double far_history_duration_sec = 0.0;
  bool far_history_valid = false;
  bool provenance_valid = false;
  bool authority_ambiguous = false;
  bool fresh = false;
};

struct HazardAssessment {
  double source_stamp_sec = 0.0;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_track_id = 0U;
  std::uint64_t obstacle_track_id = 0U;
  float footprint_distance_m = std::numeric_limits<float>::infinity();
  float obstacle_top_z_m = std::numeric_limits<float>::quiet_NaN();
  float safe_bottom_z_m = std::numeric_limits<float>::quiet_NaN();
  float conservative_clearance_m = std::numeric_limits<float>::quiet_NaN();
  float combined_uncertainty_m = std::numeric_limits<float>::quiet_NaN();
  bool valid = false;
};

struct AvoidanceDecision {
  double source_stamp_sec = 0.0;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_track_id = 0U;
  std::uint64_t obstacle_track_id = 0U;
  std::uint16_t code = 0U;
  std::uint32_t fault_mask = 0U;
  bool valid = false;
  std::string reason = "not_decided";
};

}  // namespace ndt_slam
