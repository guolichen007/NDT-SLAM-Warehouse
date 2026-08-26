#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>

#include "ndt_slam/cargo_config_validation.hpp"
#include "ndt_slam/pose_authority_identity.hpp"

namespace ndt_slam {

struct CargoSafetyTemporalConfig {
  int hazard_confirm_frames = 3;
  int clear_confirm_frames = 2;
  std::size_t minimum_hazard_cluster_points = 20U;
  double maximum_evidence_gap_sec = 0.60;
  float maximum_centroid_step_m = 0.75F;
  float maximum_distance_step_m = 0.75F;
  float maximum_clearance_step_m = 0.75F;
};

struct CargoSafetyTemporalInput {
  double stamp_sec = 0.0;
  bool raw_valid = false;
  std::uint16_t raw_code = 0U;
  std::size_t cluster_points = 0U;
  Eigen::Vector3f cluster_centroid = Eigen::Vector3f::Zero();
  float footprint_distance_m = 0.0F;
  float conservative_clearance_m = 0.0F;
  TemporalEvidenceAuthority cargo_pose_authority;
  TemporalEvidenceAuthority obstacle_pose_authority;
};

struct CargoSafetyTemporalDecision {
  bool stable = false;
  bool pending = false;
  bool newly_confirmed = false;
  bool use_current_evidence = false;
  std::uint16_t code = 0U;
  std::uint16_t candidate_code = 0U;
  std::uint16_t confirmed_code = 0U;
  int evidence_count = 0;
  TemporalEvidenceAuthority cargo_pose_authority;
  TemporalEvidenceAuthority obstacle_pose_authority;
  std::string reason = "not_evaluated";
};

/**
 * Confirms 17/18 only from spatially continuous, independently stamped
 * clusters. Sparse or jumping returns remain fail-safe pending evidence (34
 * at the integration layer) instead of becoming an immediate collision
 * alarm. Every published 14/17/18 is backed by the current confirmation
 * window. While a new CLEAR, alarm level, or cluster identity is still being
 * confirmed, integration publishes evidence-invalid (34), never a stale
 * previously confirmed alarm.
 */
class CargoSafetyTemporalFilter {
 public:
  explicit CargoSafetyTemporalFilter(
      const CargoSafetyTemporalConfig& config = CargoSafetyTemporalConfig());

  CargoConfigValidationResult setConfig(
      const CargoSafetyTemporalConfig& config);
  const CargoConfigValidationResult& configValidation() const noexcept {
    return config_validation_;
  }
  const CargoSafetyTemporalConfig& config() const noexcept { return config_; }
  void reset();
  CargoSafetyTemporalDecision update(
      const CargoSafetyTemporalInput& input);

 private:
  CargoSafetyTemporalConfig config_;
  CargoConfigValidationResult config_validation_;
  bool has_source_stamp_ = false;
  double last_source_stamp_sec_ = 0.0;

  bool candidate_valid_ = false;
  std::uint16_t candidate_code_ = 0U;
  int candidate_count_ = 0;
  double candidate_stamp_sec_ = 0.0;
  Eigen::Vector3f candidate_centroid_ = Eigen::Vector3f::Zero();
  float candidate_distance_m_ = 0.0F;
  float candidate_clearance_m_ = 0.0F;
  TemporalEvidenceAuthority candidate_cargo_pose_authority_;
  TemporalEvidenceAuthority candidate_obstacle_pose_authority_;

  bool confirmed_valid_ = false;
  std::uint16_t confirmed_code_ = 0U;
  TemporalEvidenceAuthority confirmed_cargo_pose_authority_;
  TemporalEvidenceAuthority confirmed_obstacle_pose_authority_;

  CargoSafetyTemporalDecision currentDecision(
      const std::string& reason) const;
  CargoSafetyTemporalDecision pendingDecision(
      const std::string& reason) const;
};

CargoConfigValidationResult validateCargoSafetyTemporalConfig(
    const CargoSafetyTemporalConfig& config);

}  // namespace ndt_slam
