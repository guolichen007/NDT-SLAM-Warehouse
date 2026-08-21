#pragma once

#include "ndt_slam/hook_load_evidence_policy.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ndt_slam {

enum class CargoCandidateAssociationState : std::uint8_t {
  MATCHED = 0,
  AMBIGUOUS,
  NEW_HISTORY,
};

enum class CargoPhysicalIdentityState : std::uint8_t {
  UNKNOWN = 0,
  AMBIGUOUS,
  VALIDATED,
};

enum class CargoLiftBaselineSource : std::uint8_t {
  PRE_LOAD_FROZEN_BASELINE = 0,
  POST_LOAD_FIRST_FRESH_OBSERVATION,
  UNAVAILABLE_STARTED_LOADED,
};

enum class CargoExistenceSource : std::uint8_t {
  NONE = 0,
  GRAVITY_LOADED,
  STRICT_LIDAR,
};

const char* cargoCandidateAssociationStateName(
    CargoCandidateAssociationState state) noexcept;
const char* cargoPhysicalIdentityStateName(
    CargoPhysicalIdentityState state) noexcept;
const char* cargoLiftBaselineSourceName(
    CargoLiftBaselineSource source) noexcept;
const char* cargoExistenceSourceName(CargoExistenceSource source) noexcept;

struct CargoPhysicalIdentityConfig {
  double maximum_xy_step_m = 0.30;
  double maximum_z_speed_mps = 1.50;
  double z_step_margin_m = 0.05;
  double maximum_size_relative_step = 0.60;
  double ambiguity_cost_margin = 0.08;
  double minimum_significant_change_m = 0.15;
  double significance_sigma = 3.0;
  double maximum_observation_gap_sec = 0.50;
  double maximum_source_age_sec = 0.50;
  int lift_confirm_frames = 4;
};

// A geometry hypothesis is exported before product top-1 selection. Product
// rank and prediction fields deliberately do not exist in this contract.
struct CargoPhysicalCandidateObservation {
  std::uint64_t candidate_id = 0U;
  std::vector<std::uint64_t> member_component_ids;
  double stamp_sec = 0.0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  double yaw_rad = 0.0;
  double z05 = std::numeric_limits<double>::quiet_NaN();
  double z50 = std::numeric_limits<double>::quiet_NaN();
  double z95 = std::numeric_limits<double>::quiet_NaN();
  double vertical_uncertainty_m = 0.20;
  std::size_t point_support = 0U;
};

struct CargoPhysicalGroupObservation {
  std::uint64_t frame_group_id = 0U;
  std::vector<std::uint64_t> member_component_ids;
  std::vector<CargoPhysicalCandidateObservation> hypotheses;
  bool geometry_resolved = false;
  bool group_ambiguous = false;
  CargoPhysicalCandidateObservation representative;
};

// Exact member sets form one physical group. Partial overlap/containment is
// retained as ambiguity; it is never counted as an independent second cargo.
std::vector<CargoPhysicalGroupObservation> groupCargoPhysicalCandidates(
    const std::vector<CargoPhysicalCandidateObservation>& candidates,
    double equivalent_center_tolerance_m,
    double equivalent_size_relative_tolerance);

struct CargoPhysicalIdentityInput {
  double pipeline_stamp_sec = 0.0;
  std::uint64_t lifecycle_id = 0U;
  bool rearm = false;
  bool node_started_loaded = false;
  HookLoadSignalRole hook_role = HookLoadSignalRole::REQUIRED;
  bool gravity_valid = false;
  HookLoadState gravity_state = HookLoadState::UNKNOWN;
  std::vector<CargoPhysicalGroupObservation> groups;
};

struct CargoPhysicalIdentityDecision {
  bool valid_input = false;
  bool cargo_exists = false;
  CargoExistenceSource existence_source = CargoExistenceSource::NONE;
  CargoCandidateAssociationState association =
      CargoCandidateAssociationState::NEW_HISTORY;
  CargoPhysicalIdentityState identity = CargoPhysicalIdentityState::UNKNOWN;
  CargoLiftBaselineSource baseline_source =
      CargoLiftBaselineSource::POST_LOAD_FIRST_FRESH_OBSERVATION;
  std::uint64_t load_epoch = 0U;
  std::uint64_t physical_history_id = 0U;
  std::uint64_t frame_group_id = 0U;
  std::uint64_t resolved_candidate_id = 0U;
  bool geometry_resolved = false;
  bool current_candidate_fresh = false;
  bool lift_confirmed = false;
  int lift_confirm_count = 0;
  int required_lift_confirm_frames = 0;
  double baseline_z95 = std::numeric_limits<double>::quiet_NaN();
  double current_z95 = std::numeric_limits<double>::quiet_NaN();
  double lift_delta_m = std::numeric_limits<double>::quiet_NaN();
  double lift_threshold_m = std::numeric_limits<double>::quiet_NaN();
  double evidence_age_sec = std::numeric_limits<double>::quiet_NaN();
  double identity_validation_stamp_sec = 0.0;
  std::string reason = "uninitialized";
};

class CargoPhysicalIdentityAuthority {
 public:
  explicit CargoPhysicalIdentityAuthority(
      const CargoPhysicalIdentityConfig& config =
          CargoPhysicalIdentityConfig{});

  void setConfig(const CargoPhysicalIdentityConfig& config);
  void reset(const std::string& reason = "explicit_reset");
  CargoPhysicalIdentityDecision update(
      const CargoPhysicalIdentityInput& input);
  const CargoPhysicalIdentityDecision& decision() const noexcept {
    return decision_;
  }

 private:
  struct History {
    std::uint64_t id = 0U;
    CargoPhysicalGroupObservation last_group;
    double last_stamp_sec = 0.0;
    bool has_preload = false;
    double preload_z95 = std::numeric_limits<double>::quiet_NaN();
    double preload_uncertainty_m = 0.20;
    double preload_stamp_sec = 0.0;
    bool baseline_frozen = false;
    CargoLiftBaselineSource baseline_source =
        CargoLiftBaselineSource::POST_LOAD_FIRST_FRESH_OBSERVATION;
    double baseline_z95 = std::numeric_limits<double>::quiet_NaN();
    double baseline_uncertainty_m = 0.20;
    double baseline_stamp_sec = 0.0;
    double last_consumed_evidence_stamp_sec = 0.0;
    int lift_confirm_count = 0;
    bool lift_confirmed = false;
    double validation_stamp_sec = 0.0;
    bool association_ambiguous = false;
  };

  CargoPhysicalIdentityConfig config_;
  std::vector<History> histories_;
  CargoPhysicalIdentityDecision decision_;
  std::uint64_t next_history_id_ = 1U;
  std::uint64_t load_epoch_ = 0U;
  std::uint64_t lifecycle_id_ = 0U;
  std::uint64_t validated_history_id_ = 0U;
  bool initialized_ = false;
  bool previous_existence_phase_ = false;
  bool started_loaded_without_baseline_ = false;
  double last_pipeline_stamp_sec_ = 0.0;
  std::string reset_reason_ = "constructed";
};

}  // namespace ndt_slam
