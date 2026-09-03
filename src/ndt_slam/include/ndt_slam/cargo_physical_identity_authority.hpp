#pragma once

#include "ndt_slam/cargo_vertical_evidence.hpp"
#include "ndt_slam/cargo_identity_component_lineage.hpp"
#include "ndt_slam/hook_load_evidence_policy.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace ndt_slam {

enum class CargoCandidateAssociationState : std::uint8_t {
  MATCHED = 0,
  AMBIGUOUS,
  NEW_HISTORY,
};

enum class CargoPhysicalAssociationMode : std::uint8_t {
  ANCHOR_CONTINUITY = 0,
  SUPPORT_OVERLAP_CONTINUITY,
  COMPONENT_LINEAGE_CONTINUITY,
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

enum class CargoGroupVerticalMode : std::uint8_t {
  SUPPORTED_EVIDENCE = 0,
  CONTINUITY_ONLY,
  INVALID,
};

enum class CargoVerticalEvidenceSource : std::uint8_t {
  COMPONENT_UNION = 0,
  RAW_ROI_CURRENT_FOOTPRINT,
  RAW_ROI_HISTORY_FOOTPRINT_REACQUIRE,
};

enum class CargoPreLiftReferenceState : std::uint8_t {
  UNSEEN = 0,
  ACQUIRING,
  PAUSED,
  FROZEN,
  CLOSED,
};

enum class CargoLineageRejectStage : std::uint8_t {
  NOT_ATTEMPTED = 0,
  OBSERVATION_CONTRACT,
  EXACT_PATH_WON,
  GROUP_AMBIGUOUS,
  HISTORY_PROVENANCE_NOT_FOUND,
  HISTORY_COMPETITION,
  PAIR_NOT_XY_EXTENT,
  VERTICAL_GATE,
  SELECTED,
};

const char* cargoCandidateAssociationStateName(
    CargoCandidateAssociationState state) noexcept;
const char* cargoPhysicalAssociationModeName(
    CargoPhysicalAssociationMode mode) noexcept;
const char* cargoPhysicalIdentityStateName(
    CargoPhysicalIdentityState state) noexcept;
const char* cargoLiftBaselineSourceName(
    CargoLiftBaselineSource source) noexcept;
const char* cargoExistenceSourceName(CargoExistenceSource source) noexcept;
const char* cargoGroupVerticalModeName(CargoGroupVerticalMode mode) noexcept;
const char* cargoVerticalEvidenceSourceName(
    CargoVerticalEvidenceSource source) noexcept;
const char* cargoPreLiftReferenceStateName(
    CargoPreLiftReferenceState state) noexcept;
const char* cargoLineageRejectStageName(
    CargoLineageRejectStage stage) noexcept;

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

// Component ids are frame-local and are used only to build a physical group
// before association. They are deliberately absent from cross-frame costs.
struct CargoPhysicalComponentObservation {
  std::uint64_t component_id = 0U;
  std::vector<Eigen::Vector3f> points_base;
};

// Frame-owned, immutable SHADOW input. The authority may inspect this cloud
// only during update(); no history or decision type is allowed to retain it.
struct CargoShadowFrameEvidence {
  double source_stamp_sec = 0.0;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr raw_roi_current_frame;
  bool ground_reference_valid = false;
  float ground_z_base = std::numeric_limits<float>::quiet_NaN();
};

struct CargoFootprintSnapshot {
  bool valid = false;
  Eigen::Vector2f center_base = Eigen::Vector2f::Zero();
  Eigen::Vector2f size_xy = Eigen::Vector2f::Zero();
  float yaw_base_rad = 0.0F;
  double source_stamp_sec = 0.0;
};

// This evidence can only complete the Z check for an already unique XY pair.
// Its type deliberately exposes no identity, lift, geometry or safety fields.
struct AssociationOnlyReacquiredVerticalEvidence {
  bool valid = false;
  double source_stamp_sec = 0.0;
  double top_z_base = std::numeric_limits<double>::quiet_NaN();
  double uncertainty_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t owner_overlap_cell_count = 0U;
  double owner_overlap_coverage = 0.0;
  std::string reason = "not_evaluated";
};

struct CargoPhysicalGroupingTelemetry {
  double raw_roi_vertical_total_ms = 0.0;
  std::size_t raw_roi_vertical_hypothesis_count = 0U;
  std::size_t raw_roi_vertical_points_examined = 0U;
};

struct CargoPhysicalGroupDescriptor {
  bool valid = false;
  double stamp_sec = 0.0;
  Eigen::Vector3d stable_anchor = Eigen::Vector3d::Zero();
  Eigen::Vector3d aggregate_extent = Eigen::Vector3d::Zero();
  double robust_x05 = std::numeric_limits<double>::quiet_NaN();
  double robust_x95 = std::numeric_limits<double>::quiet_NaN();
  double robust_y05 = std::numeric_limits<double>::quiet_NaN();
  double robust_y95 = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector2d robust_xy_center = Eigen::Vector2d::Zero();
  Eigen::Vector2d robust_xy_extent = Eigen::Vector2d::Zero();
  std::size_t aggregate_point_support = 0U;
  CargoGroupVerticalMode vertical_mode = CargoGroupVerticalMode::INVALID;
  CargoVerticalEvidenceSource vertical_source =
      CargoVerticalEvidenceSource::COMPONENT_UNION;
  double physical_vertical_z = std::numeric_limits<double>::quiet_NaN();
  double vertical_uncertainty_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t valid_hypothesis_top_count = 0U;
  double hypothesis_top_min = std::numeric_limits<double>::quiet_NaN();
  double hypothesis_top_max = std::numeric_limits<double>::quiet_NaN();
  double hypothesis_top_spread = std::numeric_limits<double>::quiet_NaN();
  std::size_t raw_roi_supported_hypothesis_count = 0U;
  std::size_t owner_proof_rejected_hypothesis_count = 0U;
  std::size_t owner_overlap_cell_count = 0U;
  double owner_overlap_coverage = 0.0;
  bool component_vertical_valid = false;
  double component_vertical_z = std::numeric_limits<double>::quiet_NaN();
  bool raw_roi_vertical_valid = false;
  double raw_roi_vertical_z = std::numeric_limits<double>::quiet_NaN();
  double diagnostic_z05 = std::numeric_limits<double>::quiet_NaN();
  double diagnostic_z95 = std::numeric_limits<double>::quiet_NaN();
  double diagnostic_z_extent = std::numeric_limits<double>::quiet_NaN();
  bool diagnostic_z_extent_reliable = false;
  std::string vertical_reject_reason = "not_evaluated";
};

struct CargoPhysicalGroupObservation {
  std::uint64_t frame_group_id = 0U;
  std::vector<std::uint64_t> member_component_ids;
  // Current-frame physical points assembled directly from the underlying
  // components. Each component contributes once. These points are never used
  // by cross-frame association and are not a geometry-hypothesis point cloud.
  std::vector<Eigen::Vector3f> union_points_base;
  std::vector<CargoPhysicalCandidateObservation> hypotheses;
  bool geometry_resolved = false;
  bool group_ambiguous = false;
  CargoPhysicalCandidateObservation representative;
  CargoPhysicalGroupDescriptor descriptor;
};

// Build frame-local physical groups from the component clouds already owned by
// the detector. Each component contributes points once, regardless of how many
// geometry hypotheses reference it.
std::vector<CargoPhysicalGroupObservation> groupCargoPhysicalCandidates(
    const std::vector<CargoPhysicalCandidateObservation>& candidates,
    const std::vector<CargoPhysicalComponentObservation>& components,
    const CargoShadowFrameEvidence* frame_evidence,
    bool ground_reference_valid,
    double ground_z_base,
    const CargoVerticalEvidenceConfig& vertical_config,
    double equivalent_center_tolerance_m,
    double equivalent_size_relative_tolerance,
    CargoPhysicalGroupingTelemetry* telemetry = nullptr);

struct CargoPhysicalGroupDiagnostic {
  std::uint64_t frame_group_id = 0U;
  std::vector<std::uint64_t> member_component_ids;
  Eigen::Vector3d raw_representative = Eigen::Vector3d::Zero();
  CargoPhysicalGroupDescriptor descriptor;
  std::uint64_t matched_history_id = 0U;
  CargoCandidateAssociationState association =
      CargoCandidateAssociationState::NEW_HISTORY;
  CargoPhysicalAssociationMode association_mode =
      CargoPhysicalAssociationMode::NEW_HISTORY;
  double raw_representative_xy_step_m =
      std::numeric_limits<double>::quiet_NaN();
  double xy_step_m = std::numeric_limits<double>::quiet_NaN();
  double support_xy_separation_m =
      std::numeric_limits<double>::quiet_NaN();
  double z_step_m = std::numeric_limits<double>::quiet_NaN();
  double extent_step = std::numeric_limits<double>::quiet_NaN();
  double xy_cost = std::numeric_limits<double>::quiet_NaN();
  double z_cost = std::numeric_limits<double>::quiet_NaN();
  double extent_cost = std::numeric_limits<double>::quiet_NaN();
  bool reacquired_vertical_attempted = false;
  AssociationOnlyReacquiredVerticalEvidence reacquired_vertical;
  bool lineage_attempted = false;
  bool lineage_rescue_used = false;
  bool lineage_exact_path_won = false;
  bool lineage_ambiguous = false;
  double lineage_xy_before_m = std::numeric_limits<double>::quiet_NaN();
  double lineage_xy_after_m = std::numeric_limits<double>::quiet_NaN();
  double lineage_extent_before = std::numeric_limits<double>::quiet_NaN();
  double lineage_extent_after = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t lineage_previous_component_id = 0U;
  std::uint64_t lineage_current_component_id = 0U;
  double lineage_source_age_sec =
      std::numeric_limits<double>::quiet_NaN();
  std::uint64_t lineage_source_frame_offset = 0U;
  CargoLineageRejectStage lineage_reject_stage =
      CargoLineageRejectStage::NOT_ATTEMPTED;
  std::string association_reject_reason = "NO_HISTORY";
  std::string new_history_reason = "NO_HISTORY";
  bool validated_history_conflict = false;
  std::uint64_t conflicting_history_id = 0U;
  bool frame_has_unrelated_ambiguity = false;
  CargoLiftBaselineSource baseline_source =
      CargoLiftBaselineSource::POST_LOAD_FIRST_FRESH_OBSERVATION;
  CargoPreLiftReferenceState prelift_state =
      CargoPreLiftReferenceState::UNSEEN;
  std::size_t prelift_sample_count = 0U;
  std::uint64_t physical_cargo_epoch_id = 0U;
  double prelift_reference_uncertainty_m =
      std::numeric_limits<double>::quiet_NaN();
  double prelift_reference_first_stamp = 0.0;
  double prelift_reference_last_stamp = 0.0;
  std::string prelift_close_reason = "none";
  double baseline_z = std::numeric_limits<double>::quiet_NaN();
  double lift_delta_m = std::numeric_limits<double>::quiet_NaN();
  double lift_threshold_m = std::numeric_limits<double>::quiet_NaN();
  double last_supported_evidence_stamp = 0.0;
  int lift_confirm_count = 0;
  int lift_confirm_required = 0;
  bool lift_confirmed = false;
  CargoPhysicalIdentityState identity = CargoPhysicalIdentityState::UNKNOWN;
};

struct CargoPhysicalIdentityInput {
  double pipeline_stamp_sec = 0.0;
  std::uint64_t lifecycle_id = 0U;
  bool rearm = false;
  bool node_started_loaded = false;
  HookLoadSignalRole hook_role = HookLoadSignalRole::REQUIRED;
  std::string hook_role_source = "unspecified";
  bool localization_authorized = true;
  bool pose_authority_identity_valid = true;
  bool gravity_valid = false;
  HookLoadState gravity_state = HookLoadState::UNKNOWN;
  std::vector<CargoPhysicalGroupObservation> groups;
  std::vector<CargoIdentitySupportLineageObservation> lineage_observations;
  CargoShadowFrameEvidence frame_evidence;
  CargoVerticalEvidenceConfig vertical_config;
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
  std::uint64_t physical_cargo_epoch_id = 0U;
  CargoPreLiftReferenceState prelift_state =
      CargoPreLiftReferenceState::UNSEEN;
  std::size_t prelift_sample_count = 0U;
  double prelift_reference_uncertainty_m =
      std::numeric_limits<double>::quiet_NaN();
  double prelift_reference_first_stamp = 0.0;
  double prelift_reference_last_stamp = 0.0;
  std::string prelift_close_reason = "none";
  std::string hook_role_source = "unspecified";
  bool strict_lidar_existence_path = false;
  std::uint64_t physical_history_id = 0U;
  std::uint64_t frame_group_id = 0U;
  std::uint64_t resolved_candidate_id = 0U;
  std::vector<std::uint64_t> resolved_member_component_ids;
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
  double last_supported_evidence_stamp = 0.0;
  double reacquired_vertical_compute_ms = 0.0;
  std::size_t reacquired_vertical_attempt_count = 0U;
  std::size_t reacquired_vertical_points_examined = 0U;
  CargoGroupVerticalMode current_vertical_mode =
      CargoGroupVerticalMode::INVALID;
  bool current_vertical_evidence_valid = false;
  double identity_validation_stamp_sec = 0.0;
  std::vector<CargoPhysicalGroupDiagnostic> group_diagnostics;
  std::string reason = "uninitialized";
};

// A frame-local hypothesis id is not sufficient when legacy component ids
// collide. Authoritative point lookup must also match the canonical member set.
bool matchesResolvedPhysicalHypothesis(
    const CargoPhysicalCandidateObservation& candidate,
    const CargoPhysicalIdentityDecision& decision);

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
  struct PreLiftSample {
    double stamp_sec = 0.0;
    double vertical_z = std::numeric_limits<double>::quiet_NaN();
    double uncertainty_m = std::numeric_limits<double>::quiet_NaN();
  };

  struct LineageProvenanceSnapshot {
    double source_stamp_sec = 0.0;
    std::vector<std::uint64_t> component_ids;
  };

  struct History {
    std::uint64_t id = 0U;
    CargoPhysicalGroupDescriptor last_descriptor;
    Eigen::Vector3d last_representative_center = Eigen::Vector3d::Zero();
    double last_stamp_sec = 0.0;
    CargoPreLiftReferenceState prelift_state =
        CargoPreLiftReferenceState::UNSEEN;
    std::vector<PreLiftSample> earliest_prelift_samples;
    std::uint64_t physical_cargo_epoch_id = 0U;
    bool prelift_reference_frozen = false;
    // Latched only when a valid LOADED edge observes an already-frozen
    // pre-load reference. A reference first assembled after startup-loaded
    // or during the loaded epoch can never use the reduced confirmation.
    bool loaded_reduced_confirmation_eligible = false;
    double prelift_reference_z = std::numeric_limits<double>::quiet_NaN();
    double prelift_reference_uncertainty_m = 0.20;
    double prelift_reference_first_stamp = 0.0;
    double prelift_reference_last_stamp = 0.0;
    std::string prelift_close_reason = "none";
    bool baseline_frozen = false;
    CargoLiftBaselineSource baseline_source =
        CargoLiftBaselineSource::POST_LOAD_FIRST_FRESH_OBSERVATION;
    double baseline_z95 = std::numeric_limits<double>::quiet_NaN();
    double baseline_uncertainty_m = 0.20;
    double baseline_stamp_sec = 0.0;
    double last_supported_evidence_stamp_sec = 0.0;
    double last_supported_vertical_z = std::numeric_limits<double>::quiet_NaN();
    double last_supported_vertical_uncertainty_m = 0.20;
    CargoFootprintSnapshot last_supported_footprint;
    // Bounded exact-component provenance for the same three source frames
    // inspected by CargoIdentityComponentLineage.  This is not a tracker: it
    // owns no points, pose, prediction, Cargo id, baseline, lift or geometry.
    std::deque<LineageProvenanceSnapshot> recent_lineage_provenance;
    int lift_confirm_count = 0;
    bool lift_confirmed = false;
    double validation_stamp_sec = 0.0;
    bool association_ambiguous = false;
  };

  static constexpr std::size_t kMaximumLineageProvenanceFrames = 3U;

  CargoPhysicalIdentityConfig config_;
  std::vector<History> histories_;
  CargoPhysicalIdentityDecision decision_;
  std::uint64_t next_history_id_ = 1U;
  std::uint64_t load_epoch_ = 0U;
  std::uint64_t lifecycle_id_ = 0U;
  std::uint64_t physical_cargo_epoch_id_ = 0U;
  std::uint64_t validated_history_id_ = 0U;
  bool initialized_ = false;
  bool previous_existence_phase_ = false;
  bool started_loaded_without_baseline_ = false;
  bool prelift_blocked_until_new_epoch_ = false;
  double last_pipeline_stamp_sec_ = 0.0;
  std::string reset_reason_ = "constructed";
};

}  // namespace ndt_slam
