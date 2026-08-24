#include "ndt_slam/integrated_cargo_identity_shadow.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ndt_slam {
namespace {

double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2U != 0U) return upper;
  std::nth_element(values.begin(), values.begin() + middle - 1U,
                   values.begin() + middle);
  return 0.5 * (values[middle - 1U] + upper);
}

bool finiteGeometry(
    const CargoShadowResolvedGeometryObservation& geometry) {
  return geometry.valid && geometry.source_stamp_sec > 0.0 &&
      std::isfinite(geometry.source_stamp_sec) &&
      geometry.footprint_center_base.allFinite() &&
      std::isfinite(geometry.physical_anchor_z) &&
      std::isfinite(geometry.vertical_uncertainty_m) &&
      geometry.vertical_uncertainty_m >= 0.0 &&
      geometry.size.allFinite() && (geometry.size.array() > 0.0).all() &&
      std::isfinite(geometry.yaw_rad);
}

}  // namespace

CargoPhysicalGroupEvidenceSnapshot bindCargoPhysicalGroupEvidence(
    const std::vector<CargoPhysicalGroupObservation>& groups,
    const CargoPhysicalIdentityDecision& identity,
    std::uint64_t lifecycle_id) {
  CargoPhysicalGroupEvidenceSnapshot snapshot;
  if (identity.identity != CargoPhysicalIdentityState::VALIDATED ||
      identity.physical_history_id == 0U || identity.frame_group_id == 0U) {
    return snapshot;
  }
  const CargoPhysicalGroupObservation* selected = nullptr;
  for (const CargoPhysicalGroupObservation& group : groups) {
    if (group.frame_group_id != identity.frame_group_id) continue;
    if (selected != nullptr) return CargoPhysicalGroupEvidenceSnapshot{};
    selected = &group;
  }
  if (selected == nullptr ||
      !std::isfinite(selected->descriptor.stamp_sec) ||
      selected->descriptor.stamp_sec <= 0.0 ||
      !selected->descriptor.stable_anchor.allFinite()) {
    return snapshot;
  }

  snapshot.valid = true;
  snapshot.physical_history_id = identity.physical_history_id;
  snapshot.frame_group_id = selected->frame_group_id;
  snapshot.load_epoch = identity.load_epoch;
  snapshot.lifecycle_id = lifecycle_id;
  snapshot.source_stamp_sec = selected->descriptor.stamp_sec;
  snapshot.union_points_base = selected->union_points_base;
  snapshot.stable_anchor = selected->descriptor.stable_anchor;
  snapshot.vertical_mode = selected->descriptor.vertical_mode;
  snapshot.supported_top_valid = selected->descriptor.vertical_mode ==
          CargoGroupVerticalMode::SUPPORTED_EVIDENCE &&
      std::isfinite(selected->descriptor.physical_vertical_z);
  snapshot.supported_top_z = snapshot.supported_top_valid
      ? selected->descriptor.physical_vertical_z
      : std::numeric_limits<double>::quiet_NaN();
  snapshot.vertical_uncertainty_m =
      selected->descriptor.vertical_uncertainty_m;
  snapshot.vertical_reject_reason =
      selected->descriptor.vertical_reject_reason;
  snapshot.member_component_ids = selected->member_component_ids;

  const CargoPhysicalCandidateObservation& canonical =
      selected->representative;
  snapshot.geometry_resolved = selected->geometry_resolved &&
      !selected->group_ambiguous && canonical.center.allFinite() &&
      canonical.size.allFinite() && (canonical.size.array() > 0.0).all() &&
      std::isfinite(canonical.yaw_rad);
  if (snapshot.geometry_resolved) {
    snapshot.resolved_geometry.valid = true;
    snapshot.resolved_geometry.source_stamp_sec = snapshot.source_stamp_sec;
    snapshot.resolved_geometry.footprint_center_base =
        canonical.center.head<2>();
    snapshot.resolved_geometry.physical_anchor_z =
        snapshot.stable_anchor.z();
    snapshot.resolved_geometry.vertical_uncertainty_m =
        snapshot.vertical_uncertainty_m;
    snapshot.resolved_geometry.size = canonical.size;
    snapshot.resolved_geometry.yaw_rad = canonical.yaw_rad;
    snapshot.resolved_geometry.point_support = canonical.point_support;
  }
  return snapshot;
}

bool cargoPhysicalGroupEvidenceOwnerMatches(
    const CargoPhysicalGroupEvidenceSnapshot& snapshot,
    const CargoPhysicalIdentityDecision& identity,
    std::uint64_t lifecycle_id) noexcept {
  return snapshot.valid &&
      identity.identity == CargoPhysicalIdentityState::VALIDATED &&
      snapshot.physical_history_id == identity.physical_history_id &&
      snapshot.load_epoch == identity.load_epoch &&
      snapshot.lifecycle_id == lifecycle_id &&
      std::isfinite(snapshot.source_stamp_sec) &&
      snapshot.source_stamp_sec > 0.0;
}

CargoShadowGeometryAuthority::CargoShadowGeometryAuthority(
    const CargoShadowGeometryConfig& config) {
  setConfig(config);
}

void CargoShadowGeometryAuthority::setConfig(
    const CargoShadowGeometryConfig& config) {
  config_ = config;
  config_.minimum_point_support =
      std::max<std::size_t>(1U, config_.minimum_point_support);
  config_.formal_confirm_frames = std::max(1, config_.formal_confirm_frames);
  if (!(config_.maximum_observation_gap_sec > 0.0)) {
    config_.maximum_observation_gap_sec = 0.50;
  }
  if (!(config_.maximum_xy_step_m > 0.0)) {
    config_.maximum_xy_step_m = 0.30;
  }
  if (!(config_.maximum_z_speed_mps > 0.0)) {
    config_.maximum_z_speed_mps = 1.50;
  }
  if (!(config_.z_step_margin_m >= 0.0)) config_.z_step_margin_m = 0.05;
  reset("config_changed");
}

void CargoShadowGeometryAuthority::reset(const std::string& reason) {
  window_.clear();
  decision_ = CargoShadowGeometryDecision{};
  history_id_ = 0U;
  last_stamp_sec_ = 0.0;
  reset_reason_ = reason;
}

CargoShadowGeometryDecision CargoShadowGeometryAuthority::update(
    const CargoShadowGeometryInput& input) {
  const bool identity_validated = input.identity.identity ==
      CargoPhysicalIdentityState::VALIDATED;
  if (!identity_validated) {
    reset("identity_not_validated");
    decision_.reject_reason = "identity_not_validated";
    return decision_;
  }
  const bool identity_changed = history_id_ != 0U &&
      history_id_ != input.identity.physical_history_id;
  if (identity_changed) reset("validated_identity_changed");

  CargoShadowGeometryDecision current;
  current.reference_independent = true;
  current.identity_validated = true;
  current.physical_history_id = input.identity.physical_history_id;
  current.geometry_resolved = input.identity.geometry_resolved;
  if (!finiteGeometry(input.geometry) ||
      !input.identity.current_candidate_fresh) {
    current.reject_reason = "group_geometry_not_fresh_or_finite";
    decision_ = current;
    history_id_ = input.identity.physical_history_id;
    return decision_;
  }

  // A repeated pipeline frame may project the latest evidence, but only a
  // strictly newer detector source stamp may advance formal confirmation.
  if (last_stamp_sec_ > 0.0 &&
      std::abs(input.geometry.source_stamp_sec - last_stamp_sec_) <= 1.0e-9) {
    return decision_;
  }
  if (last_stamp_sec_ > 0.0 &&
      input.geometry.source_stamp_sec < last_stamp_sec_) {
    reset("geometry_source_time_rollback");
    current.reject_reason = "geometry_source_time_rollback";
    decision_ = current;
    history_id_ = input.identity.physical_history_id;
    return decision_;
  }

  decision_ = current;
  decision_.reference_independent = true;
  history_id_ = input.identity.physical_history_id;
  decision_.physical_history_id = history_id_;
  decision_.source_stamp_sec = input.geometry.source_stamp_sec;
  const bool physical_bounds =
      input.geometry.point_support >= config_.minimum_point_support &&
      input.geometry.size.x() >= config_.minimum_length_m &&
      input.geometry.size.x() <= config_.maximum_length_m &&
      input.geometry.size.y() >= config_.minimum_width_m &&
      input.geometry.size.y() <= config_.maximum_width_m &&
      input.geometry.size.z() >= config_.minimum_height_m &&
      input.geometry.size.z() <= config_.maximum_height_m;
  if (!physical_bounds || !input.identity.geometry_resolved) {
    decision_.reject_reason = input.identity.geometry_resolved
        ? "reference_independent_physical_bounds" : "geometry_ambiguous";
    return decision_;
  }

  // A validated, fresh, plausible candidate is a positive-only Pending
  // envelope immediately. Formal clear authority requires the full window.
  decision_.pending_envelope_valid = true;
  if (last_stamp_sec_ > 0.0 &&
      (input.geometry.source_stamp_sec - last_stamp_sec_ >
           config_.maximum_observation_gap_sec)) {
    window_.clear();
  }
  if (!window_.empty()) {
    const auto& previous = window_.back();
    const double dt = input.geometry.source_stamp_sec -
        previous.source_stamp_sec;
    const double xy_step = (input.geometry.footprint_center_base -
                            previous.footprint_center_base).norm();
    const double z_limit = config_.maximum_z_speed_mps * std::max(0.0, dt) +
        config_.z_step_margin_m + input.geometry.vertical_uncertainty_m +
        previous.vertical_uncertainty_m;
    const double z_step = std::abs(
        input.geometry.physical_anchor_z - previous.physical_anchor_z);
    if (!(dt > 0.0) || xy_step > config_.maximum_xy_step_m ||
        z_step > z_limit) {
      window_.clear();
    }
  }
  if (window_.empty()) {
    decision_.window_start_stamp_sec = input.geometry.source_stamp_sec;
  }
  window_.push_back(input.geometry);
  while (window_.size() >
         static_cast<std::size_t>(config_.formal_confirm_frames)) {
    window_.pop_front();
  }
  last_stamp_sec_ = input.geometry.source_stamp_sec;
  decision_.confirm_count = static_cast<int>(window_.size());
  decision_.window_start_stamp_sec = window_.front().source_stamp_sec;

  std::vector<double> xs, ys, zs, lengths, widths, heights;
  double axial_cos = 0.0;
  double axial_sin = 0.0;
  for (const auto& sample : window_) {
    xs.push_back(sample.footprint_center_base.x());
    ys.push_back(sample.footprint_center_base.y());
    zs.push_back(sample.physical_anchor_z);
    lengths.push_back(sample.size.x());
    widths.push_back(sample.size.y());
    heights.push_back(sample.size.z());
    axial_cos += std::cos(2.0 * sample.yaw_rad);
    axial_sin += std::sin(2.0 * sample.yaw_rad);
  }
  decision_.median_center = Eigen::Vector3d(
      median(xs), median(ys), median(zs));
  decision_.median_size = Eigen::Vector3d(
      median(lengths), median(widths), median(heights));
  decision_.axial_orientation_concentration =
      std::hypot(axial_cos, axial_sin) /
      static_cast<double>(window_.size());
  double maximum_cv = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    double variance = 0.0;
    for (const auto& sample : window_) {
      const double delta = sample.size[axis] - decision_.median_size[axis];
      variance += delta * delta;
    }
    variance /= static_cast<double>(window_.size());
    maximum_cv = std::max(maximum_cv,
        std::sqrt(variance) / std::max(1.0e-9, decision_.median_size[axis]));
  }
  if (maximum_cv > config_.maximum_size_cv) {
    decision_.reject_reason = "shape_cv_unstable";
    return decision_;
  }
  if (decision_.axial_orientation_concentration <
      config_.minimum_axial_orientation_concentration) {
    decision_.reject_reason = "axial_orientation_unstable";
    return decision_;
  }
  if (decision_.confirm_count < config_.formal_confirm_frames) {
    decision_.reject_reason = "formal_geometry_confirming";
    return decision_;
  }
  decision_.formal_geometry_valid = true;
  decision_.formal_clear_authorized = true;
  decision_.reject_reason = "reference_independent_geometry_valid";
  return decision_;
}

bool shadowThicknessAuthorized(
    const CargoShadowThicknessProvenance& provenance,
    const CargoPhysicalIdentityDecision& identity,
    std::uint64_t lifecycle_id) noexcept {
  return provenance.valid &&
      identity.identity == CargoPhysicalIdentityState::VALIDATED &&
      provenance.physical_history_id == identity.physical_history_id &&
      provenance.load_epoch == identity.load_epoch &&
      provenance.lifecycle_id == lifecycle_id &&
      std::isfinite(provenance.source_stamp_sec) &&
      provenance.source_stamp_sec > 0.0 &&
      provenance.source ==
          "SHADOW_REFERENCE_INDEPENDENT_FORMAL_GEOMETRY";
}

void CargoShadowThicknessState::reset() noexcept {
  frozen_thickness_m = 0.0F;
  provenance = CargoShadowThicknessProvenance{};
}

bool CargoShadowThicknessState::freezeFromFormalGeometry(
    const CargoPhysicalGroupEvidenceSnapshot& snapshot,
    const CargoPhysicalIdentityDecision& identity,
    const CargoShadowGeometryDecision& geometry,
    float minimum_height_m,
    float maximum_height_m) {
  if (provenance.valid) {
    return shadowThicknessAuthorized(
        provenance, identity, snapshot.lifecycle_id);
  }
  const double height = geometry.median_size.z();
  if (!cargoPhysicalGroupEvidenceOwnerMatches(
          snapshot, identity, snapshot.lifecycle_id) ||
      !snapshot.geometry_resolved || !geometry.geometry_resolved ||
      !geometry.formal_geometry_valid ||
      geometry.physical_history_id != snapshot.physical_history_id ||
      !std::isfinite(height) || !std::isfinite(minimum_height_m) ||
      !std::isfinite(maximum_height_m) ||
      height < minimum_height_m || height > maximum_height_m) {
    return false;
  }
  frozen_thickness_m = static_cast<float>(height);
  provenance.valid = true;
  provenance.physical_history_id = snapshot.physical_history_id;
  provenance.load_epoch = snapshot.load_epoch;
  provenance.lifecycle_id = snapshot.lifecycle_id;
  provenance.source =
      "SHADOW_REFERENCE_INDEPENDENT_FORMAL_GEOMETRY";
  provenance.source_stamp_sec = geometry.source_stamp_sec;
  return true;
}

CargoAvoidanceFusionInput projectShadowCargoOntoCanonicalFusion(
    const CargoAvoidanceFusionInput& canonical,
    const CargoShadowFusionProjection& shadow) {
  CargoAvoidanceFusionInput result = canonical;
  result.formal_cargo_geometry_valid = shadow.formal;
  result.formal_cargo_bottom_valid = shadow.formal && shadow.bottom_valid;
  result.formal_clear_authorized = shadow.formal && shadow.bottom_valid &&
      shadow.clear_authorized;
  result.pending_envelope_valid = shadow.pending && !shadow.formal;
  if (result.pending_envelope_valid) {
    result.pending_envelope_source =
        PendingCargoEnvelopeSource::CURRENT_CANDIDATE;
    result.pending_pose_source =
        CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR;
    result.pending_recognition_state_allows_warning = true;
    result.pending_warning_query_allowed = true;
    result.pending_pose_physically_plausible = true;
    result.pending_warning_state_reason = "shadow_identity_validated_pending";
  }
  result.live = shadow.live;
  result.static_map = shadow.static_map;
  result.live.cargo_lifecycle_id = shadow.cargo_lifecycle_id;
  result.live.cargo_track_id = shadow.cargo_track_id;
  result.static_map.cargo_lifecycle_id = shadow.cargo_lifecycle_id;
  result.static_map.cargo_track_id = shadow.cargo_track_id;
  return result;
}

void updateShadowPhysicalDistanceTiming(
    double stamp_sec, double physical_distance_m,
    double level2_distance_m, double far_distance_m,
    bool identity_validated, bool pending_ready, bool formal_ready,
    CargoShadowPhysicalDistanceTiming* timing) {
  if (timing == nullptr || !std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    return;
  }
  if (identity_validated && timing->identity_validation_stamp_sec <= 0.0) {
    timing->identity_validation_stamp_sec = stamp_sec;
  }
  if (pending_ready && timing->pending_ready_stamp_sec <= 0.0) {
    timing->pending_ready_stamp_sec = stamp_sec;
  }
  if (formal_ready && timing->formal_ready_stamp_sec <= 0.0) {
    timing->formal_ready_stamp_sec = stamp_sec;
  }
  if (std::isfinite(physical_distance_m)) {
    if (physical_distance_m <= far_distance_m &&
        timing->first_obstacle_8m_stamp_sec <= 0.0) {
      timing->first_obstacle_8m_stamp_sec = stamp_sec;
    }
    if (physical_distance_m <= level2_distance_m &&
        timing->first_obstacle_5m_stamp_sec <= 0.0) {
      timing->first_obstacle_5m_stamp_sec = stamp_sec;
    }
  }
  timing->identity_validated_before_8m =
      timing->identity_validation_stamp_sec > 0.0 &&
      (timing->first_obstacle_8m_stamp_sec <= 0.0 ||
       timing->identity_validation_stamp_sec <=
           timing->first_obstacle_8m_stamp_sec);
  const double ready_stamp = timing->pending_ready_stamp_sec > 0.0
      ? timing->pending_ready_stamp_sec : timing->formal_ready_stamp_sec;
  timing->pending_or_lock_ready_before_5m = ready_stamp > 0.0 &&
      (timing->first_obstacle_5m_stamp_sec <= 0.0 ||
       ready_stamp <= timing->first_obstacle_5m_stamp_sec);
}

}  // namespace ndt_slam
