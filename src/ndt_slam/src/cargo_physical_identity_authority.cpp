#include "ndt_slam/cargo_physical_identity_authority.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <tuple>
#include <utility>

namespace ndt_slam {
namespace {

constexpr double kEpsilon = 1.0e-9;

std::vector<std::uint64_t> canonicalMembers(
    std::vector<std::uint64_t> members) {
  std::sort(members.begin(), members.end());
  members.erase(std::unique(members.begin(), members.end()), members.end());
  return members;
}

bool finiteCandidate(const CargoPhysicalCandidateObservation& candidate) {
  return candidate.stamp_sec > 0.0 && std::isfinite(candidate.stamp_sec) &&
      candidate.center.allFinite() && candidate.size.allFinite() &&
      (candidate.size.array() > 0.0).all() &&
      std::isfinite(candidate.yaw_rad) && std::isfinite(candidate.z95) &&
      std::isfinite(candidate.vertical_uncertainty_m) &&
      candidate.vertical_uncertainty_m >= 0.0 &&
      !candidate.member_component_ids.empty() && candidate.point_support > 0U;
}

bool finiteDescriptor(const CargoPhysicalGroupDescriptor& descriptor) {
  return descriptor.valid && descriptor.stamp_sec > 0.0 &&
      std::isfinite(descriptor.stamp_sec) &&
      descriptor.stable_anchor.allFinite() &&
      descriptor.aggregate_extent.allFinite() &&
      (descriptor.aggregate_extent.array() > 0.0).all() &&
      std::isfinite(descriptor.robust_x05) &&
      std::isfinite(descriptor.robust_x95) &&
      std::isfinite(descriptor.robust_y05) &&
      std::isfinite(descriptor.robust_y95) &&
      descriptor.robust_xy_center.allFinite() &&
      descriptor.robust_xy_extent.allFinite() &&
      (descriptor.robust_xy_extent.array() > 0.0).all() &&
      descriptor.aggregate_point_support > 0U &&
      descriptor.vertical_mode != CargoGroupVerticalMode::INVALID &&
      std::isfinite(descriptor.physical_vertical_z) &&
      std::isfinite(descriptor.vertical_uncertainty_m) &&
      descriptor.vertical_uncertainty_m >= 0.0;
}

bool overlaps(const std::vector<std::uint64_t>& lhs,
              const std::vector<std::uint64_t>& rhs) {
  std::vector<std::uint64_t> intersection;
  std::set_intersection(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(),
                        std::back_inserter(intersection));
  return !intersection.empty();
}

bool containsAll(const std::vector<std::uint64_t>& container,
                 const std::vector<std::uint64_t>& contents) {
  return std::includes(container.begin(), container.end(),
                       contents.begin(), contents.end());
}

bool equivalentGeometry(const CargoPhysicalCandidateObservation& lhs,
                        const CargoPhysicalCandidateObservation& rhs,
                        double center_tolerance,
                        double size_relative_tolerance) {
  if ((lhs.center - rhs.center).norm() > center_tolerance) return false;
  for (int axis = 0; axis < 3; ++axis) {
    const double denominator = std::max(
        std::max(std::abs(lhs.size[axis]), std::abs(rhs.size[axis])), kEpsilon);
    if (std::abs(lhs.size[axis] - rhs.size[axis]) / denominator >
        size_relative_tolerance) {
      return false;
    }
  }
  return true;
}

bool canonicalCandidateLess(const CargoPhysicalCandidateObservation& lhs,
                            const CargoPhysicalCandidateObservation& rhs) {
  return std::make_tuple(
             lhs.member_component_ids, lhs.center.x(), lhs.center.y(),
             lhs.center.z(), lhs.size.x(), lhs.size.y(), lhs.size.z(),
             lhs.yaw_rad, lhs.candidate_id) <
      std::make_tuple(
             rhs.member_component_ids, rhs.center.x(), rhs.center.y(),
             rhs.center.z(), rhs.size.x(), rhs.size.y(), rhs.size.z(),
             rhs.yaw_rad, rhs.candidate_id);
}

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  return values.size() % 2U == 0U
      ? 0.5 * (values[middle - 1U] + values[middle])
      : values[middle];
}

double quantile(std::vector<double> values, double probability) {
  std::sort(values.begin(), values.end());
  if (values.size() == 1U) return values.front();
  const double position = probability *
      static_cast<double>(values.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return values[lower] + fraction * (values[upper] - values[lower]);
}

double intervalSeparation(double lhs_low, double lhs_high,
                          double rhs_low, double rhs_high) {
  return std::max(0.0, std::max(lhs_low, rhs_low) -
                           std::min(lhs_high, rhs_high));
}

bool hasPositiveAreaSupportOverlap(
    const CargoPhysicalGroupDescriptor& lhs,
    const CargoPhysicalGroupDescriptor& rhs) {
  const double intersection_x =
      std::min(lhs.robust_x95, rhs.robust_x95) -
      std::max(lhs.robust_x05, rhs.robust_x05);
  const double intersection_y =
      std::min(lhs.robust_y95, rhs.robust_y95) -
      std::max(lhs.robust_y05, rhs.robust_y05);
  return intersection_x > kEpsilon && intersection_y > kEpsilon &&
      intersection_x * intersection_y > kEpsilon;
}

CargoVerticalEvidenceInput makeVerticalInput(
    const CargoPhysicalCandidateObservation& hypothesis,
    bool ground_reference_valid,
    double ground_z_base) {
  CargoVerticalEvidenceInput input;
  input.footprint_valid = true;
  input.footprint_center_base = hypothesis.center.head<2>().cast<float>();
  input.footprint_size_xy = hypothesis.size.head<2>().cast<float>();
  input.footprint_yaw_base_rad = static_cast<float>(hypothesis.yaw_rad);
  input.ground_reference_valid = ground_reference_valid;
  input.ground_z_base = static_cast<float>(ground_z_base);
  return input;
}

struct VerticalOwnershipResult {
  bool valid = false;
  std::size_t overlap_cell_count = 0U;
  double coverage = 0.0;
  std::vector<double> owned_filtered_z;
};

VerticalOwnershipResult proveVerticalOwnership(
    const CargoVerticalEvidence& evidence,
    const std::vector<Eigen::Vector3f>& owner_points,
    const CargoVerticalEvidenceInput& input,
    const CargoVerticalEvidenceConfig& config) {
  VerticalOwnershipResult result;
  if (!evidence.valid || evidence.top_surface_cell_indices.empty()) {
    return result;
  }
  std::set<CargoFootprintGridIndex> owner_cells;
  for (const Eigen::Vector3f& point : owner_points) {
    if (!point.allFinite() || !cargoPointInsideFootprint(
            point, input, config.footprint_margin_m)) {
      continue;
    }
    owner_cells.insert(makeCargoFootprintGridIndex(
        point, input, config.xy_cell_size_m));
  }
  for (const CargoFootprintGridIndex& cell :
       evidence.top_surface_cell_indices) {
    if (owner_cells.count(cell) > 0U) ++result.overlap_cell_count;
  }
  result.coverage = static_cast<double>(result.overlap_cell_count) /
      static_cast<double>(evidence.top_surface_cell_indices.size());
  result.valid = result.overlap_cell_count >= config.minimum_surface_cells &&
      result.coverage + kEpsilon >= config.minimum_surface_coverage_ratio;
  if (result.valid) {
    for (const Eigen::Vector3f& point :
         evidence.filtered_vertical_points_base) {
      if (!point.allFinite()) continue;
      const CargoFootprintGridIndex cell = makeCargoFootprintGridIndex(
          point, input, config.xy_cell_size_m);
      if (owner_cells.count(cell) > 0U) {
        result.owned_filtered_z.push_back(point.z());
      }
    }
  }
  return result;
}

CargoFootprintSnapshot footprintSnapshot(
    const CargoPhysicalCandidateObservation& hypothesis,
    double source_stamp_sec) {
  CargoFootprintSnapshot snapshot;
  snapshot.valid = hypothesis.center.allFinite() &&
      hypothesis.size.allFinite() &&
      (hypothesis.size.head<2>().array() > 0.0).all() &&
      std::isfinite(hypothesis.yaw_rad) &&
      std::isfinite(source_stamp_sec) && source_stamp_sec > 0.0;
  snapshot.center_base = hypothesis.center.head<2>().cast<float>();
  snapshot.size_xy = hypothesis.size.head<2>().cast<float>();
  snapshot.yaw_base_rad = static_cast<float>(hypothesis.yaw_rad);
  snapshot.source_stamp_sec = source_stamp_sec;
  return snapshot;
}

AssociationOnlyReacquiredVerticalEvidence reacquireAssociationVertical(
    const CargoShadowFrameEvidence& frame,
    const CargoFootprintSnapshot& footprint,
    const std::vector<Eigen::Vector3f>& owner_points,
    const CargoVerticalEvidenceConfig& config,
    double uncertainty_m) {
  AssociationOnlyReacquiredVerticalEvidence result;
  result.source_stamp_sec = frame.source_stamp_sec;
  result.uncertainty_m = uncertainty_m;
  if (!footprint.valid || !frame.raw_roi_current_frame ||
      !std::isfinite(frame.source_stamp_sec) ||
      frame.source_stamp_sec <= 0.0) {
    result.reason = "frame_or_history_footprint_invalid";
    return result;
  }
  CargoVerticalEvidenceInput input;
  input.selected_cloud_base = frame.raw_roi_current_frame;
  input.footprint_valid = true;
  input.footprint_center_base = footprint.center_base;
  input.footprint_size_xy = footprint.size_xy;
  input.footprint_yaw_base_rad = footprint.yaw_base_rad;
  input.ground_reference_valid = frame.ground_reference_valid;
  input.ground_z_base = frame.ground_z_base;
  const CargoVerticalEvidence evidence =
      extractCargoVerticalEvidence(input, config);
  if (!evidence.valid || !std::isfinite(evidence.top_z_base)) {
    result.reason = "raw_history_footprint_vertical_invalid:" +
        evidence.reject_reason;
    return result;
  }
  const VerticalOwnershipResult ownership = proveVerticalOwnership(
      evidence, owner_points, input, config);
  result.owner_overlap_cell_count = ownership.overlap_cell_count;
  result.owner_overlap_coverage = ownership.coverage;
  if (!ownership.valid) {
    result.reason = "raw_history_footprint_owner_proof_failed";
    return result;
  }
  result.valid = true;
  result.top_z_base = evidence.top_z_base;
  result.reason = "association_only_reacquired_vertical_valid";
  return result;
}

int requiredFrames(HookLoadSignalRole role, bool gravity_valid,
                   HookLoadState gravity_state, int base) {
  if (role != HookLoadSignalRole::AUXILIARY) return std::max(1, base);
  if (gravity_valid && gravity_state == HookLoadState::EMPTY) {
    return std::max(1, base) + 2;
  }
  if (!gravity_valid || gravity_state == HookLoadState::UNKNOWN) {
    return std::max(1, base) + 1;
  }
  return std::max(1, base);
}

}  // namespace

const char* cargoCandidateAssociationStateName(
    CargoCandidateAssociationState state) noexcept {
  switch (state) {
    case CargoCandidateAssociationState::MATCHED: return "MATCHED";
    case CargoCandidateAssociationState::AMBIGUOUS: return "AMBIGUOUS";
    case CargoCandidateAssociationState::NEW_HISTORY: return "NEW_HISTORY";
  }
  return "INVALID";
}

const char* cargoPhysicalAssociationModeName(
    CargoPhysicalAssociationMode mode) noexcept {
  switch (mode) {
    case CargoPhysicalAssociationMode::ANCHOR_CONTINUITY:
      return "ANCHOR_CONTINUITY";
    case CargoPhysicalAssociationMode::SUPPORT_OVERLAP_CONTINUITY:
      return "SUPPORT_OVERLAP_CONTINUITY";
    case CargoPhysicalAssociationMode::NEW_HISTORY: return "NEW_HISTORY";
  }
  return "INVALID";
}

const char* cargoPhysicalIdentityStateName(
    CargoPhysicalIdentityState state) noexcept {
  switch (state) {
    case CargoPhysicalIdentityState::UNKNOWN: return "UNKNOWN";
    case CargoPhysicalIdentityState::AMBIGUOUS: return "AMBIGUOUS";
    case CargoPhysicalIdentityState::VALIDATED: return "VALIDATED";
  }
  return "INVALID";
}

const char* cargoLiftBaselineSourceName(
    CargoLiftBaselineSource source) noexcept {
  switch (source) {
    case CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE:
      return "PRE_LOAD_FROZEN_BASELINE";
    case CargoLiftBaselineSource::POST_LOAD_FIRST_FRESH_OBSERVATION:
      return "POST_LOAD_FIRST_FRESH_OBSERVATION";
    case CargoLiftBaselineSource::UNAVAILABLE_STARTED_LOADED:
      return "UNAVAILABLE_STARTED_LOADED";
  }
  return "INVALID";
}

const char* cargoExistenceSourceName(CargoExistenceSource source) noexcept {
  switch (source) {
    case CargoExistenceSource::NONE: return "NONE";
    case CargoExistenceSource::GRAVITY_LOADED: return "GRAVITY_LOADED";
    case CargoExistenceSource::STRICT_LIDAR: return "STRICT_LIDAR";
  }
  return "INVALID";
}

const char* cargoGroupVerticalModeName(CargoGroupVerticalMode mode) noexcept {
  switch (mode) {
    case CargoGroupVerticalMode::SUPPORTED_EVIDENCE:
      return "SUPPORTED_EVIDENCE";
    case CargoGroupVerticalMode::CONTINUITY_ONLY:
      return "CONTINUITY_ONLY";
    case CargoGroupVerticalMode::INVALID: return "INVALID";
  }
  return "INVALID";
}

const char* cargoVerticalEvidenceSourceName(
    CargoVerticalEvidenceSource source) noexcept {
  switch (source) {
    case CargoVerticalEvidenceSource::COMPONENT_UNION:
      return "COMPONENT_UNION";
    case CargoVerticalEvidenceSource::RAW_ROI_CURRENT_FOOTPRINT:
      return "RAW_ROI_CURRENT_FOOTPRINT";
    case CargoVerticalEvidenceSource::RAW_ROI_HISTORY_FOOTPRINT_REACQUIRE:
      return "RAW_ROI_HISTORY_FOOTPRINT_REACQUIRE";
  }
  return "INVALID";
}

std::vector<CargoPhysicalGroupObservation> groupCargoPhysicalCandidates(
    const std::vector<CargoPhysicalCandidateObservation>& candidates,
    const std::vector<CargoPhysicalComponentObservation>& components,
    const CargoShadowFrameEvidence* frame_evidence,
    bool ground_reference_valid,
    double ground_z_base,
    const CargoVerticalEvidenceConfig& vertical_config,
    double equivalent_center_tolerance_m,
    double equivalent_size_relative_tolerance,
    CargoPhysicalGroupingTelemetry* telemetry) {
  if (telemetry) *telemetry = CargoPhysicalGroupingTelemetry{};
  std::vector<CargoPhysicalCandidateObservation> valid;
  valid.reserve(candidates.size());
  for (CargoPhysicalCandidateObservation candidate : candidates) {
    candidate.member_component_ids =
        canonicalMembers(std::move(candidate.member_component_ids));
    if (finiteCandidate(candidate)) valid.push_back(std::move(candidate));
  }
  std::sort(valid.begin(), valid.end(), canonicalCandidateLess);

  std::vector<std::size_t> parent(valid.size());
  std::iota(parent.begin(), parent.end(), 0U);
  std::function<std::size_t(std::size_t)> root = [&](std::size_t index) {
    if (parent[index] != index) parent[index] = root(parent[index]);
    return parent[index];
  };
  const auto join = [&](std::size_t lhs, std::size_t rhs) {
    lhs = root(lhs);
    rhs = root(rhs);
    if (lhs != rhs) parent[rhs] = lhs;
  };
  for (std::size_t i = 0; i < valid.size(); ++i) {
    for (std::size_t j = i + 1U; j < valid.size(); ++j) {
      if (overlaps(valid[i].member_component_ids,
                   valid[j].member_component_ids)) {
        join(i, j);
      }
    }
  }

  std::map<std::size_t, std::vector<CargoPhysicalCandidateObservation>>
      hypothesis_groups;
  for (std::size_t index = 0; index < valid.size(); ++index) {
    hypothesis_groups[root(index)].push_back(valid[index]);
  }
  std::map<std::uint64_t, std::vector<Eigen::Vector3f>> component_points;
  for (const CargoPhysicalComponentObservation& component : components) {
    component_points.emplace(component.component_id, component.points_base);
  }

  std::vector<CargoPhysicalGroupObservation> groups;
  groups.reserve(hypothesis_groups.size());
  for (auto& entry : hypothesis_groups) {
    CargoPhysicalGroupObservation group;
    group.hypotheses = std::move(entry.second);
    std::sort(group.hypotheses.begin(), group.hypotheses.end(),
              canonicalCandidateLess);
    group.representative = *std::min_element(
        group.hypotheses.begin(), group.hypotheses.end(),
        canonicalCandidateLess);
    for (const auto& hypothesis : group.hypotheses) {
      group.member_component_ids.insert(group.member_component_ids.end(),
          hypothesis.member_component_ids.begin(),
          hypothesis.member_component_ids.end());
    }
    group.member_component_ids =
        canonicalMembers(std::move(group.member_component_ids));

    group.geometry_resolved = true;
    for (std::size_t i = 1U; i < group.hypotheses.size(); ++i) {
      if (!equivalentGeometry(group.representative, group.hypotheses[i],
                              equivalent_center_tolerance_m,
                              equivalent_size_relative_tolerance)) {
        group.geometry_resolved = false;
      }
    }
    for (std::size_t i = 0; i < group.hypotheses.size(); ++i) {
      for (std::size_t j = i + 1U; j < group.hypotheses.size(); ++j) {
        if (!overlaps(group.hypotheses[i].member_component_ids,
                      group.hypotheses[j].member_component_ids)) {
          group.group_ambiguous = true;
        } else if (!containsAll(
                       group.hypotheses[i].member_component_ids,
                       group.hypotheses[j].member_component_ids) &&
                   !containsAll(
                       group.hypotheses[j].member_component_ids,
                       group.hypotheses[i].member_component_ids)) {
          group.group_ambiguous = true;
        }
      }
    }

    std::vector<Eigen::Vector3f> union_points;
    bool component_lookup_complete = true;
    for (std::uint64_t member : group.member_component_ids) {
      const auto found = component_points.find(member);
      if (found == component_points.end()) {
        component_lookup_complete = false;
        continue;
      }
      union_points.insert(union_points.end(), found->second.begin(),
                          found->second.end());
    }
    group.union_points_base = union_points;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    xs.reserve(union_points.size());
    ys.reserve(union_points.size());
    zs.reserve(union_points.size());
    Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    for (const Eigen::Vector3f& point : union_points) {
      if (!point.allFinite()) continue;
      const Eigen::Vector3d value = point.cast<double>();
      minimum = minimum.cwiseMin(value);
      maximum = maximum.cwiseMax(value);
      xs.push_back(value.x());
      ys.push_back(value.y());
      zs.push_back(value.z());
    }

    CargoPhysicalGroupDescriptor& descriptor = group.descriptor;
    descriptor.stamp_sec = group.representative.stamp_sec;
    descriptor.aggregate_point_support = zs.size();
    double maximum_uncertainty = 0.0;
    for (const auto& hypothesis : group.hypotheses) {
      maximum_uncertainty = std::max(
          maximum_uncertainty, hypothesis.vertical_uncertainty_m);
    }
    descriptor.vertical_uncertainty_m = maximum_uncertainty;
    const std::size_t minimum_points =
        std::max<std::size_t>(1U, vertical_config.minimum_surface_points);
    const bool physical_support_available = component_lookup_complete &&
        zs.size() >= minimum_points;
    if (physical_support_available) {
      descriptor.stable_anchor =
          Eigen::Vector3d(median(xs), median(ys), median(zs));
      descriptor.aggregate_extent = maximum - minimum;
      descriptor.robust_x05 = quantile(xs, 0.05);
      descriptor.robust_x95 = quantile(xs, 0.95);
      descriptor.robust_y05 = quantile(ys, 0.05);
      descriptor.robust_y95 = quantile(ys, 0.95);
      descriptor.robust_xy_center = Eigen::Vector2d(
          0.5 * (descriptor.robust_x05 + descriptor.robust_x95),
          0.5 * (descriptor.robust_y05 + descriptor.robust_y95));
      descriptor.robust_xy_extent = Eigen::Vector2d(
          descriptor.robust_x95 - descriptor.robust_x05,
          descriptor.robust_y95 - descriptor.robust_y05);
    }
    const bool continuity_available = physical_support_available &&
        !group.group_ambiguous;

    std::vector<double> component_supported_tops;
    std::vector<double> raw_owned_supported_tops;
    std::vector<double> diagnostic_owned_z;
    bool selected_raw_evidence = false;
    const bool frame_stamp_matches = frame_evidence &&
        frame_evidence->raw_roi_current_frame &&
        std::isfinite(frame_evidence->source_stamp_sec) &&
        std::abs(frame_evidence->source_stamp_sec -
                 group.representative.stamp_sec) <= kEpsilon;
    const bool ground_context_matches = frame_evidence &&
        frame_evidence->ground_reference_valid == ground_reference_valid &&
        (!ground_reference_valid ||
         (std::isfinite(frame_evidence->ground_z_base) &&
          std::isfinite(ground_z_base) &&
          std::abs(static_cast<double>(frame_evidence->ground_z_base) -
                   ground_z_base) <= kEpsilon));
    if (continuity_available) {
      for (const auto& hypothesis : group.hypotheses) {
        CargoVerticalEvidenceInput component_input = makeVerticalInput(
            hypothesis, ground_reference_valid, ground_z_base);
        component_input.selected_points_base = union_points;
        const CargoVerticalEvidence component_evidence =
            extractCargoVerticalEvidence(component_input, vertical_config);

        // RAW top ownership is hypothesis-local. A nested hypothesis may not
        // borrow occupied XY cells from components that it does not contain,
        // even though those components belong to the same physical group.
        std::vector<Eigen::Vector3f> hypothesis_owner_points;
        for (std::uint64_t member : hypothesis.member_component_ids) {
          const auto found = component_points.find(member);
          if (found == component_points.end()) continue;
          hypothesis_owner_points.insert(
              hypothesis_owner_points.end(), found->second.begin(),
              found->second.end());
        }

        bool raw_owned = false;
        CargoVerticalEvidence raw_evidence;
        VerticalOwnershipResult ownership;
        if (frame_stamp_matches && ground_context_matches) {
          const auto raw_start = std::chrono::steady_clock::now();
          CargoVerticalEvidenceInput raw_input = makeVerticalInput(
              hypothesis, ground_reference_valid, ground_z_base);
          raw_input.selected_cloud_base =
              frame_evidence->raw_roi_current_frame;
          raw_evidence = extractCargoVerticalEvidence(
              raw_input, vertical_config);
          ownership = proveVerticalOwnership(
              raw_evidence, hypothesis_owner_points, raw_input,
              vertical_config);
          const double raw_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - raw_start).count();
          if (telemetry) {
            telemetry->raw_roi_vertical_total_ms += raw_ms;
            ++telemetry->raw_roi_vertical_hypothesis_count;
            telemetry->raw_roi_vertical_points_examined +=
                frame_evidence->raw_roi_current_frame->size();
          }
          raw_owned = raw_evidence.valid && ownership.valid &&
              std::isfinite(raw_evidence.top_z_base);
          descriptor.owner_overlap_cell_count = std::max(
              descriptor.owner_overlap_cell_count,
              ownership.overlap_cell_count);
          descriptor.owner_overlap_coverage = std::max(
              descriptor.owner_overlap_coverage, ownership.coverage);
          if (raw_evidence.valid && !ownership.valid) {
            ++descriptor.owner_proof_rejected_hypothesis_count;
          }
          diagnostic_owned_z.insert(
              diagnostic_owned_z.end(), ownership.owned_filtered_z.begin(),
              ownership.owned_filtered_z.end());
        }

        if (raw_owned) {
          raw_owned_supported_tops.push_back(raw_evidence.top_z_base);
          ++descriptor.raw_roi_supported_hypothesis_count;
        }
        if (component_evidence.valid &&
            std::isfinite(component_evidence.top_z_base)) {
          component_supported_tops.push_back(component_evidence.top_z_base);
        }
      }
    }

    // A group aggregate always has one evidence lineage. Mixing component and
    // RAW tops would make the counterfactual depend on how many hypotheses
    // happened to pass each path. Prefer owner-proven RAW evidence as a set;
    // otherwise preserve the existing component-union result.
    selected_raw_evidence = !raw_owned_supported_tops.empty();
    const std::vector<double>& supported_tops = selected_raw_evidence
        ? raw_owned_supported_tops : component_supported_tops;
    const double consistency_limit =
        static_cast<double>(vertical_config.surface_band_height_m) +
        maximum_uncertainty;
    const auto aggregate_if_consistent = [consistency_limit](
        const std::vector<double>& values, bool* valid, double* aggregate) {
      if (values.empty()) return;
      const auto bounds = std::minmax_element(values.begin(), values.end());
      if (*bounds.second - *bounds.first <= consistency_limit + kEpsilon) {
        *valid = true;
        *aggregate = median(values);
      }
    };
    aggregate_if_consistent(
        component_supported_tops, &descriptor.component_vertical_valid,
        &descriptor.component_vertical_z);
    aggregate_if_consistent(
        raw_owned_supported_tops, &descriptor.raw_roi_vertical_valid,
        &descriptor.raw_roi_vertical_z);

    if (!diagnostic_owned_z.empty()) {
      descriptor.diagnostic_z05 = quantile(diagnostic_owned_z, 0.05);
      descriptor.diagnostic_z95 = quantile(diagnostic_owned_z, 0.95);
      descriptor.diagnostic_z_extent = descriptor.diagnostic_z95 -
          descriptor.diagnostic_z05;
      descriptor.diagnostic_z_extent_reliable =
          std::isfinite(descriptor.diagnostic_z_extent);
    }

    descriptor.valid_hypothesis_top_count = supported_tops.size();
    if (!supported_tops.empty()) {
      const auto bounds = std::minmax_element(supported_tops.begin(),
                                               supported_tops.end());
      descriptor.hypothesis_top_min = *bounds.first;
      descriptor.hypothesis_top_max = *bounds.second;
      descriptor.hypothesis_top_spread = *bounds.second - *bounds.first;
      if (descriptor.hypothesis_top_spread <= consistency_limit + kEpsilon) {
        descriptor.vertical_mode =
            CargoGroupVerticalMode::SUPPORTED_EVIDENCE;
        descriptor.physical_vertical_z = median(supported_tops);
        descriptor.vertical_source = selected_raw_evidence
            ? CargoVerticalEvidenceSource::RAW_ROI_CURRENT_FOOTPRINT
            : CargoVerticalEvidenceSource::COMPONENT_UNION;
        descriptor.vertical_reject_reason = selected_raw_evidence
            ? "SUPPORTED_RAW_ROI_OWNER_PROVEN_TOP_MEDIAN"
            : "SUPPORTED_HYPOTHESIS_TOP_MEDIAN";
      } else {
        descriptor.vertical_mode = CargoGroupVerticalMode::CONTINUITY_ONLY;
        descriptor.physical_vertical_z = descriptor.stable_anchor.z();
        descriptor.vertical_reject_reason =
            "CONFLICTING_HYPOTHESIS_SUPPORTED_TOPS";
      }
    } else if (continuity_available) {
      descriptor.vertical_mode = CargoGroupVerticalMode::CONTINUITY_ONLY;
      descriptor.physical_vertical_z = descriptor.stable_anchor.z();
      descriptor.vertical_reject_reason = "NO_SUPPORTED_HYPOTHESIS_TOP";
    } else {
      descriptor.vertical_mode = CargoGroupVerticalMode::INVALID;
      descriptor.vertical_reject_reason = group.group_ambiguous
          ? "PHYSICAL_GROUP_AMBIGUOUS"
          : component_lookup_complete ? "INSUFFICIENT_FINITE_GROUP_POINTS"
                                      : "COMPONENT_POINTS_UNAVAILABLE";
    }
    descriptor.valid = descriptor.vertical_mode !=
            CargoGroupVerticalMode::INVALID &&
        descriptor.stable_anchor.allFinite() &&
        descriptor.aggregate_extent.allFinite() &&
        (descriptor.aggregate_extent.array() > 0.0).all() &&
        descriptor.robust_xy_center.allFinite() &&
        descriptor.robust_xy_extent.allFinite() &&
        (descriptor.robust_xy_extent.array() > 0.0).all() &&
        std::isfinite(descriptor.physical_vertical_z) &&
        std::isfinite(descriptor.vertical_uncertainty_m);
    groups.push_back(std::move(group));
  }

  std::sort(groups.begin(), groups.end(),
            [](const CargoPhysicalGroupObservation& lhs,
               const CargoPhysicalGroupObservation& rhs) {
              return lhs.member_component_ids < rhs.member_component_ids;
            });
  for (std::size_t index = 0; index < groups.size(); ++index) {
    groups[index].frame_group_id = static_cast<std::uint64_t>(index + 1U);
  }
  return groups;
}

bool matchesResolvedPhysicalHypothesis(
    const CargoPhysicalCandidateObservation& candidate,
    const CargoPhysicalIdentityDecision& decision) {
  return decision.geometry_resolved && decision.resolved_candidate_id != 0U &&
      candidate.candidate_id == decision.resolved_candidate_id &&
      canonicalMembers(candidate.member_component_ids) ==
          canonicalMembers(decision.resolved_member_component_ids);
}

CargoPhysicalIdentityAuthority::CargoPhysicalIdentityAuthority(
    const CargoPhysicalIdentityConfig& config) {
  setConfig(config);
}

void CargoPhysicalIdentityAuthority::setConfig(
    const CargoPhysicalIdentityConfig& config) {
  config_ = config;
  if (!(config_.maximum_xy_step_m > 0.0)) config_.maximum_xy_step_m = 0.30;
  if (!(config_.maximum_z_speed_mps > 0.0)) config_.maximum_z_speed_mps = 1.50;
  if (!(config_.z_step_margin_m >= 0.0)) config_.z_step_margin_m = 0.05;
  if (!(config_.maximum_size_relative_step >= 0.0)) {
    config_.maximum_size_relative_step = 0.60;
  }
  if (!(config_.ambiguity_cost_margin >= 0.0)) {
    config_.ambiguity_cost_margin = 0.08;
  }
  if (!(config_.minimum_significant_change_m > 0.0)) {
    config_.minimum_significant_change_m = 0.15;
  }
  if (!(config_.significance_sigma > 0.0)) config_.significance_sigma = 3.0;
  if (!(config_.maximum_observation_gap_sec > 0.0)) {
    config_.maximum_observation_gap_sec = 0.50;
  }
  if (!(config_.maximum_source_age_sec >= 0.0)) {
    config_.maximum_source_age_sec = 0.50;
  }
  config_.lift_confirm_frames = std::max(1, config_.lift_confirm_frames);
  reset("config_changed");
}

void CargoPhysicalIdentityAuthority::reset(const std::string& reason) {
  histories_.clear();
  decision_ = CargoPhysicalIdentityDecision{};
  next_history_id_ = 1U;
  load_epoch_ = 0U;
  lifecycle_id_ = 0U;
  validated_history_id_ = 0U;
  initialized_ = false;
  previous_existence_phase_ = false;
  started_loaded_without_baseline_ = false;
  last_pipeline_stamp_sec_ = 0.0;
  reset_reason_ = reason;
}

CargoPhysicalIdentityDecision CargoPhysicalIdentityAuthority::update(
    const CargoPhysicalIdentityInput& input) {
  decision_ = CargoPhysicalIdentityDecision{};
  decision_.load_epoch = load_epoch_;
  const bool stamp_valid = std::isfinite(input.pipeline_stamp_sec) &&
      input.pipeline_stamp_sec > 0.0 &&
      (!initialized_ || input.pipeline_stamp_sec > last_pipeline_stamp_sec_);
  if (!stamp_valid) {
    decision_.reason = "source_time_invalid_or_not_advanced";
    return decision_;
  }
  decision_.valid_input = true;

  if (input.rearm || (initialized_ && input.lifecycle_id != lifecycle_id_)) {
    histories_.clear();
    validated_history_id_ = 0U;
    previous_existence_phase_ = false;
    started_loaded_without_baseline_ = false;
    ++load_epoch_;
  }
  lifecycle_id_ = input.lifecycle_id;
  last_pipeline_stamp_sec_ = input.pipeline_stamp_sec;
  initialized_ = true;

  const bool gravity_loaded = input.gravity_valid &&
      input.gravity_state == HookLoadState::LOADED;
  const bool gravity_empty = input.gravity_valid &&
      input.gravity_state == HookLoadState::EMPTY;
  const bool pre_load_phase = gravity_empty;
  const bool gravity_load_edge = gravity_loaded && !previous_existence_phase_;
  if (gravity_load_edge) {
    ++load_epoch_;
    started_loaded_without_baseline_ = input.node_started_loaded &&
        std::none_of(histories_.begin(), histories_.end(),
                     [](const History& history) { return history.has_preload; });
    for (History& history : histories_) {
      history.lift_confirm_count = 0;
      history.lift_confirmed = false;
      history.validation_stamp_sec = 0.0;
      history.last_supported_evidence_stamp_sec = 0.0;
      history.baseline_frozen = false;
      history.baseline_z95 = std::numeric_limits<double>::quiet_NaN();
      history.baseline_stamp_sec = 0.0;
      if (history.has_preload) {
        history.baseline_frozen = true;
        history.baseline_source =
            CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE;
        history.baseline_z95 = history.preload_z95;
        history.baseline_uncertainty_m = history.preload_uncertainty_m;
        history.baseline_stamp_sec = history.preload_stamp_sec;
        history.has_preload = false;
      }
    }
  }

  decision_.group_diagnostics.reserve(input.groups.size());
  for (const auto& group : input.groups) {
    CargoPhysicalGroupDiagnostic diagnostic;
    diagnostic.frame_group_id = group.frame_group_id;
    diagnostic.member_component_ids = group.member_component_ids;
    diagnostic.raw_representative = group.representative.center;
    diagnostic.descriptor = group.descriptor;
    decision_.group_diagnostics.push_back(std::move(diagnostic));
  }

  struct Pair {
    std::size_t group = 0U;
    std::size_t history = 0U;
    bool feasible = false;
    bool requires_post_unique_z = false;
    double cost = std::numeric_limits<double>::infinity();
    double raw_xy = std::numeric_limits<double>::quiet_NaN();
    double xy = std::numeric_limits<double>::quiet_NaN();
    double support_xy_separation =
        std::numeric_limits<double>::quiet_NaN();
    double dz = std::numeric_limits<double>::quiet_NaN();
    double extent_step = std::numeric_limits<double>::quiet_NaN();
    double xy_cost = std::numeric_limits<double>::quiet_NaN();
    double z_cost = std::numeric_limits<double>::quiet_NaN();
    double extent_cost = std::numeric_limits<double>::quiet_NaN();
    CargoPhysicalAssociationMode association_mode =
        CargoPhysicalAssociationMode::NEW_HISTORY;
    std::string reject_reason = "NO_HISTORY";
  };
  std::vector<Pair> pairs;
  for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
    const auto& group = input.groups[gi];
    if (group.group_ambiguous || !finiteDescriptor(group.descriptor)) {
      decision_.group_diagnostics[gi].association_reject_reason =
          group.group_ambiguous ? "PHYSICAL_GROUP_AMBIGUOUS"
                                : "VERTICAL_INVALID";
      decision_.group_diagnostics[gi].new_history_reason =
          decision_.group_diagnostics[gi].association_reject_reason;
      continue;
    }
    for (std::size_t hi = 0; hi < histories_.size(); ++hi) {
      const History& history = histories_[hi];
      Pair pair;
      pair.group = gi;
      pair.history = hi;
      const double dt = group.descriptor.stamp_sec - history.last_stamp_sec;
      if (!(dt > 0.0) || dt > config_.maximum_observation_gap_sec) {
        pair.reject_reason = "GAP";
        pairs.push_back(pair);
        continue;
      }
      const auto& previous = history.last_descriptor;
      pair.raw_xy = (group.representative.center.head<2>() -
                     history.last_representative_center.head<2>()).norm();
      pair.xy = (group.descriptor.robust_xy_center -
                 previous.robust_xy_center).norm();
      const double support_x_separation = intervalSeparation(
          group.descriptor.robust_x05, group.descriptor.robust_x95,
          previous.robust_x05, previous.robust_x95);
      const double support_y_separation = intervalSeparation(
          group.descriptor.robust_y05, group.descriptor.robust_y95,
          previous.robust_y05, previous.robust_y95);
      pair.support_xy_separation = std::hypot(
          support_x_separation, support_y_separation);
      if (pair.xy <= config_.maximum_xy_step_m) {
        pair.association_mode =
            CargoPhysicalAssociationMode::ANCHOR_CONTINUITY;
      } else if (hasPositiveAreaSupportOverlap(
                     group.descriptor, previous)) {
        pair.association_mode =
            CargoPhysicalAssociationMode::SUPPORT_OVERLAP_CONTINUITY;
      }
      const double z_limit = config_.maximum_z_speed_mps * dt +
          config_.z_step_margin_m + group.descriptor.vertical_uncertainty_m +
          previous.vertical_uncertainty_m;
      pair.dz = std::abs(group.descriptor.physical_vertical_z -
                         previous.physical_vertical_z);
      pair.requires_post_unique_z = group.descriptor.vertical_mode ==
          CargoGroupVerticalMode::CONTINUITY_ONLY;
      pair.extent_step = 0.0;
      pair.extent_cost = 0.0;
      bool extent_ok = true;
      const int extent_axis_count = pair.requires_post_unique_z ? 2 : 3;
      for (int axis = 0; axis < extent_axis_count; ++axis) {
        const double current_extent = axis < 2
            ? group.descriptor.robust_xy_extent[axis]
            : group.descriptor.aggregate_extent[axis];
        const double previous_extent = axis < 2
            ? previous.robust_xy_extent[axis]
            : previous.aggregate_extent[axis];
        const double denominator = std::max(
            std::max(std::abs(current_extent),
                     std::abs(previous_extent)), kEpsilon);
        const double relative = std::abs(
            current_extent - previous_extent) / denominator;
        pair.extent_step = std::max(pair.extent_step, relative);
        pair.extent_cost += relative;
        extent_ok = extent_ok &&
            relative <= config_.maximum_size_relative_step;
      }
      const double association_xy = pair.association_mode ==
              CargoPhysicalAssociationMode::ANCHOR_CONTINUITY
          ? pair.xy : pair.support_xy_separation;
      pair.xy_cost = association_xy / config_.maximum_xy_step_m;
      pair.z_cost = pair.requires_post_unique_z
          ? 0.0 : pair.dz / std::max(z_limit, kEpsilon);
      pair.cost = pair.xy_cost + pair.z_cost + pair.extent_cost;
      if (pair.association_mode ==
          CargoPhysicalAssociationMode::NEW_HISTORY) {
        pair.reject_reason = "XY_GATE";
      } else if (!pair.requires_post_unique_z && pair.dz > z_limit) {
        pair.reject_reason = "Z_GATE";
      } else if (!extent_ok) {
        pair.reject_reason = "EXTENT_GATE";
      } else {
        pair.feasible = true;
        pair.reject_reason = "NONE";
      }
      pairs.push_back(pair);
    }
  }

  for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
    Pair* best = nullptr;
    for (Pair& pair : pairs) {
      if (pair.group != gi) continue;
      if (!best || pair.cost < best->cost) best = &pair;
    }
    if (!best) continue;
    auto& diagnostic = decision_.group_diagnostics[gi];
    diagnostic.raw_representative_xy_step_m = best->raw_xy;
    diagnostic.xy_step_m = best->xy;
    diagnostic.support_xy_separation_m = best->support_xy_separation;
    diagnostic.z_step_m = best->dz;
    diagnostic.extent_step = best->extent_step;
    diagnostic.xy_cost = best->xy_cost;
    diagnostic.z_cost = best->z_cost;
    diagnostic.extent_cost = best->extent_cost;
    diagnostic.association_mode = best->association_mode;
    diagnostic.association_reject_reason = best->reject_reason;
    diagnostic.new_history_reason = best->reject_reason;
  }

  std::vector<int> group_match(input.groups.size(), -1);
  std::vector<bool> group_ambiguous(input.groups.size(), false);
  for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
    double best = std::numeric_limits<double>::infinity();
    int best_hi = -1;
    for (const Pair& pair : pairs) {
      if (pair.group == gi && pair.feasible && pair.cost + kEpsilon < best) {
        best = pair.cost;
        best_hi = static_cast<int>(pair.history);
      }
    }
    const int competitive_history_count = static_cast<int>(std::count_if(
        pairs.begin(), pairs.end(), [&](const Pair& pair) {
          return pair.group == gi && pair.feasible &&
              pair.cost <= best + config_.ambiguity_cost_margin;
        }));
    if (competitive_history_count > 1) {
      group_ambiguous[gi] = true;
      continue;
    }
    if (best_hi < 0) continue;
    int history_best_group = -1;
    double history_best = std::numeric_limits<double>::infinity();
    for (const Pair& pair : pairs) {
      if (pair.history == static_cast<std::size_t>(best_hi) && pair.feasible &&
          pair.cost + kEpsilon < history_best) {
        history_best = pair.cost;
        history_best_group = static_cast<int>(pair.group);
      }
    }
    const int competitive_group_count = static_cast<int>(std::count_if(
        pairs.begin(), pairs.end(), [&](const Pair& pair) {
          return pair.history == static_cast<std::size_t>(best_hi) &&
              pair.feasible && pair.cost <=
                  history_best + config_.ambiguity_cost_margin;
        }));
    if (competitive_group_count == 1 && history_best_group ==
        static_cast<int>(gi)) {
      group_match[gi] = best_hi;
    } else {
      group_ambiguous[gi] = true;
    }
  }

  // CONTINUITY_ONLY never selects a history by recovered Z. Reciprocal
  // uniqueness is established from current-frame XY/extent evidence first;
  // only that already-unique pair may use the prior supported footprint to
  // complete its Z gate from the current RAW ROI.
  for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
    if (group_match[gi] < 0) continue;
    Pair* matched_pair = nullptr;
    for (Pair& pair : pairs) {
      if (pair.group == gi && pair.history ==
              static_cast<std::size_t>(group_match[gi]) && pair.feasible) {
        matched_pair = &pair;
        break;
      }
    }
    if (!matched_pair || !matched_pair->requires_post_unique_z) continue;
    History& history = histories_[matched_pair->history];
    const auto& group = input.groups[gi];
    const double association_dt =
        group.descriptor.stamp_sec - history.last_stamp_sec;
    double z_limit = config_.maximum_z_speed_mps * association_dt +
        config_.z_step_margin_m +
        group.descriptor.vertical_uncertainty_m +
        history.last_descriptor.vertical_uncertainty_m;
    double z_step = std::abs(group.descriptor.physical_vertical_z -
                             history.last_descriptor.physical_vertical_z);

    auto& diagnostic = decision_.group_diagnostics[gi];
    const double supported_gap = history.last_supported_evidence_stamp_sec > 0.0
        ? group.descriptor.stamp_sec -
              history.last_supported_evidence_stamp_sec
        : std::numeric_limits<double>::infinity();
    if (history.last_supported_footprint.valid &&
        supported_gap > 0.0 &&
        supported_gap <= config_.maximum_observation_gap_sec &&
        input.frame_evidence.raw_roi_current_frame &&
        std::abs(input.frame_evidence.source_stamp_sec -
                 group.descriptor.stamp_sec) <= kEpsilon) {
      diagnostic.reacquired_vertical_attempted = true;
      const auto reacquire_start = std::chrono::steady_clock::now();
      diagnostic.reacquired_vertical = reacquireAssociationVertical(
          input.frame_evidence, history.last_supported_footprint,
          group.union_points_base, input.vertical_config,
          history.last_supported_vertical_uncertainty_m);
      const double reacquire_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - reacquire_start).count();
      decision_.reacquired_vertical_compute_ms += reacquire_ms;
      ++decision_.reacquired_vertical_attempt_count;
      decision_.reacquired_vertical_points_examined +=
          input.frame_evidence.raw_roi_current_frame->size();
      if (diagnostic.reacquired_vertical.valid &&
          std::isfinite(history.last_supported_vertical_z)) {
        z_limit = config_.maximum_z_speed_mps * supported_gap +
            config_.z_step_margin_m +
            diagnostic.reacquired_vertical.uncertainty_m +
            history.last_supported_vertical_uncertainty_m;
        z_step = std::abs(diagnostic.reacquired_vertical.top_z_base -
                          history.last_supported_vertical_z);
      }
    }
    matched_pair->dz = z_step;
    matched_pair->z_cost = z_step / std::max(z_limit, kEpsilon);
    matched_pair->cost += matched_pair->z_cost;
    diagnostic.z_step_m = z_step;
    diagnostic.z_cost = matched_pair->z_cost;
    if (z_step > z_limit) {
      matched_pair->feasible = false;
      matched_pair->reject_reason = "Z_GATE";
      diagnostic.association_reject_reason = "Z_GATE";
      diagnostic.new_history_reason = "Z_GATE";
      group_match[gi] = -1;
    }
  }

  CargoCandidateAssociationState frame_association =
      CargoCandidateAssociationState::NEW_HISTORY;
  bool frame_has_any_ambiguity = false;
  std::vector<std::uint64_t> group_history_ids(input.groups.size(), 0U);
  for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
    const auto& group = input.groups[gi];
    auto& diagnostic = decision_.group_diagnostics[gi];
    if (group.group_ambiguous || group_ambiguous[gi]) {
      frame_has_any_ambiguity = true;
      frame_association = CargoCandidateAssociationState::AMBIGUOUS;
      diagnostic.association = CargoCandidateAssociationState::AMBIGUOUS;
      diagnostic.association_mode =
          CargoPhysicalAssociationMode::NEW_HISTORY;
      diagnostic.association_reject_reason = "AMBIGUOUS";
      diagnostic.new_history_reason = "AMBIGUOUS";
      continue;
    }
    if (!finiteDescriptor(group.descriptor)) continue;

    History* history = nullptr;
    if (group_match[gi] >= 0) {
      history = &histories_[static_cast<std::size_t>(group_match[gi])];
      if (frame_association != CargoCandidateAssociationState::AMBIGUOUS) {
        frame_association = CargoCandidateAssociationState::MATCHED;
      }
      diagnostic.association = CargoCandidateAssociationState::MATCHED;
      diagnostic.association_reject_reason = "NONE";
      diagnostic.new_history_reason = "NONE";
      for (const Pair& pair : pairs) {
        if (pair.group == gi && pair.history ==
                static_cast<std::size_t>(group_match[gi]) && pair.feasible) {
          diagnostic.association_mode = pair.association_mode;
          diagnostic.raw_representative_xy_step_m = pair.raw_xy;
          diagnostic.xy_step_m = pair.xy;
          diagnostic.support_xy_separation_m =
              pair.support_xy_separation;
          diagnostic.z_step_m = pair.dz;
          diagnostic.extent_step = pair.extent_step;
          diagnostic.xy_cost = pair.xy_cost;
          diagnostic.z_cost = pair.z_cost;
          diagnostic.extent_cost = pair.extent_cost;
          break;
        }
      }
    } else {
      histories_.push_back(History{});
      history = &histories_.back();
      history->id = next_history_id_++;
      diagnostic.association = CargoCandidateAssociationState::NEW_HISTORY;
      diagnostic.association_mode =
          CargoPhysicalAssociationMode::NEW_HISTORY;
      if (histories_.size() == 1U) {
        diagnostic.association_reject_reason = "NO_HISTORY";
        diagnostic.new_history_reason = "NO_HISTORY";
      }
    }
    group_history_ids[gi] = history->id;
    diagnostic.matched_history_id = history->id;
    history->association_ambiguous = false;
    history->last_descriptor = group.descriptor;
    history->last_representative_center = group.representative.center;
    history->last_stamp_sec = group.descriptor.stamp_sec;

    const bool supported = group.descriptor.vertical_mode ==
        CargoGroupVerticalMode::SUPPORTED_EVIDENCE;
    if (pre_load_phase && supported) {
      history->has_preload = true;
      history->preload_z95 = group.descriptor.physical_vertical_z;
      history->preload_uncertainty_m =
          group.descriptor.vertical_uncertainty_m;
      history->preload_stamp_sec = group.descriptor.stamp_sec;
    }
    const bool strict_lidar_existence_path =
        input.hook_role != HookLoadSignalRole::REQUIRED;
    if (!history->baseline_frozen &&
        (!pre_load_phase || strict_lidar_existence_path) &&
        !started_loaded_without_baseline_ && supported) {
      history->baseline_frozen = true;
      history->baseline_source = history->has_preload &&
              !strict_lidar_existence_path
          ? CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE
          : CargoLiftBaselineSource::POST_LOAD_FIRST_FRESH_OBSERVATION;
      history->baseline_z95 = history->has_preload &&
              !strict_lidar_existence_path
          ? history->preload_z95 : group.descriptor.physical_vertical_z;
      history->baseline_uncertainty_m = history->has_preload &&
              !strict_lidar_existence_path
          ? history->preload_uncertainty_m
          : group.descriptor.vertical_uncertainty_m;
      history->baseline_stamp_sec = history->has_preload &&
              !strict_lidar_existence_path
          ? history->preload_stamp_sec : group.descriptor.stamp_sec;
    }

    if (!supported) {
      if (history->last_supported_evidence_stamp_sec > 0.0 &&
          group.descriptor.stamp_sec -
              history->last_supported_evidence_stamp_sec >
                  config_.maximum_observation_gap_sec) {
        history->lift_confirm_count = 0;
        history->lift_confirmed = false;
        history->validation_stamp_sec = 0.0;
      }
      continue;
    }

    const double supported_gap = group.descriptor.stamp_sec -
        history->last_supported_evidence_stamp_sec;
    if (history->last_supported_evidence_stamp_sec > 0.0 &&
        supported_gap > config_.maximum_observation_gap_sec) {
      history->lift_confirm_count = 0;
      history->lift_confirmed = false;
      history->validation_stamp_sec = 0.0;
    }
    if (!history->baseline_frozen) continue;
    const double threshold = std::max(
        config_.minimum_significant_change_m,
        config_.significance_sigma * std::hypot(
            history->baseline_uncertainty_m,
            group.descriptor.vertical_uncertainty_m));
    const double delta = group.descriptor.physical_vertical_z -
        history->baseline_z95;
    const bool evidence_advanced = group.descriptor.stamp_sec >
        history->last_supported_evidence_stamp_sec + kEpsilon;
    if (evidence_advanced) {
      history->last_supported_evidence_stamp_sec =
          group.descriptor.stamp_sec;
      history->last_supported_vertical_z =
          group.descriptor.physical_vertical_z;
      history->last_supported_vertical_uncertainty_m =
          group.descriptor.vertical_uncertainty_m;
      if (group.geometry_resolved && !group.group_ambiguous) {
        history->last_supported_footprint = footprintSnapshot(
            group.representative, group.descriptor.stamp_sec);
      }
      if (delta >= threshold) {
        ++history->lift_confirm_count;
        const int required = requiredFrames(
            input.hook_role, input.gravity_valid, input.gravity_state,
            config_.lift_confirm_frames);
        if (history->lift_confirm_count >= required &&
            !history->lift_confirmed) {
          history->lift_confirmed = true;
          history->validation_stamp_sec = input.pipeline_stamp_sec;
        }
      } else if (!history->lift_confirmed) {
        const double retention_margin = config_.significance_sigma *
            group.descriptor.vertical_uncertainty_m;
        const double retention_floor =
            std::max(0.0, threshold - retention_margin);
        if (delta < retention_floor) history->lift_confirm_count = 0;
      }
      if (delta < -threshold) {
        history->lift_confirm_count = 0;
        history->lift_confirmed = false;
        history->validation_stamp_sec = 0.0;
      }
    }
  }

  std::vector<History*> fresh_confirmed;
  for (History& history : histories_) {
    const double association_age =
        input.pipeline_stamp_sec - history.last_stamp_sec;
    const double supported_age = history.last_supported_evidence_stamp_sec > 0.0
        ? input.pipeline_stamp_sec - history.last_supported_evidence_stamp_sec
        : std::numeric_limits<double>::infinity();
    const bool fresh = association_age >= -kEpsilon &&
        association_age <= config_.maximum_source_age_sec &&
        supported_age >= -kEpsilon &&
        supported_age <= config_.maximum_observation_gap_sec;
    if (history.lift_confirmed && fresh && !history.association_ambiguous) {
      fresh_confirmed.push_back(&history);
    }
  }

  const std::uint64_t protected_history_id = fresh_confirmed.size() == 1U
      ? fresh_confirmed.front()->id : validated_history_id_;
  bool validated_history_conflict = false;
  bool frame_has_unrelated_ambiguity = false;
  for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
    if (!input.groups[gi].group_ambiguous && !group_ambiguous[gi]) continue;
    bool conflicts_with_protected_history = false;
    if (protected_history_id != 0U) {
      for (const Pair& pair : pairs) {
        if (pair.group == gi && pair.feasible &&
            histories_[pair.history].id == protected_history_id) {
          conflicts_with_protected_history = true;
          break;
        }
      }
      // A same-frame physical-group ambiguity is forbidden from association,
      // but its current support can still prove that the ambiguity directly
      // occupies the protected history's physical gate. This is revocation
      // evidence only; it never transfers history or lift evidence.
      if (!conflicts_with_protected_history &&
          input.groups[gi].group_ambiguous) {
        const auto& current = input.groups[gi].descriptor;
        const History* protected_history = nullptr;
        for (const History& history : histories_) {
          if (history.id == protected_history_id) {
            protected_history = &history;
            break;
          }
        }
        if (protected_history && current.stable_anchor.allFinite() &&
            current.robust_xy_center.allFinite() &&
            current.robust_xy_extent.allFinite() &&
            (current.robust_xy_extent.array() > 0.0).all() &&
            current.aggregate_extent.allFinite() &&
            std::isfinite(current.vertical_uncertainty_m)) {
          const auto& previous = protected_history->last_descriptor;
          const double dt = current.stamp_sec -
              protected_history->last_stamp_sec;
          const double center_step = (current.robust_xy_center -
                                      previous.robust_xy_center).norm();
          const bool positive_area_overlap =
              hasPositiveAreaSupportOverlap(current, previous);
          const double z_limit = config_.maximum_z_speed_mps * dt +
              config_.z_step_margin_m + current.vertical_uncertainty_m +
              previous.vertical_uncertainty_m;
          bool extent_ok = true;
          const int extent_axis_count = current.vertical_mode ==
                  CargoGroupVerticalMode::CONTINUITY_ONLY
              ? 2 : 3;
          for (int axis = 0; axis < extent_axis_count; ++axis) {
            const double current_extent = axis < 2
                ? current.robust_xy_extent[axis]
                : current.aggregate_extent[axis];
            const double previous_extent = axis < 2
                ? previous.robust_xy_extent[axis]
                : previous.aggregate_extent[axis];
            const double denominator = std::max(
                std::max(std::abs(current_extent),
                         std::abs(previous_extent)), kEpsilon);
            extent_ok = extent_ok &&
                std::abs(current_extent - previous_extent) / denominator <=
                    config_.maximum_size_relative_step;
          }
          conflicts_with_protected_history = dt > 0.0 &&
              dt <= config_.maximum_observation_gap_sec &&
              (center_step <= config_.maximum_xy_step_m ||
               positive_area_overlap) &&
              std::abs(current.stable_anchor.z() -
                       previous.stable_anchor.z()) <= z_limit && extent_ok;
        }
      }
    }
    auto& diagnostic = decision_.group_diagnostics[gi];
    diagnostic.validated_history_conflict =
        conflicts_with_protected_history;
    diagnostic.conflicting_history_id = conflicts_with_protected_history
        ? protected_history_id : 0U;
    validated_history_conflict = validated_history_conflict ||
        conflicts_with_protected_history;
    frame_has_unrelated_ambiguity = frame_has_unrelated_ambiguity ||
        !conflicts_with_protected_history;
  }
  for (auto& diagnostic : decision_.group_diagnostics) {
    diagnostic.frame_has_unrelated_ambiguity =
        frame_has_unrelated_ambiguity;
  }

  if (input.hook_role == HookLoadSignalRole::REQUIRED) {
    decision_.cargo_exists = gravity_loaded;
    decision_.existence_source = gravity_loaded
        ? CargoExistenceSource::GRAVITY_LOADED : CargoExistenceSource::NONE;
  } else if (gravity_loaded) {
    decision_.cargo_exists = true;
    decision_.existence_source = CargoExistenceSource::GRAVITY_LOADED;
  } else {
    decision_.cargo_exists = !fresh_confirmed.empty();
    decision_.existence_source = decision_.cargo_exists
        ? CargoExistenceSource::STRICT_LIDAR : CargoExistenceSource::NONE;
  }

  if (gravity_empty && input.hook_role == HookLoadSignalRole::REQUIRED) {
    validated_history_id_ = 0U;
  }
  if (decision_.cargo_exists && fresh_confirmed.size() == 1U &&
      !validated_history_conflict) {
    validated_history_id_ = fresh_confirmed.front()->id;
  } else if (fresh_confirmed.size() != 1U || validated_history_conflict) {
    validated_history_id_ = 0U;
  }

  if (validated_history_conflict || fresh_confirmed.size() > 1U) {
    frame_association = CargoCandidateAssociationState::AMBIGUOUS;
  } else if (validated_history_id_ != 0U) {
    frame_association = CargoCandidateAssociationState::NEW_HISTORY;
    for (std::size_t gi = 0; gi < group_history_ids.size(); ++gi) {
      if (group_history_ids[gi] == validated_history_id_) {
        frame_association = decision_.group_diagnostics[gi].association;
        break;
      }
    }
  } else if (frame_has_any_ambiguity) {
    frame_association = CargoCandidateAssociationState::AMBIGUOUS;
  }

  decision_.association = frame_association;
  decision_.load_epoch = load_epoch_;
  if (started_loaded_without_baseline_) {
    decision_.baseline_source =
        CargoLiftBaselineSource::UNAVAILABLE_STARTED_LOADED;
    decision_.identity = CargoPhysicalIdentityState::UNKNOWN;
    decision_.reason = "identity_evidence_unavailable_started_loaded";
  } else if (fresh_confirmed.size() > 1U || validated_history_conflict ||
             (fresh_confirmed.empty() && frame_has_any_ambiguity)) {
    decision_.identity = CargoPhysicalIdentityState::AMBIGUOUS;
    decision_.reason = "multiple_or_ambiguous_physical_groups";
  } else if (validated_history_id_ != 0U && decision_.cargo_exists) {
    History* selected = nullptr;
    for (History& history : histories_) {
      if (history.id == validated_history_id_) selected = &history;
    }
    if (selected) {
      const CargoPhysicalGroupObservation* current_group = nullptr;
      for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
        if (group_history_ids[gi] == selected->id) {
          current_group = &input.groups[gi];
          break;
        }
      }
      decision_.identity = CargoPhysicalIdentityState::VALIDATED;
      decision_.physical_history_id = selected->id;
      decision_.frame_group_id = current_group
          ? current_group->frame_group_id : 0U;
      decision_.current_vertical_mode = current_group
          ? current_group->descriptor.vertical_mode
          : CargoGroupVerticalMode::INVALID;
      decision_.current_vertical_evidence_valid = current_group &&
          current_group->descriptor.vertical_mode ==
              CargoGroupVerticalMode::SUPPORTED_EVIDENCE;
      decision_.geometry_resolved = current_group &&
          current_group->geometry_resolved && !current_group->group_ambiguous &&
          decision_.current_vertical_evidence_valid;
      decision_.resolved_candidate_id = decision_.geometry_resolved
          ? current_group->representative.candidate_id : 0U;
      decision_.resolved_member_component_ids = decision_.geometry_resolved
          ? current_group->representative.member_component_ids
          : std::vector<std::uint64_t>{};
      decision_.baseline_source = selected->baseline_source;
      decision_.current_candidate_fresh = current_group != nullptr;
      decision_.lift_confirmed = selected->lift_confirmed;
      decision_.lift_confirm_count = selected->lift_confirm_count;
      decision_.required_lift_confirm_frames = requiredFrames(
          input.hook_role, input.gravity_valid, input.gravity_state,
          config_.lift_confirm_frames);
      decision_.baseline_z95 = selected->baseline_z95;
      decision_.current_z95 = current_group
          ? current_group->descriptor.physical_vertical_z
          : std::numeric_limits<double>::quiet_NaN();
      if (decision_.current_vertical_evidence_valid) {
        decision_.lift_delta_m =
            decision_.current_z95 - decision_.baseline_z95;
        decision_.lift_threshold_m = std::max(
            config_.minimum_significant_change_m,
            config_.significance_sigma * std::hypot(
                selected->baseline_uncertainty_m,
                current_group->descriptor.vertical_uncertainty_m));
      }
      decision_.evidence_age_sec = input.pipeline_stamp_sec -
          selected->last_supported_evidence_stamp_sec;
      decision_.last_supported_evidence_stamp =
          selected->last_supported_evidence_stamp_sec;
      decision_.identity_validation_stamp_sec =
          selected->validation_stamp_sec;
      decision_.reason = decision_.geometry_resolved
          ? "candidate_specific_lift_validated"
          : decision_.current_vertical_evidence_valid
              ? "physical_identity_validated_geometry_ambiguous"
              : "physical_identity_retained_vertical_continuity_only";
    }
  } else {
    decision_.identity = CargoPhysicalIdentityState::UNKNOWN;
    decision_.reason = decision_.cargo_exists
        ? "existence_without_candidate_identity"
        : "cargo_existence_not_proven";
  }

  for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
    auto& diagnostic = decision_.group_diagnostics[gi];
    if (diagnostic.association == CargoCandidateAssociationState::AMBIGUOUS) {
      diagnostic.identity = CargoPhysicalIdentityState::AMBIGUOUS;
      continue;
    }
    for (const History& history : histories_) {
      if (history.id != group_history_ids[gi]) continue;
      diagnostic.baseline_source = history.baseline_source;
      diagnostic.baseline_z = history.baseline_z95;
      diagnostic.last_supported_evidence_stamp =
          history.last_supported_evidence_stamp_sec;
      diagnostic.lift_confirm_count = history.lift_confirm_count;
      diagnostic.lift_confirm_required = requiredFrames(
          input.hook_role, input.gravity_valid, input.gravity_state,
          config_.lift_confirm_frames);
      diagnostic.lift_confirmed = history.lift_confirmed;
      if (history.baseline_frozen &&
          input.groups[gi].descriptor.vertical_mode ==
              CargoGroupVerticalMode::SUPPORTED_EVIDENCE) {
        diagnostic.lift_delta_m =
            input.groups[gi].descriptor.physical_vertical_z -
            history.baseline_z95;
        diagnostic.lift_threshold_m = std::max(
            config_.minimum_significant_change_m,
            config_.significance_sigma * std::hypot(
                history.baseline_uncertainty_m,
                input.groups[gi].descriptor.vertical_uncertainty_m));
      }
      diagnostic.identity = history.id == validated_history_id_
          ? CargoPhysicalIdentityState::VALIDATED
          : CargoPhysicalIdentityState::UNKNOWN;
      break;
    }
  }

  previous_existence_phase_ = gravity_loaded || decision_.cargo_exists;
  return decision_;
}

}  // namespace ndt_slam
