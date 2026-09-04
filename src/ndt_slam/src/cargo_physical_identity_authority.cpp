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

double medianAbsoluteDeviation(const std::vector<double>& values,
                               double center) {
  std::vector<double> deviations;
  deviations.reserve(values.size());
  for (double value : values) deviations.push_back(std::abs(value - center));
  return median(std::move(deviations));
}

double liftSignificanceThreshold(
    const CargoPhysicalIdentityConfig& config,
    double baseline_uncertainty_m,
    double current_uncertainty_m) {
  return std::max(
      config.minimum_significant_change_m,
      config.significance_sigma * std::hypot(
          std::max(0.0, baseline_uncertainty_m),
          std::max(0.0, current_uncertainty_m)));
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

CargoFootprintSnapshot robustFootprintSnapshot(
    const CargoPhysicalGroupObservation& group) {
  CargoFootprintSnapshot snapshot;
  snapshot.valid = group.geometry_resolved && !group.group_ambiguous &&
      group.descriptor.robust_xy_center.allFinite() &&
      group.descriptor.robust_xy_extent.allFinite() &&
      (group.descriptor.robust_xy_extent.array() > 0.0).all() &&
      std::isfinite(group.representative.yaw_rad) &&
      std::isfinite(group.descriptor.stamp_sec) &&
      group.descriptor.stamp_sec > 0.0;
  snapshot.center_base =
      group.descriptor.robust_xy_center.cast<float>();
  snapshot.size_xy = group.descriptor.robust_xy_extent.cast<float>();
  snapshot.yaw_base_rad = static_cast<float>(group.representative.yaw_rad);
  snapshot.source_stamp_sec = group.descriptor.stamp_sec;
  return snapshot;
}

AssociationOnlyReacquiredVerticalEvidence reacquireAssociationVertical(
    const CargoShadowFrameEvidence& frame,
    const CargoFootprintSnapshot& footprint,
    const std::vector<Eigen::Vector3f>& owner_points,
    const CargoVerticalEvidenceConfig& config,
    double uncertainty_m,
    const std::vector<Eigen::Vector3f>* competing_owner_points = nullptr) {
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
  if (competing_owner_points && !competing_owner_points->empty()) {
    std::set<CargoFootprintGridIndex> competing_cells;
    for (const Eigen::Vector3f& point : *competing_owner_points) {
      if (!point.allFinite() || !cargoPointInsideFootprint(
              point, input, config.footprint_margin_m)) {
        continue;
      }
      competing_cells.insert(makeCargoFootprintGridIndex(
          point, input, config.xy_cell_size_m));
    }
    const bool competing_column_support = std::any_of(
        evidence.top_surface_cell_indices.begin(),
        evidence.top_surface_cell_indices.end(),
        [&](const CargoFootprintGridIndex& cell) {
          return competing_cells.count(cell) > 0U;
        });
    if (competing_column_support) {
      result.reason = "raw_history_footprint_competing_owner_column";
      return result;
    }
  }
  result.valid = true;
  result.top_z_base = evidence.top_z_base;
  result.reason = "association_only_reacquired_vertical_valid";
  return result;
}

// V3.1 diagnostic counterfactual: OWNER_LOCKED_FROZEN_FOOTPRINT_RAW_ROI.
// The current exact owner selects the surface cells from its own points; the
// RAW ROI only measures the dense Z within those owner-authorized cells.
// Historical Z, prediction Z, gravity Z and lineage Z never contribute.
struct OwnerLockedSurfaceResult {
  bool valid = false;
  double surface_z = std::numeric_limits<double>::quiet_NaN();
  double surface_uncertainty = std::numeric_limits<double>::quiet_NaN();
  std::size_t owner_cells = 0U;
  std::size_t frozen_cells = 0U;
  std::size_t authorized_cells = 0U;
  std::size_t raw_points_measured = 0U;
  std::string reject_reason = "not_evaluated";
};

OwnerLockedSurfaceResult computeOwnerLockedSurfaceVertical(
    const CargoShadowFrameEvidence& frame,
    const CargoFootprintSnapshot& footprint,
    const std::vector<CargoFootprintGridIndex>& frozen_owner_cells,
    const std::vector<Eigen::Vector3f>& owner_points,
    const CargoVerticalEvidenceConfig& config,
    const std::vector<Eigen::Vector3f>* competing_owner_points) {
  OwnerLockedSurfaceResult result;
  if (!footprint.valid || !frame.raw_roi_current_frame ||
      !std::isfinite(frame.source_stamp_sec) ||
      frame.source_stamp_sec <= 0.0) {
    result.reject_reason = "NO_FROZEN_FOOTPRINT";
    return result;
  }
  CargoVerticalEvidenceInput input;
  input.footprint_valid = true;
  input.footprint_center_base = footprint.center_base;
  input.footprint_size_xy = footprint.size_xy;
  input.footprint_yaw_base_rad = footprint.yaw_base_rad;
  input.ground_reference_valid = frame.ground_reference_valid;
  input.ground_z_base = frame.ground_z_base;

  std::set<CargoFootprintGridIndex> current_owner_cells;
  for (const Eigen::Vector3f& point : owner_points) {
    if (!point.allFinite() || !cargoPointInsideFootprint(
            point, input, config.footprint_margin_m)) {
      continue;
    }
    current_owner_cells.insert(makeCargoFootprintGridIndex(
        point, input, config.xy_cell_size_m));
  }
  result.owner_cells = current_owner_cells.size();
  if (current_owner_cells.empty()) {
    result.reject_reason = "NO_CURRENT_EXACT_OWNER";
    return result;
  }

  std::set<CargoFootprintGridIndex> authorized_cells;
  if (frozen_owner_cells.empty()) {
    authorized_cells = current_owner_cells;
  } else {
    result.frozen_cells = frozen_owner_cells.size();
    for (const CargoFootprintGridIndex& cell : frozen_owner_cells) {
      if (current_owner_cells.count(cell) > 0U) {
        authorized_cells.insert(cell);
      }
    }
  }
  result.authorized_cells = authorized_cells.size();
  if (authorized_cells.empty()) {
    result.reject_reason = "BASELINE_CELL_OVERLAP_INSUFFICIENT";
    return result;
  }

  std::set<CargoFootprintGridIndex> competing_cells;
  if (competing_owner_points != nullptr) {
    for (const Eigen::Vector3f& point : *competing_owner_points) {
      if (!point.allFinite() || !cargoPointInsideFootprint(
              point, input, config.footprint_margin_m)) {
        continue;
      }
      competing_cells.insert(makeCargoFootprintGridIndex(
          point, input, config.xy_cell_size_m));
    }
  }

  std::vector<Eigen::Vector3f> authorized_raw_points;
  for (const pcl::PointXYZ& point : frame.raw_roi_current_frame->points) {
    const Eigen::Vector3f p(point.x, point.y, point.z);
    if (!p.allFinite() || !cargoPointInsideFootprint(
            p, input, config.footprint_margin_m)) {
      continue;
    }
    const CargoFootprintGridIndex cell = makeCargoFootprintGridIndex(
        p, input, config.xy_cell_size_m);
    if (authorized_cells.count(cell) == 0U) {
      continue;
    }
    if (competing_cells.count(cell) > 0U) {
      result.reject_reason = "COMPETING_OWNER_COLUMN";
      return result;
    }
    authorized_raw_points.push_back(p);
  }
  result.raw_points_measured = authorized_raw_points.size();
  if (authorized_raw_points.empty()) {
    result.reject_reason = "RAW_OWNER_LOCKED_SURFACE_INVALID";
    return result;
  }

  CargoVerticalEvidenceInput owner_input = input;
  owner_input.selected_points_base = std::move(authorized_raw_points);
  const CargoVerticalEvidence evidence = extractCargoVerticalEvidence(
      owner_input, config);
  if (!evidence.valid || !std::isfinite(evidence.top_z_base)) {
    result.reject_reason = "OWNER_SURFACE_INVALID:" + evidence.reject_reason;
    return result;
  }
  result.surface_z = evidence.top_z_base;
  result.valid = true;
  result.reject_reason = "owner_locked_valid";
  return result;
}

// Diagnostic-only current-frame owner assembly (Phase A counterfactual).  It
// proves, within a single source frame and against the frozen reference cell
// mask, whether the low and high vertical fragments belong to the same cargo
// owner.  It is XY/cell authority only: it never produces Z, lift, identity,
// Bottom, Safety or map authority.
struct CurrentFrameOwnerAssemblyResult {
  bool valid = false;
  int seed_group = -1;
  std::vector<int> member_groups;
  std::size_t assembly_count = 0U;
  std::size_t authorized_owner_cells = 0U;
  std::string reject_reason = "not_evaluated";
  bool high_z_group_exists = false;
  bool high_z_group_joined = false;
  std::string high_z_group_reject_reason = "none";
  double high_z_group_z95 = std::numeric_limits<double>::quiet_NaN();
};

std::set<CargoFootprintGridIndex> cellsOfGroup(
    const CargoPhysicalGroupObservation& group,
    const CargoVerticalEvidenceInput& input,
    const CargoVerticalEvidenceConfig& config) {
  std::set<CargoFootprintGridIndex> cells;
  for (const Eigen::Vector3f& point : group.union_points_base) {
    if (!point.allFinite() || !cargoPointInsideFootprint(
            point, input, config.footprint_margin_m)) {
      continue;
    }
    cells.insert(makeCargoFootprintGridIndex(
        point, input, config.xy_cell_size_m));
  }
  return cells;
}

// Owner-local cells: the group's cells expressed in its OWN robust footprint
// frame (center + yaw).  This makes the owner-cell pattern translation and yaw
// invariant, so a lifted/swung cargo keeps matching the frozen local shape
// instead of being required to stay on the pre-load world XY.
std::set<CargoFootprintGridIndex> localCellsOfGroup(
    const CargoPhysicalGroupObservation& group,
    const CargoVerticalEvidenceConfig& config) {
  const CargoFootprintSnapshot footprint = robustFootprintSnapshot(group);
  if (!footprint.valid) return {};
  CargoVerticalEvidenceInput input;
  input.footprint_valid = true;
  input.footprint_center_base = footprint.center_base;
  input.footprint_size_xy = footprint.size_xy;
  input.footprint_yaw_base_rad = footprint.yaw_base_rad;
  return cellsOfGroup(group, input, config);
}

CurrentFrameOwnerAssemblyResult assembleCurrentFrameOwner(
    const std::vector<CargoPhysicalGroupObservation>& groups,
    const CargoFootprintSnapshot& frozen_footprint,
    const std::vector<CargoFootprintGridIndex>& frozen_owner_cells,
    const CargoVerticalEvidenceConfig& config,
    double high_z_threshold_m) {
  CurrentFrameOwnerAssemblyResult result;
  if (!frozen_footprint.valid || frozen_owner_cells.empty()) {
    result.reject_reason = "NO_FROZEN_REFERENCE";
    return result;
  }
  std::set<CargoFootprintGridIndex> frozen_mask(
      frozen_owner_cells.begin(), frozen_owner_cells.end());

  // Seed: exactly one current group whose cells overlap the frozen mask with
  // at least minimum_surface_cells support.
  int seed = -1;
  int seed_count = 0;
  std::set<CargoFootprintGridIndex> seed_cells;
  for (std::size_t gi = 0U; gi < groups.size(); ++gi) {
    const auto& g = groups[gi];
    if (g.group_ambiguous || !g.geometry_resolved ||
        g.union_points_base.empty() || !finiteDescriptor(g.descriptor)) {
      continue;
    }
    const auto cells = localCellsOfGroup(g, config);
    std::size_t overlap = 0U;
    for (const auto& cell : cells) {
      if (frozen_mask.count(cell) > 0U) ++overlap;
    }
    if (overlap >= config.minimum_surface_cells) {
      ++seed_count;
      if (seed < 0) { seed = static_cast<int>(gi); seed_cells = cells; }
    }
  }
  if (seed_count == 0) { result.reject_reason = "NO_CURRENT_OWNER"; return result; }
  if (seed_count > 1) { result.reject_reason = "CURRENT_OWNER_AMBIGUOUS"; return result; }
  result.seed_group = seed;
  result.member_groups.push_back(seed);

  // Vertical fragments: high-Z current groups that overlap both the frozen
  // mask and the seed cells (current-frame XY support) and are not a competing
  // independent owner.
  bool high_z_exists = false;
  for (std::size_t gi = 0U; gi < groups.size(); ++gi) {
    if (static_cast<int>(gi) == seed) continue;
    const auto& g = groups[gi];
    if (g.group_ambiguous || !g.geometry_resolved ||
        g.union_points_base.empty() || !finiteDescriptor(g.descriptor)) {
      continue;
    }
    const double z95 = g.descriptor.diagnostic_z95;
    const bool high_z = std::isfinite(z95) && z95 > high_z_threshold_m;
    if (high_z) {
      high_z_exists = true;
      result.high_z_group_z95 = z95;
    }
    if (!high_z) continue;
    const auto cells = localCellsOfGroup(g, config);
    std::size_t frozen_overlap = 0U;
    std::size_t seed_overlap = 0U;
    for (const auto& cell : cells) {
      if (frozen_mask.count(cell) > 0U) ++frozen_overlap;
      if (seed_cells.count(cell) > 0U) ++seed_overlap;
    }
    const bool frozen_ok = frozen_overlap >= config.minimum_surface_cells;
    const bool seed_ok = seed_overlap > 0U;
    if (!frozen_ok) {
      result.high_z_group_reject_reason = "HIGH_Z_NO_FROZEN_CELL_OVERLAP";
      continue;
    }
    if (!seed_ok) {
      result.high_z_group_reject_reason = "HIGH_Z_NO_SEED_CELL_OVERLAP";
      continue;
    }
    result.member_groups.push_back(static_cast<int>(gi));
    result.high_z_group_joined = true;
  }
  result.high_z_group_exists = high_z_exists;

  std::set<CargoFootprintGridIndex> authorized;
  for (const int gi : result.member_groups) {
    const auto cells = localCellsOfGroup(
        groups[static_cast<std::size_t>(gi)], config);
    authorized.insert(cells.begin(), cells.end());
  }
  result.authorized_owner_cells = authorized.size();
  result.assembly_count = result.member_groups.size();
  result.valid = true;
  result.reject_reason = result.high_z_group_exists && !result.high_z_group_joined
      ? "HIGH_Z_FRAGMENT_NOT_JOINED" : "assembly_valid";
  return result;
}

// Diagnostic pre-cluster vertical evidence.  It re-measures the already
// identified Cargo's current vertical from the range-filtered cloud (BEFORE
// the narrow Cargo ROI crop / voxel clustering) gated by the CURRENT owner's
// robust XY footprint.  It never selects identity; it only returns a Z.
OwnerLockedSurfaceResult computePreClusterSurfaceVertical(
    const CargoShadowFrameEvidence& frame,
    const CargoFootprintSnapshot& current_owner_footprint,
    const CargoVerticalEvidenceConfig& config) {
  OwnerLockedSurfaceResult result;
  if (!current_owner_footprint.valid ||
      !frame.range_cloud_current_frame ||
      !std::isfinite(frame.source_stamp_sec) ||
      frame.source_stamp_sec <= 0.0) {
    result.reject_reason = "NO_RANGE_CLOUD_OR_FOOTPRINT";
    return result;
  }
  CargoVerticalEvidenceInput input;
  input.footprint_valid = true;
  input.footprint_center_base = current_owner_footprint.center_base;
  input.footprint_size_xy = current_owner_footprint.size_xy;
  input.footprint_yaw_base_rad = current_owner_footprint.yaw_base_rad;
  input.ground_reference_valid = frame.ground_reference_valid;
  input.ground_z_base = frame.ground_z_base;

  std::vector<Eigen::Vector3f> footprint_points;
  for (const pcl::PointXYZ& p : frame.range_cloud_current_frame->points) {
    const Eigen::Vector3f pt(p.x, p.y, p.z);
    if (!pt.allFinite() || !cargoPointInsideFootprint(
            pt, input, config.footprint_margin_m)) {
      continue;
    }
    footprint_points.push_back(pt);
  }
  if (footprint_points.empty()) {
    result.reject_reason = "NO_RANGE_POINTS_IN_OWNER_FOOTPRINT";
    return result;
  }
  CargoVerticalEvidenceInput owner_input = input;
  owner_input.selected_points_base = std::move(footprint_points);
  const CargoVerticalEvidence evidence = extractCargoVerticalEvidence(
      owner_input, config);
  if (!evidence.valid || !std::isfinite(evidence.top_z_base)) {
    result.reject_reason = "OWNER_SURFACE_INVALID:" + evidence.reject_reason;
    return result;
  }
  result.surface_z = evidence.top_z_base;
  // Surface uncertainty comes from the measured surface band itself, never
  // from the inherited group vertical uncertainty.
  std::vector<double> surface_z_values;
  surface_z_values.reserve(evidence.top_support_points_base.size());
  for (const Eigen::Vector3f& point : evidence.top_support_points_base) {
    if (point.allFinite()) {
      surface_z_values.push_back(static_cast<double>(point.z()));
    }
  }
  const double measured_dispersion = surface_z_values.empty()
      ? 0.0 : 1.4826 * medianAbsoluteDeviation(
          surface_z_values, median(surface_z_values));
  // Sensor/voxel resolution floor: 0.05 m (the detector voxel leaf).  This is
  // the existing sensor contract, not a bag-specific value.
  constexpr double kSensorResolutionFloorM = 0.05;
  result.surface_uncertainty = std::max(
      kSensorResolutionFloorM, measured_dispersion);
  result.valid = true;
  result.reject_reason = "precluster_valid";
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
    case CargoPhysicalAssociationMode::COMPONENT_LINEAGE_CONTINUITY:
      return "COMPONENT_LINEAGE_CONTINUITY";
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

const char* cargoPreLiftReferenceStateName(
    CargoPreLiftReferenceState state) noexcept {
  switch (state) {
    case CargoPreLiftReferenceState::UNSEEN: return "UNSEEN";
    case CargoPreLiftReferenceState::ACQUIRING: return "ACQUIRING";
    case CargoPreLiftReferenceState::PAUSED: return "PAUSED";
    case CargoPreLiftReferenceState::FROZEN: return "FROZEN";
    case CargoPreLiftReferenceState::CLOSED: return "CLOSED";
  }
  return "INVALID";
}

const char* cargoLineageRejectStageName(
    CargoLineageRejectStage stage) noexcept {
  switch (stage) {
    case CargoLineageRejectStage::NOT_ATTEMPTED:
      return "NOT_ATTEMPTED";
    case CargoLineageRejectStage::OBSERVATION_CONTRACT:
      return "OBSERVATION_CONTRACT";
    case CargoLineageRejectStage::EXACT_PATH_WON:
      return "EXACT_PATH_WON";
    case CargoLineageRejectStage::GROUP_AMBIGUOUS:
      return "GROUP_AMBIGUOUS";
    case CargoLineageRejectStage::HISTORY_PROVENANCE_NOT_FOUND:
      return "HISTORY_PROVENANCE_NOT_FOUND";
    case CargoLineageRejectStage::HISTORY_COMPETITION:
      return "HISTORY_COMPETITION";
    case CargoLineageRejectStage::PAIR_NOT_XY_EXTENT:
      return "PAIR_NOT_XY_EXTENT";
    case CargoLineageRejectStage::VERTICAL_GATE:
      return "VERTICAL_GATE";
    case CargoLineageRejectStage::SELECTED:
      return "SELECTED";
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
  physical_cargo_epoch_id_ = 0U;
  validated_history_id_ = 0U;
  initialized_ = false;
  previous_existence_phase_ = false;
  previous_gravity_valid_ = false;
  previous_gravity_state_ = HookLoadState::UNKNOWN;
  started_loaded_without_baseline_ = false;
  prelift_blocked_until_new_epoch_ = false;
  last_pipeline_stamp_sec_ = 0.0;
  preload_handoff_ = PreLoadHandoffSnapshot{};
  preload_boundary_ = PendingPreLoadBoundary{};
  diagnostic_reference_lock_ = DiagnosticSurfaceReferenceLock{};
  reset_reason_ = reason;
}

bool CargoPhysicalIdentityAuthority::captureUniqueEligiblePreloadReference(
    const CargoPhysicalIdentityInput& input, PreLoadHandoffSnapshot* snapshot,
    std::size_t* eligible_count) const {
  std::vector<const History*> eligible;
  for (const History& history : histories_) {
    const double support_age = input.pipeline_stamp_sec -
        history.last_supported_evidence_stamp_sec;
    const bool fresh_exact_support =
        history.last_supported_evidence_stamp_sec > 0.0 &&
        support_age >= 0.0 &&
        support_age <= config_.maximum_observation_gap_sec;
    if (history.prelift_reference_frozen && history.baseline_frozen &&
        history.baseline_source ==
            CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE &&
        history.surface_lift_reference_frozen &&
        fresh_exact_support && !history.association_ambiguous &&
        history.frozen_preload_footprint.valid) {
      eligible.push_back(&history);
    }
  }
  if (eligible_count != nullptr) {
    *eligible_count = eligible.size();
  }
  if (eligible.size() != 1U || snapshot == nullptr) {
    return false;
  }
  const History& source = *eligible.front();
  PreLoadHandoffSnapshot handoff;
  handoff.valid = true;
  handoff.source_lifecycle_id = lifecycle_id_;
  handoff.source_physical_epoch = source.physical_cargo_epoch_id;
  handoff.source_history_id = source.id;
  handoff.baseline_z = source.surface_baseline_z;
  handoff.baseline_uncertainty_m =
      source.surface_baseline_uncertainty_m;
  handoff.baseline_stamp_sec = source.baseline_stamp_sec;
  handoff.frozen_preload_footprint = source.frozen_preload_footprint;
  handoff.last_exact_support_stamp =
      source.last_supported_evidence_stamp_sec;
  handoff.robust_xy_center =
      source.frozen_preload_footprint.center_base.cast<double>();
  handoff.robust_xy_extent =
      source.frozen_preload_footprint.size_xy.cast<double>();
  handoff.robust_x05 = handoff.robust_xy_center.x() -
      0.5 * handoff.robust_xy_extent.x();
  handoff.robust_x95 = handoff.robust_xy_center.x() +
      0.5 * handoff.robust_xy_extent.x();
  handoff.robust_y05 = handoff.robust_xy_center.y() -
      0.5 * handoff.robust_xy_extent.y();
  handoff.robust_y95 = handoff.robust_xy_center.y() +
      0.5 * handoff.robust_xy_extent.y();
  handoff.yaw_rad = source.frozen_preload_footprint.yaw_base_rad;
  handoff.captured_at_load_edge_stamp = input.pipeline_stamp_sec;
  *snapshot = handoff;
  return true;
}

CargoPhysicalIdentityDecision CargoPhysicalIdentityAuthority::update(
    const CargoPhysicalIdentityInput& input) {
  decision_ = CargoPhysicalIdentityDecision{};
  decision_.load_epoch = load_epoch_;
  decision_.physical_cargo_epoch_id = physical_cargo_epoch_id_;
  decision_.hook_role_source = input.hook_role_source;
  decision_.strict_lidar_existence_path =
      input.hook_role != HookLoadSignalRole::REQUIRED;
  const bool gravity_loaded = input.gravity_valid &&
      input.gravity_state == HookLoadState::LOADED;
  const bool gravity_empty = input.gravity_valid &&
      input.gravity_state == HookLoadState::EMPTY;
  const bool empty_to_loaded_transition = initialized_ &&
      previous_gravity_valid_ &&
      previous_gravity_state_ == HookLoadState::EMPTY && gravity_loaded;
  const bool lifecycle_changed = initialized_ &&
      input.lifecycle_id != lifecycle_id_;
  const bool time_rollback = initialized_ &&
      std::isfinite(input.pipeline_stamp_sec) &&
      input.pipeline_stamp_sec + kEpsilon < last_pipeline_stamp_sec_;
  if (time_rollback) {
    for (History& history : histories_) {
      history.prelift_state = CargoPreLiftReferenceState::CLOSED;
      history.prelift_close_reason = "SOURCE_TIME_ROLLBACK";
      history.earliest_prelift_samples.clear();
      history.prelift_reference_frozen = false;
      history.baseline_frozen = false;
      history.lift_confirm_count = 0;
      history.lift_confirmed = false;
      history.validation_stamp_sec = 0.0;
    }
    histories_.clear();
    validated_history_id_ = 0U;
    preload_handoff_ = PreLoadHandoffSnapshot{};
    preload_boundary_ = PendingPreLoadBoundary{};
    diagnostic_reference_lock_ = DiagnosticSurfaceReferenceLock{};
    prelift_blocked_until_new_epoch_ = true;
    decision_.prelift_state = CargoPreLiftReferenceState::CLOSED;
    decision_.prelift_close_reason = "SOURCE_TIME_ROLLBACK";
  }
  const bool stamp_valid = std::isfinite(input.pipeline_stamp_sec) &&
      input.pipeline_stamp_sec > 0.0 &&
      (!initialized_ || time_rollback || input.rearm ||
       input.lifecycle_id != lifecycle_id_ ||
       input.pipeline_stamp_sec > last_pipeline_stamp_sec_);
  if (!stamp_valid) {
    decision_.reason = time_rollback
        ? "source_time_rollback_closed_current_cargo_epoch"
        : "source_time_invalid_or_not_advanced";
    return decision_;
  }
  decision_.valid_input = true;

  if (preload_handoff_.valid) {
    const double handoff_age = input.pipeline_stamp_sec -
        preload_handoff_.last_exact_support_stamp;
    if (!gravity_loaded || !(handoff_age >= 0.0) ||
        handoff_age > config_.maximum_observation_gap_sec) {
      decision_.preload_handoff_reject_reason = !gravity_loaded
          ? "POSTLOAD_GRAVITY_NOT_LOADED" : "PRELOAD_HANDOFF_STALE";
      preload_handoff_ = PreLoadHandoffSnapshot{};
    }
  }

  // Capture the frozen preload reference across the load boundary.  The
  // lifecycle edge and the EMPTY -> LOADED gravity edge are independent
  // publisher events and may arrive in adjacent source frames.  A single
  // pending reference is retained until the complementary edge arrives within
  // the source-time contract; the old History itself is never retained.
  const bool boundary_edge_lifecycle = lifecycle_changed;
  const bool boundary_edge_load = empty_to_loaded_transition;
  const bool boundary_capture_allowed = !time_rollback && !input.rearm;

  if (input.rearm && (preload_handoff_.valid || preload_boundary_.valid)) {
    preload_handoff_ = PreLoadHandoffSnapshot{};
    preload_boundary_ = PendingPreLoadBoundary{};
    decision_.preload_handoff_reject_reason = "REARM_CLEARED_BOUNDARY";
  }

  // Promote a pending boundary once its complementary edge arrives within the
  // source-time contract; otherwise it expires fail-closed.
  if (boundary_capture_allowed && preload_boundary_.valid &&
      !preload_handoff_.valid) {
    const double pending_age = input.pipeline_stamp_sec -
        preload_boundary_.first_edge_stamp_sec;
    const bool complementary =
        (preload_boundary_.phase == PreLoadBoundaryPhase::WAITING_FOR_LOAD &&
         boundary_edge_load) ||
        (preload_boundary_.phase ==
             PreLoadBoundaryPhase::WAITING_FOR_LIFECYCLE &&
         boundary_edge_lifecycle);
    const bool repeat_edge =
        (preload_boundary_.phase == PreLoadBoundaryPhase::WAITING_FOR_LOAD &&
         boundary_edge_lifecycle) ||
        (preload_boundary_.phase ==
             PreLoadBoundaryPhase::WAITING_FOR_LIFECYCLE &&
         boundary_edge_load);
    const bool gravity_invalid_while_waiting =
        preload_boundary_.phase == PreLoadBoundaryPhase::WAITING_FOR_LOAD &&
        !gravity_loaded && !gravity_empty;
    if (complementary && pending_age >= 0.0 &&
        pending_age <= config_.maximum_observation_gap_sec) {
      if (preload_boundary_.phase == PreLoadBoundaryPhase::WAITING_FOR_LOAD) {
        preload_boundary_.load_edge_seen = true;
        preload_boundary_.load_edge_stamp_sec = input.pipeline_stamp_sec;
        decision_.preload_handoff_trigger_mode = "LIFECYCLE_THEN_LOAD";
      } else {
        preload_boundary_.lifecycle_edge_seen = true;
        preload_boundary_.lifecycle_edge_stamp_sec = input.pipeline_stamp_sec;
        preload_boundary_.target_lifecycle_id = input.lifecycle_id;
        decision_.preload_handoff_trigger_mode = "LOAD_THEN_LIFECYCLE";
      }
      decision_.preload_boundary_edge_delta_sec = pending_age;
      preload_handoff_ = preload_boundary_.reference;
      decision_.preload_handoff_captured = true;
      decision_.preload_handoff_source_history_id =
          preload_handoff_.source_history_id;
      decision_.preload_handoff_source_epoch =
          preload_handoff_.source_physical_epoch;
      decision_.preload_handoff_reject_reason = "NONE";
      preload_boundary_ = PendingPreLoadBoundary{};
    } else if (pending_age > config_.maximum_observation_gap_sec ||
               repeat_edge || gravity_invalid_while_waiting) {
      preload_boundary_ = PendingPreLoadBoundary{};
      decision_.preload_handoff_reject_reason = "PENDING_BOUNDARY_STALE";
    }
  }

  // Capture a fresh boundary (same-frame or the first of two adjacent edges).
  if (boundary_capture_allowed && !preload_handoff_.valid &&
      !preload_boundary_.valid &&
      (boundary_edge_lifecycle || boundary_edge_load)) {
    PreLoadHandoffSnapshot handoff;
    std::size_t eligible_count = 0U;
    if (captureUniqueEligiblePreloadReference(
            input, &handoff, &eligible_count)) {
      if (boundary_edge_lifecycle && boundary_edge_load) {
        preload_handoff_ = handoff;
        decision_.preload_handoff_captured = true;
        decision_.preload_handoff_source_history_id = handoff.source_history_id;
        decision_.preload_handoff_source_epoch = handoff.source_physical_epoch;
        decision_.preload_handoff_reject_reason = "NONE";
        decision_.preload_handoff_trigger_mode = "SAME_FRAME";
      } else if (boundary_edge_lifecycle && gravity_empty) {
        PendingPreLoadBoundary pending;
        pending.valid = true;
        pending.reference = handoff;
        pending.lifecycle_edge_seen = true;
        pending.first_edge_stamp_sec = input.pipeline_stamp_sec;
        pending.lifecycle_edge_stamp_sec = input.pipeline_stamp_sec;
        pending.source_lifecycle_id = lifecycle_id_;
        pending.target_lifecycle_id = input.lifecycle_id;
        pending.phase = PreLoadBoundaryPhase::WAITING_FOR_LOAD;
        preload_boundary_ = pending;
      } else {
        PendingPreLoadBoundary pending;
        pending.valid = true;
        pending.reference = handoff;
        pending.load_edge_seen = true;
        pending.first_edge_stamp_sec = input.pipeline_stamp_sec;
        pending.load_edge_stamp_sec = input.pipeline_stamp_sec;
        pending.source_lifecycle_id = lifecycle_id_;
        pending.target_lifecycle_id = lifecycle_id_;
        pending.phase = PreLoadBoundaryPhase::WAITING_FOR_LIFECYCLE;
        preload_boundary_ = pending;
      }
    } else {
      decision_.preload_handoff_reject_reason = eligible_count == 0U
          ? "NO_ELIGIBLE_PRELOAD_HISTORY"
          : "AMBIGUOUS_PRELOAD_HISTORIES";
    }
  }

  // Emit pending-boundary telemetry after the coalescer has run.
  if (preload_boundary_.valid) {
    decision_.preload_boundary_pending = true;
    decision_.preload_boundary_phase =
        preload_boundary_.phase == PreLoadBoundaryPhase::WAITING_FOR_LOAD
            ? "WAITING_FOR_LOAD" : "WAITING_FOR_LIFECYCLE";
    decision_.preload_boundary_lifecycle_seen =
        preload_boundary_.lifecycle_edge_seen;
    decision_.preload_boundary_load_seen = preload_boundary_.load_edge_seen;
    decision_.preload_boundary_first_edge_stamp =
        preload_boundary_.first_edge_stamp_sec;
    decision_.preload_boundary_lifecycle_stamp =
        preload_boundary_.lifecycle_edge_stamp_sec;
    decision_.preload_boundary_load_stamp =
        preload_boundary_.load_edge_stamp_sec;
  }

  if (input.rearm || (initialized_ && input.lifecycle_id != lifecycle_id_)) {
    histories_.clear();
    validated_history_id_ = 0U;
    previous_existence_phase_ = false;
    started_loaded_without_baseline_ = false;
    prelift_blocked_until_new_epoch_ = false;
    ++load_epoch_;
    physical_cargo_epoch_id_ = input.lifecycle_id != 0U
        ? input.lifecycle_id : physical_cargo_epoch_id_ + 1U;
  }
  if (!initialized_ && physical_cargo_epoch_id_ == 0U) {
    physical_cargo_epoch_id_ = input.lifecycle_id != 0U
        ? input.lifecycle_id : 1U;
  }
  lifecycle_id_ = input.lifecycle_id;
  last_pipeline_stamp_sec_ = input.pipeline_stamp_sec;
  initialized_ = true;

  const bool pre_load_phase = gravity_empty;
  if (gravity_empty && previous_existence_phase_ &&
      input.hook_role == HookLoadSignalRole::REQUIRED) {
    histories_.clear();
    validated_history_id_ = 0U;
    started_loaded_without_baseline_ = false;
    prelift_blocked_until_new_epoch_ = false;
    ++load_epoch_;
    physical_cargo_epoch_id_ = input.lifecycle_id != 0U &&
            input.lifecycle_id != lifecycle_id_
        ? input.lifecycle_id : physical_cargo_epoch_id_ + 1U;
  }
  const bool gravity_load_edge = gravity_loaded && !previous_existence_phase_;
  if (gravity_load_edge) {
    ++load_epoch_;
    if (input.hook_role == HookLoadSignalRole::REQUIRED) {
      started_loaded_without_baseline_ = input.node_started_loaded &&
          std::none_of(histories_.begin(), histories_.end(),
                       [](const History& history) {
                         return history.prelift_reference_frozen;
                       });
      for (History& history : histories_) {
        history.lift_confirm_count = 0;
        history.lift_confirmed = false;
        history.validation_stamp_sec = 0.0;
        history.last_supported_evidence_stamp_sec = 0.0;
        if (!history.prelift_reference_frozen) {
          history.prelift_state = CargoPreLiftReferenceState::CLOSED;
          history.prelift_close_reason =
              "LOAD_EDGE_BEFORE_REFERENCE_FROZEN";
          history.earliest_prelift_samples.clear();
        }
      }
    }
  }
  decision_.load_epoch = load_epoch_;
  decision_.physical_cargo_epoch_id = physical_cargo_epoch_id_;

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

  int preload_handoff_group = -1;
  std::vector<AssociationOnlyReacquiredVerticalEvidence>
      preload_handoff_vertical(input.groups.size());
  if (preload_handoff_.valid) {
    decision_.preload_handoff_source_history_id =
        preload_handoff_.source_history_id;
    decision_.preload_handoff_source_epoch =
        preload_handoff_.source_physical_epoch;
    int matching_group_count = 0;
    int geometric_candidate_count = 0;
    for (std::size_t gi = 0U; gi < input.groups.size(); ++gi) {
      const auto& group = input.groups[gi];
      if (group.group_ambiguous || group_ambiguous[gi] ||
          !group.geometry_resolved || group.union_points_base.empty() ||
          !finiteDescriptor(group.descriptor) ||
          group.descriptor.vertical_mode !=
              CargoGroupVerticalMode::SUPPORTED_EVIDENCE) {
        continue;
      }
      const double source_gap = group.descriptor.stamp_sec -
          preload_handoff_.last_exact_support_stamp;
      if (!(source_gap > 0.0) ||
          source_gap > config_.maximum_observation_gap_sec) {
        continue;
      }
      const double xy_step = (group.descriptor.robust_xy_center -
          preload_handoff_.robust_xy_center).norm();
      const double intersection_x = std::min(
          group.descriptor.robust_x95, preload_handoff_.robust_x95) -
          std::max(group.descriptor.robust_x05,
                   preload_handoff_.robust_x05);
      const double intersection_y = std::min(
          group.descriptor.robust_y95, preload_handoff_.robust_y95) -
          std::max(group.descriptor.robust_y05,
                   preload_handoff_.robust_y05);
      const bool xy_ok = xy_step <= config_.maximum_xy_step_m ||
          (intersection_x > kEpsilon && intersection_y > kEpsilon &&
           intersection_x * intersection_y > kEpsilon);
      bool extent_ok = true;
      for (int axis = 0; axis < 2; ++axis) {
        const double current_extent =
            group.descriptor.robust_xy_extent[axis];
        const double previous_extent =
            preload_handoff_.robust_xy_extent[axis];
        const double denominator = std::max(
            std::max(std::abs(current_extent), std::abs(previous_extent)),
            kEpsilon);
        extent_ok = extent_ok &&
            std::abs(current_extent - previous_extent) / denominator <=
                config_.maximum_size_relative_step;
      }
      if (!xy_ok || !extent_ok ||
          !input.frame_evidence.raw_roi_current_frame ||
          std::abs(input.frame_evidence.source_stamp_sec -
                   group.descriptor.stamp_sec) > kEpsilon) {
        continue;
      }
      ++geometric_candidate_count;
      std::vector<Eigen::Vector3f> competing_owner_points;
      for (std::size_t other = 0U; other < input.groups.size(); ++other) {
        if (other == gi) continue;
        competing_owner_points.insert(
            competing_owner_points.end(),
            input.groups[other].union_points_base.begin(),
            input.groups[other].union_points_base.end());
      }
      auto vertical = reacquireAssociationVertical(
          input.frame_evidence, preload_handoff_.frozen_preload_footprint,
          group.union_points_base, input.vertical_config,
          preload_handoff_.baseline_uncertainty_m,
          &competing_owner_points);
      if (!vertical.valid) continue;
      preload_handoff_vertical[gi] = std::move(vertical);
      preload_handoff_group = static_cast<int>(gi);
      ++matching_group_count;
    }
    if (matching_group_count != 1 || geometric_candidate_count != 1) {
      if (!input.groups.empty() || matching_group_count > 1 ||
          geometric_candidate_count > 1) {
        decision_.preload_handoff_reject_reason =
            geometric_candidate_count > 1 || matching_group_count > 1
            ? "AMBIGUOUS_POSTLOAD_GROUPS"
            : "NO_UNIQUE_POSTLOAD_MATCH";
        preload_handoff_ = PreLoadHandoffSnapshot{};
      }
      preload_handoff_group = -1;
    } else {
      decision_.preload_handoff_reject_reason = "NONE";
    }
  }

  // The exact current-group association above has absolute priority.  A
  // component-lineage observation may only rescue an otherwise-unmatched
  // current group to one already-existing history.  It cannot create a
  // history, rank competing histories, or replace exact product evidence.
  std::vector<const CargoIdentitySupportLineageObservation*> group_lineage(
      input.groups.size(), nullptr);
  std::vector<bool> history_claimed_by_exact(histories_.size(), false);
  std::vector<bool> history_has_exact_feasible(histories_.size(), false);
  for (const Pair& pair : pairs) {
    if (pair.feasible) {
      history_has_exact_feasible[pair.history] = true;
    }
  }
  for (const int matched_history : group_match) {
    if (matched_history >= 0) {
      history_claimed_by_exact[static_cast<std::size_t>(matched_history)] =
          true;
    }
  }

  struct LineageProposal {
    std::size_t group = 0U;
    std::size_t history = 0U;
    Pair* pair = nullptr;
    const CargoIdentitySupportLineageObservation* observation = nullptr;
  };
  std::vector<LineageProposal> lineage_proposals;
  for (const auto& observation : input.lineage_observations) {
    if (!observation.valid ||
        observation.state != CargoIdentityLineageState::MATCHED) {
      continue;
    }
    std::size_t group_index = input.groups.size();
    for (std::size_t gi = 0U; gi < input.groups.size(); ++gi) {
      if (input.groups[gi].frame_group_id ==
          observation.exact_seed_frame_group_id) {
        if (group_index != input.groups.size()) {
          group_index = input.groups.size();
          break;
        }
        group_index = gi;
      }
    }
    if (group_index >= input.groups.size()) continue;

    auto& diagnostic = decision_.group_diagnostics[group_index];
    diagnostic.lineage_attempted = true;
    const bool newest_diagnostic_observation =
        diagnostic.lineage_source_frame_offset == 0U ||
        observation.source_frame_offset <
            diagnostic.lineage_source_frame_offset;
    if (newest_diagnostic_observation) {
      diagnostic.lineage_previous_component_id =
          observation.previous_component_id;
      diagnostic.lineage_current_component_id =
          observation.current_component_id;
      diagnostic.lineage_source_age_sec = observation.source_age_sec;
      diagnostic.lineage_source_frame_offset =
          observation.source_frame_offset;
      diagnostic.lineage_reject_stage =
          CargoLineageRejectStage::OBSERVATION_CONTRACT;
    }
    const auto set_observation_reject_stage =
        [&](CargoLineageRejectStage stage) {
          if (newest_diagnostic_observation) {
            diagnostic.lineage_reject_stage = stage;
          }
        };
    const auto& exact_group = input.groups[group_index];
    const bool current_component_belongs_to_exact_seed =
        std::find(exact_group.member_component_ids.begin(),
                  exact_group.member_component_ids.end(),
                  observation.current_component_id) !=
        exact_group.member_component_ids.end();
    const bool observable_ego_contract =
        observation.motion_observability_state ==
            CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE &&
        std::isfinite(observation.ego_xy_step_m) &&
        observation.ego_xy_step_m >
            config_.maximum_xy_step_m + kEpsilon &&
        std::isfinite(observation.map_step_m) &&
        observation.map_step_m > config_.maximum_xy_step_m + kEpsilon;
    const bool loaded_low_ego_contract =
        observation.motion_observability_state ==
            CargoIdentityMotionObservabilityState::
                LOAD_PRESENT_UNOBSERVABLE &&
        std::isfinite(observation.ego_xy_step_m) &&
        observation.ego_xy_step_m <=
            config_.maximum_xy_step_m + kEpsilon &&
        std::isfinite(observation.map_step_m) && input.gravity_valid &&
        input.gravity_state == HookLoadState::LOADED;
    const bool observation_contract_valid =
        current_component_belongs_to_exact_seed &&
        observation.source_frame_offset >= 1U &&
        observation.source_frame_offset <= 3U &&
        std::isfinite(observation.source_age_sec) &&
        observation.source_age_sec > 0.0 &&
        observation.source_age_sec <=
            config_.maximum_observation_gap_sec + kEpsilon &&
        std::abs(observation.source_age_sec -
                 (observation.source_stamp_sec -
                  observation.previous_source_stamp_sec)) <= kEpsilon &&
        std::abs(observation.source_stamp_sec -
                 exact_group.descriptor.stamp_sec) <= kEpsilon &&
        observation.robust_xy_center.allFinite() &&
        observation.robust_xy_extent.allFinite() &&
        (observation.robust_xy_extent.array() > 0.0).all() &&
        std::isfinite(observation.base_step_m) &&
        observation.base_step_m <= config_.maximum_xy_step_m + kEpsilon &&
        (observable_ego_contract || loaded_low_ego_contract) &&
        std::isfinite(observation.extent_step) &&
        observation.extent_step <=
            config_.maximum_size_relative_step + kEpsilon;
    if (!observation_contract_valid) continue;
    if (group_match[group_index] >= 0) {
      diagnostic.lineage_exact_path_won = true;
      set_observation_reject_stage(
          CargoLineageRejectStage::EXACT_PATH_WON);
      continue;
    }
    if (input.groups[group_index].group_ambiguous ||
        group_ambiguous[group_index]) {
      diagnostic.lineage_ambiguous = true;
      set_observation_reject_stage(
          CargoLineageRejectStage::GROUP_AMBIGUOUS);
      continue;
    }

    std::vector<std::size_t> provenance_histories;
    for (std::size_t hi = 0U; hi < histories_.size(); ++hi) {
      const History& history = histories_[hi];
      bool provenance_matched = false;
      for (const LineageProvenanceSnapshot& snapshot :
           history.recent_lineage_provenance) {
        if (std::abs(snapshot.source_stamp_sec -
                     observation.previous_source_stamp_sec) > kEpsilon) {
          continue;
        }
        if (std::find(snapshot.component_ids.begin(),
                      snapshot.component_ids.end(),
                      observation.previous_component_id) !=
            snapshot.component_ids.end()) {
          provenance_matched = true;
          break;
        }
      }
      if (provenance_matched) provenance_histories.push_back(hi);
    }
    if (provenance_histories.empty()) {
      set_observation_reject_stage(
          CargoLineageRejectStage::HISTORY_PROVENANCE_NOT_FOUND);
      continue;
    }
    if (provenance_histories.size() > 1U) {
      diagnostic.lineage_ambiguous = true;
      set_observation_reject_stage(
          CargoLineageRejectStage::HISTORY_COMPETITION);
      continue;
    }

    const std::size_t history_index = provenance_histories.front();
    // Any exact-feasible current group keeps absolute priority, including
    // the case where multiple exact groups make the reciprocal result
    // ambiguous and therefore leave the history formally unclaimed.
    if (history_claimed_by_exact[history_index] ||
        history_has_exact_feasible[history_index]) {
      diagnostic.lineage_ambiguous = true;
      set_observation_reject_stage(
          CargoLineageRejectStage::HISTORY_COMPETITION);
      continue;
    }

    Pair* pair = nullptr;
    for (Pair& candidate : pairs) {
      if (candidate.group == group_index &&
          candidate.history == history_index) {
        pair = &candidate;
        break;
      }
    }
    if (!pair || (pair->reject_reason != "XY_GATE" &&
                  pair->reject_reason != "EXTENT_GATE")) {
      set_observation_reject_stage(
          CargoLineageRejectStage::PAIR_NOT_XY_EXTENT);
      continue;
    }

    const auto& group = input.groups[group_index];
    const auto& previous = histories_[history_index].last_descriptor;
    const double dt = group.descriptor.stamp_sec -
        histories_[history_index].last_stamp_sec;
    if (!(dt > 0.0) || dt > config_.maximum_observation_gap_sec) {
      set_observation_reject_stage(
          CargoLineageRejectStage::OBSERVATION_CONTRACT);
      continue;
    }

    // Family/lineage has no vertical authority.  Any vertical check remains
    // sourced exclusively from the current exact group and the existing
    // history.  CONTINUITY_ONLY performs its existing post-unique RAW-ROI Z
    // check below.
    const bool requires_post_unique_z =
        group.descriptor.vertical_mode ==
            CargoGroupVerticalMode::CONTINUITY_ONLY;
    const double z_limit = config_.maximum_z_speed_mps * dt +
        config_.z_step_margin_m +
        group.descriptor.vertical_uncertainty_m +
        previous.vertical_uncertainty_m;
    const double dz = std::abs(group.descriptor.physical_vertical_z -
                               previous.physical_vertical_z);
    if (!requires_post_unique_z && dz > z_limit) {
      set_observation_reject_stage(
          CargoLineageRejectStage::VERTICAL_GATE);
      continue;
    }
    if (!requires_post_unique_z) {
      const double current_z_extent = group.descriptor.aggregate_extent.z();
      const double previous_z_extent = previous.aggregate_extent.z();
      const double denominator = std::max(
          std::max(std::abs(current_z_extent),
                   std::abs(previous_z_extent)), kEpsilon);
      if (std::abs(current_z_extent - previous_z_extent) / denominator >
          config_.maximum_size_relative_step) {
        set_observation_reject_stage(
            CargoLineageRejectStage::VERTICAL_GATE);
        continue;
      }
    }

    diagnostic.lineage_xy_before_m = pair->xy;
    diagnostic.lineage_extent_before = pair->extent_step;
    lineage_proposals.push_back(LineageProposal{
        group_index, history_index, pair, &observation});
  }

  // Authority, not the compact correspondence cache, decides whether an
  // older source frame is legally bound to a Cargo history.  For one group,
  // multiple recent observations may refer to the same history; keep only
  // the newest such provenance.  Different histories, duplicate newest
  // claims, or multiple groups claiming one history remain fail-closed.
  std::vector<const LineageProposal*> selected_lineage(
      input.groups.size(), nullptr);
  for (std::size_t gi = 0U; gi < input.groups.size(); ++gi) {
    std::size_t selected_history = histories_.size();
    std::uint64_t best_offset = std::numeric_limits<std::uint64_t>::max();
    std::size_t best_count = 0U;
    for (const LineageProposal& proposal : lineage_proposals) {
      if (proposal.group != gi) continue;
      if (selected_history == histories_.size()) {
        selected_history = proposal.history;
      } else if (selected_history != proposal.history) {
        decision_.group_diagnostics[gi].lineage_ambiguous = true;
        decision_.group_diagnostics[gi].lineage_reject_stage =
            CargoLineageRejectStage::HISTORY_COMPETITION;
        group_ambiguous[gi] = true;
      }
      if (proposal.observation->source_frame_offset < best_offset) {
        best_offset = proposal.observation->source_frame_offset;
        selected_lineage[gi] = &proposal;
        best_count = 1U;
      } else if (proposal.observation->source_frame_offset == best_offset) {
        ++best_count;
      }
    }
    if (best_count > 1U) {
      decision_.group_diagnostics[gi].lineage_ambiguous = true;
      decision_.group_diagnostics[gi].lineage_reject_stage =
          CargoLineageRejectStage::HISTORY_COMPETITION;
      group_ambiguous[gi] = true;
    }
  }
  for (std::size_t hi = 0U; hi < histories_.size(); ++hi) {
    std::size_t claimant_count = 0U;
    for (const LineageProposal* proposal : selected_lineage) {
      if (proposal && proposal->history == hi) ++claimant_count;
    }
    if (claimant_count <= 1U) continue;
    for (std::size_t gi = 0U; gi < selected_lineage.size(); ++gi) {
      if (selected_lineage[gi] && selected_lineage[gi]->history == hi) {
        decision_.group_diagnostics[gi].lineage_ambiguous = true;
        decision_.group_diagnostics[gi].lineage_reject_stage =
            CargoLineageRejectStage::HISTORY_COMPETITION;
        group_ambiguous[gi] = true;
      }
    }
  }

  for (std::size_t gi = 0U; gi < selected_lineage.size(); ++gi) {
    const LineageProposal* selected = selected_lineage[gi];
    if (!selected || group_ambiguous[gi]) continue;
    const LineageProposal& proposal = *selected;
    auto& diagnostic = decision_.group_diagnostics[proposal.group];

    Pair& pair = *proposal.pair;
    const auto& observation = *proposal.observation;
    pair.xy = observation.base_step_m;
    pair.support_xy_separation = observation.base_step_m;
    pair.extent_step = observation.extent_step;
    pair.xy_cost = observation.base_step_m /
        config_.maximum_xy_step_m;
    pair.extent_cost = observation.extent_step;
    pair.z_cost = pair.requires_post_unique_z
        ? 0.0 : pair.dz /
            std::max(config_.maximum_z_speed_mps *
                         (input.groups[proposal.group].descriptor.stamp_sec -
                          histories_[proposal.history].last_stamp_sec) +
                     config_.z_step_margin_m +
                     input.groups[proposal.group].descriptor.
                         vertical_uncertainty_m +
                     histories_[proposal.history].last_descriptor.
                         vertical_uncertainty_m,
                     kEpsilon);
    pair.cost = pair.xy_cost + pair.extent_cost + pair.z_cost;
    pair.association_mode =
        CargoPhysicalAssociationMode::COMPONENT_LINEAGE_CONTINUITY;
    pair.reject_reason = "NONE";
    pair.feasible = true;
    group_match[proposal.group] = static_cast<int>(proposal.history);
    group_lineage[proposal.group] = proposal.observation;
    diagnostic.lineage_rescue_used = true;
    diagnostic.lineage_previous_component_id =
        observation.previous_component_id;
    diagnostic.lineage_current_component_id = observation.current_component_id;
    diagnostic.lineage_source_age_sec = observation.source_age_sec;
    diagnostic.lineage_source_frame_offset =
        observation.source_frame_offset;
    diagnostic.lineage_reject_stage = CargoLineageRejectStage::SELECTED;
    diagnostic.lineage_xy_after_m = pair.xy;
    diagnostic.lineage_extent_after = pair.extent_step;
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
      diagnostic.lineage_reject_stage =
          CargoLineageRejectStage::VERTICAL_GATE;
      group_match[gi] = -1;
      group_lineage[gi] = nullptr;
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
      history->physical_cargo_epoch_id = physical_cargo_epoch_id_;
      if (static_cast<int>(gi) == preload_handoff_group &&
          preload_handoff_.valid) {
        history->prelift_state = CargoPreLiftReferenceState::FROZEN;
        history->prelift_reference_frozen = true;
        history->prelift_reference_z = preload_handoff_.baseline_z;
        history->prelift_reference_uncertainty_m =
            preload_handoff_.baseline_uncertainty_m;
        history->prelift_reference_first_stamp =
            preload_handoff_.baseline_stamp_sec;
        history->prelift_reference_last_stamp =
            preload_handoff_.baseline_stamp_sec;
        history->baseline_frozen = true;
        history->baseline_source =
            CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE;
        history->baseline_z95 = preload_handoff_.baseline_z;
        history->baseline_uncertainty_m =
            preload_handoff_.baseline_uncertainty_m;
        history->baseline_stamp_sec = preload_handoff_.baseline_stamp_sec;
        history->frozen_preload_footprint =
            preload_handoff_.frozen_preload_footprint;
        history->surface_lift_reference_frozen = true;
        history->surface_baseline_z = preload_handoff_.baseline_z;
        history->surface_baseline_uncertainty_m =
            preload_handoff_.baseline_uncertainty_m;
        history->last_supported_footprint =
            preload_handoff_.frozen_preload_footprint;
        history->lift_confirm_count = 0;
        history->lift_confirmed = false;
        history->validation_stamp_sec = 0.0;
        decision_.preload_handoff_consumed = true;
        decision_.postload_history_id = history->id;
        decision_.preload_handoff_source_history_id =
            preload_handoff_.source_history_id;
        decision_.preload_handoff_source_epoch =
            preload_handoff_.source_physical_epoch;
        diagnostic.preload_handoff_consumed = true;
        preload_handoff_ = PreLoadHandoffSnapshot{};
      }
      if (prelift_blocked_until_new_epoch_) {
        history->prelift_state = CargoPreLiftReferenceState::CLOSED;
        history->prelift_close_reason = "CURRENT_EPOCH_PRELIFT_BLOCKED";
      }
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
    CargoPhysicalGroupDescriptor history_descriptor = group.descriptor;
    if (group_lineage[gi]) {
      const auto& lineage = *group_lineage[gi];
      history_descriptor.stable_anchor.x() = lineage.robust_xy_center.x();
      history_descriptor.stable_anchor.y() = lineage.robust_xy_center.y();
      history_descriptor.robust_xy_center = lineage.robust_xy_center;
      history_descriptor.robust_xy_extent = lineage.robust_xy_extent;
      history_descriptor.robust_x05 = lineage.robust_x05;
      history_descriptor.robust_x95 = lineage.robust_x95;
      history_descriptor.robust_y05 = lineage.robust_y05;
      history_descriptor.robust_y95 = lineage.robust_y95;
      history_descriptor.aggregate_extent.x() =
          lineage.robust_xy_extent.x();
      history_descriptor.aggregate_extent.y() =
          lineage.robust_xy_extent.y();
    }
    history->last_descriptor = history_descriptor;
    history->last_representative_center = group.representative.center;
    history->last_stamp_sec = group.descriptor.stamp_sec;
    LineageProvenanceSnapshot provenance;
    provenance.source_stamp_sec = group.descriptor.stamp_sec;
    provenance.component_ids = group_lineage[gi]
        ? std::vector<std::uint64_t>{
              group_lineage[gi]->current_component_id}
        : canonicalMembers(group.member_component_ids);
    auto& recent_provenance = history->recent_lineage_provenance;
    recent_provenance.erase(
        std::remove_if(
            recent_provenance.begin(), recent_provenance.end(),
            [&](const LineageProvenanceSnapshot& snapshot) {
              const double age = provenance.source_stamp_sec -
                  snapshot.source_stamp_sec;
              return !std::isfinite(snapshot.source_stamp_sec) ||
                  age < -kEpsilon ||
                  age > config_.maximum_observation_gap_sec + kEpsilon ||
                  std::abs(age) <= kEpsilon;
            }),
        recent_provenance.end());
    recent_provenance.push_back(std::move(provenance));
    while (recent_provenance.size() >
           kMaximumLineageProvenanceFrames) {
      recent_provenance.pop_front();
    }

    const bool supported = group.descriptor.vertical_mode ==
        CargoGroupVerticalMode::SUPPORTED_EVIDENCE;
    const bool role_allows_reference_acquisition =
        input.hook_role == HookLoadSignalRole::REQUIRED
            ? pre_load_phase : !started_loaded_without_baseline_;
    const bool surface_reference_acquisition_phase =
        role_allows_reference_acquisition &&
        (input.hook_role == HookLoadSignalRole::DISABLED || !gravity_loaded);
    const bool evidence_authorized = input.localization_authorized &&
        input.pose_authority_identity_valid && supported &&
        !group.group_ambiguous && history->physical_cargo_epoch_id ==
            physical_cargo_epoch_id_;
    const bool unique_current_exact_group = !group.group_ambiguous &&
        !group_ambiguous[gi] && group.geometry_resolved &&
        !group.union_points_base.empty() && finiteDescriptor(group.descriptor) &&
        group.descriptor.vertical_mode ==
            CargoGroupVerticalMode::SUPPORTED_EVIDENCE;
    if (!history->frozen_preload_footprint.valid &&
        unique_current_exact_group && evidence_authorized &&
        surface_reference_acquisition_phase &&
        !prelift_blocked_until_new_epoch_) {
      history->frozen_preload_footprint = robustFootprintSnapshot(group);
    }

    AssociationOnlyReacquiredVerticalEvidence current_surface_vertical;
    if (static_cast<int>(gi) == preload_handoff_group &&
        preload_handoff_vertical[gi].valid) {
      current_surface_vertical = preload_handoff_vertical[gi];
    } else if (unique_current_exact_group && evidence_authorized &&
               history->frozen_preload_footprint.valid &&
               input.frame_evidence.raw_roi_current_frame &&
               std::abs(input.frame_evidence.source_stamp_sec -
                        group.descriptor.stamp_sec) <= kEpsilon) {
      std::vector<Eigen::Vector3f> competing_owner_points;
      for (std::size_t other = 0U; other < input.groups.size(); ++other) {
        if (other == gi) continue;
        competing_owner_points.insert(
            competing_owner_points.end(),
            input.groups[other].union_points_base.begin(),
            input.groups[other].union_points_base.end());
      }
      current_surface_vertical = reacquireAssociationVertical(
          input.frame_evidence, history->frozen_preload_footprint,
          group.union_points_base, input.vertical_config,
          group.descriptor.vertical_uncertainty_m,
          &competing_owner_points);
    }
    diagnostic.current_surface_vertical_valid =
        current_surface_vertical.valid;
    diagnostic.current_surface_z = current_surface_vertical.top_z_base;
    diagnostic.current_surface_owner_overlap_cells =
        current_surface_vertical.owner_overlap_cell_count;
    diagnostic.current_surface_owner_coverage =
        current_surface_vertical.owner_overlap_coverage;
    diagnostic.current_surface_reject_reason = current_surface_vertical.reason;

    // V3.1 diagnostic counterfactual: owner-locked surface (never influences
    // the product decision in this shadow pass).
    {
      std::vector<Eigen::Vector3f> v31_competing_points;
      for (std::size_t other = 0U; other < input.groups.size(); ++other) {
        if (other == gi) continue;
        v31_competing_points.insert(
            v31_competing_points.end(),
            input.groups[other].union_points_base.begin(),
            input.groups[other].union_points_base.end());
      }
      const OwnerLockedSurfaceResult v31 = computeOwnerLockedSurfaceVertical(
          input.frame_evidence, history->frozen_preload_footprint,
          history->frozen_preload_owner_surface_cells,
          group.union_points_base, input.vertical_config,
          &v31_competing_points);
      diagnostic.v31_owner_locked_valid = v31.valid;
      diagnostic.v31_owner_locked_surface_z = v31.surface_z;
      diagnostic.v31_owner_locked_reject_reason = v31.reject_reason;
      diagnostic.v31_owner_cells = v31.owner_cells;
      diagnostic.v31_frozen_cells = v31.frozen_cells;
      diagnostic.v31_authorized_cells = v31.authorized_cells;
      diagnostic.v31_raw_points_measured = v31.raw_points_measured;
    }

    const bool surface_evidence_authorized = evidence_authorized &&
        unique_current_exact_group && current_surface_vertical.valid;
    if (!history->prelift_reference_frozen &&
        history->prelift_state != CargoPreLiftReferenceState::CLOSED &&
        surface_reference_acquisition_phase && surface_evidence_authorized &&
        !prelift_blocked_until_new_epoch_) {
      if (!history->earliest_prelift_samples.empty()) {
        const double gap = group.descriptor.stamp_sec -
            history->earliest_prelift_samples.back().stamp_sec;
        if (!(gap > 0.0) || gap > config_.maximum_observation_gap_sec) {
          // V3.1/Reference-Lock: a transient evidence gap restarts the sample
          // window instead of permanently closing acquisition.  The current
          // legal sample below becomes sample #1 of the fresh window.
          history->prelift_state = CargoPreLiftReferenceState::ACQUIRING;
          history->prelift_close_reason =
              "WINDOW_RESTART_SUPPORTED_EVIDENCE_GAP";
          history->earliest_prelift_samples.clear();
          diagnostic_reference_lock_.window_restarted = true;
          diagnostic_reference_lock_.window_restart_reason =
              "SUPPORTED_EVIDENCE_GAP";
          ++diagnostic_reference_lock_.pre_freeze_outlier_windows;
        }
      }
      if (history->prelift_state != CargoPreLiftReferenceState::CLOSED) {
        history->prelift_state = CargoPreLiftReferenceState::ACQUIRING;
        history->prelift_close_reason = "none";
        history->earliest_prelift_samples.push_back(PreLiftSample{
            group.descriptor.stamp_sec,
            current_surface_vertical.top_z_base,
            current_surface_vertical.uncertainty_m});
        const std::size_t required_samples = static_cast<std::size_t>(
            std::max(1, config_.lift_confirm_frames));
        if (history->earliest_prelift_samples.size() == required_samples) {
          std::vector<double> z_values;
          z_values.reserve(required_samples);
          double maximum_uncertainty = 0.0;
          bool nondecreasing = true;
          for (std::size_t sample_index = 0U;
               sample_index < required_samples; ++sample_index) {
            const PreLiftSample& sample =
                history->earliest_prelift_samples[sample_index];
            z_values.push_back(sample.vertical_z);
            maximum_uncertainty = std::max(
                maximum_uncertainty, sample.uncertainty_m);
            if (sample_index > 0U && sample.vertical_z + kEpsilon <
                    history->earliest_prelift_samples[
                        sample_index - 1U].vertical_z) {
              nondecreasing = false;
            }
          }
          const double reference = median(z_values);
          const double robust_dispersion =
              1.4826 * medianAbsoluteDeviation(z_values, reference);
          const double uncertainty = std::max(
              maximum_uncertainty, robust_dispersion);
          const double compatibility = std::max(
              config_.minimum_significant_change_m,
              config_.significance_sigma * maximum_uncertainty);
          const auto extrema = std::minmax_element(
              z_values.begin(), z_values.end());
          const double spread = *extrema.second - *extrema.first;
          const double departure = z_values.back() - z_values.front();
          const double departure_significance = liftSignificanceThreshold(
              config_,
              history->earliest_prelift_samples.front().uncertainty_m,
              history->earliest_prelift_samples.back().uncertainty_m);
          const bool monotonic_departure = nondecreasing &&
              departure > departure_significance + kEpsilon;
          if (spread <= compatibility && !monotonic_departure) {
            history->prelift_state = CargoPreLiftReferenceState::FROZEN;
            history->prelift_reference_frozen = true;
            history->prelift_reference_z = reference;
            history->prelift_reference_uncertainty_m = uncertainty;
            history->prelift_reference_first_stamp =
                history->earliest_prelift_samples.front().stamp_sec;
            history->prelift_reference_last_stamp =
                history->earliest_prelift_samples.back().stamp_sec;
            history->baseline_frozen = true;
            history->baseline_source =
                CargoLiftBaselineSource::PRE_LOAD_FROZEN_BASELINE;
            history->baseline_z95 = reference;
            history->baseline_uncertainty_m = uncertainty;
            history->baseline_stamp_sec =
                history->prelift_reference_last_stamp;
            history->surface_lift_reference_frozen = true;
            history->surface_baseline_z = reference;
            history->surface_baseline_uncertainty_m = uncertainty;
            // V3.1: freeze the owner surface cells (XY grid indices only).
            history->frozen_preload_owner_surface_cells.clear();
            if (history->frozen_preload_footprint.valid) {
              CargoVerticalEvidenceInput owner_cell_input;
              owner_cell_input.footprint_valid = true;
              owner_cell_input.footprint_center_base =
                  history->frozen_preload_footprint.center_base;
              owner_cell_input.footprint_size_xy =
                  history->frozen_preload_footprint.size_xy;
              owner_cell_input.footprint_yaw_base_rad =
                  history->frozen_preload_footprint.yaw_base_rad;
              std::set<CargoFootprintGridIndex> owner_cells;
              for (const Eigen::Vector3f& point : group.union_points_base) {
                if (!point.allFinite() || !cargoPointInsideFootprint(
                        point, owner_cell_input,
                        input.vertical_config.footprint_margin_m)) {
                  continue;
                }
                owner_cells.insert(makeCargoFootprintGridIndex(
                    point, owner_cell_input,
                    input.vertical_config.xy_cell_size_m));
              }
              history->frozen_preload_owner_surface_cells.assign(
                  owner_cells.begin(), owner_cells.end());
            }
            // Diagnostic Reference Lock freeze (Phase A counterfactual).
            diagnostic_reference_lock_.phase =
                DiagnosticSurfaceReferenceLock::Phase::PRELOAD_ACTIVE;
            diagnostic_reference_lock_.frozen = true;
            diagnostic_reference_lock_.baseline_z = reference;
            diagnostic_reference_lock_.baseline_uncertainty_m = uncertainty;
            diagnostic_reference_lock_.frozen_footprint =
                history->frozen_preload_footprint;
            diagnostic_reference_lock_.frozen_owner_cells =
                history->frozen_preload_owner_surface_cells;
            diagnostic_reference_lock_.source_lifecycle_id = lifecycle_id_;
            diagnostic_reference_lock_.source_physical_epoch =
                history->physical_cargo_epoch_id;
            diagnostic_reference_lock_.source_history_id = history->id;
            diagnostic_reference_lock_.current_history_id = history->id;
            diagnostic_reference_lock_.last_owner_refresh_stamp =
                group.descriptor.stamp_sec;
          } else if (monotonic_departure) {
            // A genuinely rising surface during EMPTY is a real departure, not
            // a stable reference.  Keep the pre-existing permanent close so a
            // moving surface can never slide into a lifted stable window.
            history->prelift_state = CargoPreLiftReferenceState::CLOSED;
            history->prelift_close_reason =
                "EARLIEST_PREFIX_MONOTONIC_DEPARTURE";
            history->earliest_prelift_samples.clear();
          } else {
            // Reference-Lock: an incompatible (spread) prefix restarts the
            // window instead of permanently closing acquisition.
            history->prelift_state = CargoPreLiftReferenceState::ACQUIRING;
            history->prelift_close_reason =
                "WINDOW_RESTART_PREFIX_INCOMPATIBLE";
            history->earliest_prelift_samples.clear();
            diagnostic_reference_lock_.window_restarted = true;
            diagnostic_reference_lock_.window_restart_reason =
                "PREFIX_INCOMPATIBLE";
            ++diagnostic_reference_lock_.pre_freeze_outlier_windows;
          }
        }
      }
    } else if (!history->prelift_reference_frozen &&
               history->prelift_state ==
                   CargoPreLiftReferenceState::ACQUIRING &&
               surface_reference_acquisition_phase &&
               !surface_evidence_authorized) {
      history->prelift_state = CargoPreLiftReferenceState::PAUSED;
    } else if (!history->prelift_reference_frozen &&
               history->prelift_state ==
                   CargoPreLiftReferenceState::PAUSED &&
               surface_reference_acquisition_phase &&
               surface_evidence_authorized) {
      history->prelift_state = CargoPreLiftReferenceState::ACQUIRING;
    }
    if (!history->prelift_reference_frozen && gravity_load_edge &&
        input.hook_role == HookLoadSignalRole::REQUIRED) {
      history->prelift_state = CargoPreLiftReferenceState::CLOSED;
      history->prelift_close_reason = "LOAD_EDGE_BEFORE_REFERENCE_FROZEN";
      history->earliest_prelift_samples.clear();
    }

    if (supported && group.descriptor.stamp_sec >
            history->last_supported_evidence_stamp_sec + kEpsilon) {
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
    }

    if (!surface_evidence_authorized) {
      if (history->last_lift_surface_evidence_stamp_sec > 0.0 &&
          group.descriptor.stamp_sec -
              history->last_lift_surface_evidence_stamp_sec >
                  config_.maximum_observation_gap_sec) {
        history->lift_confirm_count = 0;
        history->lift_confirmed = false;
        history->validation_stamp_sec = 0.0;
      }
      continue;
    }

    const double surface_gap = group.descriptor.stamp_sec -
        history->last_lift_surface_evidence_stamp_sec;
    if (history->last_lift_surface_evidence_stamp_sec > 0.0 &&
        surface_gap > config_.maximum_observation_gap_sec) {
      history->lift_confirm_count = 0;
      history->lift_confirmed = false;
      history->validation_stamp_sec = 0.0;
    }
    if (!history->baseline_frozen ||
        !history->surface_lift_reference_frozen) {
      continue;
    }
    const double threshold = liftSignificanceThreshold(
        config_, history->baseline_uncertainty_m,
        current_surface_vertical.uncertainty_m);
    const double delta = current_surface_vertical.top_z_base -
        history->baseline_z95;
    const bool evidence_advanced = group.descriptor.stamp_sec >
        history->last_lift_surface_evidence_stamp_sec + kEpsilon;
    if (evidence_advanced) {
      history->last_lift_surface_evidence_stamp_sec =
          group.descriptor.stamp_sec;
      history->last_lift_surface_z = current_surface_vertical.top_z_base;
      history->last_lift_surface_uncertainty_m =
          current_surface_vertical.uncertainty_m;
      history->last_lift_surface_owner_overlap_cells =
          current_surface_vertical.owner_overlap_cell_count;
      history->last_lift_surface_owner_coverage =
          current_surface_vertical.owner_overlap_coverage;
      // A valid EMPTY gravity observation is explicit evidence that lift has
      // not begun.  AUXILIARY keeps its independent LiDAR pre-lift/reference
      // path, but EMPTY may not be accumulated into a false lift transition.
      // Already-confirmed histories retain the existing conflict/retention
      // behavior; this gate only prevents an unconfirmed false -> true edge.
      const bool empty_inhibits_unconfirmed_lift =
          input.hook_role == HookLoadSignalRole::AUXILIARY &&
          gravity_empty && !history->lift_confirmed;
      if (empty_inhibits_unconfirmed_lift) {
        history->lift_confirm_count = 0;
      } else if (delta >= threshold) {
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
            current_surface_vertical.uncertainty_m;
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
  decision_.physical_cargo_epoch_id = physical_cargo_epoch_id_;
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
      std::size_t current_group_index = input.groups.size();
      for (std::size_t gi = 0; gi < input.groups.size(); ++gi) {
        if (group_history_ids[gi] == selected->id) {
          current_group = &input.groups[gi];
          current_group_index = gi;
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
      decision_.baseline_frozen = selected->baseline_frozen;
      decision_.surface_reference_frozen =
          selected->surface_lift_reference_frozen;
      decision_.surface_reference_footprint_valid =
          selected->frozen_preload_footprint.valid;
      decision_.surface_reference_footprint =
          selected->frozen_preload_footprint;
      decision_.surface_baseline_z = selected->surface_baseline_z;
      decision_.surface_baseline_uncertainty_m =
          selected->surface_baseline_uncertainty_m;
      decision_.prelift_state = selected->prelift_state;
      decision_.prelift_sample_count =
          selected->earliest_prelift_samples.size();
      decision_.prelift_reference_uncertainty_m =
          selected->prelift_reference_uncertainty_m;
      decision_.prelift_reference_first_stamp =
          selected->prelift_reference_first_stamp;
      decision_.prelift_reference_last_stamp =
          selected->prelift_reference_last_stamp;
      decision_.prelift_close_reason = selected->prelift_close_reason;
      decision_.current_candidate_fresh = current_group != nullptr;
      decision_.lift_confirmed = selected->lift_confirmed;
      decision_.lift_confirm_count = selected->lift_confirm_count;
      decision_.required_lift_confirm_frames = requiredFrames(
          input.hook_role, input.gravity_valid, input.gravity_state,
          config_.lift_confirm_frames);
      decision_.baseline_z95 = selected->baseline_z95;
      if (current_group_index < decision_.group_diagnostics.size()) {
        const auto& current_diagnostic =
            decision_.group_diagnostics[current_group_index];
        decision_.current_surface_vertical_valid =
            current_diagnostic.current_surface_vertical_valid;
        decision_.current_surface_z = current_diagnostic.current_surface_z;
        decision_.current_surface_owner_overlap_cells =
            current_diagnostic.current_surface_owner_overlap_cells;
        decision_.current_surface_owner_coverage =
            current_diagnostic.current_surface_owner_coverage;
      }
      decision_.current_z95 = decision_.current_surface_z;
      decision_.lift_vertical_source =
          decision_.current_surface_vertical_valid
              ? "FROZEN_PRELOAD_FOOTPRINT_RAW_ROI_OWNER_PROOF"
              : "NONE";
      if (decision_.current_surface_vertical_valid) {
        decision_.lift_delta_m =
            decision_.current_z95 - decision_.baseline_z95;
        decision_.lift_threshold_m = std::max(
            config_.minimum_significant_change_m,
            config_.significance_sigma * std::hypot(
                selected->baseline_uncertainty_m,
                selected->last_lift_surface_uncertainty_m));
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
      diagnostic.prelift_state = history.prelift_state;
      diagnostic.prelift_sample_count =
          history.earliest_prelift_samples.size();
      diagnostic.physical_cargo_epoch_id =
          history.physical_cargo_epoch_id;
      diagnostic.prelift_reference_uncertainty_m =
          history.prelift_reference_uncertainty_m;
      diagnostic.prelift_reference_first_stamp =
          history.prelift_reference_first_stamp;
      diagnostic.prelift_reference_last_stamp =
          history.prelift_reference_last_stamp;
      diagnostic.prelift_close_reason = history.prelift_close_reason;
      diagnostic.baseline_z = history.baseline_z95;
      diagnostic.baseline_frozen = history.baseline_frozen;
      diagnostic.surface_reference_frozen =
          history.surface_lift_reference_frozen;
      diagnostic.surface_reference_footprint_valid =
          history.frozen_preload_footprint.valid;
      diagnostic.surface_reference_footprint =
          history.frozen_preload_footprint;
      diagnostic.surface_baseline_uncertainty_m =
          history.surface_baseline_uncertainty_m;
      diagnostic.last_supported_evidence_stamp =
          history.last_supported_evidence_stamp_sec;
      diagnostic.lift_confirm_count = history.lift_confirm_count;
      diagnostic.lift_confirm_required = requiredFrames(
          input.hook_role, input.gravity_valid, input.gravity_state,
          config_.lift_confirm_frames);
      diagnostic.lift_confirmed = history.lift_confirmed;
      if (history.baseline_frozen &&
          diagnostic.current_surface_vertical_valid) {
        diagnostic.lift_delta_m =
            diagnostic.current_surface_z - history.baseline_z95;
        diagnostic.lift_threshold_m = std::max(
            config_.minimum_significant_change_m,
            config_.significance_sigma * std::hypot(
                history.baseline_uncertainty_m,
                history.last_lift_surface_uncertainty_m));
      }
      diagnostic.identity = history.id == validated_history_id_
          ? CargoPhysicalIdentityState::VALIDATED
          : CargoPhysicalIdentityState::UNKNOWN;
      break;
    }
  }

  // Before identity validation there is intentionally no selected Cargo, but
  // a single unambiguous physical group still owns the pre-lift diagnostic.
  // This copies telemetry only and does not grant identity authority.
  if (decision_.physical_history_id == 0U &&
      decision_.group_diagnostics.size() == 1U &&
      decision_.group_diagnostics.front().association !=
          CargoCandidateAssociationState::AMBIGUOUS) {
    const auto& diagnostic = decision_.group_diagnostics.front();
    decision_.prelift_state = diagnostic.prelift_state;
    decision_.prelift_sample_count = diagnostic.prelift_sample_count;
    decision_.prelift_reference_uncertainty_m =
        diagnostic.prelift_reference_uncertainty_m;
    decision_.prelift_reference_first_stamp =
        diagnostic.prelift_reference_first_stamp;
    decision_.prelift_reference_last_stamp =
        diagnostic.prelift_reference_last_stamp;
    decision_.prelift_close_reason = diagnostic.prelift_close_reason;
    decision_.baseline_frozen = diagnostic.baseline_frozen;
    decision_.surface_reference_frozen =
        diagnostic.surface_reference_frozen;
    decision_.surface_reference_footprint_valid =
        diagnostic.surface_reference_footprint_valid;
    decision_.surface_reference_footprint =
        diagnostic.surface_reference_footprint;
    decision_.surface_baseline_z = diagnostic.baseline_z;
    decision_.surface_baseline_uncertainty_m =
        diagnostic.surface_baseline_uncertainty_m;
    decision_.current_surface_vertical_valid =
        diagnostic.current_surface_vertical_valid;
    decision_.current_surface_z = diagnostic.current_surface_z;
    decision_.current_surface_owner_overlap_cells =
        diagnostic.current_surface_owner_overlap_cells;
    decision_.current_surface_owner_coverage =
        diagnostic.current_surface_owner_coverage;
    decision_.lift_vertical_source =
        diagnostic.current_surface_vertical_valid
            ? "FROZEN_PRELOAD_FOOTPRINT_RAW_ROI_OWNER_PROOF"
            : "NONE";
  }

  // Diagnostic Surface Reference Lock (Phase A counterfactual).
  {
    auto& lock = diagnostic_reference_lock_;
    if (lock.frozen && gravity_loaded &&
        lock.phase ==
            DiagnosticSurfaceReferenceLock::Phase::PRELOAD_ACTIVE) {
      lock.phase = DiagnosticSurfaceReferenceLock::Phase::POSTLOAD_ACTIVE;
      lock.lift_confirm_count = 0;
      lock.lift_confirmed = false;
    }
    if (lock.frozen &&
        lock.phase ==
            DiagnosticSurfaceReferenceLock::Phase::POSTLOAD_ACTIVE) {
      // Current-frame owner assembly: seed + high-Z vertical fragments.
      const CurrentFrameOwnerAssemblyResult assembly =
          assembleCurrentFrameOwner(input.groups, lock.frozen_footprint,
                                    lock.frozen_owner_cells,
                                    input.vertical_config, 1.2);
      decision_.assembly_valid = assembly.valid;
      decision_.assembly_seed_group = assembly.seed_group;
      decision_.assembly_member_count = assembly.member_groups.size();
      decision_.assembly_high_z_exists = assembly.high_z_group_exists;
      decision_.assembly_high_z_joined = assembly.high_z_group_joined;
      decision_.assembly_high_z_reject_reason =
          assembly.high_z_group_reject_reason;
      decision_.assembly_reject_reason = assembly.reject_reason;

      double assembly_surface_z = std::numeric_limits<double>::quiet_NaN();
      if (assembly.valid && !assembly.member_groups.empty() &&
          input.frame_evidence.raw_roi_current_frame) {
        CargoVerticalEvidenceInput asm_input;
        asm_input.footprint_valid = true;
        asm_input.footprint_center_base = lock.frozen_footprint.center_base;
        asm_input.footprint_size_xy = lock.frozen_footprint.size_xy;
        asm_input.footprint_yaw_base_rad =
            lock.frozen_footprint.yaw_base_rad;
        asm_input.ground_reference_valid =
            input.frame_evidence.ground_reference_valid;
        asm_input.ground_z_base = input.frame_evidence.ground_z_base;
        std::set<CargoFootprintGridIndex> authorized;
        for (const int gi : assembly.member_groups) {
          const auto cells = cellsOfGroup(
              input.groups[static_cast<std::size_t>(gi)], asm_input,
              input.vertical_config);
          authorized.insert(cells.begin(), cells.end());
        }
        std::vector<Eigen::Vector3f> authorized_raw;
        for (const pcl::PointXYZ& p :
             input.frame_evidence.raw_roi_current_frame->points) {
          const Eigen::Vector3f pt(p.x, p.y, p.z);
          if (!pt.allFinite() || !cargoPointInsideFootprint(
                  pt, asm_input, input.vertical_config.footprint_margin_m)) {
            continue;
          }
          const CargoFootprintGridIndex cell = makeCargoFootprintGridIndex(
              pt, asm_input, input.vertical_config.xy_cell_size_m);
          if (authorized.count(cell) == 0U) continue;
          authorized_raw.push_back(pt);
        }
        if (!authorized_raw.empty()) {
          CargoVerticalEvidenceInput owner_input = asm_input;
          owner_input.selected_points_base = std::move(authorized_raw);
          const CargoVerticalEvidence evidence =
              extractCargoVerticalEvidence(owner_input, input.vertical_config);
          if (evidence.valid && std::isfinite(evidence.top_z_base)) {
            assembly_surface_z = evidence.top_z_base;
          }
        }
      }
      decision_.assembly_surface_z = assembly_surface_z;
      const double asm_threshold = liftSignificanceThreshold(
          config_, lock.baseline_uncertainty_m, 0.20);
      const double asm_lift = std::isfinite(assembly_surface_z)
          ? assembly_surface_z - lock.baseline_z
          : std::numeric_limits<double>::quiet_NaN();
      const bool asm_significant = std::isfinite(asm_lift) &&
          asm_lift > asm_threshold + kEpsilon;
      decision_.assembly_lift_delta = asm_lift;
      decision_.assembly_lift_significant = asm_significant;
      if (asm_significant) {
        ++lock.assembly_lift_confirm_count;
      } else {
        lock.assembly_lift_confirm_count = 0;
      }
      if (lock.assembly_lift_confirm_count >=
          std::max(1, config_.lift_confirm_frames)) {
        lock.assembly_lift_confirmed = true;
      }
      decision_.assembly_lift_confirm_count =
          lock.assembly_lift_confirm_count;
      decision_.assembly_simulated_validated = lock.assembly_lift_confirmed;

      // Pre-cluster vertical: range cloud within the CURRENT owner footprint.
      {
        double precluster_z = std::numeric_limits<double>::quiet_NaN();
        double precluster_uncertainty =
            std::numeric_limits<double>::quiet_NaN();
        std::string precluster_reason = "NO_CURRENT_OWNER";
        if (assembly.valid && assembly.seed_group >= 0) {
          const CargoFootprintSnapshot seed_footprint =
              robustFootprintSnapshot(
                  input.groups[static_cast<std::size_t>(
                      assembly.seed_group)]);
          const OwnerLockedSurfaceResult pc =
              computePreClusterSurfaceVertical(
                  input.frame_evidence, seed_footprint, input.vertical_config);
          decision_.precluster_surface_valid = pc.valid;
          precluster_z = pc.surface_z;
          precluster_uncertainty = pc.surface_uncertainty;
          precluster_reason = pc.reject_reason;
        }
        decision_.precluster_surface_z = precluster_z;
        decision_.precluster_reject_reason = precluster_reason;
        const double current_uncertainty = std::isfinite(precluster_uncertainty)
            ? precluster_uncertainty : 0.20;
        const double pc_threshold = liftSignificanceThreshold(
            config_, lock.baseline_uncertainty_m, current_uncertainty);
        const double pc_lift = std::isfinite(precluster_z)
            ? precluster_z - lock.baseline_z
            : std::numeric_limits<double>::quiet_NaN();
        const bool pc_significant = std::isfinite(pc_lift) &&
            pc_lift > pc_threshold + kEpsilon;
        decision_.precluster_lift_delta = pc_lift;
        decision_.precluster_lift_significant = pc_significant;
        if (pc_significant) {
          ++lock.precluster_lift_confirm_count;
        } else {
          lock.precluster_lift_confirm_count = 0;
        }
        if (lock.precluster_lift_confirm_count >=
            std::max(1, config_.lift_confirm_frames)) {
          lock.precluster_lift_confirmed = true;
        }
        decision_.precluster_lift_confirm_count =
            lock.precluster_lift_confirm_count;
        decision_.precluster_simulated_validated =
            lock.precluster_lift_confirmed;
      }
    }
    if (lock.frozen &&
        lock.phase ==
            DiagnosticSurfaceReferenceLock::Phase::PRELOAD_ACTIVE &&
        gravity_empty) {
      // Pre-load refresh: count owner matches against the frozen mask.
      int match_count = 0;
      for (const auto& g : input.groups) {
        if (g.group_ambiguous || !g.geometry_resolved ||
            g.union_points_base.empty() || !finiteDescriptor(g.descriptor)) {
          continue;
        }
        const OwnerLockedSurfaceResult v31 = computeOwnerLockedSurfaceVertical(
            input.frame_evidence, lock.frozen_footprint,
            lock.frozen_owner_cells, g.union_points_base,
            input.vertical_config, nullptr);
        if (v31.valid) ++match_count;
      }
      if (match_count == 1) {
        lock.last_owner_refresh_stamp = input.pipeline_stamp_sec;
      }
    }
    // Emit diagnostic fields to the decision.
    decision_.ref_lock_frozen = lock.frozen;
    decision_.ref_lock_phase =
        lock.phase ==
                DiagnosticSurfaceReferenceLock::Phase::PRELOAD_ACTIVE
            ? "PRELOAD_ACTIVE"
            : (lock.phase ==
                       DiagnosticSurfaceReferenceLock::Phase::POSTLOAD_ACTIVE
                   ? "POSTLOAD_ACTIVE"
                   : "NONE");
    decision_.ref_lock_source_history_id = lock.source_history_id;
    decision_.ref_lock_current_history_id = lock.current_history_id;
    decision_.ref_lock_history_id_changed =
        lock.current_history_id != 0U &&
        lock.current_history_id != lock.source_history_id;
    decision_.ref_lock_postload_history_id_change_count =
        lock.postload_history_id_change_count;
    decision_.ref_lock_lift_confirm_count = lock.lift_confirm_count;
    decision_.ref_lock_lift_confirmed = lock.lift_confirmed;
    decision_.ref_lock_simulated_validated =
        lock.lift_confirmed &&
        lock.phase ==
            DiagnosticSurfaceReferenceLock::Phase::POSTLOAD_ACTIVE;
    decision_.ref_lock_empty_lift_confirm_advance =
        lock.empty_lift_confirm_advance;
    decision_.ref_lock_significant_frames_after_split =
        lock.significant_frames_after_split;
    decision_.ref_lock_v31_valid =
        lock.phase ==
            DiagnosticSurfaceReferenceLock::Phase::POSTLOAD_ACTIVE &&
        decision_.ref_lock_lift_confirm_count > 0;
  }

  previous_existence_phase_ = gravity_loaded || decision_.cargo_exists;
  previous_gravity_valid_ = input.gravity_valid;
  previous_gravity_state_ = input.gravity_state;
  return decision_;
}

}  // namespace ndt_slam
