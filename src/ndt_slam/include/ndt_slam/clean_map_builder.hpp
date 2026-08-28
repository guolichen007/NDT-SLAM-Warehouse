#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ndt_slam {

inline constexpr float kCleanMapCellSizeM = 0.15F;

using CleanMapCell = std::pair<int, int>;

struct CleanMapDenyRange {
    float z_min = 0.0F;
    float z_max = 0.0F;
};

struct CleanMapBuildInput {
    float cell_size_m = kCleanMapCellSizeM;
    std::vector<Eigen::Vector3f> object_points;
    // The last coherent clean layer is a retention baseline, not a synthetic
    // observation. A cell may reuse these exact points while its current raw
    // content has not yet reached the temporal observation threshold.
    std::vector<Eigen::Vector3f> previous_clean_points;
    std::vector<Eigen::Vector3f> payload_candidate_points;
    std::map<CleanMapCell, int> observation_counts;
    std::set<CleanMapCell> deny_cells;
    std::set<CleanMapCell> protect_cells;
    std::set<CleanMapCell> human_deny_cells;
    std::map<CleanMapCell, std::vector<CleanMapDenyRange>> deny_ranges;
    bool use_human_deny = false;
    bool use_3d_deny = false;
};

struct CleanMapBuildResult {
    bool valid = false;
    std::string reason = "not_built";
    std::vector<Eigen::Vector3f> clean_points;
    int total_cells = 0;
    int passed_cells = 0;
    int denied_cells = 0;
    int denied_points = 0;
    int protected_cells = 0;
    int protected_points = 0;
    int human_denied_cells = 0;
    int retained_cells = 0;
    int retained_points = 0;
};

enum class CleanMapBuildAction {
    APPLY,
    DISCARD_INVALID,
    // The raw snapshot and its derived clean layer remain a publishable,
    // coherent bundle even when the working map has advanced. It is simply
    // not installed as the current clean working layer.
    PUBLISH_SNAPSHOT_ONLY
};

CleanMapBuildAction evaluateCleanMapBuildAction(
    bool build_valid,
    bool newer_request_pending,
    std::uint64_t source_objects_version,
    std::uint64_t current_objects_version);

// Pure, thread-safe clean-layer reconstruction from immutable snapshots.
CleanMapBuildResult buildCleanMapFromSnapshot(
    const CleanMapBuildInput& input);

}  // namespace ndt_slam
