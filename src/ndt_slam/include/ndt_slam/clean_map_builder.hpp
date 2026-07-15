#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ndt_slam {

using CleanMapCell = std::pair<int, int>;

struct CleanMapDenyRange {
    float z_min = 0.0F;
    float z_max = 0.0F;
};

struct CleanMapBuildInput {
    float cell_size_m = 0.15F;
    std::vector<Eigen::Vector3f> object_points;
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
};

enum class CleanMapBuildAction {
    APPLY,
    DISCARD_INVALID,
    DISCARD_SUPERSEDED,
    DISCARD_STALE_OBJECTS
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
