#pragma once

#include <cstdint>
#include <string>

namespace ndt_slam {

// The AcceptedPose generation is audit metadata, not a freshness gate. An
// older immutable snapshot remains usable while its localization/map lineage
// is unchanged; only its working-layer installation authority is downgraded
// when a newer objects version exists.
struct CleanWorkerLineage {
  std::uint64_t localization_continuity_generation = 0U;
  std::uint64_t localization_map_generation = 0U;
  std::string localization_map_uuid;
  std::uint64_t lifecycle_epoch = 0U;
  std::uint64_t static_evidence_epoch = 0U;
  std::uint64_t source_accepted_pose_generation = 0U;
  std::uint64_t source_objects_version = 0U;
};

enum class CleanWorkerLineageAction : std::uint8_t {
  DISCARD = 0,
  INSTALL_CURRENT = 1,
  PUBLISH_SNAPSHOT_ONLY = 2,
};

struct CleanWorkerLineageDecision {
  CleanWorkerLineageAction action = CleanWorkerLineageAction::DISCARD;
  bool static_observation_authorized = false;
  std::string reason = "not_evaluated";
};

CleanWorkerLineageDecision evaluateCleanWorkerLineage(
    bool worker_valid,
    const CleanWorkerLineage& source,
    const CleanWorkerLineage& current);

}  // namespace ndt_slam
