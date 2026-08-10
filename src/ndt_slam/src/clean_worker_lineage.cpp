#include "ndt_slam/clean_worker_lineage.hpp"

namespace ndt_slam {

CleanWorkerLineageDecision evaluateCleanWorkerLineage(
    bool worker_valid,
    const CleanWorkerLineage& source,
    const CleanWorkerLineage& current) {
  CleanWorkerLineageDecision decision;
  if (!worker_valid) {
    decision.reason = "worker_result_invalid";
    return decision;
  }
  if (source.localization_continuity_generation == 0U ||
      source.localization_continuity_generation !=
          current.localization_continuity_generation) {
    decision.reason = "localization_continuity_mismatch";
    return decision;
  }
  if (source.localization_map_generation !=
          current.localization_map_generation ||
      source.localization_map_uuid.empty() ||
      source.localization_map_uuid != current.localization_map_uuid) {
    decision.reason = "localization_map_identity_mismatch";
    return decision;
  }
  // Epoch zero is the valid initial map lifecycle.  Freshness comes from
  // equality in this version domain, not from assuming that every domain has
  // already advanced at least once.
  if (source.lifecycle_epoch != current.lifecycle_epoch) {
    decision.reason = "lifecycle_epoch_mismatch";
    return decision;
  }
  if (source.static_evidence_epoch == 0U ||
      source.static_evidence_epoch != current.static_evidence_epoch) {
    decision.reason = "static_evidence_epoch_mismatch";
    return decision;
  }
  if (source.source_accepted_pose_generation == 0U) {
    decision.reason = "source_accepted_pose_missing";
    return decision;
  }
  if (source.source_objects_version == 0U ||
      current.source_objects_version == 0U) {
    decision.reason = "objects_version_invalid";
    return decision;
  }

  decision.static_observation_authorized = true;
  if (source.source_objects_version == current.source_objects_version) {
    decision.action = CleanWorkerLineageAction::INSTALL_CURRENT;
    decision.reason = "lineage_current_objects_match";
  } else if (source.source_objects_version <
             current.source_objects_version) {
    decision.action = CleanWorkerLineageAction::PUBLISH_SNAPSHOT_ONLY;
    decision.reason = "lineage_valid_historical_objects_snapshot";
  } else {
    decision.static_observation_authorized = false;
    decision.reason = "source_objects_version_from_future";
  }
  return decision;
}

}  // namespace ndt_slam
