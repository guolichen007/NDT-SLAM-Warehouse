#include "ndt_slam/registration_target_snapshot.hpp"

namespace ndt_slam {

RegistrationTargetSource registrationTargetSourceFromName(
    const std::string& source) noexcept {
  if (source == "cropped_localization_target") {
    return RegistrationTargetSource::CROPPED_ACTIVE_MAP;
  }
  if (source == "localization_target_snapshot") {
    return RegistrationTargetSource::LOCALIZATION_MAP;
  }
  if (source == "persistent_map") {
    return RegistrationTargetSource::PERSISTENT_MAP;
  }
  if (source == "local_map" || source == "bootstrap_local_map" ||
      source == "fallback_local_map") {
    return RegistrationTargetSource::GLOBAL_MAP;
  }
  return RegistrationTargetSource::UNKNOWN;
}

RegistrationTargetSnapshot makeRegistrationTargetSnapshot(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    RegistrationTargetSource source,
    std::uint64_t content_version,
    std::uint64_t map_rebuild_generation,
    const std::string& map_frame_uuid,
    const std::string& crop_identity) {
  RegistrationTargetSnapshot snapshot;
  snapshot.cloud = cloud;
  snapshot.source = source;
  snapshot.content_version = content_version;
  snapshot.map_rebuild_generation = map_rebuild_generation;
  snapshot.map_frame_uuid = map_frame_uuid;
  snapshot.crop_identity = crop_identity;
  if (!cloud || cloud->empty() ||
      source == RegistrationTargetSource::UNKNOWN) {
    return snapshot;
  }
  RegistrationTargetIdentityInput identity;
  identity.source = source;
  identity.content_version = content_version;
  identity.map_rebuild_generation = map_rebuild_generation;
  identity.map_frame_uuid = map_frame_uuid;
  identity.crop_identity = crop_identity;
  snapshot.target_snapshot_id = makeRegistrationTargetSnapshotId(identity);
  return snapshot;
}

}  // namespace ndt_slam
