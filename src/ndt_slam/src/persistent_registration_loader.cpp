#include "ndt_slam/persistent_registration_loader.hpp"

#include "ndt_slam/map_session_snapshot.hpp"

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace ndt_slam {
namespace {

namespace fs = std::filesystem;

constexpr char kManifestName[] = "persistent_map_manifest.json";
constexpr char kRegistrationDirectory[] = "tiles_registration";
constexpr char kManifestSchema[] = "ndt_slam_persistent_tile_catalog";
constexpr std::uint32_t kLegacyManifestSchemaVersion = 1U;
constexpr std::uint32_t kManifestSchemaVersion = 2U;
constexpr const char* kLayerDirectories[] = {
    "tiles_registration", "tiles_display", "tiles_ground", "tiles_objects"};

PersistentRegistrationLoadResult invalidResult(const std::string& reason) {
  PersistentRegistrationLoadResult result;
  result.status =
      PersistentRegistrationLoadStatus::INVALID_EXISTING_MAP;
  result.reason = reason;
  return result;
}

bool enumerateRegistrationTiles(const fs::path& directory,
                                std::vector<fs::path>* paths,
                                std::string* reason) {
  paths->clear();
  std::error_code error;
  if (!fs::exists(directory, error)) {
    if (error && reason) *reason = "registration_directory_stat_failed";
    return !error;
  }
  if (!fs::is_directory(directory, error) || error) {
    if (reason) *reason = "registration_path_not_directory";
    return false;
  }
  for (fs::directory_iterator it(directory, error), end;
       !error && it != end; it.increment(error)) {
    if (it->is_regular_file(error) && !error &&
        it->path().extension() == ".pcd") {
      paths->push_back(it->path());
    }
  }
  if (error) {
    if (reason) *reason = "registration_tile_scan_failed";
    return false;
  }
  std::sort(paths->begin(), paths->end());
  return true;
}

bool isSafeRegistrationPath(const std::string& value) {
  const fs::path relative(value);
  if (value.empty() || relative.is_absolute() || relative.has_root_name() ||
      relative.extension() != ".pcd") {
    return false;
  }
  if (value.find('\\') != std::string::npos ||
      relative.parent_path().generic_string() != kRegistrationDirectory) {
    return false;
  }
  for (const auto& component : relative) {
    if (component == ".." || component == ".") return false;
  }
  return true;
}

bool hasAnyPersistentTileOnDisk(const fs::path& root, std::string* reason) {
  for (const char* layer : kLayerDirectories) {
    std::error_code error;
    const fs::path directory = root / layer;
    if (!fs::exists(directory, error)) {
      if (error) {
        if (reason) *reason = "persistent_layer_stat_failed";
        return true;
      }
      continue;
    }
    if (!fs::is_directory(directory, error) || error) {
      if (reason) *reason = "persistent_layer_not_directory";
      return true;
    }
    for (fs::directory_iterator it(directory, error), end;
         !error && it != end; it.increment(error)) {
      if (it->is_regular_file(error) && !error &&
          it->path().extension() == ".pcd") {
        return true;
      }
    }
    if (error) {
      if (reason) *reason = "persistent_layer_scan_failed";
      return true;
    }
  }
  return false;
}

bool validateLayerCatalog(const fs::path& root,
                          const char* manifest_layer_name,
                          const char* directory_name,
                          const YAML::Node& entries,
                          std::string* reason) {
  if (!entries.IsSequence()) {
    if (reason) {
      *reason = std::string("persistent_layer_catalog_invalid:") +
          manifest_layer_name;
    }
    return false;
  }
  std::set<std::string> catalog_paths;
  for (std::size_t item_index = 0U;
       item_index < entries.size(); ++item_index) {
    const YAML::Node item = entries[item_index];
    if (!item.IsMap()) {
      if (reason) *reason = "persistent_layer_entry_invalid";
      return false;
    }
    const std::string relative_value = item["path"].as<std::string>("");
    const fs::path relative(relative_value);
    if (relative_value.empty() || relative.is_absolute() ||
        relative.has_root_name() ||
        relative.parent_path().generic_string() != directory_name ||
        relative.extension() != ".pcd" ||
        relative_value.find('\\') != std::string::npos ||
        !catalog_paths.insert(relative.generic_string()).second) {
      if (reason) *reason = "persistent_layer_path_invalid";
      return false;
    }
    for (const auto& component : relative) {
      if (component == ".." || component == ".") {
        if (reason) *reason = "persistent_layer_path_invalid";
        return false;
      }
    }
    const fs::path tile_path = root / relative;
    std::error_code file_error;
    if (!fs::is_regular_file(tile_path, file_error) || file_error) {
      if (reason) *reason = "persistent_layer_tile_missing:" + relative_value;
      return false;
    }
    const std::uintmax_t expected_bytes =
        item["bytes"].as<std::uintmax_t>();
    if (fs::file_size(tile_path, file_error) != expected_bytes || file_error) {
      if (reason) *reason = "persistent_layer_size_mismatch:" + relative_value;
      return false;
    }
    const std::string expected_hash = item["sha256"].as<std::string>("");
    std::string hash_reason;
    if (expected_hash.size() != 64U ||
        MapSessionSnapshot::sha256File(tile_path.string(), &hash_reason) !=
            expected_hash) {
      if (reason) *reason = "persistent_layer_hash_mismatch:" + relative_value;
      return false;
    }
  }

  std::set<std::string> disk_paths;
  const fs::path directory = root / directory_name;
  std::error_code directory_error;
  const bool directory_exists = fs::exists(directory, directory_error);
  if (directory_error) {
    if (reason) *reason = "persistent_layer_stat_failed";
    return false;
  }
  if (directory_exists) {
    if (!fs::is_directory(directory, directory_error) || directory_error) {
      if (reason) *reason = "persistent_layer_not_directory";
      return false;
    }
    for (fs::directory_iterator it(directory, directory_error), end;
         !directory_error && it != end; it.increment(directory_error)) {
      if (it->is_regular_file(directory_error) && !directory_error &&
          it->path().extension() == ".pcd") {
        disk_paths.insert(
            (fs::path(directory_name) / it->path().filename())
                .generic_string());
      }
    }
  }
  if (directory_error || disk_paths != catalog_paths) {
    if (reason) {
      *reason = std::string("persistent_layer_catalog_mismatch:") +
          manifest_layer_name;
    }
    return false;
  }
  return true;
}

}  // namespace

PersistentRegistrationLoadResult loadPersistentRegistrationLayer(
    const std::string& root_directory,
    const std::string& expected_map_uuid,
    double expected_tile_size_m) {
  const fs::path root(root_directory);
  const fs::path registration_directory = root / kRegistrationDirectory;
  const fs::path manifest_path = root / kManifestName;

  std::vector<fs::path> disk_tiles;
  std::string scan_reason;
  if (!enumerateRegistrationTiles(
          registration_directory, &disk_tiles, &scan_reason)) {
    return invalidResult(scan_reason);
  }

  std::error_code manifest_error;
  const bool manifest_path_exists = fs::exists(manifest_path, manifest_error);
  if (manifest_error) return invalidResult("persistent_manifest_stat_failed");
  if (!manifest_path_exists) {
    std::string layer_reason;
    if (!hasAnyPersistentTileOnDisk(root, &layer_reason)) {
      PersistentRegistrationLoadResult result;
      result.status =
          PersistentRegistrationLoadStatus::NEW_MAP_BOOTSTRAP;
      result.reason = "new_map_no_registration_tiles";
      return result;
    }
    return invalidResult(
        layer_reason.empty() ? "persistent_manifest_missing" : layer_reason);
  }
  if (!fs::is_regular_file(manifest_path, manifest_error) || manifest_error) {
    return invalidResult("persistent_manifest_not_regular_file");
  }

  try {
    const YAML::Node manifest = YAML::LoadFile(manifest_path.string());
    const std::uint32_t schema_version =
        manifest["schema_version"].as<std::uint32_t>(0U);
    if (!manifest.IsMap() ||
        manifest["schema"].as<std::string>("") != kManifestSchema ||
        (schema_version != kLegacyManifestSchemaVersion &&
         schema_version != kManifestSchemaVersion)) {
      return invalidResult("persistent_manifest_schema_invalid");
    }
    // Schema 1 used a root-path-derived catalog identifier. Preserve that
    // loader contract for old deployments. Schema 2 carries a path-independent
    // map-frame identity, so moving the exact catalog into a sandbox cannot
    // change its rail/yaw identity.
    if (schema_version == kLegacyManifestSchemaVersion &&
        manifest["map_uuid"].as<std::string>("") != expected_map_uuid) {
      return invalidResult("persistent_manifest_uuid_mismatch");
    }

    std::string map_frame_uuid;
    RailYawReference yaw_reference;
    bool rail_write_authorized = false;
    if (schema_version == kManifestSchemaVersion) {
      map_frame_uuid = manifest["map_frame_uuid"].as<std::string>("");
      if (map_frame_uuid.empty()) {
        return invalidResult("persistent_manifest_map_frame_uuid_missing");
      }
      const YAML::Node reference = manifest["yaw_reference"];
      if (reference && reference.IsMap()) {
        yaw_reference.schema_version =
            reference["schema_version"].as<std::uint32_t>(0U);
        yaw_reference.verified = reference["verified"].as<bool>(false);
        yaw_reference.rail_yaw_in_map_rad =
            reference["rail_yaw_in_map_rad"].as<double>(0.0);
        if (!yawReferenceSourceFromName(
                reference["source"].as<std::string>(""),
                &yaw_reference.source)) {
          return invalidResult("persistent_manifest_yaw_source_invalid");
        }
        yaw_reference.map_frame_uuid =
            reference["map_frame_uuid"].as<std::string>("");
        yaw_reference.map_frame_id =
            reference["map_frame_id"].as<std::string>("");
        yaw_reference.base_frame_id =
            reference["base_frame_id"].as<std::string>("");
        yaw_reference.map_frame_convention_id =
            reference["map_frame_convention_id"].as<std::string>("");
        yaw_reference.sensor_rig_calibration_id =
            reference["sensor_rig_calibration_id"].as<std::string>("");
        yaw_reference.reference_uuid =
            reference["reference_uuid"].as<std::string>("");
        yaw_reference.reference_hash =
            reference["reference_hash"].as<std::string>("");
        const std::string canonical_hash =
            semanticYawReferenceHash(yaw_reference);
        if (yaw_reference.verified &&
            (yaw_reference.map_frame_uuid != map_frame_uuid ||
             yaw_reference.reference_hash.empty() ||
             yaw_reference.reference_hash != canonical_hash)) {
          return invalidResult("persistent_manifest_yaw_reference_invalid");
        }
        rail_write_authorized = yaw_reference.verified;
      }
    }
    const double manifest_tile_size =
        manifest["tile_size_m"].as<double>(0.0);
    if (!std::isfinite(manifest_tile_size) ||
        !std::isfinite(expected_tile_size_m) ||
        expected_tile_size_m <= 0.0 ||
        std::abs(manifest_tile_size - expected_tile_size_m) > 1.0e-6) {
      return invalidResult("persistent_manifest_tile_size_mismatch");
    }

    const YAML::Node layers = manifest["layers"];
    if (!layers.IsMap()) {
      return invalidResult("persistent_layers_catalog_invalid");
    }
    bool manifest_has_any_tile = false;
    for (const char* layer : {"registration", "display", "ground", "objects"}) {
      const YAML::Node catalog = layers[layer];
      if (!catalog.IsSequence()) {
        return invalidResult("persistent_layer_catalog_invalid");
      }
      manifest_has_any_tile = manifest_has_any_tile || catalog.size() > 0U;
    }
    const YAML::Node registration = layers["registration"];
    if (!registration.IsSequence()) {
      return invalidResult("persistent_registration_catalog_invalid");
    }
    if (registration.size() == 0U) {
      std::string layer_reason;
      if (!manifest_has_any_tile &&
          !hasAnyPersistentTileOnDisk(root, &layer_reason)) {
        PersistentRegistrationLoadResult result;
        result.status =
            PersistentRegistrationLoadStatus::NEW_MAP_BOOTSTRAP;
        result.reason = "new_map_empty_registration_catalog";
        return result;
      }
      return invalidResult(
          layer_reason.empty()
              ? "persistent_registration_catalog_empty"
              : layer_reason);
    }

    const std::pair<const char*, const char*> other_layers[] = {
        {"display", "tiles_display"},
        {"ground", "tiles_ground"},
        {"objects", "tiles_objects"},
    };
    for (const auto& layer : other_layers) {
      std::string layer_reason;
      if (!validateLayerCatalog(
              root, layer.first, layer.second, layers[layer.first],
              &layer_reason)) {
        return invalidResult(layer_reason);
      }
    }

    std::set<std::string> catalog_paths;
    struct CatalogEntry {
      fs::path absolute_path;
      std::string relative_path;
      std::uintmax_t bytes = 0U;
      std::string sha256;
    };
    std::vector<CatalogEntry> entries;
    entries.reserve(registration.size());
    for (std::size_t item_index = 0U;
         item_index < registration.size(); ++item_index) {
      const YAML::Node item = registration[item_index];
      if (!item.IsMap()) {
        return invalidResult("persistent_registration_entry_invalid");
      }
      CatalogEntry entry;
      entry.relative_path = item["path"].as<std::string>("");
      if (!isSafeRegistrationPath(entry.relative_path) ||
          !catalog_paths.insert(entry.relative_path).second) {
        return invalidResult("persistent_registration_path_invalid");
      }
      entry.bytes = item["bytes"].as<std::uintmax_t>();
      entry.sha256 = item["sha256"].as<std::string>("");
      if (entry.sha256.size() != 64U) {
        return invalidResult("persistent_registration_hash_invalid");
      }
      entry.absolute_path = root / fs::path(entry.relative_path);
      entries.push_back(std::move(entry));
    }

    std::set<std::string> disk_paths;
    for (const fs::path& path : disk_tiles) {
      disk_paths.insert(
          (fs::path(kRegistrationDirectory) / path.filename())
              .generic_string());
    }
    if (catalog_paths != disk_paths) {
      return invalidResult("persistent_registration_catalog_mismatch");
    }

    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (const CatalogEntry& entry : entries) {
      std::error_code file_error;
      if (!fs::is_regular_file(entry.absolute_path, file_error) || file_error) {
        return invalidResult("persistent_registration_tile_missing:" +
                             entry.relative_path);
      }
      if (fs::file_size(entry.absolute_path, file_error) != entry.bytes ||
          file_error) {
        return invalidResult("persistent_registration_size_mismatch:" +
                             entry.relative_path);
      }
      std::string hash_reason;
      const std::string actual_hash = MapSessionSnapshot::sha256File(
          entry.absolute_path.string(), &hash_reason);
      if (actual_hash.empty() || actual_hash != entry.sha256) {
        return invalidResult("persistent_registration_hash_mismatch:" +
                             entry.relative_path);
      }
      pcl::PointCloud<pcl::PointXYZ> tile;
      if (pcl::io::loadPCDFile(entry.absolute_path.string(), tile) < 0 ||
          tile.empty()) {
        return invalidResult("persistent_registration_decode_failed:" +
                             entry.relative_path);
      }
      *cloud += tile;
    }
    if (cloud->empty()) {
      return invalidResult("persistent_registration_empty");
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1U;

    PersistentRegistrationLoadResult result;
    result.status = PersistentRegistrationLoadStatus::RESTORED;
    result.reason = "persistent_registration_restored";
    result.cloud = std::move(cloud);
    result.tile_count = entries.size();
    result.map_frame_uuid = std::move(map_frame_uuid);
    result.yaw_reference = std::move(yaw_reference);
    result.rail_write_authorized = rail_write_authorized;
    return result;
  } catch (const std::exception& error) {
    return invalidResult(
        std::string("persistent_manifest_parse_failed:") + error.what());
  }
}

}  // namespace ndt_slam
