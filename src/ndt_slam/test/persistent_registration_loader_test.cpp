#include "ndt_slam/persistent_registration_loader.hpp"

#include "ndt_slam/map_session_snapshot.hpp"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace ndt_slam {
namespace {

namespace fs = std::filesystem;

class PersistentRegistrationLoaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
        ("ndt-registration-loader-test-" +
         MapSessionSnapshot::generateUuid());
    fs::create_directories(root_ / "tiles_registration");
  }

  void TearDown() override {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  fs::path writeTile(const std::string& name = "x0_y0.pcd") {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.0F, 2.0F, 3.0F));
    cloud.push_back(pcl::PointXYZ(2.0F, 3.0F, 4.0F));
    const fs::path path = root_ / "tiles_registration" / name;
    EXPECT_EQ(pcl::io::savePCDFileBinary(path.string(), cloud), 0);
    return path;
  }

  void writeManifest(const fs::path& tile,
                     const std::string& map_uuid = "test-map",
                     const std::string& hash_override = "") {
    std::string hash = hash_override;
    if (hash.empty()) {
      hash = MapSessionSnapshot::sha256File(tile.string());
    }
    std::ofstream output(root_ / "persistent_map_manifest.json");
    output << "{\n"
           << "  \"schema\": \"ndt_slam_persistent_tile_catalog\",\n"
           << "  \"schema_version\": 1,\n"
           << "  \"map_uuid\": \"" << map_uuid << "\",\n"
           << "  \"tile_size_m\": 20.0,\n"
           << "  \"layers\": {\n"
           << "    \"registration\": [{\"path\": \"tiles_registration/"
           << tile.filename().string() << "\", \"bytes\": "
           << fs::file_size(tile) << ", \"sha256\": \"" << hash
           << "\"}],\n"
           << "    \"display\": [],\n"
           << "    \"ground\": [],\n"
           << "    \"objects\": []\n"
           << "  }\n"
           << "}\n";
  }

  void writeSemanticManifest(const fs::path& tile,
                             const RailYawReference& reference) {
    const std::string hash = MapSessionSnapshot::sha256File(tile.string());
    std::ofstream output(root_ / "persistent_map_manifest.json");
    output << "{\n"
           << "  \"schema\": \"ndt_slam_persistent_tile_catalog\",\n"
           << "  \"schema_version\": 2,\n"
           << "  \"map_uuid\": \"legacy-path-id\",\n"
           << "  \"map_frame_uuid\": \"" << reference.map_frame_uuid
           << "\",\n"
           << "  \"yaw_reference\": {\n"
           << "    \"schema_version\": 1,\n"
           << "    \"verified\": true,\n"
           << "    \"rail_yaw_in_map_rad\": "
           << reference.rail_yaw_in_map_rad << ",\n"
           << "    \"source\": \""
           << yawReferenceSourceName(reference.source) << "\",\n"
           << "    \"map_frame_uuid\": \"" << reference.map_frame_uuid
           << "\",\n"
           << "    \"map_frame_id\": \"" << reference.map_frame_id
           << "\",\n"
           << "    \"base_frame_id\": \"" << reference.base_frame_id
           << "\",\n"
           << "    \"map_frame_convention_id\": \""
           << reference.map_frame_convention_id << "\",\n"
           << "    \"sensor_rig_calibration_id\": \""
           << reference.sensor_rig_calibration_id << "\",\n"
           << "    \"reference_uuid\": \"" << reference.reference_uuid
           << "\",\n"
           << "    \"reference_hash\": \"" << reference.reference_hash
           << "\"\n"
           << "  },\n"
           << "  \"tile_size_m\": 20.0,\n"
           << "  \"layers\": {\n"
           << "    \"registration\": [{\"path\": \"tiles_registration/"
           << tile.filename().string() << "\", \"bytes\": "
           << fs::file_size(tile) << ", \"sha256\": \"" << hash
           << "\"}],\n"
           << "    \"display\": [],\n"
           << "    \"ground\": [],\n"
           << "    \"objects\": []\n"
           << "  }\n"
           << "}\n";
  }

  fs::path root_;
};

TEST_F(PersistentRegistrationLoaderTest, RestoresVerifiedRegistrationLayer) {
  const fs::path tile = writeTile();
  writeManifest(tile);
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "test-map", 20.0);
  ASSERT_EQ(result.status, PersistentRegistrationLoadStatus::RESTORED)
      << result.reason;
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.cloud->size(), 2U);
  EXPECT_EQ(result.tile_count, 1U);
}

TEST_F(PersistentRegistrationLoaderTest, EmptyRootUsesNewMapBootstrap) {
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "test-map", 20.0);
  EXPECT_EQ(result.status,
            PersistentRegistrationLoadStatus::NEW_MAP_BOOTSTRAP);
  EXPECT_FALSE(result.cloud);
}

TEST_F(PersistentRegistrationLoaderTest,
       VerifiedEmptyCatalogUsesNewMapBootstrap) {
  std::ofstream output(root_ / "persistent_map_manifest.json");
  output << "{\n"
         << "  \"schema\": \"ndt_slam_persistent_tile_catalog\",\n"
         << "  \"schema_version\": 1,\n"
         << "  \"map_uuid\": \"test-map\",\n"
         << "  \"tile_size_m\": 20.0,\n"
         << "  \"layers\": {\"registration\": [], \"display\": [], "
            "\"ground\": [], \"objects\": []}\n"
         << "}\n";
  output.close();
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "test-map", 20.0);
  EXPECT_EQ(result.status,
            PersistentRegistrationLoadStatus::NEW_MAP_BOOTSTRAP);
  EXPECT_EQ(result.reason, "new_map_empty_registration_catalog");
}

TEST_F(PersistentRegistrationLoaderTest,
       ExistingTileWithoutManifestFailsClosed) {
  writeTile();
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "test-map", 20.0);
  EXPECT_EQ(result.status,
            PersistentRegistrationLoadStatus::INVALID_EXISTING_MAP);
  EXPECT_EQ(result.reason, "persistent_manifest_missing");
}

TEST_F(PersistentRegistrationLoaderTest,
       OtherPersistentLayerWithoutRegistrationFailsClosed) {
  fs::create_directories(root_ / "tiles_objects");
  std::ofstream(root_ / "tiles_objects" / "x0_y0.pcd") << "not-empty\n";
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "test-map", 20.0);
  EXPECT_EQ(result.status,
            PersistentRegistrationLoadStatus::INVALID_EXISTING_MAP);
  EXPECT_EQ(result.reason, "persistent_manifest_missing");
}

TEST_F(PersistentRegistrationLoaderTest, MapUuidMismatchFailsClosed) {
  const fs::path tile = writeTile();
  writeManifest(tile, "wrong-map");
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "test-map", 20.0);
  EXPECT_EQ(result.status,
            PersistentRegistrationLoadStatus::INVALID_EXISTING_MAP);
  EXPECT_EQ(result.reason, "persistent_manifest_uuid_mismatch");
}

TEST_F(PersistentRegistrationLoaderTest,
       SemanticMapFrameIdentitySurvivesSandboxPathChange) {
  const fs::path tile = writeTile();
  RailYawReference reference;
  reference.verified = true;
  reference.rail_yaw_in_map_rad = 0.125;
  reference.source = YawReferenceSource::VERIFIED_MAP_SESSION;
  reference.map_frame_uuid = "stable-map-frame";
  reference.map_frame_convention_id = "rail-x-positive";
  reference.sensor_rig_calibration_id = "test-rig";
  reference.reference_uuid = "stable-yaw-reference";
  reference.reference_hash = semanticYawReferenceHash(reference);
  writeSemanticManifest(tile, reference);

  // The expected legacy catalog id deliberately differs, as it would after
  // copying an identical map root into a replay sandbox.
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "different-sandbox-path-id", 20.0);
  ASSERT_EQ(result.status, PersistentRegistrationLoadStatus::RESTORED)
      << result.reason;
  EXPECT_EQ(result.map_frame_uuid, reference.map_frame_uuid);
  EXPECT_EQ(result.yaw_reference.reference_hash, reference.reference_hash);
  EXPECT_TRUE(result.rail_write_authorized);
}

TEST_F(PersistentRegistrationLoaderTest,
       LegacyManifestWithoutVerifiedYawReferenceIsRailReadOnly) {
  const fs::path tile = writeTile();
  writeManifest(tile);
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "test-map", 20.0);
  ASSERT_EQ(result.status, PersistentRegistrationLoadStatus::RESTORED)
      << result.reason;
  EXPECT_FALSE(result.rail_write_authorized);
  EXPECT_TRUE(result.map_frame_uuid.empty());
}

TEST_F(PersistentRegistrationLoaderTest, HashMismatchFailsClosed) {
  const fs::path tile = writeTile();
  writeManifest(tile, "test-map", std::string(64U, '0'));
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "test-map", 20.0);
  EXPECT_EQ(result.status,
            PersistentRegistrationLoadStatus::INVALID_EXISTING_MAP);
  EXPECT_EQ(result.reason,
            "persistent_registration_hash_mismatch:"
            "tiles_registration/x0_y0.pcd");
}

TEST_F(PersistentRegistrationLoaderTest, UnlistedTileFailsClosed) {
  const fs::path tile = writeTile();
  writeManifest(tile);
  writeTile("x1_y0.pcd");
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "test-map", 20.0);
  EXPECT_EQ(result.status,
            PersistentRegistrationLoadStatus::INVALID_EXISTING_MAP);
  EXPECT_EQ(result.reason, "persistent_registration_catalog_mismatch");
}

TEST_F(PersistentRegistrationLoaderTest,
       UnlistedObjectTileFailsClosedBeforeRegistrationRestore) {
  const fs::path tile = writeTile();
  writeManifest(tile);
  fs::create_directories(root_ / "tiles_objects");
  std::ofstream(root_ / "tiles_objects" / "x0_y0.pcd") << "not-empty\n";
  const auto result = loadPersistentRegistrationLayer(
      root_.string(), "test-map", 20.0);
  EXPECT_EQ(result.status,
            PersistentRegistrationLoadStatus::INVALID_EXISTING_MAP);
  EXPECT_EQ(result.reason, "persistent_layer_catalog_mismatch:objects");
}

}  // namespace
}  // namespace ndt_slam
