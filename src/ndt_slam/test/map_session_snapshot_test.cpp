#include "ndt_slam/map_session_snapshot.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace ndt_slam {
namespace {

namespace fs = std::filesystem;

class MapSessionSnapshotTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
        ("ndt-map-session-test-" + MapSessionSnapshot::generateUuid());
    fs::create_directories(root_);
  }

  void TearDown() override {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  MapSessionSaveRequest request(const fs::path& target) {
    auto make_cloud = []() {
      auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
          new pcl::PointCloud<pcl::PointXYZ>);
      cloud->push_back(pcl::PointXYZ(1.0F, 2.0F, 3.0F));
      cloud->push_back(pcl::PointXYZ(2.0F, 3.0F, 4.0F));
      return cloud;
    };
    registration_ = make_cloud();
    display_ = make_cloud();
    ground_ = make_cloud();
    objects_raw_ = make_cloud();
    objects_clean_ = make_cloud();
    MapSessionSaveRequest value;
    value.target_directory = target.string();
    value.layers = {registration_, display_, ground_, objects_raw_,
                    objects_clean_};
    value.metadata.map_uuid = "11111111-2222-4333-8444-555555555555";
    value.metadata.map_generation = 9U;
    value.metadata.objects_content_version = 12U;
    value.metadata.clean_build_version = 7U;
    auto evidence = std::make_shared<StaticEvidenceSnapshot>();
    evidence->map_generation = 9U;
    evidence->revision = 3U;
    evidence->authority =
        StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE;
    value.static_evidence = evidence;
    value.write_extras = [](const std::string& temp, std::string*) {
      std::ofstream output(fs::path(temp) / "poses_raw.txt");
      output << "test\n";
      return output.good();
    };
    return value;
  }

  fs::path root_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registration_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr display_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects_raw_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects_clean_;
};

TEST_F(MapSessionSnapshotTest, AtomicRoundTripVerifiesAllFormalLayers) {
  const fs::path target = root_ / "session";
  std::string reason;
  ASSERT_TRUE(MapSessionSnapshot::saveAtomic(request(target), &reason))
      << reason;
  const auto loaded = MapSessionSnapshot::loadVerified(target.string());
  ASSERT_TRUE(loaded.valid) << loaded.reason;
  EXPECT_EQ(loaded.metadata.map_generation, 9U);
  EXPECT_FALSE(loaded.metadata.map_frame_uuid.empty());
  EXPECT_FALSE(loaded.metadata.rail_write_authorized);
  EXPECT_EQ(loaded.layers.registration->size(), 2U);
  EXPECT_EQ(loaded.static_authority,
            StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE);
  EXPECT_TRUE(fs::is_regular_file(target / "poses_raw.txt"));
  EXPECT_FALSE(fs::exists(target.string() + ".tmp"));
}

TEST_F(MapSessionSnapshotTest,
       SandboxPathChangeCannotAlterYawReferenceIdentity) {
  auto first = request(root_ / "sandbox-a" / "session");
  first.metadata.map_frame_uuid = "map-frame-stable-uuid";
  first.metadata.frame_id = "map";
  first.metadata.base_frame_id = "base_link";
  first.metadata.map_frame_convention_id = "rail-x-axis-v1";
  first.metadata.sensor_rig_calibration_id = "test-rig-v1";
  first.metadata.yaw_reference.verified = true;
  first.metadata.yaw_reference.rail_yaw_in_map_rad = 0.125;
  first.metadata.yaw_reference.source =
      YawReferenceSource::CONFIG_SITE_REFERENCE;
  first.metadata.yaw_reference.map_frame_uuid =
      first.metadata.map_frame_uuid;
  first.metadata.yaw_reference.map_frame_id = first.metadata.frame_id;
  first.metadata.yaw_reference.base_frame_id = first.metadata.base_frame_id;
  first.metadata.yaw_reference.map_frame_convention_id =
      first.metadata.map_frame_convention_id;
  first.metadata.yaw_reference.sensor_rig_calibration_id =
      first.metadata.sensor_rig_calibration_id;
  first.metadata.yaw_reference.reference_uuid = "reference-stable-uuid";
  first.metadata.yaw_reference.reference_hash =
      semanticYawReferenceHash(first.metadata.yaw_reference);
  auto second = first;
  second.target_directory = (root_ / "sandbox-b" / "session").string();
  std::string reason;
  ASSERT_TRUE(MapSessionSnapshot::saveAtomic(first, &reason)) << reason;
  ASSERT_TRUE(MapSessionSnapshot::saveAtomic(second, &reason)) << reason;
  const auto loaded_first =
      MapSessionSnapshot::loadVerified(first.target_directory);
  const auto loaded_second =
      MapSessionSnapshot::loadVerified(second.target_directory);
  ASSERT_TRUE(loaded_first.valid) << loaded_first.reason;
  ASSERT_TRUE(loaded_second.valid) << loaded_second.reason;
  EXPECT_TRUE(loaded_first.metadata.rail_write_authorized);
  EXPECT_TRUE(loaded_second.metadata.rail_write_authorized);
  EXPECT_EQ(loaded_first.metadata.map_frame_uuid,
            loaded_second.metadata.map_frame_uuid);
  EXPECT_EQ(loaded_first.metadata.yaw_reference.reference_hash,
            loaded_second.metadata.yaw_reference.reference_hash);
}

TEST_F(MapSessionSnapshotTest, Sha256MatchesPublishedVector) {
  const fs::path input = root_ / "abc.txt";
  std::ofstream stream(input, std::ios::binary);
  stream << "abc";
  stream.close();
  std::string reason;
  EXPECT_EQ(
      MapSessionSnapshot::sha256File(input.string(), &reason),
      "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(reason, "ok");
}

TEST_F(MapSessionSnapshotTest, HashMismatchRejectsWholeSession) {
  const fs::path target = root_ / "session";
  std::string reason;
  ASSERT_TRUE(MapSessionSnapshot::saveAtomic(request(target), &reason));
  std::ofstream corrupt(target / "map_objects_clean.pcd",
                        std::ios::binary | std::ios::app);
  corrupt << "corrupt";
  corrupt.close();
  const auto loaded = MapSessionSnapshot::loadVerified(target.string());
  EXPECT_FALSE(loaded.valid);
  EXPECT_EQ(loaded.reason, "hash_mismatch:objects_clean");
}

TEST_F(MapSessionSnapshotTest, ExtrasFailureLeavesNoPublishedSession) {
  const fs::path target = root_ / "session";
  auto value = request(target);
  value.write_extras = [](const std::string&, std::string* reason) {
    if (reason) *reason = "injected_failure";
    return false;
  };
  std::string reason;
  EXPECT_FALSE(MapSessionSnapshot::saveAtomic(value, &reason));
  EXPECT_FALSE(fs::exists(target));
}

TEST_F(MapSessionSnapshotTest, RuntimeHeightSelectionKeepsOnlyMatureCells) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0.1F, 0.1F, 1.0F));
  cloud.push_back(pcl::PointXYZ(1.1F, 0.1F, 1.0F));
  StaticEvidenceSnapshot evidence;
  evidence.cell_size_m = 1.0F;
  evidence.map_generation = 9U;
  evidence.authority = StaticEvidenceAuthority::RUNTIME_MATURE;
  StaticEvidenceCell mature;
  mature.key = packStaticEvidenceCell(0, 0);
  mature.clean_map_confirmed = true;
  mature.temporally_mature = true;
  mature.map_generation = 9U;
  evidence.cells.emplace(mature.key, mature);
  StaticEvidenceCell immature = mature;
  immature.key = packStaticEvidenceCell(1, 0);
  immature.temporally_mature = false;
  evidence.cells.emplace(immature.key, immature);
  const auto selected = selectStaticHeightPointsForAuthority(cloud, evidence);
  ASSERT_EQ(selected.size(), 1U);
  EXPECT_FLOAT_EQ(selected.front().x(), 0.1F);
}

TEST_F(MapSessionSnapshotTest, OperatorHeightSelectionRequiresExplicitCell) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0.1F, 0.1F, 1.0F));
  cloud.push_back(pcl::PointXYZ(1.1F, 0.1F, 1.0F));
  StaticEvidenceSnapshot evidence;
  evidence.cell_size_m = 1.0F;
  evidence.map_generation = 9U;
  evidence.authority =
      StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE;
  StaticEvidenceCell approved;
  approved.key = packStaticEvidenceCell(0, 0);
  approved.clean_map_confirmed = true;
  approved.map_generation = 9U;
  evidence.cells.emplace(approved.key, approved);
  const auto selected = selectStaticHeightPointsForAuthority(cloud, evidence);
  ASSERT_EQ(selected.size(), 1U);
  EXPECT_FLOAT_EQ(selected.front().x(), 0.1F);
}

}  // namespace
}  // namespace ndt_slam
