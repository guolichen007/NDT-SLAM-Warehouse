#include "ndt_slam/durable_map_store.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace ndt_slam {
namespace {

namespace fs = std::filesystem;

class DurableMapStoreTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
        ("ndt-durable-map-test-" + MapSessionSnapshot::generateUuid());
  }

  void TearDown() override {
    std::error_code ignored;
    fs::remove_all(root_, ignored);
  }

  MapSessionSaveRequest request(std::uint64_t generation) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->push_back({1.0F, 2.0F, 3.0F});
    clouds_.push_back(cloud);
    auto evidence = std::make_shared<StaticEvidenceSnapshot>();
    evidence->map_generation = generation;
    evidence->authority = StaticEvidenceAuthority::RUNTIME_MATURE;
    evidence_.push_back(evidence);
    MapSessionSaveRequest value;
    value.layers = {cloud, cloud, cloud, cloud, cloud};
    value.metadata.map_uuid = "11111111-2222-4333-8444-555555555555";
    value.metadata.map_generation = generation;
    value.static_evidence = evidence;
    return value;
  }

  fs::path root_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
  std::vector<std::shared_ptr<StaticEvidenceSnapshot>> evidence_;
};

TEST_F(DurableMapStoreTest, LoadsCurrentThenFallsBackToPrevious) {
  DurableMapStore store(root_.string());
  std::string reason;
  ASSERT_TRUE(store.save(request(1U), &reason)) << reason;
  ASSERT_TRUE(store.save(request(2U), &reason)) << reason;
  auto loaded = store.loadBest();
  ASSERT_EQ(loaded.status, DurableMapLoadStatus::LOADED) << loaded.reason;
  EXPECT_EQ(loaded.pointer, "CURRENT");
  EXPECT_EQ(loaded.session.metadata.map_generation, 2U);

  std::ofstream corrupt(fs::path(loaded.session.session_directory) /
                        "map_registration.pcd", std::ios::app);
  corrupt << "corrupt";
  corrupt.close();
  loaded = store.loadBest();
  ASSERT_EQ(loaded.status, DurableMapLoadStatus::LOADED) << loaded.reason;
  EXPECT_EQ(loaded.pointer, "PREVIOUS");
  EXPECT_EQ(loaded.session.metadata.map_generation, 1U);
}

TEST_F(DurableMapStoreTest, FailedSaveDoesNotChangeCurrent) {
  DurableMapStore store(root_.string());
  std::string reason;
  ASSERT_TRUE(store.save(request(1U), &reason)) << reason;
  auto failing = request(2U);
  failing.write_extras = [](const std::string&, std::string* failure) {
    if (failure) *failure = "injected";
    return false;
  };
  EXPECT_FALSE(store.save(std::move(failing), &reason));
  const auto loaded = store.loadBest();
  ASSERT_EQ(loaded.status, DurableMapLoadStatus::LOADED);
  EXPECT_EQ(loaded.session.metadata.map_generation, 1U);
}

TEST_F(DurableMapStoreTest, IgnoresPartialStagingAndDistinguishesFirstBoot) {
  DurableMapStore store(root_.string());
  std::string reason;
  ASSERT_TRUE(store.initialize(&reason)) << reason;
  fs::create_directories(root_ / "staging" / "gen_000000000001.partial");
  EXPECT_EQ(store.loadBest().status, DurableMapLoadStatus::FIRST_BOOT);
  std::ofstream pointer(root_ / "CURRENT");
  pointer << "gen_000000000001\n";
  pointer.close();
  EXPECT_EQ(store.loadBest().status,
            DurableMapLoadStatus::REFERENCE_CORRUPTED);
}

TEST_F(DurableMapStoreTest, ExistingEmptyCurrentIsCorruptionNotFirstBoot) {
  DurableMapStore store(root_.string());
  std::string reason;
  ASSERT_TRUE(store.initialize(&reason)) << reason;
  std::ofstream(root_ / "CURRENT").close();
  const auto loaded = store.loadBest();
  EXPECT_EQ(loaded.status, DurableMapLoadStatus::REFERENCE_CORRUPTED);
  EXPECT_NE(loaded.reason.find("pointer_empty_or_unreadable"),
            std::string::npos);
}

}  // namespace
}  // namespace ndt_slam
