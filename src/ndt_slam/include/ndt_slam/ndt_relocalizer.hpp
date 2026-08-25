#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ndt_slam {

enum class RelocalizationMode : std::uint8_t {
    LOCAL = 0,
    GLOBAL = 1
};

struct RelocalizationConfig {
    bool enabled = true;
    int min_source_points = 800;
    int min_target_points = 1200;
    int max_candidates = 64;
    int max_iterations = 35;
    int num_threads = 2;
    double resolution = 1.0;
    double step_size = 0.20;
    double transformation_epsilon = 0.01;
    double target_crop_radius_m = 18.0;
    double source_voxel_m = 0.30;
    double target_voxel_m = 0.40;
    double max_fitness = 2.0;
    double min_probability = 0.0;
    double max_local_seed_correction_m = 3.0;
    double max_local_seed_yaw_correction_deg = 20.0;
    double max_roll_pitch_deg = 3.0;
    double max_z_correction_m = 0.50;
};

struct RelocalizationSeed {
    Sophus::SE3d pose;
    std::string source;
    int keyframe_id = -1;
    double descriptor_similarity = 0.0;
};

struct RelocalizationJob {
    std::uint64_t frame_index = 0;
    std::uint64_t map_generation = 0;
    std::uint64_t pose_version = 0;
    std::uint64_t map_content_version = 0;
    double stamp_sec = 0.0;
    RelocalizationMode mode = RelocalizationMode::LOCAL;
    Sophus::SE3d reference_pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map;
    std::vector<RelocalizationSeed> seeds;
    int candidate_limit = 0;
    std::string map_source;
};

struct RelocalizationResult {
    bool valid = false;
    std::uint64_t frame_index = 0;
    std::uint64_t map_generation = 0;
    std::uint64_t pose_version = 0;
    std::uint64_t map_content_version = 0;
    double stamp_sec = 0.0;
    RelocalizationMode mode = RelocalizationMode::LOCAL;
    Sophus::SE3d reference_pose;
    Sophus::SE3d pose;
    double fitness = std::numeric_limits<double>::infinity();
    double probability = -std::numeric_limits<double>::infinity();
    double elapsed_ms = 0.0;
    int candidates_tested = 0;
    int keyframe_id = -1;
    std::string seed_source;
    std::string map_source;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr fixed_yaw_source;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr fixed_yaw_target;
    std::string target_crop_identity;
    bool fixed_yaw_verified = false;
    std::uint64_t target_snapshot_id = 0;
    std::uint64_t yaw_authority_generation = 0;
    std::string yaw_reference_hash;
    std::string reason;
};

// Owns a single long-lived worker. Jobs and results are latest-only so a slow
// recovery attempt can never build an unbounded queue behind the live SLAM
// path. The class never mutates the runtime pose or maps.
class NdtRelocalizer {
public:
    NdtRelocalizer() = default;
    ~NdtRelocalizer();

    NdtRelocalizer(const NdtRelocalizer&) = delete;
    NdtRelocalizer& operator=(const NdtRelocalizer&) = delete;

    void configure(const RelocalizationConfig& config);
    void start();
    void stop();
    bool submit(RelocalizationJob job);
    bool takeResult(RelocalizationResult& result);
    bool busy() const { return busy_.load(std::memory_order_acquire); }

private:
    void workerLoop();
    RelocalizationResult run(const RelocalizationJob& job) const;

    RelocalizationConfig config_;
    std::thread worker_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool stopping_ = false;
    bool started_ = false;
    bool has_job_ = false;
    bool has_result_ = false;
    RelocalizationJob pending_job_;
    RelocalizationResult result_;
    std::atomic<bool> busy_{false};
};

}  // namespace ndt_slam
