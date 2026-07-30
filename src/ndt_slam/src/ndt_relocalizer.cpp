#include "ndt_slam/ndt_relocalizer.hpp"
#include "ndt_slam/rigid_transform_conversion.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace ndt_slam {
namespace {

double wrapAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

double yawOf(const Sophus::SE3d& pose) {
    const Eigen::Matrix3d r = pose.so3().matrix();
    return std::atan2(r(1, 0), r(0, 0));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, double leaf) {
    auto output = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (!input || input->empty()) return output;
    if (leaf <= 0.0) {
        *output = *input;
        return output;
    }
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                       static_cast<float>(leaf));
    filter.filter(*output);
    return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cropAround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& map,
    const Eigen::Vector3d& center, double radius) {
    auto output = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (!map || map->empty()) return output;
    const double radius_sq = radius * radius;
    output->reserve(map->size());
    for (const auto& point : map->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) continue;
        const double dx = point.x - center.x();
        const double dy = point.y - center.y();
        if (dx * dx + dy * dy <= radius_sq) output->push_back(point);
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = false;
    return output;
}

}  // namespace

NdtRelocalizer::~NdtRelocalizer() { stop(); }

void NdtRelocalizer::configure(const RelocalizationConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    config_.min_source_points = std::max(100, config_.min_source_points);
    config_.min_target_points = std::max(300, config_.min_target_points);
    config_.max_candidates = std::clamp(config_.max_candidates, 1, 64);
    config_.max_iterations = std::clamp(config_.max_iterations, 5, 100);
    config_.num_threads = std::clamp(config_.num_threads, 1, 4);
    config_.resolution = std::max(0.10, config_.resolution);
    config_.step_size = std::max(0.01, config_.step_size);
    config_.transformation_epsilon =
        std::max(1.0e-5, config_.transformation_epsilon);
    config_.target_crop_radius_m = std::max(5.0, config_.target_crop_radius_m);
    config_.source_voxel_m = std::max(0.05, config_.source_voxel_m);
    config_.target_voxel_m = std::max(0.05, config_.target_voxel_m);
    config_.max_fitness = std::max(0.0, config_.max_fitness);
    config_.max_local_seed_correction_m =
        std::max(0.10, config_.max_local_seed_correction_m);
    config_.max_local_seed_yaw_correction_deg =
        std::max(1.0, config_.max_local_seed_yaw_correction_deg);
    config_.max_roll_pitch_deg = std::max(0.10, config_.max_roll_pitch_deg);
    config_.max_z_correction_m = std::max(0.05, config_.max_z_correction_m);
}

void NdtRelocalizer::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_) return;
    stopping_ = false;
    started_ = true;
    worker_ = std::thread(&NdtRelocalizer::workerLoop, this);
}

void NdtRelocalizer::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!started_) return;
        stopping_ = true;
        has_job_ = false;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    started_ = false;
    busy_.store(false, std::memory_order_release);
}

bool NdtRelocalizer::submit(RelocalizationJob job) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_ || stopping_ || !config_.enabled || !job.source ||
        !job.map || job.seeds.empty()) return false;
    // Replace a queued-but-not-started job. Never replace the job currently
    // owned by the worker.
    pending_job_ = std::move(job);
    has_job_ = true;
    cv_.notify_one();
    return true;
}

bool NdtRelocalizer::takeResult(RelocalizationResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_result_) return false;
    result = std::move(result_);
    has_result_ = false;
    return true;
}

void NdtRelocalizer::workerLoop() {
    while (true) {
        RelocalizationJob job;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return stopping_ || has_job_; });
            if (stopping_) break;
            job = std::move(pending_job_);
            has_job_ = false;
            busy_.store(true, std::memory_order_release);
        }

        RelocalizationResult result = run(job);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // A newer result always supersedes an unconsumed older result.
            result_ = std::move(result);
            has_result_ = true;
            busy_.store(false, std::memory_order_release);
        }
    }
}

RelocalizationResult NdtRelocalizer::run(const RelocalizationJob& job) const {
    const auto started = std::chrono::steady_clock::now();
    RelocalizationResult best;
    best.frame_index = job.frame_index;
    best.map_generation = job.map_generation;
    best.pose_version = job.pose_version;
    best.stamp_sec = job.stamp_sec;
    best.mode = job.mode;
    best.reference_pose = job.reference_pose;
    best.map_source = job.map_source;

    RelocalizationConfig config;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        config = config_;
    }

    auto source = voxelized(job.source, config.source_voxel_m);
    if (static_cast<int>(source->size()) < config.min_source_points) {
        best.reason = "source_below_min_points";
        return best;
    }

    const int requested_candidates =
        job.candidate_limit > 0 ? job.candidate_limit : config.max_candidates;
    const int candidate_count = std::min<int>(
        std::min(config.max_candidates, requested_candidates),
        static_cast<int>(job.seeds.size()));
    for (int index = 0; index < candidate_count; ++index) {
        const RelocalizationSeed& seed = job.seeds[index];
        auto cropped = cropAround(job.map, seed.pose.translation(),
                                  config.target_crop_radius_m);
        auto target = voxelized(cropped, config.target_voxel_m);
        if (static_cast<int>(target->size()) < config.min_target_points) continue;

        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        ndt.setResolution(config.resolution);
        ndt.setStepSize(config.step_size);
        ndt.setTransformationEpsilon(config.transformation_epsilon);
        ndt.setMaximumIterations(config.max_iterations);
        ndt.setNumThreads(config.num_threads);
        ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ndt.setInputSource(source);
        ndt.setInputTarget(target);

        pcl::PointCloud<pcl::PointXYZ> aligned;
        ndt.align(aligned, seed.pose.matrix().cast<float>());
        ++best.candidates_tested;
        if (!ndt.hasConverged()) continue;

        const Eigen::Matrix4f transform = ndt.getFinalTransformation();
        const SafeSE3Result converted =
            makeSafeSE3FromMatrix(transform.cast<double>());
        if (!converted.valid) continue;
        const Sophus::SE3d candidate = converted.pose;
        const double fitness = ndt.getFitnessScore();
        const double probability = ndt.getTransformationProbability();
        if (!std::isfinite(fitness) || fitness > config.max_fitness ||
            !std::isfinite(probability) || probability < config.min_probability) {
            continue;
        }
        const Eigen::Matrix3d candidate_rotation = candidate.so3().matrix();
        const double roll = std::atan2(candidate_rotation(2, 1),
                                       candidate_rotation(2, 2));
        const double pitch = std::atan2(
            -candidate_rotation(2, 0),
            std::hypot(candidate_rotation(2, 1), candidate_rotation(2, 2)));
        if (std::abs(roll) * 180.0 / M_PI > config.max_roll_pitch_deg ||
            std::abs(pitch) * 180.0 / M_PI > config.max_roll_pitch_deg ||
            std::abs(candidate.translation().z() - seed.pose.translation().z()) >
                config.max_z_correction_m) {
            continue;
        }

        if (job.mode == RelocalizationMode::LOCAL) {
            const double translation =
                (candidate.translation().head<2>() -
                 seed.pose.translation().head<2>()).norm();
            const double yaw_delta_deg = std::abs(wrapAngle(
                yawOf(candidate) - yawOf(seed.pose))) * 180.0 / M_PI;
            if (translation > config.max_local_seed_correction_m ||
                yaw_delta_deg > config.max_local_seed_yaw_correction_deg) continue;
        }

        if (!best.valid || fitness < best.fitness) {
            best.valid = true;
            best.pose = candidate;
            best.fitness = fitness;
            best.probability = probability;
            best.keyframe_id = seed.keyframe_id;
            best.seed_source = seed.source;
            best.reason = "accepted_candidate";
        }
    }

    if (!best.valid) best.reason = "no_candidate_passed_gates";
    best.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    return best;
}

}  // namespace ndt_slam
