#include <ndt_slam/loop_closure.hpp>
#include <ndt_slam/sha256.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace {

struct ArchivedFrame {
    std::uint64_t sequence = 0U;
    double stamp_sec = 0.0;
    std::string segment_uuid;
    std::string survey_pass_id;
    Sophus::SE3d pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw{
        new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr registration{
        new pcl::PointCloud<pcl::PointXYZ>};
};

struct CellSupport {
    std::set<std::string> episodes;
    std::set<std::string> survey_passes;
    std::map<std::string, double> last_seen_by_segment;
    std::map<std::string, std::uint64_t> episode_ordinal_by_segment;
    std::size_t points = 0U;
    float minimum_z = std::numeric_limits<float>::infinity();
    float maximum_z = -std::numeric_limits<float>::infinity();
    std::map<int, std::size_t> z_histogram;
    float robust_minimum_z = std::numeric_limits<float>::infinity();
    float robust_maximum_z = -std::numeric_limits<float>::infinity();
};

float histogramQuantile(const std::map<int, std::size_t>& histogram,
                        std::size_t total,
                        double quantile,
                        double bin_size) {
    if (histogram.empty() || total == 0U || !std::isfinite(quantile) ||
        !std::isfinite(bin_size) || bin_size <= 0.0) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const std::size_t rank = static_cast<std::size_t>(std::floor(
        std::clamp(quantile, 0.0, 1.0) *
        static_cast<double>(total - 1U)));
    std::size_t cumulative = 0U;
    for (const auto& item : histogram) {
        cumulative += item.second;
        if (cumulative > rank) {
            return static_cast<float>(
                static_cast<double>(item.first) * bin_size);
        }
    }
    return static_cast<float>(
        static_cast<double>(histogram.rbegin()->first) * bin_size);
}

bool verifySidecar(const fs::path& path) {
    std::ifstream sidecar(path.string() + ".sha256");
    std::string expected;
    sidecar >> expected;
    return !expected.empty() && ndt_slam::sha256File(path.string()) == expected;
}

bool segmentAllowsCertification(const fs::path& pose_path,
                                const std::string& segment_uuid,
                                const std::string& survey_pass_id) {
    try {
        const fs::path segment_dir = pose_path.parent_path().parent_path();
        const fs::path state_path = segment_dir / "state.json";
        if (!verifySidecar(state_path)) return false;
        const YAML::Node state = YAML::LoadFile(state_path.string());
        if (state["segment_uuid"].as<std::string>("") != segment_uuid ||
            state["survey_pass_id"].as<std::string>("") != survey_pass_id) {
            return false;
        }
        const std::string terminal = state["state"].as<std::string>("");
        // FAILED_CLOSED preserves only the healthy, already archived prefix.
        // RUNNING, crash-aborted and archive-incomplete segments never carry
        // certification authority.
        return terminal == "CLOSED" || terminal == "FAILED_CLOSED";
    } catch (const std::exception&) {
        return false;
    }
}

bool loadFrame(const fs::path& pose_path, ArchivedFrame* frame) {
    try {
        const YAML::Node pose = YAML::LoadFile(pose_path.string());
        frame->sequence = pose["sequence"].as<std::uint64_t>();
        frame->stamp_sec = pose["source_stamp"].as<double>();
        frame->segment_uuid = pose["segment_uuid"].as<std::string>();
        frame->survey_pass_id = pose["survey_pass_id"].as<std::string>();
        if (!segmentAllowsCertification(pose_path, frame->segment_uuid,
                                        frame->survey_pass_id)) {
            return false;
        }
        const YAML::Node translation = pose["translation"];
        const YAML::Node quaternion = pose["quaternion_xyzw"];
        if (!translation || translation.size() != 3U ||
            !quaternion || quaternion.size() != 4U) return false;
        Eigen::Quaterniond q(
            quaternion[3].as<double>(), quaternion[0].as<double>(),
            quaternion[1].as<double>(), quaternion[2].as<double>());
        if (!q.coeffs().allFinite() || q.norm() < 1.0e-9) return false;
        q.normalize();
        const Eigen::Vector3d t(
            translation[0].as<double>(), translation[1].as<double>(),
            translation[2].as<double>());
        if (!t.allFinite()) return false;
        frame->pose = Sophus::SE3d(q.toRotationMatrix(), t);
        std::string stem = pose_path.filename().string();
        const std::string suffix = "_accepted_pose.json";
        if (stem.size() <= suffix.size() ||
            stem.substr(stem.size() - suffix.size()) != suffix) return false;
        stem.resize(stem.size() - suffix.size());
        const fs::path raw = pose_path.parent_path() / (stem + "_raw.pcd");
        const fs::path registration = pose_path.parent_path() /
            (stem + "_registration.pcd");
        if (!verifySidecar(pose_path) || !verifySidecar(raw) ||
            !verifySidecar(registration)) return false;
        return pcl::io::loadPCDFile(raw.string(), *frame->raw) == 0 &&
            pcl::io::loadPCDFile(registration.string(),
                                 *frame->registration) == 0 &&
            !frame->raw->empty() && !frame->registration->empty();
    } catch (const std::exception&) {
        return false;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, float leaf) {
    auto output = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.filter(*output);
    return output;
}

bool atomicSavePcd(const fs::path& target,
                   const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    fs::create_directories(target.parent_path());
    const fs::path temporary = target.string() + ".tmp";
    if (pcl::io::savePCDFileBinary(temporary.string(), cloud) != 0) {
        return false;
    }
    std::error_code error;
#ifdef _WIN32
    fs::remove(target, error);
    error.clear();
#endif
    fs::rename(temporary, target, error);
    if (error) return false;
    const std::string hash = ndt_slam::sha256File(target.string());
    if (hash.empty()) return false;
    std::ofstream sidecar(target.string() + ".sha256", std::ios::trunc);
    sidecar << hash << "  " << target.filename().string() << '\n';
    return sidecar.good();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: static_map_rebuilder CAMPAIGN_ROOT CONFIG_YAML "
                     "OUTPUT_DIR\n";
        return 2;
    }
    const fs::path campaign_root(argv[1]);
    const std::string config_path(argv[2]);
    const fs::path output_dir(argv[3]);
    std::vector<ArchivedFrame> frames;
    std::error_code error;
    for (const fs::directory_entry& entry :
         fs::recursive_directory_iterator(campaign_root, error)) {
        if (error) break;
        if (!entry.is_regular_file()) continue;
        const std::string name = entry.path().filename().string();
        if (name.size() < 20U ||
            name.rfind("_accepted_pose.json") != name.size() - 19U) {
            continue;
        }
        ArchivedFrame frame;
        if (!loadFrame(entry.path(), &frame)) {
            std::cerr << "invalid critical archive frame: "
                      << entry.path() << '\n';
            return 3;
        }
        frames.push_back(std::move(frame));
    }
    std::stable_sort(frames.begin(), frames.end(),
        [](const ArchivedFrame& lhs, const ArchivedFrame& rhs) {
            return std::tie(lhs.stamp_sec, lhs.segment_uuid, lhs.sequence) <
                std::tie(rhs.stamp_sec, rhs.segment_uuid, rhs.sequence);
        });
    if (frames.size() < 3U) {
        std::cerr << "at least three archived frames are required\n";
        return 4;
    }

    ndt_slam::LoopClosureDetector detector;
    detector.configureFromYaml(config_path);
    ndt_slam::PoseGraphOptimizer optimizer;
    std::deque<ndt_slam::KeyFrame> keyframes;
    ndt_slam::ScanContext scan_context;
    YAML::Node config = YAML::LoadFile(config_path);
    double episode_gap_sec = 10.0;
    double robust_z_bin_size_m = 0.05;
    double robust_z_lower_quantile = 0.01;
    double robust_z_upper_quantile = 0.99;
    double robust_z_margin_m = 0.05;
    if (config["offline_certification"]) {
        const YAML::Node offline = config["offline_certification"];
        episode_gap_sec = std::max(
            0.1, config["offline_certification"]["episode_gap_sec"]
                     .as<double>(10.0));
        robust_z_bin_size_m = std::clamp(
            offline["robust_z_bin_size_m"].as<double>(0.05), 0.01, 0.50);
        robust_z_lower_quantile = std::clamp(
            offline["robust_z_lower_quantile"].as<double>(0.01),
            0.0, 0.49);
        robust_z_upper_quantile = std::clamp(
            offline["robust_z_upper_quantile"].as<double>(0.99),
            0.51, 1.0);
        robust_z_margin_m = std::clamp(
            offline["robust_z_margin_m"].as<double>(0.05), 0.0, 0.50);
    }
    if (config["scan_context"]) {
        const YAML::Node scan = config["scan_context"];
        scan_context.configure(
            scan["num_rings"].as<int>(20),
            scan["num_sectors"].as<int>(60),
            scan["max_range"].as<double>(80.0));
    }
    Eigen::Matrix<double, 6, 6> odometry_information =
        Eigen::Matrix<double, 6, 6>::Identity() * 100.0;
    Eigen::Matrix<double, 6, 6> loop_information =
        Eigen::Matrix<double, 6, 6>::Identity() * 50.0;
    int loop_edges = 0;
    std::vector<std::vector<std::size_t>> pose_graph_adjacency(
        frames.size());
    for (std::size_t index = 0U; index < frames.size(); ++index) {
        ros::Time frame_stamp;
        frame_stamp.fromSec(frames[index].stamp_sec);
        ndt_slam::KeyFrame keyframe(
            index, frame_stamp, frames[index].pose,
            frames[index].registration);
        keyframe.scan_context_ = scan_context.generate(
            keyframe.cloud_, Eigen::Vector3d::Zero());
        keyframes.push_back(keyframe);
        optimizer.addKeyFrame(keyframe);
        if (index > 0U && frames[index - 1U].segment_uuid ==
                              frames[index].segment_uuid) {
            // Pose continuity ends with the segment. A new operator segment
            // has its own local origin and can be connected only by an
            // independently verified loop edge, never by a fabricated
            // cross-segment odometry constraint.
            optimizer.addOdometryEdge(
                static_cast<int>(index - 1U), static_cast<int>(index),
                frames[index - 1U].pose.inverse() * frames[index].pose,
                odometry_information);
            pose_graph_adjacency[index - 1U].push_back(index);
            pose_graph_adjacency[index].push_back(index - 1U);
        }
        const ndt_slam::LoopCandidate loop = detector.detectLoop(keyframes);
        if (loop.current_keyframe_id >= 0 &&
            loop.candidate_keyframe_id >= 0 &&
            static_cast<std::size_t>(loop.current_keyframe_id) <
                frames.size() &&
            static_cast<std::size_t>(loop.candidate_keyframe_id) <
                frames.size()) {
            optimizer.addLoopEdge(loop.candidate_keyframe_id,
                                  loop.current_keyframe_id,
                                  loop.relative_pose, loop_information);
            const std::size_t current = static_cast<std::size_t>(
                loop.current_keyframe_id);
            const std::size_t candidate = static_cast<std::size_t>(
                loop.candidate_keyframe_id);
            pose_graph_adjacency[current].push_back(candidate);
            pose_graph_adjacency[candidate].push_back(current);
            ++loop_edges;
        }
    }
    // Every operator segment starts a new localization continuity and has no
    // fabricated cross-segment odometry edge.  Refuse to optimize/reproject
    // unless independently verified loop edges connect every archived frame
    // to the single anchored graph component.  Otherwise g2o can return a
    // numerically valid but gauge-free subgraph that has no common-map
    // authority.
    std::vector<bool> connected(frames.size(), false);
    std::queue<std::size_t> pending;
    connected.front() = true;
    pending.push(0U);
    std::size_t connected_count = 0U;
    while (!pending.empty()) {
        const std::size_t current = pending.front();
        pending.pop();
        ++connected_count;
        for (const std::size_t neighbor : pose_graph_adjacency[current]) {
            if (!connected[neighbor]) {
                connected[neighbor] = true;
                pending.push(neighbor);
            }
        }
    }
    if (connected_count != frames.size()) {
        std::cerr << "pose graph is disconnected: connected="
                  << connected_count << " total=" << frames.size()
                  << "; independent segment loop authority is required\n";
        return 5;
    }
    if (!optimizer.optimize(30)) {
        std::cerr << "g2o optimization failed\n";
        return 5;
    }

    auto raw_reprojected = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    auto registration_reprojected = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::map<std::pair<int, int>, CellSupport> support;
    constexpr double cell_size = 0.20;
    for (std::size_t index = 0U; index < frames.size(); ++index) {
        const Sophus::SE3d optimized = optimizer.getOptimizedPose(
            static_cast<int>(index));
        pcl::PointCloud<pcl::PointXYZ> transformed_raw;
        pcl::PointCloud<pcl::PointXYZ> transformed_registration;
        pcl::transformPointCloud(*frames[index].raw, transformed_raw,
                                 optimized.matrix().cast<float>());
        pcl::transformPointCloud(*frames[index].registration,
                                 transformed_registration,
                                 optimized.matrix().cast<float>());
        *raw_reprojected += transformed_raw;
        *registration_reprojected += transformed_registration;
        std::set<std::pair<int, int>> frame_cells;
        for (const pcl::PointXYZ& point : transformed_registration.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) continue;
            const std::pair<int, int> cell{
                static_cast<int>(std::floor(point.x / cell_size)),
                static_cast<int>(std::floor(point.y / cell_size))};
            CellSupport& value = support[cell];
            ++value.points;
            value.minimum_z = std::min(value.minimum_z, point.z);
            value.maximum_z = std::max(value.maximum_z, point.z);
            const double z_bin = std::floor(
                static_cast<double>(point.z) / robust_z_bin_size_m);
            if (z_bin >= static_cast<double>(
                    std::numeric_limits<int>::lowest()) &&
                z_bin <= static_cast<double>(
                    std::numeric_limits<int>::max())) {
                ++value.z_histogram[static_cast<int>(z_bin)];
            }
            frame_cells.insert(cell);
        }
        for (const auto& cell : frame_cells) {
            CellSupport& value = support[cell];
            const std::string& segment = frames[index].segment_uuid;
            const auto previous = value.last_seen_by_segment.find(segment);
            if (previous == value.last_seen_by_segment.end() ||
                frames[index].stamp_sec - previous->second > episode_gap_sec) {
                const std::uint64_t ordinal =
                    ++value.episode_ordinal_by_segment[segment];
                value.episodes.insert(
                    segment + "#" + std::to_string(ordinal));
            }
            value.last_seen_by_segment[segment] = frames[index].stamp_sec;
            value.survey_passes.insert(frames[index].survey_pass_id);
        }
    }

    auto supported_raw = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    auto supported_registration = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& item : support) {
        CellSupport& value = item.second;
        const float lower_bin = histogramQuantile(
            value.z_histogram, value.points, robust_z_lower_quantile,
            robust_z_bin_size_m);
        const float upper_bin = histogramQuantile(
            value.z_histogram, value.points, robust_z_upper_quantile,
            robust_z_bin_size_m);
        value.robust_minimum_z = std::isfinite(lower_bin)
            ? lower_bin - static_cast<float>(robust_z_margin_m)
            : value.minimum_z;
        value.robust_maximum_z = std::isfinite(upper_bin)
            ? upper_bin + static_cast<float>(robust_z_bin_size_m +
                                             robust_z_margin_m)
            : value.maximum_z;
    }
    const auto keep_supported = [&support, cell_size](
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
        output->reserve(input->size());
        for (const pcl::PointXYZ& point : input->points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) continue;
            const std::pair<int, int> cell{
                static_cast<int>(std::floor(point.x / cell_size)),
                static_cast<int>(std::floor(point.y / cell_size))};
            const auto found = support.find(cell);
            if (found != support.end() &&
                found->second.episodes.size() >= 3U &&
                found->second.survey_passes.size() >= 2U &&
                point.z >= found->second.robust_minimum_z &&
                point.z <= found->second.robust_maximum_z) {
                output->push_back(point);
            }
        }
    };
    keep_supported(raw_reprojected, supported_raw);
    keep_supported(registration_reprojected, supported_registration);
    if (supported_raw->empty() || supported_registration->empty()) {
        std::cerr << "no cells reached 3 episodes and 2 survey passes\n";
        return 6;
    }
    const auto localization = voxelized(supported_raw, 0.30F);
    const auto avoidance = voxelized(supported_registration, 0.10F);
    const fs::path localization_path =
        output_dir / "candidate_localization_reference.pcd";
    const fs::path avoidance_path =
        output_dir / "candidate_avoidance_static_baseline.pcd";
    if (!atomicSavePcd(localization_path, *localization) ||
        !atomicSavePcd(avoidance_path, *avoidance)) {
        std::cerr << "candidate PCD write failed\n";
        return 7;
    }
    fs::create_directories(output_dir);
    const fs::path evidence_path =
        output_dir / "static_evidence_candidate.csv";
    {
        std::ofstream evidence(evidence_path);
        evidence << "cell_x,cell_y,point_count,episode_count,pass_count,min_z,max_z\n";
        for (const auto& item : support) {
            evidence << item.first.first << ',' << item.first.second << ','
                     << item.second.points << ','
                     << item.second.episodes.size() << ','
                     << item.second.survey_passes.size() << ','
                     << item.second.robust_minimum_z << ','
                     << item.second.robust_maximum_z
                     << '\n';
        }
        evidence.flush();
        if (!evidence.good()) return 8;
    }
    const std::string evidence_hash =
        ndt_slam::sha256File(evidence_path.string());
    if (evidence_hash.empty()) return 8;
    const std::string localization_hash =
        ndt_slam::sha256File(localization_path.string());
    const std::string avoidance_hash =
        ndt_slam::sha256File(avoidance_path.string());
    std::ofstream manifest(output_dir / "candidate_manifest.yaml");
    manifest << "schema_version: 1\n"
             << "authority: CANDIDATE_NOT_CERTIFIED\n"
             << "frame_id: map\n"
             << "source: immutable_raw_segments\n"
             << "config_sha256: " << ndt_slam::sha256File(config_path) << "\n"
             << "frame_count: " << frames.size() << "\n"
             << "loop_edges: " << loop_edges << "\n"
             << "optimizer: scan_context_icp_g2o\n"
             << "robust_z_bin_size_m: " << robust_z_bin_size_m << "\n"
             << "robust_z_lower_quantile: "
             << robust_z_lower_quantile << "\n"
             << "robust_z_upper_quantile: "
             << robust_z_upper_quantile << "\n"
             << "robust_z_margin_m: " << robust_z_margin_m << "\n"
             << "static_evidence_candidate_sha256: " << evidence_hash << "\n"
             << "localization_reference_sha256: " << localization_hash << "\n"
             << "avoidance_static_baseline_sha256: " << avoidance_hash << "\n";
    manifest.flush();
    if (!manifest.good()) return 8;
    manifest.close();
    const fs::path manifest_path = output_dir / "candidate_manifest.yaml";
    const std::string manifest_hash =
        ndt_slam::sha256File(manifest_path.string());
    if (manifest_hash.empty()) return 8;
    std::ofstream manifest_sidecar(
        manifest_path.string() + ".sha256", std::ios::trunc);
    manifest_sidecar << manifest_hash << "  "
                     << manifest_path.filename().string() << '\n';
    manifest_sidecar.flush();
    if (!manifest_sidecar.good()) return 8;
    std::cout << "candidate rebuilt; certification is still required\n";
    return 0;
}
