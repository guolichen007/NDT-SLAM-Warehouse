#include "ndt_slam/crane_place_descriptor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

double descriptorSimilarity(const std::vector<float>& query,
                            const std::vector<float>& stored, int rings,
                            int sectors, int shift) {
  double dot = 0.0;
  double query_norm = 0.0;
  double stored_norm = 0.0;
  for (int ring = 0; ring < rings; ++ring) {
    for (int sector = 0; sector < sectors; ++sector) {
      const int shifted = (sector + shift) % sectors;
      const float lhs = query[ring * sectors + sector];
      const float rhs = stored[ring * sectors + shifted];
      dot += static_cast<double>(lhs) * rhs;
      query_norm += static_cast<double>(lhs) * lhs;
      stored_norm += static_cast<double>(rhs) * rhs;
    }
  }
  if (query_norm <= 1.0e-12 || stored_norm <= 1.0e-12) return 0.0;
  return dot / std::sqrt(query_norm * stored_norm);
}

}  // namespace

CranePlaceDescriptor::CranePlaceDescriptor(
    const CranePlaceDescriptorConfig& config) {
  configure(config);
}

void CranePlaceDescriptor::configure(
    const CranePlaceDescriptorConfig& config) {
  config_ = config;
  config_.rings = std::clamp(config_.rings, 4, 80);
  config_.sectors = std::clamp(config_.sectors, 12, 180);
  config_.maximum_radius_m = std::max(1.0, config_.maximum_radius_m);
  config_.minimum_similarity =
      std::clamp(config_.minimum_similarity, -1.0, 1.0);
  entries_.clear();
}

std::vector<float> CranePlaceDescriptor::describe(
    const pcl::PointCloud<pcl::PointXYZ>& cloud) const {
  std::vector<float> descriptor(
      static_cast<std::size_t>(config_.rings * config_.sectors), 0.0F);
  for (const auto& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const double radius = std::hypot(point.x, point.y);
    if (radius <= 0.0 || radius > config_.maximum_radius_m) continue;
    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) angle += 2.0 * M_PI;
    const int ring = std::min(config_.rings - 1, static_cast<int>(
        radius / config_.maximum_radius_m * config_.rings));
    const int sector = std::min(config_.sectors - 1, static_cast<int>(
        angle / (2.0 * M_PI) * config_.sectors));
    float& bin = descriptor[ring * config_.sectors + sector];
    bin = std::max(bin, point.z);
  }
  return descriptor;
}

bool CranePlaceDescriptor::addPlace(
    std::uint64_t id, const Sophus::SE3d& pose,
    const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  if (id == 0U || cloud.empty() || !pose.matrix().allFinite()) return false;
  auto descriptor = describe(cloud);
  if (std::none_of(descriptor.begin(), descriptor.end(),
                   [](float value) { return value != 0.0F; })) {
    return false;
  }
  entries_.push_back({id, pose, std::move(descriptor)});
  return true;
}

std::vector<CranePlaceCandidate> CranePlaceDescriptor::query(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    std::size_t maximum_candidates) const {
  std::vector<CranePlaceCandidate> candidates;
  if (maximum_candidates == 0U || cloud.empty()) return candidates;
  const auto query_descriptor = describe(cloud);
  for (const auto& entry : entries_) {
    double best_similarity = -std::numeric_limits<double>::infinity();
    int best_shift = 0;
    for (int shift = 0; shift < config_.sectors; ++shift) {
      const double similarity = descriptorSimilarity(
          query_descriptor, entry.descriptor, config_.rings,
          config_.sectors, shift);
      if (similarity > best_similarity) {
        best_similarity = similarity;
        best_shift = shift;
      }
    }
    if (best_similarity < config_.minimum_similarity) continue;
    const double yaw_offset = -2.0 * M_PI * best_shift / config_.sectors;
    const Eigen::Matrix3d rotation = entry.pose.so3().matrix() *
        Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    candidates.push_back({entry.id,
                          Sophus::SE3d(rotation, entry.pose.translation()),
                          best_similarity, yaw_offset});
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const auto& lhs, const auto& rhs) {
              if (lhs.similarity != rhs.similarity)
                return lhs.similarity > rhs.similarity;
              return lhs.id < rhs.id;
            });
  if (candidates.size() > maximum_candidates)
    candidates.resize(maximum_candidates);
  return candidates;
}

void CranePlaceDescriptor::clear() { entries_.clear(); }

}  // namespace ndt_slam
