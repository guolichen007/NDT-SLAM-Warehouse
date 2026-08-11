#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ndt_slam {

struct CranePlaceDescriptorConfig {
  int rings = 20;
  int sectors = 60;
  double maximum_radius_m = 80.0;
  double minimum_similarity = 0.15;
};

struct CranePlaceEntry {
  std::uint64_t id = 0U;
  Sophus::SE3d pose;
  std::vector<float> descriptor;
};

struct CranePlaceCandidate {
  std::uint64_t id = 0U;
  Sophus::SE3d prior_pose;
  double similarity = 0.0;
  double yaw_offset_rad = 0.0;
};

// Independently implemented crane descriptor. It is retrieval-only: no API in
// this class can mutate runtime pose or create AcceptedPose authority.
class CranePlaceDescriptor {
 public:
  explicit CranePlaceDescriptor(
      const CranePlaceDescriptorConfig& config = {});

  void configure(const CranePlaceDescriptorConfig& config);
  bool addPlace(std::uint64_t id, const Sophus::SE3d& pose,
                const pcl::PointCloud<pcl::PointXYZ>& cloud);
  std::vector<CranePlaceCandidate> query(
      const pcl::PointCloud<pcl::PointXYZ>& cloud,
      std::size_t maximum_candidates) const;
  void clear();
  std::size_t size() const { return entries_.size(); }

 private:
  std::vector<float> describe(
      const pcl::PointCloud<pcl::PointXYZ>& cloud) const;

  CranePlaceDescriptorConfig config_;
  std::vector<CranePlaceEntry> entries_;
};

}  // namespace ndt_slam
