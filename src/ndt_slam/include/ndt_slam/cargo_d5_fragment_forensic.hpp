#pragma once

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ndt_slam {

// Diagnostic-only D5 fragment lineage. This module is a pure offline
// observer: it never mutates product decision state and is compiled out of
// every product path unless the diagnostic flag is enabled. Its only purpose
// is to answer WHY a cargo top/edge support ends 0.7~1.3 m away from the
// primary body in 3D Euclidean clustering, so the downstream product repair
// (owned by Codex) can pick a safe evidence channel.

enum class D5VoxelAssignment : std::uint8_t {
  PRIMARY_ACCEPTED = 0,
  WEAK_REJECTED = 1,   // part of a <min_cluster_size connected fragment
  UNASSIGNED = 2,      // isolated single voxel (no fragment neighbor)
};

struct D5FragmentForensicConfig {
  float cluster_tolerance_m = 0.20F;   // mirror of component_cluster_tolerance_m
  int min_cluster_size = 10;           // mirror of weak_min_points
  float voxel_leaf_size_m = 0.05F;     // mirror of the detector VoxelGrid leaf
};

struct D5VoxelLineage {
  double stamp_sec = 0.0;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  std::int32_t voxel_ix = 0;
  std::int32_t voxel_iy = 0;
  std::int32_t voxel_iz = 0;
  std::int32_t cluster_label = -1;   // -1 unassigned, >=0 primary cluster id
  std::int32_t cluster_size = 0;     // primary cluster size, 0 when unassigned
  D5VoxelAssignment assignment = D5VoxelAssignment::UNASSIGNED;
  std::int32_t fragment_id = -1;     // weak fragment id, -1 for primary voxels
};

struct D5WeakFragment {
  std::int32_t fragment_id = -1;
  std::int32_t point_count = 0;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f min_xyz = Eigen::Vector3f::Zero();
  Eigen::Vector3f max_xyz = Eigen::Vector3f::Zero();
  float z_p05 = 0.0F;
  float z_p50 = 0.0F;
  float z_p95 = 0.0F;
  std::int32_t nearest_primary_id = -1;
  float nearest_primary_3d_distance = 0.0F;
  float nearest_primary_xy_distance = 0.0F;
  float vertical_separation = 0.0F;         // |frag z center - primary z center|
  float xy_projected_overlap = 0.0F;        // overlap area / frag xy area
  float xy_projected_gap = 0.0F;            // 0 when overlapping, else min gap
  std::int32_t candidate_primary_neighbor_count = 0;
  std::int32_t oracle_high_surface = 0;     // bag oracle only: any point z>=1.3
};

struct D5FragmentForensicResult {
  std::vector<D5VoxelLineage> voxel_lineage;
  std::vector<D5WeakFragment> weak_fragments;
};

// Pure, diagnostic-only. Inputs are the same voxel cloud and primary clusters
// the product EuclideanClusterExtraction just produced. Returns per-voxel
// lineage plus the weak (<min_cluster_size) fragments it discarded.
D5FragmentForensicResult analyzeD5FragmentForensic(
    const pcl::PointCloud<pcl::PointXYZ>& voxel_cloud,
    const std::vector<pcl::PointIndices>& primary_clusters,
    double stamp_sec,
    const D5FragmentForensicConfig& config);

}  // namespace ndt_slam
