#include "ndt_slam/cargo_d5_fragment_forensic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

namespace ndt_slam {
namespace {

float quantile(std::vector<float> values, double q) {
  if (values.empty()) return 0.0F;
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
      std::min<std::size_t>(values.size() - 1U,
                            static_cast<std::size_t>(q * (values.size() - 1U))));
  return values[index];
}

float median(std::vector<float> values) {
  if (values.empty()) return 0.0F;
  std::sort(values.begin(), values.end());
  return values[values.size() / 2U];
}

}  // namespace

D5FragmentForensicResult analyzeD5FragmentForensic(
    const pcl::PointCloud<pcl::PointXYZ>& voxel_cloud,
    const std::vector<pcl::PointIndices>& primary_clusters,
    double stamp_sec,
    const D5FragmentForensicConfig& config) {
  D5FragmentForensicResult result;
  const std::size_t point_count = voxel_cloud.size();
  result.voxel_lineage.reserve(point_count);

  std::vector<std::int32_t> cluster_label(point_count, -1);
  std::vector<std::int32_t> cluster_size(point_count, 0);
  for (std::size_t cluster_id = 0U;
       cluster_id < primary_clusters.size(); ++cluster_id) {
    const pcl::PointIndices& indices = primary_clusters[cluster_id];
    const std::int32_t size =
        static_cast<std::int32_t>(indices.indices.size());
    for (int point_index : indices.indices) {
      if (point_index < 0 ||
          static_cast<std::size_t>(point_index) >= point_count) {
        continue;
      }
      cluster_label[point_index] = static_cast<std::int32_t>(cluster_id);
      cluster_size[point_index] = size;
    }
  }

  const float voxel_leaf = config.voxel_leaf_size_m > 0.0F
      ? config.voxel_leaf_size_m : 0.05F;
  const float tolerance = config.cluster_tolerance_m;

  // Collect unassigned voxels and weak-cluster them with the same local
  // connectivity semantics, but without the product min_cluster_size cutoff.
  std::vector<std::size_t> unassigned;
  unassigned.reserve(point_count);
  for (std::size_t index = 0U; index < point_count; ++index) {
    if (cluster_label[index] < 0) unassigned.push_back(index);
  }

  std::vector<std::size_t> parent(unassigned.size());
  std::iota(parent.begin(), parent.end(), 0U);
  const auto root = [&](std::size_t index) {
    std::size_t current = index;
    while (parent[current] != current) {
      parent[current] = parent[parent[current]];
      current = parent[current];
    }
    return current;
  };
  const auto join = [&](std::size_t lhs, std::size_t rhs) {
    lhs = root(lhs);
    rhs = root(rhs);
    if (lhs != rhs) parent[rhs] = lhs;
  };
  for (std::size_t i = 0U; i < unassigned.size(); ++i) {
    const pcl::PointXYZ& a = voxel_cloud.points[unassigned[i]];
    for (std::size_t j = i + 1U; j < unassigned.size(); ++j) {
      const pcl::PointXYZ& b = voxel_cloud.points[unassigned[j]];
      const float dx = a.x - b.x;
      const float dy = a.y - b.y;
      const float dz = a.z - b.z;
      if (dx * dx + dy * dy + dz * dz <= tolerance * tolerance) {
        join(i, j);
      }
    }
  }

  // Map each unassigned root to a fragment id; single-voxel roots stay
  // UNASSIGNED, multi-voxel roots become WEAK_REJECTED.
  std::vector<std::int32_t> unassigned_fragment_id(unassigned.size(), -1);
  std::vector<std::vector<std::size_t>> fragment_members;
  std::vector<std::int32_t> root_to_fragment(unassigned.size(), -1);
  for (std::size_t i = 0U; i < unassigned.size(); ++i) {
    const std::size_t r = root(i);
    if (root_to_fragment[r] < 0) {
      root_to_fragment[r] = static_cast<std::int32_t>(fragment_members.size());
      fragment_members.emplace_back();
    }
    const std::int32_t fragment_id = root_to_fragment[r];
    unassigned_fragment_id[i] = fragment_id;
    fragment_members[fragment_id].push_back(unassigned[i]);
  }

  // Build lineage for every voxel.
  std::vector<std::int32_t> primary_fragment_id(unassigned.size(), -1);
  for (std::size_t i = 0U; i < unassigned.size(); ++i) {
    const std::size_t global_index = unassigned[i];
    const pcl::PointXYZ& point = voxel_cloud.points[global_index];
    const std::int32_t fragment_id = unassigned_fragment_id[i];
    const bool multi_point = fragment_members[fragment_id].size() > 1U;

    D5VoxelLineage lineage;
    lineage.stamp_sec = stamp_sec;
    lineage.x = point.x;
    lineage.y = point.y;
    lineage.z = point.z;
    lineage.voxel_ix = static_cast<std::int32_t>(
        std::floor(point.x / voxel_leaf));
    lineage.voxel_iy = static_cast<std::int32_t>(
        std::floor(point.y / voxel_leaf));
    lineage.voxel_iz = static_cast<std::int32_t>(
        std::floor(point.z / voxel_leaf));
    lineage.cluster_label = -1;
    lineage.cluster_size = 0;
    lineage.assignment = multi_point ? D5VoxelAssignment::WEAK_REJECTED
                                     : D5VoxelAssignment::UNASSIGNED;
    lineage.fragment_id = multi_point ? fragment_id : -1;
    result.voxel_lineage.push_back(std::move(lineage));
  }
  for (std::size_t index = 0U; index < point_count; ++index) {
    if (cluster_label[index] < 0) continue;
    const pcl::PointXYZ& point = voxel_cloud.points[index];
    D5VoxelLineage lineage;
    lineage.stamp_sec = stamp_sec;
    lineage.x = point.x;
    lineage.y = point.y;
    lineage.z = point.z;
    lineage.voxel_ix = static_cast<std::int32_t>(
        std::floor(point.x / voxel_leaf));
    lineage.voxel_iy = static_cast<std::int32_t>(
        std::floor(point.y / voxel_leaf));
    lineage.voxel_iz = static_cast<std::int32_t>(
        std::floor(point.z / voxel_leaf));
    lineage.cluster_label = cluster_label[index];
    lineage.cluster_size = cluster_size[index];
    lineage.assignment = D5VoxelAssignment::PRIMARY_ACCEPTED;
    lineage.fragment_id = -1;
    result.voxel_lineage.push_back(std::move(lineage));
  }

  // Precompute primary cluster XY bboxes and z centers for fragment relations.
  struct PrimaryGeometry {
    Eigen::Vector2f xy_min = Eigen::Vector2f::Constant(
        std::numeric_limits<float>::infinity());
    Eigen::Vector2f xy_max = Eigen::Vector2f::Constant(
        -std::numeric_limits<float>::infinity());
    std::vector<float> zs;
    std::vector<std::size_t> member_indices;
  };
  std::vector<PrimaryGeometry> primary_geometry(primary_clusters.size());
  for (std::size_t cluster_id = 0U;
       cluster_id < primary_clusters.size(); ++cluster_id) {
    PrimaryGeometry& geometry = primary_geometry[cluster_id];
    for (int point_index : primary_clusters[cluster_id].indices) {
      if (point_index < 0 ||
          static_cast<std::size_t>(point_index) >= point_count) {
        continue;
      }
      const pcl::PointXYZ& point = voxel_cloud.points[point_index];
      geometry.xy_min = geometry.xy_min.cwiseMin(
          Eigen::Vector2f(point.x, point.y));
      geometry.xy_max = geometry.xy_max.cwiseMax(
          Eigen::Vector2f(point.x, point.y));
      geometry.zs.push_back(point.z);
      geometry.member_indices.push_back(
          static_cast<std::size_t>(point_index));
    }
  }

  for (std::size_t fragment_index = 0U;
       fragment_index < fragment_members.size(); ++fragment_index) {
    const std::vector<std::size_t>& members = fragment_members[fragment_index];
    if (members.size() <= 1U) continue;  // isolated voxel, not a fragment

    D5WeakFragment fragment;
    fragment.fragment_id = static_cast<std::int32_t>(fragment_index);
    fragment.point_count = static_cast<std::int32_t>(members.size());

    std::vector<float> xs, ys, zs;
    xs.reserve(members.size());
    ys.reserve(members.size());
    zs.reserve(members.size());
    Eigen::Vector3f minimum = Eigen::Vector3f::Constant(
        std::numeric_limits<float>::infinity());
    Eigen::Vector3f maximum = Eigen::Vector3f::Constant(
        -std::numeric_limits<float>::infinity());
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    bool has_high = false;
    for (std::size_t member : members) {
      const pcl::PointXYZ& point = voxel_cloud.points[member];
      xs.push_back(point.x);
      ys.push_back(point.y);
      zs.push_back(point.z);
      minimum = minimum.cwiseMin(Eigen::Vector3f(point.x, point.y, point.z));
      maximum = maximum.cwiseMax(Eigen::Vector3f(point.x, point.y, point.z));
      sum += Eigen::Vector3f(point.x, point.y, point.z);
      if (point.z >= 1.3F) has_high = true;  // bag oracle marker only
    }
    fragment.min_xyz = minimum;
    fragment.max_xyz = maximum;
    fragment.center = sum / static_cast<float>(members.size());
    fragment.z_p05 = quantile(zs, 0.05);
    fragment.z_p50 = quantile(zs, 0.50);
    fragment.z_p95 = quantile(zs, 0.95);
    fragment.oracle_high_surface = has_high ? 1 : 0;

    const Eigen::Vector2f frag_xy_min = Eigen::Vector2f(minimum.x(), minimum.y());
    const Eigen::Vector2f frag_xy_max = Eigen::Vector2f(maximum.x(), maximum.y());
    const float frag_xy_area = std::max(1.0e-6F,
        (frag_xy_max.x() - frag_xy_min.x()) *
        (frag_xy_max.y() - frag_xy_min.y()));

    float best_3d = std::numeric_limits<float>::infinity();
    float best_xy = std::numeric_limits<float>::infinity();
    std::int32_t best_id = -1;
    float best_vertical_separation = 0.0F;
    std::int32_t overlapping_primary_count = 0;

    for (std::size_t cluster_id = 0U;
         cluster_id < primary_geometry.size(); ++cluster_id) {
      const PrimaryGeometry& geometry = primary_geometry[cluster_id];
      if (geometry.member_indices.empty()) continue;

      const Eigen::Vector2f prim_xy_min = geometry.xy_min;
      const Eigen::Vector2f prim_xy_max = geometry.xy_max;
      const bool xy_overlaps =
          frag_xy_max.x() >= prim_xy_min.x() &&
          frag_xy_min.x() <= prim_xy_max.x() &&
          frag_xy_max.y() >= prim_xy_min.y() &&
          frag_xy_min.y() <= prim_xy_max.y();
      if (xy_overlaps) ++overlapping_primary_count;

      const float prim_z_center = median(geometry.zs);
      const float vertical_separation =
          std::abs(fragment.center.z() - prim_z_center);

      for (std::size_t member : members) {
        const pcl::PointXYZ& fp = voxel_cloud.points[member];
        for (std::size_t prim_member : geometry.member_indices) {
          const pcl::PointXYZ& pp = voxel_cloud.points[prim_member];
          const float dx = fp.x - pp.x;
          const float dy = fp.y - pp.y;
          const float dz = fp.z - pp.z;
          const float d3 = std::sqrt(dx * dx + dy * dy + dz * dz);
          const float dxy = std::sqrt(dx * dx + dy * dy);
          if (d3 < best_3d) {
            best_3d = d3;
            best_id = static_cast<std::int32_t>(cluster_id);
            best_vertical_separation = vertical_separation;
          }
          if (dxy < best_xy) best_xy = dxy;
        }
      }
    }

    fragment.nearest_primary_id = best_id;
    fragment.nearest_primary_3d_distance = best_3d;
    fragment.nearest_primary_xy_distance = best_xy;
    fragment.vertical_separation = best_vertical_separation;
    fragment.candidate_primary_neighbor_count = overlapping_primary_count;

    if (best_id >= 0) {
      const PrimaryGeometry& best = primary_geometry[best_id];
      const Eigen::Vector2f prim_xy_min = best.xy_min;
      const Eigen::Vector2f prim_xy_max = best.xy_max;
      const float overlap_x = std::max(0.0F,
          std::min(frag_xy_max.x(), prim_xy_max.x()) -
          std::max(frag_xy_min.x(), prim_xy_min.x()));
      const float overlap_y = std::max(0.0F,
          std::min(frag_xy_max.y(), prim_xy_max.y()) -
          std::max(frag_xy_min.y(), prim_xy_min.y()));
      const float overlap_area = overlap_x * overlap_y;
      fragment.xy_projected_overlap = overlap_area / frag_xy_area;
      if (overlap_area <= 0.0F) {
        const float gap_x = std::max(0.0F,
            std::max(frag_xy_min.x() - prim_xy_max.x(),
                     prim_xy_min.x() - frag_xy_max.x()));
        const float gap_y = std::max(0.0F,
            std::max(frag_xy_min.y() - prim_xy_max.y(),
                     prim_xy_min.y() - frag_xy_max.y()));
        fragment.xy_projected_gap = std::sqrt(gap_x * gap_x + gap_y * gap_y);
      }
    }

    result.weak_fragments.push_back(std::move(fragment));
  }

  return result;
}

}  // namespace ndt_slam
