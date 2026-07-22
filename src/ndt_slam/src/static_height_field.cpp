#include "ndt_slam/static_height_field.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <utility>

namespace ndt_slam {
namespace {

bool validConfig(const StaticHeightFieldConfig& config) {
  return std::isfinite(config.cell_size_m) && config.cell_size_m > 0.0F &&
      std::isfinite(config.z_bin_m) && config.z_bin_m > 0.0F &&
      config.maximum_layers_per_cell > 0U &&
      config.maximum_layers_per_cell <= 8U &&
      config.minimum_points_per_layer > 0U &&
      std::isfinite(config.maximum_merge_gap_m) &&
      config.maximum_merge_gap_m >= config.z_bin_m &&
      config.ground_fit_max_z_m > config.ground_fit_min_z_m &&
      config.support_neighbor_outer_m >= config.support_neighbor_inner_m &&
      config.minimum_support_uncertainty_m > 0.0F &&
      config.maximum_support_uncertainty_m >=
          config.minimum_support_uncertainty_m &&
      config.maximum_query_area_cells > 0U;
}

float quantile(const std::vector<float>& sorted, float q) {
  if (sorted.empty()) return 0.0F;
  const float clamped = std::clamp(q, 0.0F, 1.0F);
  const float position = clamped * static_cast<float>(sorted.size() - 1U);
  const std::size_t low = static_cast<std::size_t>(std::floor(position));
  const std::size_t high = static_cast<std::size_t>(std::ceil(position));
  const float fraction = position - static_cast<float>(low);
  return sorted[low] * (1.0F - fraction) + sorted[high] * fraction;
}

std::int64_t keyFor(const Eigen::Vector2f& xy, float cell_size) {
  return packStaticEvidenceCell(
      static_cast<std::int32_t>(std::floor(xy.x() / cell_size)),
      static_cast<std::int32_t>(std::floor(xy.y() / cell_size)));
}

Eigen::Vector2f centerFor(std::int64_t key, float cell_size) {
  const auto xy = unpackStaticEvidenceCell(key);
  return Eigen::Vector2f(
      (static_cast<float>(xy.first) + 0.5F) * cell_size,
      (static_cast<float>(xy.second) + 0.5F) * cell_size);
}

float authorityRank(StaticEvidenceAuthority authority) {
  switch (authority) {
    case StaticEvidenceAuthority::RUNTIME_MATURE: return 3.0F;
    case StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE: return 2.0F;
    case StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN: return 1.0F;
  }
  return 0.0F;
}

}  // namespace

StaticHeightField::StaticHeightField(
    const StaticHeightFieldConfig& config) {
  setConfig(config);
}

void StaticHeightField::setConfig(const StaticHeightFieldConfig& config) {
  config_ = validConfig(config) ? config : StaticHeightFieldConfig{};
  clear();
}

void StaticHeightField::clear() {
  cells_.clear();
  support_plane_.setZero();
  support_plane_valid_ = false;
  map_generation_ = 0U;
}

StaticHeightFieldBuildResult StaticHeightField::build(
    const std::vector<Eigen::Vector3f>& object_points,
    const std::vector<Eigen::Vector3f>& ground_points,
    StaticEvidenceAuthority authority,
    std::uint32_t observation_count,
    std::uint64_t map_generation) {
  clear();
  map_generation_ = map_generation;
  StaticHeightFieldBuildResult result;
  if (!validConfig(config_)) {
    result.reason = "invalid_config";
    return result;
  }

  std::map<std::int64_t, std::vector<float>> object_z;
  for (const Eigen::Vector3f& point : object_points) {
    if (!point.allFinite()) continue;
    ++result.finite_object_points;
    object_z[keyFor(point.head<2>(), config_.cell_size_m)].push_back(
        point.z());
  }
  for (auto& item : object_z) {
    std::vector<float>& values = item.second;
    std::sort(values.begin(), values.end());
    std::vector<std::vector<float>> clusters;
    for (const float z : values) {
      if (clusters.empty() ||
          z - clusters.back().back() > config_.maximum_merge_gap_m) {
        clusters.push_back({z});
      } else {
        clusters.back().push_back(z);
      }
    }
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
                       [this](const std::vector<float>& cluster) {
                         return cluster.size() <
                             config_.minimum_points_per_layer;
                       }),
        clusters.end());
    if (clusters.size() > config_.maximum_layers_per_cell) {
      std::stable_sort(
          clusters.begin(), clusters.end(),
          [](const auto& lhs, const auto& rhs) {
            return lhs.size() > rhs.size();
          });
      clusters.resize(config_.maximum_layers_per_cell);
      std::sort(clusters.begin(), clusters.end(),
                [](const auto& lhs, const auto& rhs) {
                  return lhs.front() < rhs.front();
                });
    }
    if (clusters.empty()) continue;
    StaticHeightCell& output = cells_[item.first];
    output.key = item.first;
    for (const std::vector<float>& cluster : clusters) {
      StaticHeightLayer layer;
      layer.z_low = cluster.front();
      layer.z_high = cluster.back();
      layer.z05 = quantile(cluster, 0.05F);
      layer.z50 = quantile(cluster, 0.50F);
      layer.z95 = quantile(cluster, 0.95F);
      double variance = 0.0;
      for (const float z : cluster) {
        const double residual = static_cast<double>(z - layer.z50);
        variance += residual * residual;
      }
      variance /= static_cast<double>(cluster.size());
      layer.roughness_m = static_cast<float>(std::sqrt(variance));
      layer.uncertainty_m = std::max(
          0.5F * config_.z_bin_m,
          layer.roughness_m /
              std::sqrt(static_cast<float>(cluster.size())));
      layer.point_count = static_cast<std::uint32_t>(cluster.size());
      layer.observation_count = observation_count;
      layer.authority = authority;
      output.layers.push_back(layer);
      ++result.layer_count;
    }
    ++result.occupied_cells;
  }

  std::vector<Eigen::Vector3f> floor_candidates;
  floor_candidates.reserve(ground_points.size());
  for (const Eigen::Vector3f& point : ground_points) {
    if (!point.allFinite()) continue;
    ++result.finite_ground_points;
    if (point.z() >= config_.ground_fit_min_z_m &&
        point.z() <= config_.ground_fit_max_z_m) {
      floor_candidates.push_back(point);
    }
  }
  if (floor_candidates.size() >= 3U) {
    std::vector<Eigen::Vector3f> inliers = floor_candidates;
    for (int iteration = 0; iteration < 3 && inliers.size() >= 3U;
         ++iteration) {
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      for (const Eigen::Vector3f& point : inliers) {
        const Eigen::Vector3d row(point.x(), point.y(), 1.0);
        normal += row * row.transpose();
        rhs += row * static_cast<double>(point.z());
      }
      const Eigen::Vector3d plane = normal.ldlt().solve(rhs);
      if (!plane.allFinite()) break;
      support_plane_ = plane.cast<float>();
      std::vector<float> residuals;
      residuals.reserve(floor_candidates.size());
      for (const Eigen::Vector3f& point : floor_candidates) {
        residuals.push_back(std::abs(
            point.z() - (support_plane_.x() * point.x() +
                         support_plane_.y() * point.y() +
                         support_plane_.z())));
      }
      std::sort(residuals.begin(), residuals.end());
      const float mad = quantile(residuals, 0.50F);
      const float threshold = std::clamp(
          3.0F * 1.4826F * mad, 0.05F, 0.25F);
      inliers.clear();
      for (std::size_t i = 0U; i < floor_candidates.size(); ++i) {
        if (residuals.empty()) break;
        const Eigen::Vector3f& point = floor_candidates[i];
        const float residual = std::abs(
            point.z() - (support_plane_.x() * point.x() +
                         support_plane_.y() * point.y() +
                         support_plane_.z()));
        if (residual <= threshold) inliers.push_back(point);
      }
    }
    support_plane_valid_ = support_plane_.allFinite();
  }
  result.support_plane = support_plane_;

  std::map<std::int64_t, std::vector<float>> support_z;
  double residual_square_sum = 0.0;
  std::size_t residual_count = 0U;
  for (const Eigen::Vector3f& point : ground_points) {
    if (!point.allFinite()) continue;
    const float plane_z = support_plane_valid_
        ? support_plane_.x() * point.x() +
              support_plane_.y() * point.y() + support_plane_.z()
        : 0.0F;
    const float residual = point.z() - plane_z;
    if (std::abs(residual) <= 0.25F) {
      support_z[keyFor(point.head<2>(), config_.cell_size_m)].push_back(
          point.z());
      residual_square_sum += static_cast<double>(residual * residual);
      ++residual_count;
    } else {
      ++result.elevated_ground_points;
    }
  }
  result.support_residual_std_m = residual_count == 0U ? 0.0F :
      static_cast<float>(std::sqrt(
          residual_square_sum / static_cast<double>(residual_count)));
  for (auto& item : support_z) {
    std::vector<float>& values = item.second;
    std::sort(values.begin(), values.end());
    StaticHeightCell& output = cells_[item.first];
    output.key = item.first;
    StaticSupportSurfaceCell support;
    support.valid = true;
    support.z = quantile(values, 0.20F);
    double variance = 0.0;
    for (const float z : values) {
      const double residual = static_cast<double>(z - support.z);
      variance += residual * residual;
    }
    variance /= static_cast<double>(values.size());
    support.variance_m2 = static_cast<float>(variance);
    support.uncertainty_m = std::clamp(
        static_cast<float>(std::sqrt(variance)),
        config_.minimum_support_uncertainty_m,
        config_.maximum_support_uncertainty_m);
    support.point_count = static_cast<std::uint32_t>(values.size());
    output.support = support;
    ++result.support_cells;
  }

  result.valid = !cells_.empty();
  result.reason = result.valid ? "complete" : "no_finite_layers_or_support";
  return result;
}

const StaticHeightCell* StaticHeightField::cell(std::int64_t key) const {
  const auto found = cells_.find(key);
  return found == cells_.end() ? nullptr : &found->second;
}

StaticSupportSurfaceCell StaticHeightField::supportAt(
    const Eigen::Vector2f& xy) const {
  const std::int64_t direct_key = keyFor(xy, config_.cell_size_m);
  const StaticHeightCell* direct = cell(direct_key);
  if (direct && direct->support.valid) return direct->support;

  StaticSupportSurfaceCell result;
  double weight_sum = 0.0;
  double z_sum = 0.0;
  double variance_sum = 0.0;
  const int radius = static_cast<int>(std::ceil(
      config_.support_neighbor_outer_m / config_.cell_size_m));
  const auto origin = unpackStaticEvidenceCell(direct_key);
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      const float distance = config_.cell_size_m *
          std::hypot(static_cast<float>(dx), static_cast<float>(dy));
      if (distance < config_.support_neighbor_inner_m ||
          distance > config_.support_neighbor_outer_m) continue;
      const StaticHeightCell* neighbor = cell(packStaticEvidenceCell(
          origin.first + dx, origin.second + dy));
      if (!neighbor || !neighbor->support.valid) continue;
      const double weight = 1.0 / std::max(0.05F, distance);
      weight_sum += weight;
      z_sum += weight * neighbor->support.z;
      variance_sum += weight * neighbor->support.variance_m2;
      result.point_count += neighbor->support.point_count;
    }
  }
  if (weight_sum > 0.0) {
    result.valid = true;
    result.interpolated = true;
    result.z = static_cast<float>(z_sum / weight_sum);
    result.variance_m2 = static_cast<float>(variance_sum / weight_sum);
    result.uncertainty_m = std::clamp(
        std::max(config_.default_support_uncertainty_m,
                 static_cast<float>(std::sqrt(result.variance_m2))),
        config_.minimum_support_uncertainty_m,
        config_.maximum_support_uncertainty_m);
  } else if (support_plane_valid_) {
    result.valid = true;
    result.interpolated = true;
    result.z = support_plane_.x() * xy.x() +
        support_plane_.y() * xy.y() + support_plane_.z();
    result.uncertainty_m = config_.maximum_support_uncertainty_m;
  }
  return result;
}

StaticHeightQueryResult StaticHeightField::query(
    const StaticHeightQuery& input) const {
  StaticHeightQueryResult result;
  if (!input.center_map.allFinite() || !std::isfinite(input.length_m) ||
      !std::isfinite(input.width_m) || !std::isfinite(input.yaw_map_rad) ||
      !std::isfinite(input.shell_m) || input.length_m <= 0.0F ||
      input.width_m <= 0.0F || input.shell_m < 0.0F ||
      input.maximum_z < input.minimum_z) {
    result.reason = "invalid_query";
    return result;
  }
  const float half_diagonal = 0.5F * std::hypot(
      input.length_m, input.width_m) + input.shell_m;
  const int min_x = static_cast<int>(std::floor(
      (input.center_map.x() - half_diagonal) / config_.cell_size_m));
  const int max_x = static_cast<int>(std::floor(
      (input.center_map.x() + half_diagonal) / config_.cell_size_m));
  const int min_y = static_cast<int>(std::floor(
      (input.center_map.y() - half_diagonal) / config_.cell_size_m));
  const int max_y = static_cast<int>(std::floor(
      (input.center_map.y() + half_diagonal) / config_.cell_size_m));
  const std::size_t width = static_cast<std::size_t>(max_x - min_x + 1);
  const std::size_t height = static_cast<std::size_t>(max_y - min_y + 1);
  const std::size_t maximum_cells = input.maximum_cells == 0U
      ? config_.maximum_query_area_cells :
        std::min(input.maximum_cells, config_.maximum_query_area_cells);
  if (width == 0U || height == 0U ||
      width > maximum_cells || height > maximum_cells ||
      width * height > maximum_cells) {
    result.bounded = false;
    result.reason = "query_area_exceeded";
    return result;
  }

  const float cosine = std::cos(input.yaw_map_rad);
  const float sine = std::sin(input.yaw_map_rad);
  const float half_length = 0.5F * input.length_m;
  const float half_width = 0.5F * input.width_m;
  const bool exclusion_identity_valid =
      input.exclusion_authorized &&
      input.excluded_component_id != 0U &&
      input.excluded_component_generation != 0U &&
      input.excluded_component_generation == map_generation_;
  for (int x = min_x; x <= max_x; ++x) {
    for (int y = min_y; y <= max_y; ++y) {
      ++result.queried_cells;
      const std::int64_t key = packStaticEvidenceCell(x, y);
      const Eigen::Vector2f delta =
          centerFor(key, config_.cell_size_m) - input.center_map;
      const float local_x = cosine * delta.x() + sine * delta.y();
      const float local_y = -sine * delta.x() + cosine * delta.y();
      const float outside_x = std::max(std::abs(local_x) - half_length, 0.0F);
      const float outside_y = std::max(std::abs(local_y) - half_width, 0.0F);
      const float distance = std::hypot(outside_x, outside_y);
      if (distance > input.shell_m + 0.5F * config_.cell_size_m) continue;
      ++result.clear_shell_queried_cells;
      const StaticHeightCell* height_cell = cell(key);
      if (!height_cell) continue;
      const bool raw_covered =
          height_cell->support.valid || !height_cell->layers.empty();
      if (raw_covered) ++result.raw_covered_cells;
      bool matched_cell = false;
      bool cell_has_excluded_layer = false;
      bool cell_has_external_layer = false;
      for (std::size_t layer_index = 0U;
           layer_index < height_cell->layers.size(); ++layer_index) {
        const StaticHeightLayer& layer = height_cell->layers[layer_index];
        if (exclusion_identity_valid &&
            input.excluded_members.count(StaticHeightLayerNodeId{
                key, static_cast<std::uint16_t>(layer_index)}) > 0U) {
          cell_has_excluded_layer = true;
          ++result.excluded_layer_count;
          continue;
        }
        cell_has_external_layer = true;
        if (layer.z95 < input.minimum_z || layer.z05 > input.maximum_z) {
          continue;
        }
        matched_cell = true;
        ++result.matched_layers;
        result.nearest_horizontal_distance_m = std::min(
            result.nearest_horizontal_distance_m, distance);
        if (layer.z95 > result.highest_z95_m) {
          result.highest_z95_m = layer.z95;
          result.highest_uncertainty_m = layer.uncertainty_m;
        }
        if (authorityRank(layer.authority) >
            authorityRank(result.strongest_authority)) {
          result.strongest_authority = layer.authority;
        }
      }
      if (cell_has_excluded_layer) {
        ++result.excluded_origin_cells;
      }
      // Support beneath an excluded origin component cannot prove that the
      // external shell is clear. A mixed cell may still report an external
      // hazard above, but it contributes no formal CLEAR coverage.
      const bool effective_external_covered = !cell_has_excluded_layer &&
          (height_cell->support.valid || cell_has_external_layer);
      if (effective_external_covered) {
        ++result.effective_external_covered_cells;
        ++result.clear_shell_covered_cells;
      }
      if (matched_cell) {
        ++result.matched_cells;
        result.matched_cell_keys.push_back(key);
      }
    }
  }
  result.valid = true;
  result.covered_cells = result.effective_external_covered_cells;
  result.effective_coverage_ratio = result.clear_shell_queried_cells > 0U
      ? static_cast<float>(result.effective_external_covered_cells) /
            static_cast<float>(result.clear_shell_queried_cells)
      : 0.0F;
  result.clear_shell_coverage_ratio =
      result.clear_shell_queried_cells > 0U
      ? static_cast<float>(result.clear_shell_covered_cells) /
            static_cast<float>(result.clear_shell_queried_cells)
      : 0.0F;
  result.coverage_ratio = result.effective_coverage_ratio;
  result.reason = result.matched_cells > 0U ? "matched" : "clear_in_bounds";
  return result;
}

std::size_t StaticHeightField::layerCount() const noexcept {
  std::size_t count = 0U;
  for (const auto& item : cells_) count += item.second.layers.size();
  return count;
}

}  // namespace ndt_slam
