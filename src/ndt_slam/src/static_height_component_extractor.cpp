#include "ndt_slam/static_height_component_extractor.hpp"

#include "ndt_slam/cargo_oriented_footprint.hpp"
#include "ndt_slam/static_evidence_authorization.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <tuple>

namespace ndt_slam {
namespace {

bool validConfig(const StaticHeightComponentExtractorConfig& config) {
  return std::isfinite(config.neighbor_height_center_tolerance_m) &&
      config.neighbor_height_center_tolerance_m >= 0.0F &&
      std::isfinite(config.neighbor_interval_gap_tolerance_m) &&
      config.neighbor_interval_gap_tolerance_m >= 0.0F &&
      std::isfinite(config.maximum_support_height_difference_m) &&
      config.maximum_support_height_difference_m >= 0.0F &&
      config.maximum_component_cells > 0U &&
      config.minimum_component_cells > 0U &&
      config.minimum_component_cells <= config.maximum_component_cells &&
      std::isfinite(config.maximum_anchor_distance_m) &&
      config.maximum_anchor_distance_m > 0.0F &&
      std::isfinite(config.minimum_candidate_overlap) &&
      config.minimum_candidate_overlap >= 0.0F &&
      config.minimum_candidate_overlap <= 1.0F;
}

Eigen::Vector2f cellCenter(std::int64_t key, float cell_size_m) {
  const std::pair<std::int32_t, std::int32_t> xy =
      unpackStaticEvidenceCell(key);
  return Eigen::Vector2f(
      (static_cast<float>(xy.first) + 0.5F) * cell_size_m,
      (static_cast<float>(xy.second) + 0.5F) * cell_size_m);
}

float intervalGap(const StaticHeightLayer& lhs,
                  const StaticHeightLayer& rhs) {
  if (lhs.z95 >= rhs.z05 && rhs.z95 >= lhs.z05) return 0.0F;
  return std::max(lhs.z05, rhs.z05) - std::min(lhs.z95, rhs.z95);
}

bool insideObb(const Eigen::Vector2f& point,
               const Eigen::Vector2f& center,
               float length_m,
               float width_m,
               float yaw_rad,
               float margin_m = 0.0F) {
  const Eigen::Vector2f delta = point - center;
  const float cosine = std::cos(yaw_rad);
  const float sine = std::sin(yaw_rad);
  const float local_x = cosine * delta.x() + sine * delta.y();
  const float local_y = -sine * delta.x() + cosine * delta.y();
  return std::abs(local_x) <= 0.5F * length_m + margin_m &&
      std::abs(local_y) <= 0.5F * width_m + margin_m;
}

std::uint64_t stableComponentId(
    const std::vector<StaticHeightLayerNodeId>& members,
    StaticEvidenceAuthority authority) {
  std::uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](std::uint64_t value) {
    for (int byte = 0; byte < 8; ++byte) {
      hash ^= (value >> (8 * byte)) & 0xffU;
      hash *= 1099511628211ULL;
    }
  };
  mix(static_cast<std::uint64_t>(authority));
  for (const StaticHeightLayerNodeId& member : members) {
    mix(static_cast<std::uint64_t>(member.cell_key));
    mix(static_cast<std::uint64_t>(member.layer_index));
  }
  return hash == 0U ? 1U : hash;
}

float median(std::vector<float> values) {
  if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  float result = values[middle];
  if (values.size() % 2U == 0U) {
    const float lower = *std::max_element(
        values.begin(), values.begin() + middle);
    result = 0.5F * (lower + result);
  }
  return result;
}

int authorityRank(StaticEvidenceAuthority authority) {
  switch (authority) {
    case StaticEvidenceAuthority::RUNTIME_MATURE: return 2;
    case StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE: return 3;
    case StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN: return 1;
  }
  return 0;
}

}  // namespace

StaticHeightComponentExtractor::StaticHeightComponentExtractor(
    const StaticHeightComponentExtractorConfig& config) {
  setConfig(config);
}

void StaticHeightComponentExtractor::setConfig(
    const StaticHeightComponentExtractorConfig& config) {
  config_ = validConfig(config)
      ? config : StaticHeightComponentExtractorConfig{};
}

std::vector<StaticHeightComponent> StaticHeightComponentExtractor::extract(
    const StaticHeightField& field,
    const StaticHeightComponentQuery& query) const {
  std::vector<StaticHeightComponent> components;
  if (!query.hook_anchor_map.allFinite() ||
      query.map_generation == 0U ||
      field.mapGeneration() != query.map_generation) {
    return components;
  }

  typedef std::map<StaticHeightLayerNodeId, const StaticHeightLayer*> NodeMap;
  NodeMap nodes;
  for (std::map<std::int64_t, StaticHeightCell>::const_iterator cell_it =
           field.cells().begin(); cell_it != field.cells().end(); ++cell_it) {
    const Eigen::Vector2f center =
        cellCenter(cell_it->first, field.config().cell_size_m);
    if ((center - query.hook_anchor_map).norm() >
        config_.maximum_anchor_distance_m + field.config().cell_size_m) {
      continue;
    }
    const StaticHeightCell& cell = cell_it->second;
    for (std::size_t layer_index = 0U;
         layer_index < cell.layers.size(); ++layer_index) {
      const StaticHeightLayer& layer = cell.layers[layer_index];
      if (!authorizeStaticEvidence(layer.authority)
               .formal_origin_authorized) {
        continue;
      }
      nodes[StaticHeightLayerNodeId{
          cell_it->first, static_cast<std::uint16_t>(layer_index)}] = &layer;
    }
  }

  std::set<StaticHeightLayerNodeId> visited;
  for (NodeMap::const_iterator seed_it = nodes.begin();
       seed_it != nodes.end(); ++seed_it) {
    if (visited.count(seed_it->first) > 0U) continue;
    std::deque<StaticHeightLayerNodeId> queue;
    std::vector<StaticHeightLayerNodeId> members;
    bool component_too_large = false;
    queue.push_back(seed_it->first);
    visited.insert(seed_it->first);
    while (!queue.empty()) {
      const StaticHeightLayerNodeId current = queue.front();
      queue.pop_front();
      if (members.size() < config_.maximum_component_cells) {
        members.push_back(current);
      } else {
        // Continue walking the connected set so an oversized component
        // cannot be fragmented into smaller, apparently valid candidates.
        component_too_large = true;
      }
      const StaticHeightLayer& current_layer = *nodes.find(current)->second;
      const StaticHeightCell* current_cell = field.cell(current.cell_key);
      const std::pair<std::int32_t, std::int32_t> xy =
          unpackStaticEvidenceCell(current.cell_key);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          const std::int64_t neighbor_key = packStaticEvidenceCell(
              xy.first + dx, xy.second + dy);
          const StaticHeightCell* neighbor_cell = field.cell(neighbor_key);
          if (!neighbor_cell) continue;
          for (std::size_t neighbor_index = 0U;
               neighbor_index < neighbor_cell->layers.size();
               ++neighbor_index) {
            const StaticHeightLayerNodeId neighbor{
                neighbor_key,
                static_cast<std::uint16_t>(neighbor_index)};
            NodeMap::const_iterator neighbor_it = nodes.find(neighbor);
            if (neighbor_it == nodes.end() ||
                visited.count(neighbor) > 0U) {
              continue;
            }
            const StaticHeightLayer& neighbor_layer = *neighbor_it->second;
            if (neighbor_layer.authority != current_layer.authority ||
                std::abs(neighbor_layer.z50 - current_layer.z50) >
                    config_.neighbor_height_center_tolerance_m ||
                intervalGap(neighbor_layer, current_layer) >
                    config_.neighbor_interval_gap_tolerance_m) {
              continue;
            }
            if (current_cell && current_cell->support.valid &&
                neighbor_cell->support.valid &&
                std::abs(current_cell->support.z -
                         neighbor_cell->support.z) >
                    config_.maximum_support_height_difference_m) {
              continue;
            }
            visited.insert(neighbor);
            queue.push_back(neighbor);
          }
        }
      }
    }
    if (component_too_large) continue;

    std::set<std::int64_t> unique_cells;
    for (const StaticHeightLayerNodeId& member : members) {
      unique_cells.insert(member.cell_key);
    }
    if (unique_cells.size() < config_.minimum_component_cells) continue;

    StaticHeightComponent component;
    component.members = members;
    std::sort(component.members.begin(), component.members.end());
    component.map_generation = query.map_generation;
    component.authority = nodes.find(component.members.front())->second->authority;
    component.component_id = stableComponentId(
        component.members, component.authority);

    std::vector<Eigen::Vector2f> centers;
    std::vector<float> top_values;
    std::vector<float> support_values;
    centers.reserve(unique_cells.size());
    for (const std::int64_t cell_key : unique_cells) {
      centers.push_back(cellCenter(cell_key, field.config().cell_size_m));
      const StaticHeightCell* cell = field.cell(cell_key);
      if (cell && cell->support.valid) support_values.push_back(cell->support.z);
    }
    for (const StaticHeightLayerNodeId& member : component.members) {
      const StaticHeightLayer& layer = *nodes.find(member)->second;
      top_values.push_back(layer.z95);
      component.point_count += layer.point_count;
      component.uncertainty_m = std::max(
          component.uncertainty_m, layer.uncertainty_m);
    }
    Eigen::Vector2f pca_center = Eigen::Vector2f::Zero();
    for (const Eigen::Vector2f& center : centers) pca_center += center;
    pca_center /= static_cast<float>(centers.size());

    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    for (const Eigen::Vector2f& center : centers) {
      const Eigen::Vector2f delta = center - pca_center;
      covariance.noalias() += delta * delta.transpose();
    }
    covariance /= static_cast<float>(centers.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
    Eigen::Vector2f long_axis = Eigen::Vector2f::UnitX();
    if (solver.info() == Eigen::Success &&
        solver.eigenvectors().allFinite()) {
      long_axis = solver.eigenvectors().col(1).normalized();
    }
    const Eigen::Vector2f short_axis(-long_axis.y(), long_axis.x());
    float min_long = std::numeric_limits<float>::infinity();
    float max_long = -std::numeric_limits<float>::infinity();
    float min_short = std::numeric_limits<float>::infinity();
    float max_short = -std::numeric_limits<float>::infinity();
    for (const Eigen::Vector2f& center : centers) {
      const Eigen::Vector2f delta = center - pca_center;
      min_long = std::min(min_long, delta.dot(long_axis));
      max_long = std::max(max_long, delta.dot(long_axis));
      min_short = std::min(min_short, delta.dot(short_axis));
      max_short = std::max(max_short, delta.dot(short_axis));
    }
    component.center_map = pca_center +
        0.5F * (min_long + max_long) * long_axis +
        0.5F * (min_short + max_short) * short_axis;
    component.length_m = max_long - min_long + field.config().cell_size_m;
    component.width_m = max_short - min_short + field.config().cell_size_m;
    component.yaw_map_rad = normalizeAxialYaw(
        std::atan2(long_axis.y(), long_axis.x()));
    if (component.width_m > component.length_m) {
      std::swap(component.length_m, component.width_m);
      component.yaw_map_rad = normalizeAxialYaw(
          component.yaw_map_rad + 0.5F * 3.14159265358979323846F);
    }
    component.top_z95_map = median(top_values);
    component.support_z_map = median(support_values);
    if (!std::isfinite(component.support_z_map)) {
      const StaticSupportSurfaceCell support =
          field.supportAt(component.center_map);
      if (support.valid) {
        component.support_z_map = support.z;
        component.uncertainty_m = std::max(
            component.uncertainty_m, support.uncertainty_m);
      }
    }
    component.hook_anchor_distance_m =
        (component.center_map - query.hook_anchor_map).norm();
    component.anchor_overlap = insideObb(
        query.hook_anchor_map, component.center_map,
        component.length_m, component.width_m,
        component.yaw_map_rad, field.config().cell_size_m)
        ? 1.0F : 0.0F;
    if (query.candidate_valid && query.candidate_center_map.allFinite() &&
        query.candidate_length_m > 0.0F && query.candidate_width_m > 0.0F) {
      std::size_t overlap = 0U;
      for (const Eigen::Vector2f& center : centers) {
        if (insideObb(center, query.candidate_center_map,
                      query.candidate_length_m, query.candidate_width_m,
                      query.candidate_yaw_map_rad,
                      0.5F * field.config().cell_size_m)) {
          ++overlap;
        }
      }
      component.candidate_overlap = static_cast<float>(overlap) /
          static_cast<float>(centers.size());
    }
    component.valid = component.center_map.allFinite() &&
        std::isfinite(component.top_z95_map) &&
        std::isfinite(component.support_z_map) &&
        component.top_z95_map > component.support_z_map &&
        component.hook_anchor_distance_m <= config_.maximum_anchor_distance_m &&
        (!query.candidate_valid ||
         component.candidate_overlap >= config_.minimum_candidate_overlap);
    component.reason = component.valid
        ? "connected_layer_component" : "component_constraints_failed";
    if (component.valid) components.push_back(component);
  }

  std::stable_sort(
      components.begin(), components.end(),
      [&query](const StaticHeightComponent& lhs,
               const StaticHeightComponent& rhs) {
        const float lhs_top_error = query.expected_top_valid
            ? std::abs(lhs.top_z95_map - query.expected_top_z_map) : 0.0F;
        const float rhs_top_error = query.expected_top_valid
            ? std::abs(rhs.top_z95_map - query.expected_top_z_map) : 0.0F;
        const int lhs_previous =
            lhs.component_id == query.previous_component_id ? 1 : 0;
        const int rhs_previous =
            rhs.component_id == query.previous_component_id ? 1 : 0;
        const auto size_error = [&query](
            const StaticHeightComponent& component) {
          if (!query.candidate_valid || query.candidate_length_m <= 0.0F ||
              query.candidate_width_m <= 0.0F) {
            return 0.0F;
          }
          const float direct =
              std::abs(component.length_m - query.candidate_length_m) +
              std::abs(component.width_m - query.candidate_width_m);
          const float swapped =
              std::abs(component.length_m - query.candidate_width_m) +
              std::abs(component.width_m - query.candidate_length_m);
          return std::min(direct, swapped);
        };
        const float lhs_size_error = size_error(lhs);
        const float rhs_size_error = size_error(rhs);
        return std::make_tuple(
                   lhs.candidate_overlap, -lhs.hook_anchor_distance_m,
                   -lhs_top_error, -lhs_size_error,
                   authorityRank(lhs.authority), lhs.point_count,
                   lhs_previous, lhs.anchor_overlap) >
            std::make_tuple(
                   rhs.candidate_overlap, -rhs.hook_anchor_distance_m,
                   -rhs_top_error, -rhs_size_error,
                   authorityRank(rhs.authority), rhs.point_count,
                   rhs_previous, rhs.anchor_overlap);
      });
  return components;
}

}  // namespace ndt_slam
