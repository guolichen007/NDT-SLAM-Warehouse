#include "ndt_slam/rail_translation_pose_graph.hpp"

#include <Eigen/SparseCholesky>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

bool finiteInformation(const Eigen::Matrix2d& information) {
  return information.allFinite() && information.isApprox(
      information.transpose(), 1.0e-10) &&
      information.diagonal().minCoeff() > 0.0;
}

double huberWeight(double residual_norm, double delta) {
  if (residual_norm <= delta) return 1.0;
  return delta / std::max(residual_norm, 1.0e-12);
}

}  // namespace

bool RailTranslationPoseGraph::addNode(
    std::uint64_t id, const Eigen::Vector2d& xy, bool fixed) {
  if (!xy.allFinite() || nodes_.count(id) != 0U) return false;
  nodes_.emplace(id, Node{xy, fixed});
  return true;
}

bool RailTranslationPoseGraph::addEdge(const RailTranslationEdge& edge) {
  if (edge.from_id == edge.to_id ||
      nodes_.count(edge.from_id) == 0U ||
      nodes_.count(edge.to_id) == 0U ||
      !edge.measured_translation.allFinite() ||
      !finiteInformation(edge.information)) {
    return false;
  }
  edges_.push_back(edge);
  return true;
}

bool RailTranslationPoseGraph::addOdometryEdge(
    std::uint64_t from_id, std::uint64_t to_id,
    const Eigen::Vector2d& measured_translation,
    const Eigen::Matrix2d& information) {
  return addEdge({from_id, to_id, measured_translation, information, false});
}

bool RailTranslationPoseGraph::addLoopEdge(
    std::uint64_t from_id, std::uint64_t to_id,
    const Eigen::Vector2d& measured_translation,
    const Eigen::Matrix2d& information) {
  return addEdge({from_id, to_id, measured_translation, information, true});
}

RailTranslationGraphResult RailTranslationPoseGraph::optimize(
    int maximum_irls_iterations, double huber_delta) const {
  RailTranslationGraphResult result;
  if (nodes_.size() < 2U || edges_.empty() ||
      maximum_irls_iterations <= 0 || !std::isfinite(huber_delta) ||
      huber_delta <= 0.0) {
    result.reason = "invalid_rail_translation_graph";
    return result;
  }
  std::size_t fixed_count = 0U;
  for (const auto& item : nodes_) fixed_count += item.second.fixed ? 1U : 0U;
  if (fixed_count != 1U) {
    result.reason = "rail_translation_graph_requires_one_fixed_origin";
    return result;
  }

  std::map<std::uint64_t, int> variable_offset;
  int variable_count = 0;
  for (const auto& item : nodes_) {
    if (!item.second.fixed) {
      variable_offset[item.first] = 2 * variable_count++;
    }
    result.optimized_xy[item.first] = item.second.xy;
  }
  const int dimension = 2 * variable_count;
  if (dimension == 0) {
    result.reason = "rail_translation_graph_no_variables";
    return result;
  }

  for (int iteration = 0; iteration < maximum_irls_iterations; ++iteration) {
    std::vector<Eigen::Triplet<double>> triplets;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dimension);
    double cost = 0.0;
    for (const auto& edge : edges_) {
      const Eigen::Vector2d residual =
          (result.optimized_xy.at(edge.to_id) -
           result.optimized_xy.at(edge.from_id)) -
          edge.measured_translation;
      const double whitened_norm = std::sqrt(std::max(
          0.0, residual.dot(edge.information * residual)));
      const double weight = edge.robust_loop
          ? huberWeight(whitened_norm, huber_delta) : 1.0;
      const Eigen::Matrix2d information = weight * edge.information;
      cost += edge.robust_loop && whitened_norm > huber_delta
          ? huber_delta * (2.0 * whitened_norm - huber_delta)
          : whitened_norm * whitened_norm;

      const auto from = variable_offset.find(edge.from_id);
      const auto to = variable_offset.find(edge.to_id);
      const auto add_block = [&triplets](int row, int col,
                                         const Eigen::Matrix2d& block) {
        for (int r = 0; r < 2; ++r) {
          for (int c = 0; c < 2; ++c) {
            triplets.emplace_back(row + r, col + c, block(r, c));
          }
        }
      };
      if (from != variable_offset.end()) {
        add_block(from->second, from->second, information);
        gradient.segment<2>(from->second) -= information * residual;
      }
      if (to != variable_offset.end()) {
        add_block(to->second, to->second, information);
        gradient.segment<2>(to->second) += information * residual;
      }
      if (from != variable_offset.end() && to != variable_offset.end()) {
        add_block(from->second, to->second, -information);
        add_block(to->second, from->second, -information);
      }
    }
    Eigen::SparseMatrix<double> hessian(dimension, dimension);
    hessian.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(hessian);
    if (solver.info() != Eigen::Success) {
      result.reason = "rail_translation_graph_factorization_failed";
      return result;
    }
    const Eigen::VectorXd step = solver.solve(-gradient);
    if (solver.info() != Eigen::Success || !step.allFinite()) {
      result.reason = "rail_translation_graph_solve_failed";
      return result;
    }
    double maximum_step = 0.0;
    for (const auto& item : variable_offset) {
      const Eigen::Vector2d delta = step.segment<2>(item.second);
      result.optimized_xy[item.first] += delta;
      maximum_step = std::max(maximum_step, delta.norm());
    }
    result.irls_iterations = iteration + 1;
    result.final_cost = cost;
    if (maximum_step <= 1.0e-6) break;
  }
  result.valid = true;
  result.reason = "rail_translation_graph_optimized";
  return result;
}

}  // namespace ndt_slam
