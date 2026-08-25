#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ndt_slam {

struct RailTranslationEdge {
  std::uint64_t from_id = 0U;
  std::uint64_t to_id = 0U;
  Eigen::Vector2d measured_translation = Eigen::Vector2d::Zero();
  Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
  bool robust_loop = false;
};

struct RailTranslationGraphResult {
  bool valid = false;
  std::map<std::uint64_t, Eigen::Vector2d> optimized_xy;
  int irls_iterations = 0;
  double final_cost = 0.0;
  std::string reason = "not_evaluated";
};

// Translation-only pose graph for the rail product path.  Yaw is absent from
// the state and therefore cannot be deformed per keyframe or written back by
// loop closure.
class RailTranslationPoseGraph {
 public:
  bool addNode(std::uint64_t id, const Eigen::Vector2d& xy,
               bool fixed = false);
  bool addOdometryEdge(std::uint64_t from_id, std::uint64_t to_id,
                       const Eigen::Vector2d& measured_translation,
                       const Eigen::Matrix2d& information);
  bool addLoopEdge(std::uint64_t from_id, std::uint64_t to_id,
                   const Eigen::Vector2d& measured_translation,
                   const Eigen::Matrix2d& information);
  RailTranslationGraphResult optimize(int maximum_irls_iterations,
                                      double huber_delta) const;

 private:
  struct Node {
    Eigen::Vector2d xy = Eigen::Vector2d::Zero();
    bool fixed = false;
  };
  bool addEdge(const RailTranslationEdge& edge);

  std::map<std::uint64_t, Node> nodes_;
  std::vector<RailTranslationEdge> edges_;
};

}  // namespace ndt_slam
