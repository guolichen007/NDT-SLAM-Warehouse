#pragma once

// Protocol composition owns final 14/17/18/29/30-35 selection. The concrete
// types remain declared in cargo_safety_evaluator.hpp for source compatibility.
#include "ndt_slam/cargo_safety_evaluator.hpp"

namespace ndt_slam {

struct CargoAnomalyReviewProjection {
  bool enabled = false;
  std::uint16_t output_code = CargoSafetyProtocol::kAnomalyReview;
  std::string event = "NONE";
};

class AvoidanceDecisionOwner {
 public:
  // Preview is a non-authoritative candidate used only by the stateful review
  // episode tracker. decide() remains the sole final protocol writer.
  CargoSafetyDecision preview(
      const CargoSafetyDecisionInput& input) const;
  CargoSafetyDecision decide(
      const CargoSafetyDecisionInput& input) const;
  CargoSafetyDecision decide(
      const CargoSafetyDecisionInput& input,
      const CargoAnomalyReviewProjection& review) const;
};

}  // namespace ndt_slam
