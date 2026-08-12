#pragma once

// Protocol composition owns final 14/17/18/29/30-35 selection. The concrete
// types remain declared in cargo_safety_evaluator.hpp for source compatibility.
#include "ndt_slam/cargo_safety_evaluator.hpp"

namespace ndt_slam {

class AvoidanceDecisionOwner {
 public:
  CargoSafetyDecision decide(
      const CargoSafetyDecisionInput& input) const {
    return composeCargoSafetyDecision(input);
  }
};

}  // namespace ndt_slam
