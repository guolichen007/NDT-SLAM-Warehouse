#include "ndt_slam/avoidance_decision.hpp"

namespace ndt_slam {

CargoSafetyPhaseSelection deriveCargoSafetyDecisionPhase(
    const CargoSafetyDecisionInput& input) {
    CargoSafetyPhaseSelection selection;
    const bool gravity_required_unavailable =
        input.hook_signal_role == HookLoadSignalRole::REQUIRED &&
        (!input.gravity_valid ||
         (!input.gravity_empty && !input.gravity_loaded));
    const bool warning_code_known =
        input.warning_code == 0U ||
        input.warning_code == CargoSafetyProtocol::kClear ||
        input.warning_code == CargoSafetyProtocol::kLevel1Warning ||
        input.warning_code == CargoSafetyProtocol::kLevel2Warning ||
        input.warning_code == CargoSafetyProtocol::kAnomalyReview;
    const bool positive_warning = input.warning_valid &&
        (input.warning_code == CargoSafetyProtocol::kLevel1Warning ||
         input.warning_code == CargoSafetyProtocol::kLevel2Warning ||
         input.warning_code == CargoSafetyProtocol::kAnomalyReview) &&
        input.obstacle_evidence_ready &&
        (input.pending_positive_warning || input.formal_cargo_valid) &&
        !input.cargo_fault && !input.obstacle_fault;

    if (input.internal_fault) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "explicit_internal_fault";
    } else if (input.warning_valid && !warning_code_known) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "invalid_warning_protocol_code";
    } else if (input.hook_signal_role != HookLoadSignalRole::DISABLED &&
               input.gravity_empty && input.gravity_loaded) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "gravity_state_not_exclusive";
    } else if (!input.system_ready) {
        selection.phase = CargoSafetyDecisionPhase::STARTUP_NOT_READY;
        selection.reason = "system_not_ready";
    } else if (!input.localization_valid) {
        selection.phase = CargoSafetyDecisionPhase::LOCALIZATION_INVALID;
        selection.reason = "localization_unreliable";
    } else if (gravity_required_unavailable) {
        selection.phase = CargoSafetyDecisionPhase::GRAVITY_REQUIRED_INVALID;
        selection.reason = "required_gravity_unavailable";
    } else if (positive_warning) {
        selection.phase = CargoSafetyDecisionPhase::VALID_WARNING_OR_CLEAR;
        selection.reason = input.gravity_conflict
            ? "gravity_lidar_conflict_hazard_retained" : "positive_hazard";
    } else if (input.gravity_conflict &&
               input.hook_signal_role != HookLoadSignalRole::DISABLED) {
        selection.phase = CargoSafetyDecisionPhase::
            CARGO_EXPECTED_NOT_AUTHORITATIVE;
        selection.reason = "gravity_lidar_conflict";
    } else if (input.safe_empty && input.hook_loaded) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "safe_empty_conflicts_with_hook_loaded";
    } else if (input.safe_empty && input.formal_cargo_valid) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "safe_empty_conflicts_with_formal_cargo";
    } else if (input.safe_empty && input.cargo_fault) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "safe_empty_conflicts_with_cargo_fault";
    } else if (input.safe_empty && input.obstacle_fault) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "safe_empty_conflicts_with_obstacle_fault";
    } else if (input.formal_cargo_valid) {
        const bool formal_clear = input.warning_valid &&
            input.warning_code == CargoSafetyProtocol::kClear &&
            input.formal_clear_authorized &&
            input.obstacle_evidence_ready && !input.cargo_fault &&
            !input.obstacle_fault && !input.gravity_conflict;
        if (formal_clear) {
            selection.phase = CargoSafetyDecisionPhase::VALID_WARNING_OR_CLEAR;
            selection.reason = "formal_live_static_clear";
        } else if (input.cargo_fault) {
            selection.phase = CargoSafetyDecisionPhase::
                CARGO_EXPECTED_NOT_AUTHORITATIVE;
            selection.reason = "formal_cargo_invalid";
        } else {
            selection.phase = CargoSafetyDecisionPhase::
                CARGO_FORMAL_OBSTACLE_NOT_READY;
            selection.reason = "formal_cargo_clear_authority_incomplete";
        }
    } else if (input.safe_empty) {
        const bool required_empty_valid =
            input.hook_signal_role != HookLoadSignalRole::REQUIRED ||
            (input.gravity_valid && input.gravity_empty &&
             !input.gravity_loaded);
        const bool auxiliary_loaded_conflict =
            input.hook_signal_role == HookLoadSignalRole::AUXILIARY &&
            input.gravity_valid && input.gravity_loaded;
        if (required_empty_valid && !auxiliary_loaded_conflict &&
            !input.cargo_fault && !input.obstacle_fault) {
            selection.phase = CargoSafetyDecisionPhase::SAFE_EMPTY;
            selection.reason = "strict_lidar_empty_confirmed";
        } else {
            selection.phase = CargoSafetyDecisionPhase::
                CARGO_EXPECTED_NOT_AUTHORITATIVE;
            selection.reason = "safe_empty_authority_incomplete";
        }
    } else {
        selection.phase = CargoSafetyDecisionPhase::
            CARGO_EXPECTED_NOT_AUTHORITATIVE;
        selection.reason = input.pending_positive_warning
            ? "pending_hazard_not_authoritative"
            : "cargo_expected_not_authoritative";
    }
    return selection;
}

bool cargoSafetyDecisionSelfConsistent(
    const CargoSafetyDecision& decision) {
    const bool warning =
        decision.requested_code == CargoSafetyProtocol::kClear ||
        decision.requested_code == CargoSafetyProtocol::kLevel1Warning ||
        decision.requested_code == CargoSafetyProtocol::kLevel2Warning ||
        decision.requested_code == CargoSafetyProtocol::kAnomalyReview;
    if (warning) {
        return decision.valid && decision.warning_valid &&
            decision.warning_code == decision.requested_code &&
            decision.fault_code == 0 && decision.fault_mask == 0U;
    }
    return !decision.valid && !decision.warning_valid &&
        decision.warning_code == 0 &&
        decision.requested_code == decision.fault_code &&
        decision.fault_code >= CargoSafetyProtocol::kSystemNotReady &&
        decision.fault_code <= CargoSafetyProtocol::kInternalError &&
        decision.fault_mask != 0U;
}

CargoSafetyDecision composeCargoSafetyDecision(
    const CargoSafetyDecisionInput& input) {
    CargoSafetyDecision decision;
    decision.fault_mask = 0U;
    const CargoSafetyPhaseSelection selection =
        deriveCargoSafetyDecisionPhase(input);
    const bool use_selection_reason =
        selection.phase == CargoSafetyDecisionPhase::
            INTERNAL_CONTRACT_ERROR || input.gravity_conflict;
    const std::string reason = use_selection_reason ||
        input.evidence_reason.empty()
        ? selection.reason : input.evidence_reason;
    switch (selection.phase) {
        case CargoSafetyDecisionPhase::STARTUP_NOT_READY:
            decision.fault_code = CargoSafetyProtocol::kSystemNotReady;
            decision.fault_mask = CargoSafetyProtocol::kFaultStatusStale;
            break;
        case CargoSafetyDecisionPhase::LOCALIZATION_INVALID:
            decision.fault_code = CargoSafetyProtocol::kLocalizationInvalid;
            decision.fault_mask = CargoSafetyProtocol::kFaultLocalization;
            break;
        case CargoSafetyDecisionPhase::GRAVITY_REQUIRED_INVALID:
            decision.fault_code = CargoSafetyProtocol::kGravityInvalid;
            decision.fault_mask = CargoSafetyProtocol::kFaultGravity;
            break;
        case CargoSafetyDecisionPhase::CARGO_EXPECTED_NOT_AUTHORITATIVE:
            decision.fault_code = CargoSafetyProtocol::kCargoInvalid;
            decision.fault_mask = CargoSafetyProtocol::kFaultCargo;
            break;
        case CargoSafetyDecisionPhase::CARGO_FORMAL_OBSTACLE_NOT_READY:
            decision.fault_code = CargoSafetyProtocol::kObstacleInvalid;
            decision.fault_mask = CargoSafetyProtocol::kFaultObstacle;
            break;
        case CargoSafetyDecisionPhase::SAFE_EMPTY:
            decision.fault_code = 0;
            decision.warning_code = CargoSafetyProtocol::kClear;
            break;
        case CargoSafetyDecisionPhase::VALID_WARNING_OR_CLEAR:
            decision.fault_code = 0;
            decision.warning_code = input.warning_code;
            break;
        case CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR:
        default:
            decision.fault_code = CargoSafetyProtocol::kInternalError;
            decision.fault_mask = CargoSafetyProtocol::kFaultInternal;
            break;
    }
    decision.warning_valid = decision.fault_code == 0;
    decision.valid = decision.warning_valid;
    decision.requested_code = decision.fault_code != 0
        ? decision.fault_code : decision.warning_code;
    decision.reason = reason.empty() ? "safety_decision" : reason;
    if (!cargoSafetyDecisionSelfConsistent(decision)) {
        decision = CargoSafetyDecision{};
        decision.requested_code = CargoSafetyProtocol::kInternalError;
        decision.fault_code = CargoSafetyProtocol::kInternalError;
        decision.fault_mask = CargoSafetyProtocol::kFaultInternal;
        decision.reason = "decision_self_consistency_failure";
    }
    return decision;
}

}  // namespace ndt_slam
