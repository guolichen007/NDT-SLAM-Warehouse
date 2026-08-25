from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(encoding="utf-8")
AUTHORITY = (
    ROOT / "src/ndt_slam/src/rail_localization_authority.cpp"
).read_text(encoding="utf-8")
CONFIRMATION = (
    ROOT / "src/ndt_slam/src/relocalization_confirmation_policy.cpp"
).read_text(encoding="utf-8")
RAIL_GRAPH_HEADER = (
    ROOT / "src/ndt_slam/include/ndt_slam/rail_translation_pose_graph.hpp"
).read_text(encoding="utf-8")
RAIL_GRAPH = (
    ROOT / "src/ndt_slam/src/rail_translation_pose_graph.cpp"
).read_text(encoding="utf-8")


def _function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_normal_relocalization_uses_rail_xy_confirmation_interface():
    consume = _function_body(
        NODE,
        "void NdtSlamNode::consumeRelocalizationResult(",
        "void NdtSlamNode::updateRelocalization(",
    )
    assert "evaluateRailRelocalizationConfirmation" in consume
    assert "yaw_authority_mode_ == YawAuthorityMode::RAIL_AUTHORITY" in consume


def test_rail_confirmation_never_uses_free_yaw_delta():
    rail = _function_body(
        CONFIRMATION,
        "RelocalizationConfirmationDecision evaluateRailRelocalizationConfirmation(",
        "}  // namespace ndt_slam",
    )
    assert "maximum_yaw_delta_deg" not in rail
    assert "yawOf(" not in rail
    assert "correction_xy" in rail


def test_normal_proposals_have_no_yaw_authority_writer():
    proposal = _function_body(
        AUTHORITY,
        "void RailYawAuthority::observeProposalYaw(",
        "void RailYawAuthority::observeRelocalizationProposalYaw(",
    )
    relocation = _function_body(
        AUTHORITY,
        "void RailYawAuthority::observeRelocalizationProposalYaw(",
        "void RailYawAuthority::handleTimestampRollback(",
    )
    assert "assignReference" not in proposal
    assert "reference_ =" not in proposal
    assert "assignReference" not in relocation
    assert "reference_ =" not in relocation


def test_rail_recovery_bypasses_soft_yaw_and_independent_icp_rotation():
    apply_recovery = _function_body(
        NODE,
        "void NdtSlamNode::applyRelocalizedPose(",
        "void NdtSlamNode::resetCargoAfterPoseDiscontinuity(",
    )
    assert "applyRailYawAuthorityConstraint" in apply_recovery
    assert "yaw_authority_mode_ != YawAuthorityMode::RAIL_AUTHORITY" in apply_recovery
    assert (
        "yaw_authority_mode_ != YawAuthorityMode::RAIL_AUTHORITY &&\n"
        "                icp_refine_cfg_.enabled"
    ) in NODE


def test_rail_health_not_raw_proposal_drives_relocalization_bad_frames():
    assert "increment_relocalization_bad_frames" in NODE
    assert "raw_ndt_proposal_healthy = raw_proposal_healthy" in NODE
    rail_branch = _function_body(
        NODE,
        "if (ndt_attempted_this_frame) {",
        "// ========== 阶段 7",
    )
    assert "authoritative_frame_healthy" in rail_branch
    assert "increment_relocalization_bad_frames" in rail_branch


def test_rail_graph_has_no_yaw_or_se3_state_and_uses_bounded_irls():
    assert "Sophus" not in RAIL_GRAPH_HEADER
    assert "yaw_rad" not in RAIL_GRAPH_HEADER
    assert "double yaw" not in RAIL_GRAPH_HEADER
    assert "maximum_irls_iterations" in RAIL_GRAPH
    assert "huberWeight" in RAIL_GRAPH
    assert "for (int iteration = 0; iteration < maximum_irls_iterations" in RAIL_GRAPH


def test_rail_loop_apply_never_calls_se3_pose_writer():
    consume = _function_body(
        NODE,
        "void NdtSlamNode::consumeLoopClosureResult(",
        "void NdtSlamNode::updatePoseFromLoopClosure(",
    )
    assert "if (result.rail_translation_only)" in consume
    assert "applyRailOptimizedTranslations" in consume
    rail_branch = consume.split("if (result.rail_translation_only)", 1)[1]
    rail_branch = rail_branch.split("} else {", 1)[0]
    assert "applyOptimizedPoses" not in rail_branch
