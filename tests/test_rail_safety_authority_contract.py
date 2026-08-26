from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(encoding="utf-8")


def test_rail_safety_uses_frame_context_and_fails_closed() -> None:
    start = NODE.index("void NdtSlamNode::updateAndPublishCargoSafetyPipeline")
    end = NODE.index("void NdtSlamNode::", start + 20)
    body = NODE[start:end]
    assert "safetyFrameAuthorityMatches" in body
    assert "publishRelocalizationSafetyInvalid" in body
    assert "frame_authority_context_invalid" in body
    assert "mixed_pose_generation_safety_frame_count_" in body
    assert "last_cargo_bottom_result_.pose_authority" in body
    assert "obstacle_track_decision.selected_pose_authority" in body
    assert "formal_static_hazard_decision.pose_authority" in body
    assert "const PoseAuthorityIdentity cargo_pose_identity" not in body
    assert "const PoseAuthorityIdentity obstacle_pose_identity" not in body
    pending_start = NODE.index("void NdtSlamNode::runPendingCargoAvoidance")
    pending_end = NODE.index("void NdtSlamNode::", pending_start + 20)
    pending_body = NODE[pending_start:pending_end]
    assert pending_body.index(
        "pending_temporal_evidence_authority_mismatch"
    ) < pending_body.index("fuseCargoAvoidanceRisk(")
    formal_mismatch = body.index("formal_temporal_evidence_authority_mismatch")
    formal_fusion = body.index("fuseCargoAvoidanceRisk(", formal_mismatch)
    assert formal_mismatch < formal_fusion


def test_rail_pose_is_the_single_map_and_avoidance_pose() -> None:
    assert "feature_cloud, filtered_cloud, frame_authority_context" in NODE
    assert "frame_context.runtime_pose" in NODE
    assert "frame_authority_context.rail_authority_mode" in NODE
    assert "? frame_authority_context.runtime_pose" in NODE


def test_safety_code_semantics_and_obstacle_pipeline_remain_frozen() -> None:
    # The combined phase adds provenance gates, not a second warning mapper.
    assert "fuseCargoAvoidanceRisk(" in NODE
    assert "CargoSafetyEvaluator::kSafeCode" in NODE
    assert "CargoSafetyEvaluator::kLevel1Code" in NODE
    assert "CargoSafetyEvaluator::kLevel2Code" in NODE
