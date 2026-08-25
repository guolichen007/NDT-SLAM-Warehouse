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
