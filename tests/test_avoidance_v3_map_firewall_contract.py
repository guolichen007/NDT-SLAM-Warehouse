from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
NDT_CPP = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(encoding="utf-8")
HUMAN_CPP = (ROOT / "src/ndt_slam/src/human_object_filter.cpp").read_text(
    encoding="utf-8"
)
CLEAN_CPP = (ROOT / "src/ndt_slam/src/clean_map_builder.cpp").read_text(
    encoding="utf-8"
)


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    open_brace = source.index("{", start)
    depth = 0
    for index in range(open_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


def test_map_commit_worker_cannot_advance_or_read_live_human_state():
    body = function_body(
        NDT_CPP,
        "bool NdtSlamNode::commitKeyFrameWithDynamicFiltering(",
    )
    forbidden = (
        "human_filter_.processFrame",
        "human_filter_.updateMapTracks",
        "human_filter_.getDenyCellsSnapshot",
        "dynamic_event_manager_.createHumanEvent",
        "current_pose_",
    )
    for token in forbidden:
        assert token not in body
    assert "job.avoidance_map_mutation.human_points.owns(point)" in body


def test_active_frame_path_has_one_human_state_owner():
    active_source = re.sub(r"#if 0.*?#endif", "", NDT_CPP, flags=re.DOTALL)
    assert active_source.count("human_filter_.updateMapTracks(") == 1
    assert active_source.count("human_filter_.classifyFrame(") == 1


def test_map_commit_pose_and_snapshot_are_frame_local():
    enqueue = function_body(
        NDT_CPP,
        "void NdtSlamNode::enqueueMapCommitJob(",
    )
    assert "job.pose = frame_context.runtime_pose" in enqueue
    assert "job.runtime_pose = frame_context.runtime_pose" in enqueue
    assert "job.runtime_pose = current_pose_" not in enqueue
    assert "job.avoidance_map_mutation = avoidance_map_mutation" in enqueue


def test_stale_avoidance_snapshot_drops_whole_map_commit():
    authority = function_body(
        NDT_CPP,
        "bool NdtSlamNode::isMapCommitAuthorityCurrent(",
    )
    assert "job.avoidance_map_mutation.validFor(" in authority


def test_human_deny_cannot_invalidate_existing_static_authority():
    rebuild = function_body(
        NDT_CPP,
        "void NdtSlamNode::startCleanMapRebuildJob()",
    )
    invalidation_tail = rebuild[rebuild.index("appendStaticEvidenceInvalidations("):]
    assert "appendStaticEvidenceInvalidations(\n        input.human_deny_cells" not in invalidation_tail
    assert "input.human_deny_cells = human_filter_.getDenyCellsSnapshot" not in rebuild
    assert "previous_clean_indices.find(cell)" in CLEAN_CPP


def test_pre_registration_classification_has_no_temporal_writes():
    classification = function_body(
        HUMAN_CPP,
        "HumanFrameClassification HumanObjectDynamicFilter::classifyFrame(",
    )
    for token in (
        "updateTracks(",
        "cleanupExpiredTracks(",
        "addDenyCells(",
        "trajectory_capsules_",
        "human_deny_cells_",
    ):
        assert token not in classification


def test_dynamic_event_human_has_no_active_product_writer():
    active_worker = function_body(
        NDT_CPP,
        "bool NdtSlamNode::commitKeyFrameWithDynamicFiltering(",
    )
    assert "createHumanEvent" not in active_worker
