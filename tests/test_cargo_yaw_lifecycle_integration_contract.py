from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_frame_authority_context_is_created_once_and_shared() -> None:
    source = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(
        encoding="utf-8"
    )
    frame_block = source[source.index("FrameAuthorityContext frame_authority_context") :]
    assert frame_block.count("FrameAuthorityContext frame_authority_context") == 1
    assert "feature_cloud, filtered_cloud, frame_authority_context" in frame_block
    assert "filtered_cloud, frame_authority_context, publish_time" in frame_block
    assert "frame_authority_context.safetyAuthorized()" in frame_block


def test_cargo_rollback_and_yaw_rollback_have_deliberately_different_contracts() -> None:
    cargo_test = (
        ROOT
        / "src/ndt_slam/test/cargo_physical_identity_authority_test.cpp"
    ).read_text(encoding="utf-8")
    yaw_test = (
        ROOT / "src/ndt_slam/test/rail_localization_authority_test.cpp"
    ).read_text(encoding="utf-8")
    assert "TimestampRollbackClosesCargoPreLiftForCurrentEpoch" in cargo_test
    assert "CURRENT_EPOCH_PRELIFT_BLOCKED" in cargo_test
    assert "TimeRollbackPreservesRailYawAuthority" in yaw_test
    assert "EXPECT_EQ(authority.generation(), generation)" in yaw_test


def test_map_commit_provenance_is_copied_from_frame_context() -> None:
    source = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(
        encoding="utf-8"
    )
    start = source.index("void NdtSlamNode::enqueueMapCommitJob")
    end = source.index("void NdtSlamNode::mapCommitThread", start)
    enqueue = source[start:end]
    for field in (
        "map_rebuild_generation",
        "keyframe_pose_version",
        "yaw_authority_generation",
        "map_frame_uuid",
        "yaw_reference_hash",
        "target_snapshot_id",
    ):
        assert f"frame_context.pose_identity.{field}" in enqueue


def test_shadow_geometry_and_fusion_use_physical_cargo_epoch() -> None:
    source = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(
        encoding="utf-8"
    )
    start = source.index(
        "void NdtSlamNode::evaluateIntegratedCargoIdentityShadow"
    )
    end = source.index("void NdtSlamNode::", start + 20)
    evaluation = source[start:end]
    assert (
        "integrated_identity_decision_.physical_cargo_epoch_id"
        in evaluation
    )
    assert "integrated_identity_decision_.load_epoch" not in evaluation
