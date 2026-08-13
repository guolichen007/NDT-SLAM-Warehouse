from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "src/ndt_slam/include/ndt_slam/ndt_slam.hpp").read_text(
    encoding="utf-8"
)
KEYFRAME = (ROOT / "src/ndt_slam/src/keyframe_manager.cpp").read_text(
    encoding="utf-8"
)
KEYFRAME_TEST = (
    ROOT / "src/ndt_slam/test/keyframe_manager_time_epoch_test.cpp"
).read_text(encoding="utf-8")


def section(start: str, end: str) -> str:
    first = NODE.index(start)
    last = NODE.index(end, first)
    return NODE[first:last]


class TimeEpochRecoveryContractTest(unittest.TestCase):
    def test_confirmed_outer_gate_is_rebased_without_pose_reset(self):
        rollback = section(
            "void NdtSlamNode::handleLidarTimeRollback(",
            "// CRITICAL RUNTIME CHAIN - DO NOT MODIFY",
        )
        self.assertIn("last_keyframe_time_for_gate_ = current_stamp", rollback)
        self.assertNotIn("last_keyframe_pose_for_gate_ =", rollback)
        self.assertIn("delta_translation_ = 0.0", rollback)
        self.assertIn("delta_yaw_ = 0.0", rollback)

    def test_keyframe_manager_reset_is_narrow_and_tested(self):
        reset = KEYFRAME.split(
            "void KeyFrameManager::resetTemporalGateForSourceEpoch", 1
        )[1].split("bool KeyFrameManager::saveKeyFrameDatabase", 1)[0]
        self.assertIn("last_keyframe_time_ = new_epoch_stamp", reset)
        for forbidden in (
            "keyframes_",
            "last_keyframe_id_",
            "last_keyframe_pose_",
            "spatial_index_",
        ):
            self.assertNotIn(forbidden, reset)
        self.assertIn("RebaseChangesOnlyTemporalEligibilityGate", KEYFRAME_TEST)

    def test_map_commit_has_dequeue_prewrite_and_completion_fences(self):
        worker = section(
            "void NdtSlamNode::mapCommitThread()",
            "void NdtSlamNode::consumeMapCommitCompletion()",
        )
        self.assertGreaterEqual(worker.count("job.lifecycle_epoch"), 4)
        self.assertGreaterEqual(
            worker.count("isMapCommitLifecycleCurrent"), 3
        )
        self.assertIn("map_commit_lifecycle_mutex_", worker)
        self.assertIn("completion_current", worker)
        self.assertIn("map_commit_stale_.fetch_add", worker)

    def test_epoch_transition_preserves_dirty_retry_authority(self):
        rollback = section(
            "void NdtSlamNode::handleLidarTimeRollback(",
            "// CRITICAL RUNTIME CHAIN - DO NOT MODIFY",
        )
        self.assertIn("map_commit_queue_.clear()", rollback)
        self.assertIn("stale_queued_jobs", rollback)
        self.assertNotIn("dirty_tiles_.clear()", rollback)
        self.assertNotIn("failed_tile_flush_batch_.clear()", rollback)
        self.assertIn("flush_tiles_pending_.store(true", rollback)

    def test_lock_order_and_worker_queue_release_contract(self):
        process = section(
            "void NdtSlamNode::processCloudThread()",
            "void NdtSlamNode::publishOdometry(",
        )
        rollback_call = process.index("handleLidarTimeRollback(")
        runtime_lock = process.index("runtime_state_mutex_")
        self.assertLess(runtime_lock, rollback_call)

        rollback = section(
            "void NdtSlamNode::handleLidarTimeRollback(",
            "// CRITICAL RUNTIME CHAIN - DO NOT MODIFY",
        )
        lifecycle = rollback.index("map_commit_lifecycle_mutex_")
        keyframe_lock_wrapper = rollback.index(
            "resetTemporalGateForSourceEpoch(current_stamp)"
        )
        queue_lock = rollback.index("map_commit_queue_mutex_")
        self.assertLess(lifecycle, keyframe_lock_wrapper)
        self.assertLess(keyframe_lock_wrapper, queue_lock)

    def test_process_schedules_are_steady_and_ros_flush_is_metadata_only(self):
        process = section(
            "void NdtSlamNode::processCloudThread()",
            "void NdtSlamNode::publishOdometry(",
        )
        self.assertIn("using DiagClock = std::chrono::steady_clock", process)
        self.assertIn("last_status_schedule_time", process)
        self.assertIn("last_flush_schedule_time", process)
        self.assertNotIn("ros::Time last_status_write_time", process)
        self.assertNotIn("ros::Time last_diag_health_time", process)
        self.assertNotIn("ros::Time last_flush_time_;", HEADER)
        self.assertIn("createWallTimer", NODE)
        self.assertNotIn("timer_ = nh_.createTimer", NODE)

    def test_per_epoch_diagnostics_and_status_liveness_fields_exist(self):
        for milestone in (
            "RESET_BEGIN",
            "MAP_STATE_PRESERVED",
            "MAPPING_TEMPORAL_STATE_RESET",
            "FIRST_NDT_AFTER_RESET",
            "FIRST_ACCEPT_AFTER_RESET",
            "MAP_COMMIT_REARM_AFTER_RESET",
            "FIRST_KEYFRAME_AFTER_RESET",
            "FIRST_TILE_FLUSH_AFTER_RESET",
        ):
            self.assertIn(milestone, NODE)
        for field in (
            "runtime_status_seq",
            "cloud_callback_count",
            "ndt_attempt_count",
            "accepted_localization_count",
            "keyframe_count",
            "map_commit_completed_count",
            "tile_flush_completed_count",
            "time_epoch_reset_count",
            "current_time_epoch_id",
        ):
            self.assertIn(f'\\"{field}\\"', NODE)
        rename = NODE.index("std::rename(tmp_file.c_str(), status_file.c_str())")
        sequence = NODE.index("runtime_status_seq_.store", rename)
        self.assertLess(rename, sequence)

    def test_reset_storm_detection_updates_previous_stamp_each_frame(self):
        process = section(
            "void NdtSlamNode::processCloudThread()",
            "void NdtSlamNode::publishOdometry(",
        )
        detection = process.index("const bool lidar_time_rollback")
        self.assertIn("isSourceTimestampRollback", process[detection:])
        reset = process.index("handleLidarTimeRollback(", detection)
        update = process.index(
            "last_processed_frame_stamp_ = msg->header.stamp", reset
        )
        self.assertLess(detection, reset)
        self.assertLess(reset, update)


if __name__ == "__main__":
    unittest.main()
