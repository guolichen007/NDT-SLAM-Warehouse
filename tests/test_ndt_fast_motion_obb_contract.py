from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(encoding="utf-8")
NODE_HEADER = (
    ROOT / "src/ndt_slam/include/ndt_slam/ndt_slam.hpp"
).read_text(encoding="utf-8")
EKF = (ROOT / "src/ndt_slam/src/crane_motion_ekf.cpp").read_text(
    encoding="utf-8"
)
EKF_HEADER = (
    ROOT / "src/ndt_slam/include/ndt_slam/crane_motion_ekf.hpp"
).read_text(encoding="utf-8")
OBB = (
    ROOT / "src/ndt_slam/src/cargo_oriented_footprint.cpp"
).read_text(encoding="utf-8")
OBB_HEADER = (
    ROOT / "src/ndt_slam/include/ndt_slam/cargo_oriented_footprint.hpp"
).read_text(encoding="utf-8")
TRACK_POLICY = (
    ROOT / "src/ndt_slam/src/cargo_track_policy.cpp"
).read_text(encoding="utf-8")
CONFIG = (
    ROOT / "src/ndt_slam/config/live_longterm_mapping.yaml"
).read_text(encoding="utf-8")
RVIZ = (ROOT / "src/ndt_slam/launch/rviz.rviz").read_text(encoding="utf-8")


class NdtFastMotionObbContractTest(unittest.TestCase):
    def test_raw_initial_guess_correction_is_not_a_vehicle_step_gate(self):
        self.assertNotIn("isNonPhysicalStep", NODE)
        self.assertNotIn("isNonPhysicalStep", EKF_HEADER)
        self.assertIn("correction_nominal_limit_m = 0.35", EKF_HEADER)
        self.assertIn("correction_soft_limit_m = 1.00", EKF_HEADER)
        self.assertIn("NDT_CORRECTION_HARD_LIMIT", EKF)
        self.assertIn("status_.correction_soft", EKF)

    def test_soft_output_step_blocks_map_commit_without_rejecting_odom(self):
        self.assertIn("output_soft_limit_ratio = 1.50", EKF_HEADER)
        self.assertIn("status_.output_step_soft = true", EKF)
        self.assertIn("status_.map_commit_safe", EKF)
        self.assertIn("crane_motion_ekf_.status().map_commit_safe", NODE)
        self.assertIn("absolute_output_step_limit_m: 2.50", CONFIG)

    def test_vehicle_yaw_never_rejects_xy_and_only_releases_on_recovery(self):
        self.assertNotIn("checkRuntimeYaw", NODE)
        self.assertNotIn("checkRuntimeYaw", EKF_HEADER)
        self.assertIn("observeRuntimeYaw", NODE)
        self.assertIn("XY measurement retained", NODE)
        self.assertIn("rigidly mounted on a rail crane", EKF)
        self.assertIn("Preserve the six-frame acquired heading exactly", EKF)
        self.assertIn("crane_motion_ekf_.unlatchYaw();", NODE)
        self.assertIn("accepted_yaw_valid_ = false;", NODE)

    def test_runtime_relocalization_is_confirmed_and_reseeded_once(self):
        self.assertIn("confirm_frames: 3", CONFIG)
        self.assertIn("relocalization_reseeded_this_episode_", NODE_HEADER)
        self.assertIn("reseedFromRelocalization", EKF_HEADER)
        self.assertIn("RELOCALIZATION_RESEED_VERIFYING", EKF)
        self.assertIn(
            "else if (!relocalization_reseeded_this_episode_)", NODE
        )
        self.assertIn("relocalization_reseed_already_applied", NODE)

    def test_anchor_grid_obb_rejects_disconnected_growth(self):
        self.assertIn("refineCargoAnchorGridFootprint", OBB_HEADER)
        self.assertIn("Drop isolated single cells", OBB)
        self.assertIn("best_distance", OBB)
        self.assertIn("maximum_growth_ratio = 1.20F", OBB_HEADER)
        self.assertIn("center_x = cx;", NODE)
        self.assertIn("center_y = cy;", NODE)
        self.assertIn("xy_percentile_low: 0.03", CONFIG)
        self.assertIn("xy_percentile_high: 0.97", CONFIG)
        self.assertIn("percentile_low: 0.08", CONFIG)
        self.assertIn("percentile_high: 0.92", CONFIG)
        self.assertIn("tight_box.xy_percentile_low", NODE)
        self.assertIn("tight_box.percentile_low", NODE)
        self.assertIn(
            "candidate_footprint_config.percentile_low =\n"
            "        odom_anchor_config_.tight_box.xy_percentile_low;",
            NODE,
        )
        self.assertIn(
            "const float z_low = odom_anchor_config_.tight_box.percentile_low;",
            NODE,
        )
        self.assertIn("margin_xy_m: 0.10", CONFIG)

    def test_live_obb_uses_multiframe_evidence_and_base_link(self):
        self.assertIn("minimum_valid_frames = 3U", OBB_HEADER)
        self.assertIn("expansion_valid_frames = 5U", OBB_HEADER)
        self.assertIn("shrink_valid_frames = 7U", OBB_HEADER)
        self.assertIn('"/cargo_avoidance/live_obb_marker"', NODE)
        self.assertIn('live_marker.header.frame_id = "base_link"', NODE)
        self.assertIn("live_marker.frame_locked = true", NODE)
        self.assertIn("getCargoAnchorXY()", NODE)
        self.assertIn("size_mad", OBB_HEADER)
        self.assertIn("3.0F * size_mad", OBB)

    def test_new_cargo_obb_requires_loaded_hook_and_anchor_identity(self):
        self.assertIn("gravity_identity_required", NODE)
        self.assertIn("gravity_loaded_now", NODE)
        self.assertIn("valid gravity LOADED required", NODE)
        self.assertIn("require_hook_containment = true", NODE)
        self.assertIn("maximum_hook_center_distance_m", NODE)
        self.assertIn("hook_anchor_outside_candidate_obb", TRACK_POLICY)
        self.assertIn("candidate_center_too_far_from_hook_anchor", TRACK_POLICY)
        self.assertIn("configuration-independent invariant", NODE)
        self.assertIn("measured.x() = anchor.x();", NODE)
        self.assertIn("measured.y() = anchor.y();", NODE)

    def test_cargo_yaw_expands_xy_without_replacing_height_fusion(self):
        self.assertIn("projected_live_length", NODE)
        self.assertIn("projected_live_width", NODE)
        projection = NODE[NODE.index("projected_live_length") :]
        projection = projection[:2200]
        self.assertNotIn("height_m = live_obb", projection)
        self.assertNotIn("bottom_z = live_obb", projection)

    def test_objects_clean_is_latched_sealed_map_snapshot_only(self):
        self.assertNotIn("objects_clean_live", NODE)
        self.assertNotIn("objects_clean_live", NODE_HEADER)
        self.assertNotIn("objects_clean_live", RVIZ)
        self.assertIn("Topic: /display_map_objects_clean", RVIZ)
        self.assertIn("latest_completed_map_bundle_ = result.bundle", NODE)


if __name__ == "__main__":
    unittest.main()
