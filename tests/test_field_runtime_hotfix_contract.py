from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(
    encoding="utf-8"
)
HEADER = (ROOT / "src/ndt_slam/include/ndt_slam/ndt_slam.hpp").read_text(
    encoding="utf-8"
)
CONFIG = (ROOT / "src/ndt_slam/config/live_longterm_mapping.yaml").read_text(
    encoding="utf-8"
)
LAUNCH = (
    ROOT / "src/ndt_slam/launch/warehouse_live_longterm_mapping.launch"
).read_text(encoding="utf-8")
RVIZ = (ROOT / "src/ndt_slam/launch/rviz.rviz").read_text(
    encoding="utf-8"
)


class FieldRuntimeHotfixContractTest(unittest.TestCase):
    def test_OrientedFootprintCannotBypassStrictOdomAnchor(self):
        self.assertIn('anchor_symmetry_mode = "strict"', HEADER)
        self.assertIn('anchor_symmetry_mode: "strict"', CONFIG)
        self.assertIn("center_x = cx;", NODE)
        self.assertIn("center_y = cy;", NODE)
        self.assertIn("result.footprint_length_width = Eigen::Vector2f(sx, sy);", NODE)
        self.assertIn("hook_lock_.live_pose_measured_base = raw_measured;", NODE)

    def test_CleanMapUsesOnlySealedMapFrameSnapshot(self):
        self.assertNotIn("objects_clean_live", HEADER)
        self.assertNotIn("objects_clean_live", NODE)
        self.assertNotIn("objects_clean_live", RVIZ)
        self.assertIn('"/display_map_objects_clean", 1, true', NODE)
        self.assertIn("Topic: /display_map_objects_clean", RVIZ)
        self.assertIn("latest_completed_map_bundle_ = result.bundle;", NODE)
        self.assertNotIn("const bool localization_authorized", NODE)
        self.assertNotIn("const bool health_allows_commit", NODE)
        self.assertIn("result.bundle.lifecycle_epoch", NODE)
        self.assertIn("result.source_objects_version", NODE)

    def test_WatchdogUsesActualGlobalServiceAndBoundedRetries(self):
        self.assertIn(
            '<param name="relocalize_service" value="/relocalize"/>',
            LAUNCH,
        )
        self.assertIn(
            '<param name="service_failure_threshold" value="3"/>',
            LAUNCH,
        )
        self.assertIn("request_coalesced_active_episode", NODE)
        self.assertIn("crane_motion_ekf_.unlatchYaw();", NODE)


if __name__ == "__main__":
    unittest.main()
