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

    def test_LiveCleanPreviewIsDisplayOnlyAndDoesNotRelaxPersistence(self):
        self.assertIn("ros::Publisher objects_clean_live_pub_;", HEADER)
        self.assertIn('"/display_map_objects_clean_live"', NODE)
        self.assertIn('live_clean_message.header.frame_id = "base_link";', NODE)
        self.assertIn(
            "*registration_partition.static_objects,\n"
            "                live_clean_message",
            NODE,
        )
        self.assertIn(
            "const bool localization_authorized =\n"
            "        relocalization_pose_reliable_",
            NODE,
        )
        self.assertIn("Topic: /display_map_objects_clean_live", RVIZ)

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
