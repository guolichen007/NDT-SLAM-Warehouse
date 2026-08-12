import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8", errors="replace")


class CargoAvoidanceRefactorContractTest(unittest.TestCase):
    def test_baseline_census_and_public_transport_are_locked(self):
        census = read("docs/design/cargo_avoidance_architecture_census_7a7eb00.md")
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        launch = read(
            "src/ndt_slam/launch/warehouse_live_longterm_mapping.launch"
        )
        self.assertIn("7a7eb007e74c660e0fef584c8bad45b00acc8358", census)
        self.assertIn("runPendingCargoAvoidance", census)
        self.assertIn('"/cargo_avoidance/safety_status"', node)
        self.assertIn('"/cargo_avoidance/external_obstacle_cloud"', node)
        self.assertIn('"/cargo_avoidance/pending_external_shell_cloud"', node)
        self.assertIn('/cargo_avoidance/status_code', launch)
        self.assertIn('publish_legacy_alarm_topic" value="false', launch)

    def test_existing_golden_tests_cover_geometry_tracking_and_decision(self):
        geometry = read("src/ndt_slam/test/cargo_geometry_fusion_test.cpp")
        evaluator = read("src/ndt_slam/test/cargo_safety_evaluator_test.cpp")
        tracker = read("src/ndt_slam/test/cargo_obstacle_tracker_test.cpp")
        heartbeat = read(
            "src/ndt_slam/test/cargo_alarm_heartbeat_state_machine_test.cpp"
        )
        self.assertIn("FormalThicknessIsInvariantToPositiveOnlyPromotionPath", geometry)
        self.assertIn("PartialSideCannotFreezeFormalEnvelope", geometry)
        self.assertIn("ExactDistanceAndClearanceBoundaries", evaluator)
        self.assertIn("EntireClusterAboveCargoDoesNotWarn", evaluator)
        self.assertIn("GlobalAssociationIsInvariantToObservationInputOrder", tracker)
        self.assertIn("FarHistoryUsesDistanceMinusUncertainty", tracker)
        self.assertIn("DuplicateSourceStampCannotChangeFormalCode", heartbeat)

    def test_pending_path_is_observable_and_warms_formal_history(self):
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        start = node.index("void NdtSlamNode::runPendingCargoAvoidance(")
        end = node.index("void NdtSlamNode::cargoSwingHookAnchorCallback(", start)
        body = node[start:end]
        self.assertIn("pending_tracking_query_allowed", body)
        self.assertIn("cargo_pending_external_shell_pub_", body)
        self.assertIn("pending_cargo_obstacle_tracker_.update", body)
        self.assertIn("cargo_obstacle_tracker_.update", body)

    def test_physical_perception_is_separate_from_vertical_authority(self):
        evaluator = read(
            "src/ndt_slam/src/cargo_safety_evaluator.cpp"
        )
        tracker = read(
            "src/ndt_slam/include/ndt_slam/cargo_obstacle_tracker.hpp"
        )
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        self.assertIn("CargoSafetyEvaluator::perceive", evaluator)
        self.assertIn("hazard_geometry_valid", tracker)
        self.assertIn("formal_obstacle_perception", node)
        self.assertIn("external_live_perception", node)
        self.assertNotIn(
            "static_cast<bool>(observation_cloud_base) &&\n"
            "        last_cargo_bottom_result_.geometry_valid",
            node,
        )

    def test_invalid_policy_is_fail_closed_without_silent_defaults(self):
        tracker = read("src/ndt_slam/src/cargo_obstacle_tracker.cpp")
        temporal = read("src/ndt_slam/src/cargo_safety_temporal_filter.cpp")
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        self.assertNotIn(
            "validConfig(config) ? config : CargoObstacleTrackerConfig{}",
            tracker,
        )
        self.assertNotIn(
            "validConfig(config) ? config : CargoSafetyTemporalConfig{}",
            temporal,
        )
        self.assertIn("CargoConfigValidationResult", tracker)
        self.assertIn("refusing Cargo/avoidance authority with Code35", node)
        self.assertNotIn("restoring 3.0/5.0/0.80", node)
        self.assertIn("cargo_safety_config_error_detail_", node)


if __name__ == "__main__":
    unittest.main()
