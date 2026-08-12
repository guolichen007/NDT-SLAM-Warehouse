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
        self.assertIn("physical_obstacle_track_store_.update", body)
        self.assertNotIn("pending_cargo_obstacle_tracker_", node)

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
        self.assertIn("canonical_perception", node)
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

    def test_ambiguous_association_cannot_transfer_authority(self):
        tracker = read("src/ndt_slam/src/cargo_obstacle_tracker.cpp")
        tracker_test = read("src/ndt_slam/test/cargo_obstacle_tracker_test.cpp")
        self.assertIn("reciprocal_unique_match", tracker)
        self.assertIn("ambiguous_non_reciprocal_authority_frozen", tracker)
        self.assertIn("current_source_validated = false", tracker)
        self.assertIn("current_warning_eligible = false", tracker)
        self.assertIn(
            "TwoByTwoEqualCostAssociationFreezesAllAuthorityMaturity",
            tracker_test,
        )
        self.assertIn(
            "AmbiguousFarSamplesCannotTransferOrAdvanceFarHistory",
            tracker_test,
        )

    def test_time_sensitive_domain_contracts_are_explicit(self):
        contracts = read(
            "src/ndt_slam/include/ndt_slam/cargo_domain_contracts.hpp"
        )
        for contract in (
            "CargoTrackSnapshot",
            "CargoGeometryEstimate",
            "CargoSafetyEnvelope",
            "ObstacleObservation",
            "ObstacleTrackSnapshot",
            "HazardAssessment",
            "AvoidanceDecision",
        ):
            self.assertIn(f"struct {contract}", contracts)
        for field in (
            "source_stamp_sec",
            "frame_id",
            "cargo_lifecycle_id",
            "uncertainty",
            "source",
            "valid",
            "fresh",
        ):
            self.assertIn(field, contracts)

    def test_capability_is_pure_and_positive_only_cannot_clear(self):
        capability = read("src/ndt_slam/src/cargo_capability.cpp")
        capability_test = read("src/ndt_slam/test/cargo_capability_test.cpp")
        self.assertIn("deriveCargoCapability", capability)
        self.assertNotIn("ros::", capability)
        self.assertIn("VerticalInvalidPreservesPhysicalTrackingOnly", capability_test)
        self.assertIn("PositiveOnlyCanWarnButCannotClearRemoveOrMap", capability_test)

    def test_formal_pending_share_phase_neutral_canonical_perception(self):
        perception = read(
            "src/ndt_slam/include/ndt_slam/obstacle_perception.hpp"
        )
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        evaluator_test = read(
            "src/ndt_slam/test/cargo_safety_evaluator_test.cpp"
        )
        self.assertNotIn("perception_phase", perception)
        self.assertNotIn("formal_phase", perception)
        self.assertNotIn("pending_phase", perception)
        self.assertEqual(node.count("canonical_perception ="), 1)
        self.assertIn(
            "FormalPendingTransportLabelsCannotChangeCanonicalPerception",
            evaluator_test,
        )

    def test_physical_history_has_one_stateful_owner(self):
        header = read("src/ndt_slam/include/ndt_slam/ndt_slam.hpp")
        tracker = read(
            "src/ndt_slam/include/ndt_slam/cargo_obstacle_tracker.hpp"
        )
        tracker_test = read(
            "src/ndt_slam/test/cargo_obstacle_tracker_test.cpp"
        )
        self.assertEqual(header.count("PhysicalObstacleTrackStore "), 1)
        self.assertNotIn("pending_cargo_obstacle_tracker_", header)
        self.assertIn("CargoObstacleAuthorityPolicy", tracker)
        self.assertIn(
            "PendingToFormalChangesAuthorityWithoutDuplicatingHistory",
            tracker_test,
        )

    def test_hazard_and_final_protocol_have_explicit_single_owners(self):
        evaluator = read("src/ndt_slam/src/cargo_safety_evaluator.cpp")
        hazard = read("src/ndt_slam/src/hazard_evaluator.cpp")
        decision = read("src/ndt_slam/src/avoidance_decision.cpp")
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        self.assertIn("HazardEvaluator", evaluator)
        self.assertIn("conservative_clearance_m", hazard)
        self.assertNotIn("composeCargoSafetyDecision", evaluator)
        self.assertEqual(decision.count("composeCargoSafetyDecision("), 1)
        self.assertIn("avoidance_decision_owner_.decide", node)

    def test_operational_diagnostics_are_read_only_post_decision_snapshots(self):
        diagnostics = read(
            "src/ndt_slam/include/ndt_slam/avoidance_diagnostics.hpp"
        )
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        for field in (
            "perception_phase",
            "perception_executed",
            "query_horizontal_valid",
            "query_vertical_valid",
            "external_point_count",
            "cluster_count",
            "observation_count",
            "warning_authority_valid",
            "block_reason",
        ):
            self.assertIn(field, diagnostics)
            self.assertIn(f'\\"{field}\\"', node)
        self.assertIn("avoidance_diagnostics_.replace", node)
        self.assertNotIn("requested_code", diagnostics)
        self.assertNotIn("warning_code", diagnostics)

    def test_build_has_four_in_process_core_boundaries_and_one_ros_node(self):
        cmake = read("src/ndt_slam/CMakeLists.txt")
        for target in (
            "ndt_localization_core",
            "ndt_cargo_core",
            "ndt_avoidance_core",
            "ndt_runtime_support",
        ):
            self.assertIn(f"add_library({target} OBJECT", cmake)
            self.assertIn(f"$<TARGET_OBJECTS:{target}>", cmake)
        self.assertEqual(cmake.count("add_executable(ndt_slam_node"), 1)
        self.assertIn("add_library(ndt_slam_lib SHARED", cmake)

    def test_cargo_safety_is_the_only_effective_threshold_snapshot(self):
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        config = read("src/ndt_slam/config/live_longterm_mapping.yaml")
        self.assertIn(
            'cargo_safety["cargo_bottom_extra_margin_m"]', node
        )
        self.assertIn(
            'cargo_safety["obstacle_min_cluster_points"]', node
        )
        self.assertIn("compatibility mirror", node)
        self.assertIn("only effective Cargo/avoidance policy", config)

    def test_explicit_out_of_range_policy_fails_before_defensive_bounds(self):
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        validation = node.index("Validate every explicitly supplied scalar")
        first_clamp = node.index(
            "static_map_config.minimum_cell_overlap = std::clamp"
        )
        self.assertLess(validation, first_clamp)
        for key in (
            "static_map_minimum_cell_overlap",
            "fusion_pending_minimum_authority_confidence",
            "motion_corridor_forward_half_angle_deg",
            "residual_minimum_motion_match_score",
        ):
            self.assertIn(f'validate_range("{key}"', node)
        self.assertIn("cargo_safety_config_error_detail_", node)


if __name__ == "__main__":
    unittest.main()
