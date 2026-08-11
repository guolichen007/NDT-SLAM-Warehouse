import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "ndt_slam"


def read(path):
    return path.read_text(encoding="utf-8")


class StartupRecoveryV1ContractTest(unittest.TestCase):
    def test_required_modules_are_built(self):
        cmake = read(PKG / "CMakeLists.txt")
        for module in (
            "durable_map_store",
            "recovery_checkpoint",
            "accepted_keyframe_journal",
            "crane_place_descriptor",
            "crane_startup_relocalizer",
            "startup_recovery_controller",
            "map_write_rearm_policy",
        ):
            self.assertIn(f"src/{module}.cpp", cmake)
            self.assertIn(f"test/{module}_test.cpp", cmake)

    def test_controller_has_only_automatic_states(self):
        header = read(
            PKG / "include" / "ndt_slam" / "startup_recovery_controller.hpp"
        )
        for state in (
            "BOOT",
            "LOAD_REFERENCE",
            "SENSOR_WARMUP",
            "LOCAL_RECOVERY",
            "GLOBAL_RECOVERY",
            "VERIFYING",
            "READONLY_STABILIZING",
            "ACTIVE",
            "REFERENCE_INVALID",
            "RECOVERY_RETRY",
        ):
            self.assertIn(state, header)
        self.assertNotIn("WAIT_OPERATOR", header)
        self.assertNotIn("WAIT_STATIONARY", header)
        self.assertNotIn("WAITING_STATIONARY", header)

    def test_single_map_authority_contains_recovery_gates(self):
        header = read(PKG / "include" / "ndt_slam" / "map_write_authority.hpp")
        source = read(PKG / "src" / "map_write_authority.cpp")
        self.assertIn("startup_recovery_verified", header)
        self.assertIn("map_write_rearmed", header)
        self.assertIn("startup_recovery_not_verified", source)
        self.assertIn("map_write_not_rearmed", source)
        self.assertNotIn("RecoveryMapWriteAuthority", source)

    def test_durable_store_uses_generation_pointers_and_fsync(self):
        source = read(PKG / "src" / "durable_map_store.cpp")
        for value in (
            '"CURRENT"',
            '"PREVIOUS"',
            '"PREVIOUS_2"',
            '"generations"',
            '"staging"',
            "::fsync",
            "MapSessionSnapshot::loadVerified",
        ):
            self.assertIn(value, source)

    def test_checkpoint_cannot_promote_pose(self):
        header = read(PKG / "include" / "ndt_slam" / "recovery_checkpoint.hpp")
        self.assertIn("loadVerified", header)
        self.assertNotIn("AcceptedPose", header)

    def test_runtime_loss_recovery_is_not_enabled_in_v1(self):
        node = read(PKG / "src" / "ndt_slam.cpp")
        self.assertIn("This branch is startup recovery only", node)
        self.assertNotIn("StartupLocalizationState::WAITING_STATIONARY", node)
        self.assertIn('"ACTIVE_ROOT"', node)
        self.assertIn('"isolated/" + localization_map_uuid_', node)

    def test_checkpoint_recovery_is_local_before_global_fallback(self):
        node = read(PKG / "src" / "ndt_slam.cpp")
        self.assertIn(
            '"persistent_map_requires_checkpoint_local_search"', node
        )
        self.assertIn("local_seed_pose", node)
        self.assertIn("localization_checkpoint_pose_", node)
        self.assertIn("local_failed.local_recovery_succeeded = false", node)

    def test_verified_journal_extends_readonly_recovery_reference(self):
        node = read(PKG / "src" / "ndt_slam.cpp")
        header = read(PKG / "include" / "ndt_slam" / "ndt_slam.hpp")
        self.assertIn("loadRecoveryJournalReference", node)
        self.assertIn("AcceptedKeyframeJournal::loadLastVerified", node)
        self.assertIn('"durable_plus_verified_journal"', node)
        self.assertIn("recovery_reference_snapshot_", header)

    def test_service_restarts_crashes_only_and_watchdog_is_off(self):
        service = read(PKG / "scripts" / "ops" / "ndt-slam.service.in")
        self.assertIn("Restart=on-failure", service)
        self.assertIn("RestartSec=5", service)
        self.assertIn("use_ndt_recovery_watchdog:=false", service)
        self.assertNotIn("Restart=always", service)

if __name__ == "__main__":
    unittest.main()
