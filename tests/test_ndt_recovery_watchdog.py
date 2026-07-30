import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (
    ROOT / "src/ndt_slam/scripts/ops/ndt_recovery_watchdog.py"
)
SPEC = importlib.util.spec_from_file_location(
    "ndt_recovery_watchdog", SCRIPT
)
WATCHDOG = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = WATCHDOG
SPEC.loader.exec_module(WATCHDOG)


class NdtRecoveryWatchdogTest(unittest.TestCase):
    def test_ParseRelocalizationStatusWithCompoundDetail(self):
        status = WATCHDOG.parse_relocalization_status(
            "state=SEARCHING_GLOBAL detail=target=objects_clean_static "
            "candidates=48 bad_frames=411"
        )
        self.assertIsNotNone(status)
        self.assertEqual(status.state, "SEARCHING_GLOBAL")
        self.assertEqual(status.bad_frames, 411)

    def test_SoftRecoveryPrecedesBoundedHardRestart(self):
        config = WATCHDOG.WatchdogConfig(
            startup_grace_sec=5.0,
            soft_relocalize_after_sec=8.0,
            hard_restart_after_sec=45.0,
            hard_restart_bad_frames=300,
            hard_restart_bad_frames_after_sec=15.0,
        )
        policy = WATCHDOG.RecoveryWatchdogPolicy(config, 100.0)
        degraded = WATCHDOG.RecoveryStatus(
            "DEGRADED", 20, "state=DEGRADED bad_frames=20"
        )
        self.assertEqual(policy.observe(degraded, 101.0).action, "none")
        self.assertEqual(
            policy.observe(degraded, 109.1).action, "soft_relocalize"
        )
        high_bad = WATCHDOG.RecoveryStatus(
            "SEARCHING_GLOBAL", 350,
            "state=SEARCHING_GLOBAL bad_frames=350",
        )
        self.assertEqual(
            policy.observe(high_bad, 116.0).action, "hard_restart"
        )

    def test_RestartBudgetSuppressesRestartStorm(self):
        config = WATCHDOG.WatchdogConfig(
            startup_grace_sec=0.0,
            soft_relocalize_after_sec=1.0,
            hard_restart_after_sec=2.0,
            restart_window_sec=900.0,
            max_restarts_in_window=3,
        )
        policy = WATCHDOG.RecoveryWatchdogPolicy(
            config, 100.0, (10.0, 50.0, 99.0)
        )
        degraded = WATCHDOG.RecoveryStatus(
            "DEGRADED", 5, "state=DEGRADED bad_frames=5"
        )
        policy.observe(degraded, 100.0)
        decision = policy.observe(degraded, 103.0)
        self.assertEqual(decision.action, "restart_suppressed")
        self.assertEqual(
            policy.observe(degraded, 104.0).action, "none"
        )

    def test_EvidenceWritersUseLfAndRoundTripHistory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            event_path = root / "events.jsonl"
            state_path = root / "state.json"
            WATCHDOG.append_jsonl(event_path, {"action": "hard_restart"})
            WATCHDOG.atomic_write_json(
                state_path, {"restart_history": [1.0, 2.0]}
            )
            self.assertNotIn(b"\r\n", event_path.read_bytes())
            self.assertNotIn(b"\r\n", state_path.read_bytes())
            self.assertEqual(
                WATCHDOG.read_restart_history(state_path), [1.0, 2.0]
            )


if __name__ == "__main__":
    unittest.main()
