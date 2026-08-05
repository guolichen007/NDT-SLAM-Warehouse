import importlib.util
from pathlib import Path
import sys
import tempfile
import threading
import unittest
import json
from unittest import mock


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

    def test_ResponsiveDegradationNeverRequestsBlindHardRestart(self):
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
            policy.observe(high_bad, 200.0).action, "none"
        )

    def test_ProcessFaultUsesBoundedRestartBudget(self):
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
        decision = policy.process_fault(103.0, "health_stream_stale")
        self.assertEqual(decision.action, "restart_suppressed")
        self.assertEqual(
            policy.process_fault(104.0, "health_stream_stale").action,
            "none",
        )

    def test_LocalizationHealthRequiresSchemaAndStrictFlags(self):
        health = WATCHDOG.parse_localization_health(
            '{"schema_version":1,"startup_state":"VERIFYING",'
            '"relocalization_state":"COOLDOWN","pose_reliable":false,'
            '"strict_verified":false,"reason":"strict_frame_qualified"}'
        )
        self.assertIsNotNone(health)
        self.assertEqual(health.startup_state, "VERIFYING")
        self.assertFalse(health.pose_reliable)
        self.assertIsNone(
            WATCHDOG.parse_localization_health(
                '{"schema_version":2,"startup_state":"HEALTHY"}'
            )
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

    def test_EventLogRotationIsBounded(self):
        with tempfile.TemporaryDirectory() as directory:
            event_path = Path(directory) / "events.jsonl"
            WATCHDOG.append_jsonl(
                event_path, {"event": "first"}, max_bytes=1, backups=2
            )
            WATCHDOG.append_jsonl(
                event_path, {"event": "second"}, max_bytes=1, backups=2
            )
            self.assertTrue(
                event_path.with_name("events.jsonl.1").exists()
            )
            self.assertLessEqual(
                len(list(Path(directory).glob("events.jsonl*"))), 3
            )

    def test_HardRestartRequestIsAtomicAndGenerationBound(self):
        class FakeRospy:
            @staticmethod
            def logerr(*_args):
                pass

            @staticmethod
            def logfatal(*_args):
                pass

        with tempfile.TemporaryDirectory() as directory:
            watchdog = WATCHDOG.RosRecoveryWatchdog.__new__(
                WATCHDOG.RosRecoveryWatchdog
            )
            root = Path(directory)
            watchdog.rospy = FakeRospy()
            watchdog.health = None
            watchdog.health_received_at = None
            watchdog.supervisor_run_id = "run-generation-7"
            watchdog.state_path = root / "state.json"
            watchdog.events_path = root / "events.jsonl"
            watchdog.restart_request_path = root / "restart_request.json"
            watchdog.policy = WATCHDOG.RecoveryWatchdogPolicy(
                WATCHDOG.WatchdogConfig(startup_grace_sec=0.0), 1.0
            )
            watchdog.restart_requested = threading.Event()
            decision = watchdog.policy.process_fault(
                10.0, "localization_health_stream_stale"
            )
            status = WATCHDOG.RecoveryStatus(
                "UNKNOWN", 0, "state=UNKNOWN bad_frames=0"
            )
            watchdog._request_hard_restart(decision, status, 10.0)
            request = json.loads(
                watchdog.restart_request_path.read_text(encoding="utf-8")
            )
            self.assertEqual(
                request["supervisor_run_id"], "run-generation-7"
            )
            self.assertEqual(request["action"], "hard_restart")
            self.assertTrue(watchdog.restart_requested.is_set())
            self.assertFalse(
                watchdog.restart_request_path.with_suffix(
                    ".json.tmp"
                ).exists()
            )

    def test_HardRestartReturns75FromMainThreadAfterCleanup(self):
        calls = []

        class FakeTimer:
            def __init__(self):
                self.shutdown_called = False

            def shutdown(self):
                self.shutdown_called = True
                calls.append("timer.shutdown")

        class FakeRospy:
            def __init__(self):
                self.shutdown_reason = ""

            def is_shutdown(self):
                return False

            def signal_shutdown(self, reason):
                self.shutdown_reason = reason
                calls.append("rospy.signal_shutdown")

        watchdog = WATCHDOG.RosRecoveryWatchdog.__new__(
            WATCHDOG.RosRecoveryWatchdog
        )
        watchdog.rospy = FakeRospy()
        watchdog.timer = FakeTimer()
        watchdog.restart_requested = threading.Event()
        watchdog.restart_requested.set()

        self.assertEqual(watchdog.run(), 75)
        self.assertTrue(watchdog.timer.shutdown_called)
        self.assertEqual(
            watchdog.rospy.shutdown_reason,
            "persistent localization failure",
        )
        self.assertEqual(
            calls, ["timer.shutdown", "rospy.signal_shutdown"]
        )

    def test_ExternalShutdownReturnsZeroWithoutRestartCleanup(self):
        class UnexpectedTimer:
            def shutdown(self):
                raise AssertionError(
                    "external shutdown must not stop the timer twice"
                )

        class FakeRospy:
            def __init__(self):
                self.shutdown_reason = None

            def is_shutdown(self):
                return True

            def signal_shutdown(self, reason):
                self.shutdown_reason = reason

        watchdog = WATCHDOG.RosRecoveryWatchdog.__new__(
            WATCHDOG.RosRecoveryWatchdog
        )
        watchdog.rospy = FakeRospy()
        watchdog.timer = UnexpectedTimer()
        watchdog.restart_requested = threading.Event()

        self.assertEqual(watchdog.run(), 0)
        self.assertIsNone(watchdog.rospy.shutdown_reason)

    def test_TimerCallbackStopsAfterRestartWasRequested(self):
        class UnexpectedRospy:
            def __getattr__(self, name):
                raise AssertionError(
                    f"timer callback continued via rospy.{name}"
                )

        watchdog = WATCHDOG.RosRecoveryWatchdog.__new__(
            WATCHDOG.RosRecoveryWatchdog
        )
        watchdog.rospy = UnexpectedRospy()
        watchdog.status = object()
        watchdog.restart_requested = threading.Event()
        watchdog.restart_requested.set()

        watchdog._timer_callback(None)

    def test_LiveLegacyProcessingSuppressesMissingHealthRestartLoop(self):
        warnings = []

        class FakeRospy:
            @staticmethod
            def logwarn_throttle(period, message):
                warnings.append((period, message))

        watchdog = WATCHDOG.RosRecoveryWatchdog.__new__(
            WATCHDOG.RosRecoveryWatchdog
        )
        watchdog.rospy = FakeRospy()
        watchdog.health = None
        watchdog.health_received_at = None
        watchdog.status = WATCHDOG.RecoveryStatus(
            "DEGRADED", 40, "state=DEGRADED bad_frames=40"
        )
        watchdog.restart_requested = threading.Event()
        watchdog.policy = WATCHDOG.RecoveryWatchdogPolicy(
            WATCHDOG.WatchdogConfig(
                startup_grace_sec=0.0, health_stale_sec=3.0
            ),
            1.0,
        )

        with mock.patch.object(WATCHDOG.time, "time", return_value=10.0):
            watchdog.status_received_at = 1.0
            watchdog.odom_received_at = 9.0
            watchdog._timer_callback(None)

        self.assertFalse(watchdog.restart_requested.is_set())
        self.assertEqual(len(warnings), 1)
        self.assertIn("rebuild the workspace", warnings[0][1])


if __name__ == "__main__":
    unittest.main()
