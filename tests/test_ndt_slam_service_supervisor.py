from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (
    ROOT / "src/ndt_slam/scripts/ops/ndt_slam_service_supervisor.py"
)
SPEC = importlib.util.spec_from_file_location(
    "ndt_slam_service_supervisor", SCRIPT
)
SUPERVISOR = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = SUPERVISOR
SPEC.loader.exec_module(SUPERVISOR)

SERVICE = (
    ROOT / "src/ndt_slam/scripts/ops/ndt-slam.service.in"
).read_text(encoding="utf-8")
INSTALLER = (
    ROOT / "src/ndt_slam/scripts/ops/install_server_services.sh"
).read_text(encoding="utf-8")
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(
    encoding="utf-8"
)


class NdtSlamServiceSupervisorTest(unittest.TestCase):
    def test_RecoverableTrackingFailureMapsToExit75(self) -> None:
        self.assertEqual(
            SUPERVISOR.classified_exit_code(
                3, "RECOVERABLE_TRACKING_DEGRADATION", True
            ),
            75,
        )

    def test_NonrecoverableFaultsMapToExit78(self) -> None:
        for failure_class in (
            "NONRECOVERABLE_REFERENCE_CONFIG",
            "NONRECOVERABLE_MAP_IDENTITY",
            "INTERNAL_CONTRACT_ERROR",
        ):
            with self.subTest(failure_class=failure_class):
                self.assertEqual(
                    SUPERVISOR.classified_exit_code(
                        3, failure_class, True
                    ),
                    78,
                )

    def test_StartupFailureWithoutFreshStatusIsNonrecoverable(self) -> None:
        self.assertEqual(
            SUPERVISOR.classified_exit_code(3, "", False), 78
        )
        self.assertEqual(
            SUPERVISOR.classified_exit_code(0, "", False), 0
        )

    def test_StalePriorProcessStatusCannotClassifyNewProcess(self) -> None:
        baseline = SUPERVISOR.StatusSnapshot(
            sequence=9,
            modified_ns=100,
            failure_class="NONRECOVERABLE_MAP_IDENTITY",
            payload={},
        )
        same = SUPERVISOR.StatusSnapshot(
            sequence=9,
            modified_ns=100,
            failure_class="NONRECOVERABLE_MAP_IDENTITY",
            payload={},
        )
        restarted = SUPERVISOR.StatusSnapshot(
            sequence=1,
            modified_ns=101,
            failure_class="NONE",
            payload={},
        )
        self.assertFalse(SUPERVISOR.status_is_fresh(same, baseline))
        self.assertTrue(SUPERVISOR.status_is_fresh(restarted, baseline))

    def test_RuntimeStatusRoundTripsFailureClass(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime_status.json"
            path.write_text(
                json.dumps(
                    {
                        "runtime_status_seq": 4,
                        "localization_failure_class":
                            "NONRECOVERABLE_REFERENCE_CONFIG",
                    }
                ),
                encoding="utf-8",
            )
            snapshot = SUPERVISOR.read_status_snapshot(path)
            self.assertIsNotNone(snapshot)
            self.assertEqual(snapshot.sequence, 4)
            self.assertEqual(
                snapshot.failure_class,
                "NONRECOVERABLE_REFERENCE_CONFIG",
            )

    def test_ServicePreventsRestartForExit78Only(self) -> None:
        unit, service = SERVICE.split("[Service]", 1)
        self.assertIn("StartLimitIntervalSec=300", unit)
        self.assertIn("StartLimitBurst=5", unit)
        self.assertIn("Restart=on-failure", service)
        self.assertIn("RestartPreventExitStatus=78", service)
        self.assertNotIn("Restart=always", service)
        self.assertIn("ndt_slam_service_supervisor.py", service)
        self.assertIn("service_exit_classification.json", service)

    def test_InstallerAuditsEffectiveRestartContract(self) -> None:
        self.assertIn(
            "require_effective_exact ndt-slam.service Restart on-failure",
            INSTALLER,
        )
        self.assertIn(
            "require_effective_exact ndt-slam.service "
            "RestartPreventExitStatus 78",
            INSTALLER,
        )

    def test_RelocalizationAndRuntimeStatusPublishFailureClass(self) -> None:
        self.assertIn(
            '" failure_class=" + localizationFailureClassName', NODE
        )
        self.assertIn("localization_failure_class", NODE)


if __name__ == "__main__":
    unittest.main()
