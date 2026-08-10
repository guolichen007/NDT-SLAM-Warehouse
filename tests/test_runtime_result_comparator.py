import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from scripts.regression.compare_runtime_results import (
    HARD_MAXIMUMS,
    HIGHER_IS_BETTER,
    LOWER_IS_BETTER,
)


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "regression" / "compare_runtime_results.py"


class RuntimeResultComparatorTest(unittest.TestCase):
    def metrics(self):
        value = {key: 1.0 for key in LOWER_IS_BETTER}
        value.update({key: 1.0 for key in HIGHER_IS_BETTER})
        value.update(
            {
                "crash_count": 0,
                "objects_clean_height_spike_count": 0,
                "nearby_object_identity_steal_count": 0,
                "worker_starvation_count": 0,
                "archive_critical_refused_count": 0,
                "archive_incomplete_count": 0,
            }
        )
        value.update({key: maximum for key, maximum in HARD_MAXIMUMS.items()})
        return value

    def run_compare(self, control, candidate):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            control_path = root / "control.json"
            candidate_path = root / "candidate.json"
            control_path.write_text(json.dumps(control), encoding="utf-8")
            candidate_path.write_text(json.dumps(candidate), encoding="utf-8")
            return subprocess.run(
                [sys.executable, str(SCRIPT), str(control_path), str(candidate_path)],
                cwd=ROOT,
                text=True,
                capture_output=True,
                check=False,
            )

    def test_identical_finite_metrics_pass(self):
        result = self.run_compare(self.metrics(), self.metrics())
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_nonfinite_metric_cannot_bypass_gate(self):
        control = self.metrics()
        candidate = self.metrics()
        candidate["fitness_p95"] = float("nan")
        result = self.run_compare(control, candidate)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("non-finite metric: fitness_p95", result.stdout)

    def test_archive_byte_and_job_hard_limits_are_enforced(self):
        control = self.metrics()
        candidate = self.metrics()
        candidate["archive_queue_peak_mib"] = 256.01
        result = self.run_compare(control, candidate)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("archive_queue_peak_mib exceeds hard limit", result.stdout)


if __name__ == "__main__":
    unittest.main()
