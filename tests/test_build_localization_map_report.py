import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (
    ROOT
    / "src"
    / "ndt_slam"
    / "scripts"
    / "postprocess"
    / "build_localization_map.py"
)
SPEC = importlib.util.spec_from_file_location(
    "build_localization_map", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class LocalizationMapReportTest(unittest.TestCase):
    def report(self):
        return {
            "input_file": "display_map.pcd",
            "ground_model_file": "ground_model.csv",
            "original_points": 100,
            "output_points": 80,
            "config": {"voxel_size": 0.1},
        }

    def test_atomic_report_is_valid_json_and_uses_lf(self):
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "localization_map_report.json"
            MODULE.write_json_atomic(
                report_path,
                self.report(),
                MODULE.LOCALIZATION_REPORT_REQUIRED_KEYS,
            )

            self.assertEqual(
                json.loads(report_path.read_text(encoding="utf-8")),
                self.report(),
            )
            raw = report_path.read_bytes()
            self.assertTrue(raw.endswith(b"\n"))
            self.assertNotIn(b"\r\n", raw)
            self.assertEqual(
                list(Path(directory).glob("*.tmp")),
                [],
            )

    def test_failed_serialization_preserves_previous_report(self):
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "localization_map_report.json"
            previous = self.report()
            MODULE.write_json_atomic(
                report_path,
                previous,
                MODULE.LOCALIZATION_REPORT_REQUIRED_KEYS,
            )
            invalid = dict(previous)
            invalid["config"] = {"not_json": object()}

            with self.assertRaises(TypeError):
                MODULE.write_json_atomic(
                    report_path,
                    invalid,
                    MODULE.LOCALIZATION_REPORT_REQUIRED_KEYS,
                )

            self.assertEqual(
                json.loads(report_path.read_text(encoding="utf-8")),
                previous,
            )
            self.assertEqual(
                list(Path(directory).glob("*.tmp")),
                [],
            )

    def test_missing_contract_field_is_rejected_before_write(self):
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "localization_map_report.json"
            report = self.report()
            del report["output_points"]

            with self.assertRaises(ValueError):
                MODULE.write_json_atomic(
                    report_path,
                    report,
                    MODULE.LOCALIZATION_REPORT_REQUIRED_KEYS,
                )

            self.assertFalse(report_path.exists())


if __name__ == "__main__":
    unittest.main()
