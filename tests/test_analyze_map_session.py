import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest


MODULE_PATH = Path(__file__).parents[1] / "tools" / "analyze_map_session.py"
SPEC = importlib.util.spec_from_file_location("analyze_map_session", MODULE_PATH)
assert SPEC and SPEC.loader
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def write_binary_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
        "TYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n"
    ).encode("ascii")
    payload = b"".join(struct.pack("<fff", *point) for point in points)
    path.write_bytes(header + payload)


class AnalyzeMapSessionTest(unittest.TestCase):
    def test_binary_reader_and_relationships(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = [(1.0, 2.0, 3.0), (1.1, 2.0, 3.0)]
            for filename in audit.FORMAL_LAYERS.values():
                write_binary_pcd(root / filename, points)
            (root / "poses_raw.txt").write_text("", encoding="utf-8")
            (root / "keyframes").mkdir()
            report = audit.audit_session(root)
            self.assertEqual(
                report["layers"]["registration"]["finite_points"], 2
            )
            self.assertEqual(
                report["relationships"]["point_set_overlap"]
                ["objects_clean_vs_objects_raw"]["lhs_contained_ratio"],
                1.0,
            )
            codes = {finding["code"] for finding in report["findings"]}
            self.assertIn("SESSION_MANIFEST_MISSING", codes)
            self.assertIn("STATIC_EVIDENCE_MISSING", codes)
            projection = report["static_height_field_projection"]
            self.assertEqual(projection["occupied_cells"], 0)

    def test_height_projection_keeps_two_supported_layers(self) -> None:
        points = []
        for z in (1.0, 2.0):
            points.extend((0.1, 0.1, z + 0.001 * index) for index in range(6))
        projection = audit.project_static_height_layers(
            audit.np.asarray(points), cell_size=0.25
        )
        self.assertEqual(projection["occupied_cells"], 1)
        self.assertEqual(projection["layers"], 2)
        self.assertEqual(projection["layer_histogram"], {"2": 1})

    def test_payload_size_mismatch_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.pcd"
            write_binary_pcd(path, [(1.0, 2.0, 3.0)])
            path.write_bytes(path.read_bytes()[:-1])
            with self.assertRaisesRegex(ValueError, "payload size"):
                audit.read_pcd_xyz(path)


if __name__ == "__main__":
    unittest.main()
