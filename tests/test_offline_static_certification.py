import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
OFFLINE = REPO / "src" / "ndt_slam" / "scripts" / "offline"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class OfflineStaticCertificationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.candidate = self.root / "candidate"
        self.candidate.mkdir()
        self.localization = self.candidate / "candidate_localization_reference.pcd"
        self.avoidance = self.candidate / "candidate_avoidance_static_baseline.pcd"
        self.localization.write_bytes(b"localization-reference")
        self.avoidance.write_bytes(b"avoidance-baseline")
        self.evidence = self.candidate / "static_evidence_candidate.csv"
        self.evidence.write_text(
            "cell_x,cell_y,point_count,episode_count,pass_count,min_z,max_z\n"
            "1,2,42,3,2,0.1,2.0\n",
            encoding="utf-8",
        )
        self.candidate_manifest = self.candidate / "candidate_manifest.yaml"
        self.candidate_manifest.write_text(
            "authority: CANDIDATE_NOT_CERTIFIED\n"
            "frame_id: map\n"
            f"localization_reference_sha256: {digest(self.localization)}\n"
            f"avoidance_static_baseline_sha256: {digest(self.avoidance)}\n"
            f"static_evidence_candidate_sha256: {digest(self.evidence)}\n"
            f"config_sha256: {'a' * 64}\n",
            encoding="utf-8",
        )
        Path(str(self.candidate_manifest) + ".sha256").write_text(
            f"{digest(self.candidate_manifest)}  {self.candidate_manifest.name}\n",
            encoding="utf-8",
        )
        self.validation = self.root / "validation.json"
        self.validation.write_text(
            json.dumps({
                "passed": True,
                "route_independent": True,
                "baseline_not_used_for_collection": True,
                "survey_pass_id": "VALIDATION_PASS_001",
            }),
            encoding="utf-8",
        )
        self.extrinsic = self.root / "extrinsic.yaml"
        self.extrinsic.write_text("frame: sensor_body\n", encoding="utf-8")
        self.source = self.root / "control_manifest.json"
        self.source.write_text('{"schema_version":1}\n', encoding="utf-8")

    def tearDown(self):
        self.temporary.cleanup()

    def run_python(self, script: str, *arguments: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, str(OFFLINE / script), *arguments],
            cwd=REPO, text=True, capture_output=True, check=False,
        )

    def test_certify_and_install_immutable_split_baseline(self):
        certified = self.root / "certified"
        result = self.run_python(
            "static_map_certifier.py",
            str(self.candidate), str(self.validation), str(certified),
            "--extrinsic", str(self.extrinsic),
            "--source-manifest", str(self.source),
            "--approve-by", "unit-test-operator",
            "--minimum-mature-cells", "1",
            "--minimum-mature-coverage", "1.0",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        manifest = json.loads(
            (certified / "certified_baseline_manifest.json").read_text(
                encoding="utf-8"))
        self.assertEqual(manifest["authority"], "CERTIFIED")
        self.assertNotEqual(
            manifest["localization_reference"]["sha256"],
            manifest["avoidance_static_baseline"]["sha256"],
        )

        installed = self.root / "installed"
        first = self.run_python(
            "baseline_installer.py", str(certified), str(installed))
        second = self.run_python(
            "baseline_installer.py", str(certified), str(installed))
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertEqual(second.returncode, 0, second.stderr)
        current = json.loads((installed / "CURRENT.json").read_text("utf-8"))
        self.assertEqual(current["baseline_uuid"], manifest["baseline_uuid"])

    def test_runtime_authority_cannot_be_promoted(self):
        manifest = self.candidate_manifest
        manifest.write_text(
            manifest.read_text(encoding="utf-8").replace(
                "CANDIDATE_NOT_CERTIFIED", "RUNTIME_MATURE"),
            encoding="utf-8",
        )
        Path(str(manifest) + ".sha256").write_text(
            f"{digest(manifest)}  {manifest.name}\n", encoding="utf-8")
        result = self.run_python(
            "static_map_certifier.py",
            str(self.candidate), str(self.validation), str(self.root / "out"),
            "--extrinsic", str(self.extrinsic),
            "--source-manifest", str(self.source),
            "--approve-by", "unit-test-operator",
            "--minimum-mature-cells", "1",
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("cannot be certified", result.stderr)

    def test_tampered_static_evidence_is_rejected(self):
        self.evidence.write_text(
            self.evidence.read_text(encoding="utf-8") +
            "9,9,999,9,9,-5.0,99.0\n",
            encoding="utf-8",
        )
        result = self.run_python(
            "static_map_certifier.py",
            str(self.candidate), str(self.validation), str(self.root / "out"),
            "--extrinsic", str(self.extrinsic),
            "--source-manifest", str(self.source),
            "--approve-by", "unit-test-operator",
            "--minimum-mature-cells", "1",
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("checksum mismatch", result.stderr)


if __name__ == "__main__":
    unittest.main()
