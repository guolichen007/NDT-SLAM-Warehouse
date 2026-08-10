import argparse
import json
import tempfile
import unittest
from pathlib import Path

from scripts.regression import control_manifest


class ControlManifestTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.config = self.root / "config.yaml"
        self.config.write_text(
            "runtime_profile: STATIC_MAP_BUILD_FAIL_CLOSED\n"
            "sensor_body_self_mask:\n"
            "  enabled: true\n"
            "  commissioned: true\n"
            "  frame_id: sensor_body\n",
            encoding="utf-8",
        )
        self.ros_params = self.root / "ros_params.json"
        self.ros_params.write_text('{"z":2,"a":1}\n', encoding="utf-8")
        self.topics = self.root / "topics.json"
        self.topics.write_text('{"cloud":"/rslidar_points"}\n', encoding="utf-8")
        self.flags = self.root / "flags.json"
        self.flags.write_text(
            '{"tracking_ephemeral_map_enabled":false}\n', encoding="utf-8"
        )
        self.extrinsic = self.root / "extrinsic.yaml"
        self.extrinsic.write_text("x: 0\n", encoding="utf-8")
        self.target = self.root / "target.pcd"
        self.target.write_bytes(b"fixed registration target\n")

    def tearDown(self):
        self.temp.cleanup()

    def args(self, output: Path, experiment: str = "control"):
        return argparse.Namespace(
            experiment_name=experiment,
            commit="a" * 40,
            bag=Path(control_manifest.FIXED_BAG_PATH),
            bag_sha256=control_manifest.FIXED_BAG_SHA256,
            playback_rate=1.0,
            config=self.config,
            ros_parameters_json=self.ros_params,
            sensor_topics_json=self.topics,
            tf_extrinsic=self.extrinsic,
            registration_target=self.target,
            persistent_map=None,
            runtime_profile="STATIC_MAP_BUILD_FAIL_CLOSED",
            feature_flags_json=self.flags,
            output=output,
        )

    def test_generation_is_deterministic_and_uses_empty_sentinel(self):
        self.assertEqual(
            control_manifest.FIXED_BAG_PATH,
            "/home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag",
        )
        first = self.root / "first.json"
        second = self.root / "second.json"
        self.assertEqual(control_manifest.create(self.args(first)), 0)
        self.assertEqual(control_manifest.create(self.args(second)), 0)
        self.assertEqual(first.read_bytes(), second.read_bytes())
        value = json.loads(first.read_text(encoding="utf-8"))
        self.assertEqual(value["persistent_map_initial_state_sha256"], "EMPTY")
        self.assertEqual(value["bag_sha256"], control_manifest.FIXED_BAG_SHA256)
        self.assertEqual(control_manifest.validate(value), [])

    def test_verify_files_rehashes_all_manifest_inputs(self):
        bag = self.root / "bag.bag"
        bag.write_bytes(b"fixed bag bytes")
        output = self.root / "verified.json"
        args = self.args(output)
        args.bag = bag
        args.bag_sha256 = ""
        self.assertEqual(control_manifest.create(args), 0)
        value = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(control_manifest.validate(value, verify_files=True), [])
        self.extrinsic.write_text("x: 1\n", encoding="utf-8")
        self.assertIn(
            "input content no longer matches manifest: tf_extrinsic",
            control_manifest.validate(value, verify_files=True),
        )

    def test_control_comparison_rejects_map_or_transform_change(self):
        value = {
            "experiment_name": "control",
            "registration_target_snapshot_sha256": "a" * 64,
            "tf_extrinsic_sha256": "b" * 64,
            "feature_flags": {"tracking_ephemeral_map_enabled": False},
        }
        changed = dict(value)
        changed["experiment_name"] = "changed"
        changed["registration_target_snapshot_sha256"] = "c" * 64
        self.assertEqual(
            control_manifest.compare(value, changed, False),
            ["registration_target_snapshot_sha256"],
        )

    def test_ephemeral_ab_allows_only_the_one_feature_flag(self):
        left = {
            "experiment_name": "off",
            "feature_flags": {
                "tracking_ephemeral_map_enabled": False,
                "other": 1,
            },
        }
        right = {
            "experiment_name": "on",
            "feature_flags": {
                "tracking_ephemeral_map_enabled": True,
                "other": 1,
            },
        }
        self.assertEqual(control_manifest.compare(left, right, True), [])
        right["feature_flags"]["other"] = 2
        self.assertTrue(control_manifest.compare(left, right, True))

    def test_candidate_code_allows_only_commit_and_experiment_identity(self):
        control = {
            "experiment_name": "ec64a9f-control",
            "commit_sha": "a" * 40,
            "bag_sha256": "b" * 64,
        }
        candidate = dict(control)
        candidate["experiment_name"] = "phase-a-candidate"
        candidate["commit_sha"] = "c" * 40
        self.assertEqual(
            control_manifest.compare(
                control, candidate, False, candidate_code=True),
            [],
        )
        candidate["bag_sha256"] = "d" * 64
        self.assertEqual(
            control_manifest.compare(
                control, candidate, False, candidate_code=True),
            ["bag_sha256"],
        )


if __name__ == "__main__":
    unittest.main()
