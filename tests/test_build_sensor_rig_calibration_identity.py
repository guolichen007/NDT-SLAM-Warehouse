#!/usr/bin/env python3
"""sensor_rig_calibration_id 只绑定双雷达外参，不绑定运行参数。"""
import copy
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import unittest

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOL = os.path.join(REPO, "scripts", "validation",
                    "build_sensor_rig_calibration_identity.py")
MERGER = os.path.join(REPO, "src", "ndt_slam", "config", "merger_params.yaml")


def run_tool(path):
    r = subprocess.run([sys.executable, TOOL, "--merger-params", path],
                       capture_output=True, text=True)
    self = None
    d = json.loads(r.stdout)
    return d["sensor_rig_calibration_id"], d["merger_runtime_config_sha256"]


class CalibrationIdentityTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="calib-id-")
        self.path = os.path.join(self.tmp, "merger_params.yaml")
        import shutil
        shutil.copy(MERGER, self.path)
        self.cfg = yaml.safe_load(open(self.path))
        self.calib_id, self.config_sha = run_tool(self.path)

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write_cfg(self, cfg):
        yaml.safe_dump(cfg, open(self.path, "w"))

    def test_voxel_size_change_keeps_calibration_id(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["voxel_size"] = 0.30
        self.write_cfg(cfg)
        calib, config = run_tool(self.path)
        self.assertEqual(calib, self.calib_id)
        self.assertNotEqual(config, self.config_sha)

    def test_pair_dt_change_keeps_calibration_id(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["sync"]["max_pair_dt_sec"] = 0.090
        self.write_cfg(cfg)
        calib, _ = run_tool(self.path)
        self.assertEqual(calib, self.calib_id)

    def test_extrinsic_change_changes_calibration_id(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["lidars"][0]["Lidar2BaseExtrinsic"][11] += 0.001  # 平移 Z
        self.write_cfg(cfg)
        calib, _ = run_tool(self.path)
        self.assertNotEqual(calib, self.calib_id)

    def test_swap_sensor_identity_changes_calibration_id(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["lidars"][0]["name"], cfg["lidars"][1]["name"] = \
            cfg["lidars"][1]["name"], cfg["lidars"][0]["name"]
        self.write_cfg(cfg)
        calib, _ = run_tool(self.path)
        self.assertNotEqual(calib, self.calib_id)


if __name__ == "__main__":
    unittest.main()
