#!/usr/bin/env python3
"""migrate_legacy_map_identity.py 的迁移合同测试。

验证 schema1→schema2 身份迁移的 path-independence / idempotence / fail-closed。
只用临时 sandbox，不修改生产 maps/live/current。
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROD_MAP = os.path.join(REPO, "maps", "live", "current")
TOOL = os.path.join(REPO, "scripts", "validation",
                    "migrate_legacy_map_identity.py")
CALIB = "sha256:bcefe07e764434a3fe84c26054f78ee971b4a8a35427839ca86b3f902172411d"


def run_tool(map_root, output="persistent_map_manifest.schema2.json"):
    return subprocess.run(
        [sys.executable, TOOL, "--map-root", map_root,
         "--output-manifest", output, "--sensor-rig-calibration-id", CALIB,
         "--source-git-sha", "test"],
        capture_output=True, text=True)


class MigrationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="yaw-migrate-")
        self.map_root = os.path.join(self.tmp, "map")
        shutil.copytree(PROD_MAP, self.map_root)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def read_manifest(self, root, name="persistent_map_manifest.schema2.json"):
        return json.load(open(os.path.join(root, name)))

    def test_legacy_schema1_read_and_migrate(self):
        legacy = json.load(open(os.path.join(
            self.map_root, "persistent_map_manifest.json")))
        self.assertEqual(legacy["schema_version"], 1)
        r = run_tool(self.map_root)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = self.read_manifest(self.map_root)
        self.assertEqual(out["schema_version"], 2)
        self.assertTrue(out["map_frame_uuid"])
        self.assertIn("MAP_FRAME_UUID_MINTED", r.stdout)

    def test_same_map_different_path_same_uuid(self):
        run_tool(self.map_root)
        other = os.path.join(self.tmp, "other-path")
        shutil.copytree(self.map_root, other)
        r = run_tool(other)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("MAP_FRAME_UUID_REUSED", r.stdout)
        a = self.read_manifest(self.map_root)["map_frame_uuid"]
        b = self.read_manifest(other)["map_frame_uuid"]
        self.assertEqual(a, b)

    def test_second_migration_does_not_remint(self):
        run_tool(self.map_root)
        first = self.read_manifest(self.map_root)["map_frame_uuid"]
        r = run_tool(self.map_root)
        self.assertIn("MAP_FRAME_UUID_REUSED", r.stdout)
        self.assertEqual(self.read_manifest(self.map_root)["map_frame_uuid"],
                         first)

    def test_tile_changed_changes_fingerprint_and_rejects(self):
        run_tool(self.map_root)
        manifest = json.load(open(os.path.join(
            self.map_root, "persistent_map_manifest.json")))
        tile = manifest["layers"]["registration"][0]
        p = os.path.join(self.map_root, tile["path"])
        with open(p, "ab") as f:
            f.write(b"\x00")
        r = run_tool(self.map_root)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("tile_hash_mismatch", r.stderr)

    def test_manifest_hash_corruption_rejects(self):
        run_tool(self.map_root)
        # 破坏 credential 内容（模拟篡改 content fingerprint）
        cred = json.load(open(os.path.join(
            self.map_root, "map_frame_identity_migration.json")))
        cred["map_content_fingerprint"] = "deadbeef"
        json.dump(cred, open(os.path.join(
            self.map_root, "map_frame_identity_migration.json"), "w"))
        r = run_tool(self.map_root)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("content_changed_cannot_reuse_map_frame_uuid", r.stderr)


if __name__ == "__main__":
    unittest.main()
