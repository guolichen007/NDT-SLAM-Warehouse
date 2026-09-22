#!/usr/bin/env python3
"""Legacy schema1 → semantic schema2 persistent map identity migration.

只做身份迁移：为冻结的 persistent map 目录 mint 一个 path-independent
map_frame_uuid，并写回 schema2 manifest。不改点云、不改坐标、不改 tile catalog。

map_frame_uuid 语义 = "当前这套冻结地图坐标系"的永久 ID。第一次迁移 mint 一次并
写入 map_frame_identity_migration.json；之后永远复用；复制目录 / sandbox 路径变化 /
重启都不会改变它。若内容 fingerprint 变化则拒绝（不允许悄悄 mint 新 UUID）。
"""
import argparse
import hashlib
import json
import os
import uuid

MANIFEST_SCHEMA = "ndt_slam_persistent_tile_catalog"
LAYERS = ("registration", "display", "ground", "objects")


def content_fingerprint(manifest, map_root):
    lines = []
    for layer in LAYERS:
        for tile in manifest.get("layers", {}).get(layer, []):
            path = os.path.join(map_root, tile["path"])
            if not os.path.isfile(path):
                raise SystemExit(f"tile_missing:{tile['path']}")
            digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
            if digest != tile.get("sha256", ""):
                raise SystemExit(f"tile_hash_mismatch:{tile['path']}")
            lines.append(f"{layer}:{tile['path']}:{digest}")
    lines.sort()
    return hashlib.sha256("\n".join(lines).encode()).hexdigest()


def build_schema2(manifest, map_frame_uuid, convention_id, calibration_id):
    out = dict(manifest)
    out["schema"] = MANIFEST_SCHEMA
    out["schema_version"] = 2
    out["map_frame_uuid"] = map_frame_uuid
    out["map_frame_convention_id"] = convention_id
    out["sensor_rig_calibration_id"] = calibration_id
    out["yaw_reference"] = {
        "schema_version": 1,
        "verified": False,
        "rail_yaw_in_map_rad": 0.0,
        "source": "CONFIG_SITE_REFERENCE",
        "map_frame_uuid": map_frame_uuid,
        "map_frame_id": "map",
        "base_frame_id": "base_link",
        "map_frame_convention_id": convention_id,
        "sensor_rig_calibration_id": calibration_id,
        "reference_uuid": "",
        "reference_hash": "",
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-root", required=True)
    ap.add_argument("--manifest", default="persistent_map_manifest.json")
    ap.add_argument("--output-manifest", required=True)
    ap.add_argument("--migration-credential",
                    default="map_frame_identity_migration.json")
    ap.add_argument("--map-frame-convention-id", default="")
    ap.add_argument("--sensor-rig-calibration-id", default="")
    ap.add_argument("--source-git-sha", default="")
    args = ap.parse_args()

    manifest_path = os.path.join(args.map_root, args.manifest)
    manifest = json.load(open(manifest_path))
    if manifest.get("schema_version") != 1:
        raise SystemExit("not_legacy_schema1")
    if manifest.get("schema") != MANIFEST_SCHEMA:
        raise SystemExit("unexpected_manifest_schema")

    fingerprint = content_fingerprint(manifest, args.map_root)

    credential_path = os.path.join(args.map_root, args.migration_credential)
    if os.path.exists(credential_path):
        credential = json.load(open(credential_path))
        if credential.get("map_content_fingerprint") != fingerprint:
            raise SystemExit("content_changed_cannot_reuse_map_frame_uuid")
        map_frame_uuid = credential["map_frame_uuid"]
        print(f"MAP_FRAME_UUID_REUSED={map_frame_uuid}")
    else:
        map_frame_uuid = str(uuid.uuid4())
        credential = {
            "schema_version": 2,
            "migration_version": 1,
            "legacy_map_uuid": manifest.get("map_uuid"),
            "legacy_manifest_sha256":
                hashlib.sha256(open(manifest_path, "rb").read()).hexdigest(),
            "map_content_fingerprint": fingerprint,
            "map_frame_uuid": map_frame_uuid,
            "frame_id": "map",
            "base_frame_id": "base_link",
            "source_git_sha": args.source_git_sha,
        }
        with open(credential_path, "w") as f:
            json.dump(credential, f, indent=2)
        print(f"MAP_FRAME_UUID_MINTED={map_frame_uuid}")

    out = build_schema2(manifest, map_frame_uuid,
                        args.map_frame_convention_id,
                        args.sensor_rig_calibration_id)
    out_path = os.path.join(args.map_root, args.output_manifest)
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(f"MAP_CONTENT_FINGERPRINT={fingerprint}")
    print(f"SCHEMA2_MANIFEST_WRITTEN={out_path}")


if __name__ == "__main__":
    main()
