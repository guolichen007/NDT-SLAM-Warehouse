#!/usr/bin/env python3
"""Sensor rig calibration identity（只 hash 双雷达外参，不含运行参数）。

merger_params.yaml 同时包含双雷达外参（Lidar2BaseExtrinsic）与运行参数
（voxel_size / max_pair_dt / 队列等）。标定 ID 必须只绑定外参 + 坐标系/矩阵约定，
否则把 voxel_size 从 0.15 改到别的值会被误判为"雷达重新标定"。

输出两个互不混用的 SHA：
  SENSOR_RIG_CALIBRATION_ID = sha256(canonical extrinsic payload)
  MERGER_RUNTIME_CONFIG_SHA256 = sha256(entire merger_params.yaml)
"""
import argparse
import hashlib
import json
import sys

import yaml

CANONICAL = {
    "schema_version": 1,
    "target_frame": "base_link",
    "matrix_order": "row-major",
    "translation_unit": "m",
    "rotation_convention": "rh-zup-yaw-ccw",  # 右手系，+Z 向上，yaw 绕 +Z 逆时针
}


def canonical_extrinsic_payload(cfg):
    lidars = []
    for lidar in cfg.get("lidars", []):
        lidars.append({
            "logical_name": lidar.get("name"),
            "source_topic": lidar.get("topic"),
            "Lidar2BaseExtrinsic": lidar.get("Lidar2BaseExtrinsic"),
        })
    payload = dict(CANONICAL)
    payload["sensors"] = sorted(lidars, key=lambda s: s["logical_name"] or "")
    return payload


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--merger-params", required=True)
    ap.add_argument("--output", default=None)
    args = ap.parse_args()

    raw = open(args.merger_params, "rb").read()
    cfg = yaml.safe_load(open(args.merger_params))

    payload = canonical_extrinsic_payload(cfg)
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    calibration_id = "sha256:" + hashlib.sha256(canonical.encode()).hexdigest()
    config_sha = hashlib.sha256(raw).hexdigest()

    result = {
        "sensor_rig_calibration_id": calibration_id,
        "merger_runtime_config_sha256": config_sha,
        "canonical_payload": payload,
        "source_file": args.merger_params,
    }
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w") as f:
            f.write(rendered + "\n")
    print(rendered)
    return 0


if __name__ == "__main__":
    sys.exit(main())
