#!/usr/bin/env python3
"""Yaw reference manifest builder（与 C++ semanticYawReferenceHash 严格一致）。

输入 reference 字段，输出 canonical semantic string + reference_hash + manifest。
hash = hex64(fnv1a64(canonicalReference(reference)))，其中 fnv1a64 的 offset basis
与 rail_localization_authority.cpp 一致（1469598103934665603，非标准 FNV offset）。
"""
import argparse
import json
import math
import sys

FNV_OFFSET = 1469598103934665603
FNV_PRIME = 1099511628211


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def fnv1a64(text):
    h = FNV_OFFSET
    for byte in text.encode("utf-8"):
        h ^= byte
        h = (h * FNV_PRIME) & 0xFFFFFFFFFFFFFFFF
    return h


def hex64(value):
    return format(value, "016x")


def canonical_reference(ref):
    return (
        f"schema={ref['schema_version']};"
        f"verified={1 if ref['verified'] else 0};"
        f"yaw={normalize_angle(ref['rail_yaw_in_map_rad']):.17g};"
        f"source={ref['source']};"
        f"map_frame_uuid={ref['map_frame_uuid']};"
        f"map_frame_id={ref['map_frame_id']};"
        f"base_frame_id={ref['base_frame_id']};"
        f"convention={ref['map_frame_convention_id']};"
        f"sensor_rig={ref['sensor_rig_calibration_id']};"
        f"reference_uuid={ref['reference_uuid']}"
    )


def reference_hash(ref):
    return hex64(fnv1a64(canonical_reference(ref)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--schema-version", type=int, default=1)
    ap.add_argument("--verified", type=bool, default=False)
    ap.add_argument("--rail-yaw-in-map-rad", type=float, default=0.0)
    ap.add_argument("--source", default="CONFIG_SITE_REFERENCE")
    ap.add_argument("--map-frame-uuid", required=True)
    ap.add_argument("--map-frame-id", default="map")
    ap.add_argument("--base-frame-id", default="base_link")
    ap.add_argument("--map-frame-convention-id", default="")
    ap.add_argument("--sensor-rig-calibration-id", default="")
    ap.add_argument("--reference-uuid", required=True)
    ap.add_argument("--output", default=None)
    args = ap.parse_args()

    ref = {
        "schema_version": args.schema_version,
        "verified": args.verified,
        "rail_yaw_in_map_rad": args.rail_yaw_in_map_rad,
        "source": args.source,
        "map_frame_uuid": args.map_frame_uuid,
        "map_frame_id": args.map_frame_id,
        "base_frame_id": args.base_frame_id,
        "map_frame_convention_id": args.map_frame_convention_id,
        "sensor_rig_calibration_id": args.sensor_rig_calibration_id,
        "reference_uuid": args.reference_uuid,
    }
    canonical = canonical_reference(ref)
    ref["reference_hash"] = reference_hash(ref)
    result = {"reference": ref, "canonical_string": canonical}
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w") as f:
            f.write(rendered + "\n")
    print(rendered)
    return 0


if __name__ == "__main__":
    sys.exit(main())
