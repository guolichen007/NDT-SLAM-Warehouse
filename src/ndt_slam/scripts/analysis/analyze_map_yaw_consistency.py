#!/usr/bin/env python3
"""Read-only axial-orientation audit for persistent/localization PCD maps."""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path
from typing import Iterable


def axial_delta_deg(angle_deg: float, reference_deg: float) -> float:
    """Smallest signed axial delta; directions 180 degrees apart are equal."""
    return ((angle_deg - reference_deg + 90.0) % 180.0) - 90.0


def choose_axial_orientation_deg(
    principal_deg: float, reference_deg: float | None
) -> float:
    principal = principal_deg % 180.0
    if reference_deg is None:
        return principal
    orthogonal = (principal + 90.0) % 180.0
    return min(
        (principal, orthogonal),
        key=lambda angle: abs(axial_delta_deg(angle, reference_deg)),
    )


def _parse_pcd(path: Path) -> list[tuple[float, float, float]]:
    with path.open("rb") as stream:
        header: dict[str, list[str]] = {}
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError("pcd_header_incomplete")
            line = raw.decode("ascii", errors="strict").strip()
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            header[key.upper()] = values
            if key.upper() == "DATA":
                break
        fields = header.get("FIELDS", header.get("FIELD", []))
        if not all(axis in fields for axis in ("x", "y", "z")):
            raise ValueError("pcd_xyz_fields_missing")
        points = int(header.get("POINTS", ["0"])[0])
        data_mode = header["DATA"][0].lower()
        if data_mode == "ascii":
            result = []
            indices = [fields.index(axis) for axis in ("x", "y", "z")]
            for raw in stream:
                values = raw.decode("ascii").split()
                if len(values) < len(fields):
                    continue
                xyz = tuple(float(values[index]) for index in indices)
                if all(math.isfinite(value) for value in xyz):
                    result.append(xyz)
            return result
        if data_mode != "binary":
            raise ValueError("pcd_data_mode_unsupported")
        sizes = [int(value) for value in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        formats = {("F", 4): "f", ("F", 8): "d", ("I", 1): "b",
                   ("I", 2): "h", ("I", 4): "i", ("U", 1): "B",
                   ("U", 2): "H", ("U", 4): "I"}
        tokens: list[str] = []
        expanded_fields: list[str] = []
        for field, size, field_type, count in zip(fields, sizes, types, counts):
            token = formats.get((field_type.upper(), size))
            if token is None:
                raise ValueError("pcd_field_type_unsupported")
            tokens.extend([token] * count)
            expanded_fields.extend([field] * count)
        record = struct.Struct("<" + "".join(tokens))
        indices = [expanded_fields.index(axis) for axis in ("x", "y", "z")]
        payload = stream.read()
        if points <= 0:
            points = len(payload) // record.size
        result = []
        for offset in range(0, min(len(payload), points * record.size), record.size):
            values = record.unpack_from(payload, offset)
            xyz = tuple(float(values[index]) for index in indices)
            if all(math.isfinite(value) for value in xyz):
                result.append(xyz)
        return result


def estimate_axial_orientation(
    points: Iterable[tuple[float, float, float]],
    reference_deg: float | None = None,
) -> dict[str, float | str]:
    xy = [(x, y) for x, y, _ in points if math.isfinite(x) and math.isfinite(y)]
    if len(xy) < 3:
        return {"status": "INCONCLUSIVE", "reason": "too_few_points"}
    mean_x = sum(point[0] for point in xy) / len(xy)
    mean_y = sum(point[1] for point in xy) / len(xy)
    xx = sum((x - mean_x) ** 2 for x, _ in xy)
    yy = sum((y - mean_y) ** 2 for _, y in xy)
    xy_cov = sum((x - mean_x) * (y - mean_y) for x, y in xy)
    trace = xx + yy
    discriminant = math.hypot(xx - yy, 2.0 * xy_cov)
    confidence = discriminant / trace if trace > 1.0e-12 else 0.0
    if confidence < 0.10:
        return {
            "status": "INCONCLUSIVE",
            "reason": "orthogonal_or_isotropic_structure",
            "confidence": confidence,
        }
    principal_deg = math.degrees(0.5 * math.atan2(2.0 * xy_cov, xx - yy))
    orientation = choose_axial_orientation_deg(principal_deg, reference_deg)
    result: dict[str, float | str] = {
        "status": "VALID",
        "reason": "axial_pca",
        "orientation_deg": orientation,
        "confidence": confidence,
    }
    if reference_deg is not None:
        result["delta_from_reference_deg"] = axial_delta_deg(
            orientation, reference_deg
        )
    return result


def analyze_map(
    path: Path, segment_length_m: float, reference_deg: float | None
) -> dict[str, object]:
    points = _parse_pcd(path)
    if not points:
        return {"status": "INCONCLUSIVE", "reason": "map_empty", "segments": []}
    x_min = min(point[0] for point in points)
    x_max = max(point[0] for point in points)
    segments = []
    start = math.floor(x_min / segment_length_m) * segment_length_m
    while start <= x_max:
        selected = [point for point in points if start <= point[0] < start + segment_length_m]
        estimate = estimate_axial_orientation(selected, reference_deg)
        estimate.update({"x_min": start, "x_max": start + segment_length_m,
                         "point_count": len(selected)})
        segments.append(estimate)
        start += segment_length_m
    valid = [segment for segment in segments if segment["status"] == "VALID"]
    if not valid:
        return {"status": "INCONCLUSIVE", "reason": "no_valid_segments",
                "segments": segments}
    deltas = [float(segment.get("delta_from_reference_deg", 0.0)) for segment in valid]
    spread = max(deltas) - min(deltas)
    return {
        "status": "VALID",
        "classification": "SEGMENTED_SKEW_OR_WARP" if spread > 1.0 else "RIGID_OFFSET",
        "delta_spread_deg": spread,
        "segments": segments,
        "read_only": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_pcd", type=Path)
    parser.add_argument("--segment-length-m", type=float, default=10.0)
    parser.add_argument("--reference-yaw-deg", type=float)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.segment_length_m <= 0.0:
        parser.error("--segment-length-m must be positive")
    report = analyze_map(
        args.map_pcd, args.segment_length_m, args.reference_yaw_deg
    )
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if report["status"] == "VALID" else 2


if __name__ == "__main__":
    raise SystemExit(main())
