#!/usr/bin/env python3
"""Audit a saved NDT-SLAM map session without ROS or PCL.

The reader supports the PCL binary and ASCII encodings used by production
sessions. It intentionally rejects binary_compressed rather than silently
misreading it. Results are deterministic JSON and include file hashes,
finite-point bounds, sparse cell/voxel counts, layer identity and selected
point-set overlap checks.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable
import zipfile

import numpy as np


FORMAL_LAYERS = {
    "registration": "map_registration.pcd",
    "display": "map_display.pcd",
    "display_full": "map_display_full.pcd",
    "ground": "map_ground.pcd",
    "objects_raw": "map_objects_raw.pcd",
    "objects_clean": "map_objects_clean.pcd",
    "objects_filtered": "map_objects_filtered.pcd",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def _read_header(stream: Any) -> tuple[dict[str, list[str]], int]:
    header: dict[str, list[str]] = {}
    while True:
        raw = stream.readline()
        if not raw:
            raise ValueError("PCD header ended before DATA")
        try:
            line = raw.decode("ascii").strip()
        except UnicodeDecodeError as error:
            raise ValueError("PCD header is not ASCII") from error
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        header[parts[0].upper()] = parts[1:]
        if parts[0].upper() == "DATA":
            return header, stream.tell()


def read_pcd_xyz(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    with path.open("rb") as stream:
        header, data_offset = _read_header(stream)
        fields = header.get("FIELDS", header.get("FIELD", []))
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(value) for value in header.get("COUNT", [])]
        if not counts:
            counts = [1] * len(fields)
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("PCD field metadata lengths differ")
        if not all(axis in fields for axis in ("x", "y", "z")):
            raise ValueError("PCD has no x/y/z fields")
        points = int(header.get("POINTS", ["0"])[0])
        encoding = header["DATA"][0].lower()
        if encoding == "binary_compressed":
            raise ValueError("binary_compressed PCD is not supported")
        if encoding == "ascii":
            stream.seek(data_offset)
            values = np.loadtxt(stream, dtype=np.float64, ndmin=2)
            scalar_offsets: dict[str, int] = {}
            offset = 0
            for name, count in zip(fields, counts):
                scalar_offsets[name] = offset
                offset += count
            xyz = values[:, [scalar_offsets[axis] for axis in "xyz"]]
        elif encoding == "binary":
            type_codes = {
                ("F", 4): "<f4",
                ("F", 8): "<f8",
                ("I", 1): "i1",
                ("I", 2): "<i2",
                ("I", 4): "<i4",
                ("I", 8): "<i8",
                ("U", 1): "u1",
                ("U", 2): "<u2",
                ("U", 4): "<u4",
                ("U", 8): "<u8",
            }
            names: list[str] = []
            formats: list[Any] = []
            offsets: list[int] = []
            byte_offset = 0
            axis_names: dict[str, str] = {}
            for name, size, kind, count in zip(fields, sizes, types, counts):
                code = type_codes.get((kind.upper(), size))
                if code is None:
                    raise ValueError(f"unsupported PCD scalar: {kind}{size}")
                unique_name = name
                suffix = 1
                while unique_name in names:
                    suffix += 1
                    unique_name = f"{name}_{suffix}"
                names.append(unique_name)
                formats.append(code if count == 1 else (code, (count,)))
                offsets.append(byte_offset)
                if name in ("x", "y", "z"):
                    axis_names[name] = unique_name
                byte_offset += size * count
            dtype = np.dtype(
                {"names": names, "formats": formats, "offsets": offsets,
                 "itemsize": byte_offset}
            )
            stream.seek(data_offset)
            payload = stream.read()
            if points == 0:
                points = len(payload) // dtype.itemsize
            expected = points * dtype.itemsize
            if len(payload) != expected:
                raise ValueError(
                    f"PCD payload size {len(payload)} != expected {expected}"
                )
            records = np.frombuffer(payload, dtype=dtype, count=points)
            xyz = np.column_stack(
                [records[axis_names[axis]].astype(np.float64, copy=False)
                 for axis in "xyz"]
            )
        else:
            raise ValueError(f"unsupported PCD DATA encoding: {encoding}")
    metadata = {
        "encoding": encoding,
        "header_points": points,
        "fields": fields,
        "point_step_bytes": int(sum(s * c for s, c in zip(sizes, counts))),
    }
    return np.asarray(xyz), metadata


def _unique_grid_count(xyz: np.ndarray, resolution: float) -> int:
    if xyz.size == 0:
        return 0
    quantized = np.floor(xyz / resolution).astype(np.int64)
    return int(np.unique(quantized, axis=0).shape[0])


def _row_keys(xyz: np.ndarray) -> np.ndarray:
    canonical = np.ascontiguousarray(xyz.astype("<f4", copy=False))
    return canonical.view(np.dtype((np.void, canonical.dtype.itemsize * 3))).ravel()


def point_set_overlap(lhs: np.ndarray, rhs: np.ndarray) -> dict[str, Any]:
    lhs_unique = np.unique(_row_keys(lhs))
    rhs_unique = np.unique(_row_keys(rhs))
    intersection = np.intersect1d(lhs_unique, rhs_unique, assume_unique=True)
    return {
        "lhs_unique": int(lhs_unique.size),
        "rhs_unique": int(rhs_unique.size),
        "intersection_unique": int(intersection.size),
        "lhs_contained_ratio": (
            float(intersection.size / lhs_unique.size) if lhs_unique.size else 1.0
        ),
        "rhs_contained_ratio": (
            float(intersection.size / rhs_unique.size) if rhs_unique.size else 1.0
        ),
    }


def project_static_height_layers(
    xyz: np.ndarray,
    *,
    cell_size: float,
    maximum_merge_gap: float = 0.18,
    minimum_points_per_layer: int = 6,
    maximum_layers_per_cell: int = 3,
) -> dict[str, Any]:
    """Project clean points with the same XY/gap/count policy as C++ V1."""
    if xyz.size == 0:
        return {"occupied_cells": 0, "layers": 0, "layer_histogram": {}}
    xy = np.floor(xyz[:, :2] / cell_size).astype(np.int64)
    order = np.lexsort((xyz[:, 2], xy[:, 1], xy[:, 0]))
    sorted_xy = xy[order]
    sorted_z = xyz[order, 2]
    boundary = np.empty(sorted_z.size, dtype=bool)
    boundary[0] = True
    boundary[1:] = np.any(sorted_xy[1:] != sorted_xy[:-1], axis=1)
    starts = np.flatnonzero(boundary)
    ends = np.r_[starts[1:], sorted_z.size]
    layer_histogram: dict[int, int] = {}
    accepted_cells = 0
    layer_count = 0
    discarded_small_clusters = 0
    for start, end in zip(starts, ends):
        values = sorted_z[start:end]
        split = np.r_[0, np.flatnonzero(np.diff(values) > maximum_merge_gap) + 1,
                      values.size]
        sizes = np.diff(split)
        discarded_small_clusters += int(np.count_nonzero(
            sizes < minimum_points_per_layer
        ))
        accepted = sizes[sizes >= minimum_points_per_layer]
        if accepted.size > maximum_layers_per_cell:
            accepted = np.sort(accepted)[-maximum_layers_per_cell:]
        count = int(accepted.size)
        if count:
            accepted_cells += 1
            layer_count += count
            layer_histogram[count] = layer_histogram.get(count, 0) + 1
    return {
        "cell_size_m": cell_size,
        "maximum_merge_gap_m": maximum_merge_gap,
        "minimum_points_per_layer": minimum_points_per_layer,
        "maximum_layers_per_cell": maximum_layers_per_cell,
        "occupied_cells": accepted_cells,
        "layers": layer_count,
        "multi_layer_cells": sum(
            count for layers, count in layer_histogram.items() if layers > 1
        ),
        "layer_histogram": {
            str(key): value for key, value in sorted(layer_histogram.items())
        },
        "discarded_small_clusters": discarded_small_clusters,
    }


def locate_session(root: Path) -> Path:
    if (root / "map_registration.pcd").is_file():
        return root
    candidates = sorted(
        path.parent for path in root.rglob("map_registration.pcd")
        if path.is_file()
    )
    if not candidates:
        raise FileNotFoundError(f"no map_registration.pcd below {root}")
    if len(candidates) > 1:
        candidates.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    return candidates[0]


def audit_session(
    session: Path,
    *,
    cell_size: float = 0.25,
    voxel_size: float = 0.25,
) -> dict[str, Any]:
    session = session.resolve()
    layers: dict[str, Any] = {}
    clouds: dict[str, np.ndarray] = {}
    for name, filename in FORMAL_LAYERS.items():
        path = session / filename
        if not path.is_file():
            layers[name] = {"present": False, "path": filename}
            continue
        xyz, pcd = read_pcd_xyz(path)
        finite_mask = np.isfinite(xyz).all(axis=1)
        finite = xyz[finite_mask]
        clouds[name] = finite
        layer: dict[str, Any] = {
            "present": True,
            "path": filename,
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
            "points": int(xyz.shape[0]),
            "finite_points": int(finite.shape[0]),
            "nonfinite_points": int(xyz.shape[0] - finite.shape[0]),
            "pcd": pcd,
            "occupied_xy_cells": _unique_grid_count(finite[:, :2], cell_size),
            "occupied_xyz_voxels": _unique_grid_count(finite, voxel_size),
        }
        if finite.size:
            layer["minimum_xyz"] = finite.min(axis=0).tolist()
            layer["maximum_xyz"] = finite.max(axis=0).tolist()
        layers[name] = layer

    identical: list[list[str]] = []
    present = [name for name, value in layers.items() if value["present"]]
    for index, lhs in enumerate(present):
        for rhs in present[index + 1:]:
            if layers[lhs]["sha256"] == layers[rhs]["sha256"]:
                identical.append([lhs, rhs])

    overlap: dict[str, Any] = {}
    for lhs, rhs in (
        ("objects_clean", "objects_raw"),
        ("objects_filtered", "objects_clean"),
        ("display_full", "objects_filtered"),
    ):
        if lhs in clouds and rhs in clouds:
            overlap[f"{lhs}_vs_{rhs}"] = point_set_overlap(
                clouds[lhs], clouds[rhs]
            )

    manifest_path = session / "manifest.yaml"
    static_evidence_path = session / "static_evidence.csv"
    keyframes = session / "keyframes"
    report = {
        "schema": "ndt_slam_real_map_audit",
        "schema_version": 1,
        "session_directory": str(session),
        "parameters": {
            "cell_size_m": cell_size,
            "voxel_size_m": voxel_size,
        },
        "contract": {
            "manifest_present": manifest_path.is_file(),
            "static_evidence_present": static_evidence_path.is_file(),
            "poses_raw_present": (session / "poses_raw.txt").is_file(),
            "keyframe_directory_present": keyframes.is_dir(),
            "keyframe_pcd_count": (
                sum(1 for _ in keyframes.glob("*.pcd")) if keyframes.is_dir()
                else 0
            ),
        },
        "layers": layers,
        "relationships": {
            "byte_identical_layers": identical,
            "point_set_overlap": overlap,
        },
        "static_height_field_projection": (
            project_static_height_layers(
                clouds["objects_clean"], cell_size=cell_size
            ) if "objects_clean" in clouds else None
        ),
    }
    report["findings"] = build_findings(report)
    return report


def build_findings(report: dict[str, Any]) -> list[dict[str, str]]:
    findings: list[dict[str, str]] = []
    contract = report["contract"]
    if not contract["manifest_present"]:
        findings.append({
            "severity": "error",
            "code": "SESSION_MANIFEST_MISSING",
            "detail": "layer generation, UUID and hashes cannot be proven",
        })
    if not contract["static_evidence_present"]:
        findings.append({
            "severity": "error",
            "code": "STATIC_EVIDENCE_MISSING",
            "detail": "clean PCD alone has no temporal/operator authority",
        })
    identical = report["relationships"]["byte_identical_layers"]
    if ["display_full", "objects_filtered"] in identical or [
        "objects_filtered", "display_full"
    ] in identical:
        findings.append({
            "severity": "warning",
            "code": "DISPLAY_FULL_IS_FILTERED_OBJECTS_ONLY",
            "detail": "legacy display_full is not a coherent full display layer",
        })
    mature_path = report["layers"].get("objects_clean", {})
    raw_path = report["layers"].get("objects_raw", {})
    if mature_path.get("present") and raw_path.get("present") and (
        mature_path["points"] > raw_path["points"]
    ):
        findings.append({
            "severity": "warning",
            "code": "CLEAN_EXCEEDS_RAW_POINT_COUNT",
            "detail": "inspect layer provenance and generation coherence",
        })
    return findings


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="session directory, root, or ZIP")
    parser.add_argument("--output", type=Path, help="write JSON here")
    parser.add_argument("--cell-size", type=float, default=0.25)
    parser.add_argument("--voxel-size", type=float, default=0.25)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.cell_size <= 0 or args.voxel_size <= 0:
        raise SystemExit("cell and voxel sizes must be positive")
    temporary: tempfile.TemporaryDirectory[str] | None = None
    try:
        source = args.input.resolve()
        if source.suffix.lower() == ".zip":
            temporary = tempfile.TemporaryDirectory(prefix="ndt-map-audit-")
            with zipfile.ZipFile(source) as archive:
                archive.extractall(temporary.name)
            root = Path(temporary.name)
        else:
            root = source
        session = locate_session(root)
        report = audit_session(
            session, cell_size=args.cell_size, voxel_size=args.voxel_size
        )
        try:
            report["session_directory"] = session.relative_to(root).as_posix()
        except ValueError:
            report["session_directory"] = session.name
        report["source"] = {
            "input": source.name,
            "archive_bytes": source.stat().st_size if source.is_file() else None,
            "archive_sha256": sha256_file(source) if source.is_file() else None,
        }
        text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text + "\n", encoding="utf-8")
        else:
            print(text)
    finally:
        if temporary is not None:
            temporary.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
