#!/usr/bin/env python3
"""Static gate for the Phase B1 Cargo vertical-evidence SHADOW boundary."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def main() -> int:
    failures: list[str] = []
    node = read("src/ndt_slam/src/ndt_slam.cpp")
    extractor = read("src/ndt_slam/src/cargo_vertical_evidence.cpp")
    config = read("src/ndt_slam/config/cargo_vertical_evidence_v2.yaml")
    cmake = read("src/ndt_slam/CMakeLists.txt")

    required = {
        "shadow config rejects product mode":
            "PRODUCT_MODE_NOT_IMPLEMENTED_IN_SHADOW_BUILD" in node,
        "shadow-only config is committed": "shadow_only: true" in config,
        "extractor is compiled": "src/cargo_vertical_evidence.cpp" in cmake,
        "extractor tests are registered":
            "cargo_vertical_evidence_test" in cmake,
        "product observation is evaluated independently":
            "cargo_bottom_fusion_.update(observation)" in node,
        "shadow uses a separate fusion instance":
            "cargo_bottom_shadow_fusion_.update(shadow_observation)" in node,
        "invalid ground is not defaulted":
            "input.ground_reference_valid" in extractor and
            "input.ground_z_base" in extractor,
        "thickness cannot create a surface":
            "no_supported_upper_surface" in extractor,
    }
    for label, satisfied in required.items():
        if not satisfied:
            failures.append(label)

    forbidden = {
        "shadow top assigned to product detection":
            "hook_fixed_cargo_.z95 = last_shadow" in node,
        "shadow result assigned to product bottom":
            "last_cargo_bottom_result_ = last_shadow" in node,
        "vertical helper changes identity scoring":
            "scoreCargoCandidateIdentity" in extractor,
        "vertical helper changes obstacle tracking":
            "CargoObstacleTracker" in extractor,
    }
    for label, present in forbidden.items():
        if present:
            failures.append(label)

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    print("PASS: Cargo Vertical Evidence V2 remains SHADOW-only")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
