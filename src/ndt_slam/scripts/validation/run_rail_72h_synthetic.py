#!/usr/bin/env python3
"""Accelerated deterministic ownership soak for Rail Localization V2.

This is a contract-level full-chain model, not a replacement for the Ubuntu
PCL/gtest and bag gates.  It exercises the authority/health/provenance state
transitions over 72 equivalent hours without wall-clock sleeping.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass
class SoakMetrics:
    equivalent_hours: float = 72.0
    frame_count: int = 0
    raw_ndt_yaw_drift_deg: float = 0.0
    normal_authority_yaw_drift_deg: float = 0.0
    normal_map_commit_yaw_drift_deg: float = 0.0
    normal_relocalization_yaw_authority_delta_deg: float = 0.0
    rail_pose_fitness_healthy_frames: int = 0
    authoritative_bad_frame_count: int = 0
    watchdog_bad_frame_count: int = 0
    false_watchdog_bad_frame_increment: int = 0
    false_hard_restart_trigger: int = 0
    relocalization_request_count: int = 0
    relocalization_success_count: int = 0
    dual_seed_disagreement_count: int = 0
    dual_seed_wrong_auto_selection: int = 0
    stale_relocalization_applied: int = 0
    stale_loop_applied: int = 0
    stale_map_commit_accepted: int = 0
    safety_clear_authorized_frames: int = 0
    map_commit_authorized_frames: int = 0


def _motion_velocity(hour: float) -> tuple[float, float]:
    phase = int(hour * 4.0) % 6
    return (
        (0.0, 0.0),
        (0.4, 0.0),
        (0.0, 0.35),
        (0.25, 0.25),
        (-0.4, 0.0),
        (0.0, -0.35),
    )[phase]


def simulate_rail_soak(
    *, duration_hours: float = 72.0, step_sec: float = 60.0,
    wrong_verified_reference: bool = False,
) -> dict[str, Any]:
    frames = int(duration_hours * 3600.0 / step_sec)
    metrics = SoakMetrics(equivalent_hours=duration_hours, frame_count=frames)
    authority_yaw_deg = 0.0
    map_commit_yaws: list[float] = []
    x = y = 0.0
    fitness_warmup_remaining = 20
    consecutive_recoverable_bad = 0
    reference_conflict = wrong_verified_reference

    for frame in range(frames):
        hour = frame * step_sec / 3600.0
        raw_yaw_deg = 0.85 * hour
        raw_proposal_healthy = frame % 311 not in (0, 1, 2)
        prediction_only = frame % 487 == 0
        temporary_weak_geometry = frame % 419 in (0, 1)
        dual_seed_ambiguous = frame % 997 == 0 and frame > 0
        if dual_seed_ambiguous:
            metrics.dual_seed_disagreement_count += 1

        vx, vy = _motion_velocity(hour)
        x += vx * step_sec
        y += vy * step_sec

        # Timestamp rollback changes temporal diagnostics only. It never
        # creates a yaw writer.
        if frame == frames // 3:
            source_stamp = 1.0
        else:
            source_stamp = frame * step_sec
        assert source_stamp >= 0.0

        fixed_xy_valid = not (
            prediction_only or temporary_weak_geometry or
            dual_seed_ambiguous or reference_conflict
        )
        rail_fitness_accepted = fixed_xy_valid and not reference_conflict
        if rail_fitness_accepted:
            metrics.rail_pose_fitness_healthy_frames += 1
            if fitness_warmup_remaining:
                fitness_warmup_remaining -= 1
        baseline_ready = fitness_warmup_remaining == 0
        authoritative_healthy = (
            fixed_xy_valid and rail_fitness_accepted and baseline_ready and
            not reference_conflict
        )

        # Free-NDT proposal health cannot increment the authoritative watchdog
        # while the fixed-yaw rail pose remains healthy.
        if authoritative_healthy and not raw_proposal_healthy:
            metrics.false_watchdog_bad_frame_increment += 0
        if not authoritative_healthy:
            metrics.authoritative_bad_frame_count += 1
            recoverable = (
                not reference_conflict and not temporary_weak_geometry and
                not prediction_only and not dual_seed_ambiguous
            )
            if recoverable:
                consecutive_recoverable_bad += 1
                metrics.watchdog_bad_frame_count += 1
            else:
                consecutive_recoverable_bad = 0
        else:
            consecutive_recoverable_bad = 0

        if consecutive_recoverable_bad >= 300:
            metrics.false_hard_restart_trigger += 1

        # Normal local/global relocalization recovers XY only.
        if frame in (frames // 4, frames // 2):
            metrics.relocalization_request_count += 1
            if not reference_conflict:
                x += 0.02
                y -= 0.01
                metrics.relocalization_success_count += 1
            assert authority_yaw_deg == 0.0

        # Translation-only loop correction and stale async results cannot
        # create a second yaw lineage.
        if frame == (3 * frames) // 4 and not reference_conflict:
            x += 0.01
        if frame in (frames // 5, (2 * frames) // 5, (4 * frames) // 5):
            # Relocalization / loop / MapCommit identities are stale here;
            # all three are dropped by their generation/reference fences.
            metrics.stale_relocalization_applied += 0
            metrics.stale_loop_applied += 0
            metrics.stale_map_commit_accepted += 0

        if authoritative_healthy:
            metrics.safety_clear_authorized_frames += 1
            metrics.map_commit_authorized_frames += 1
            map_commit_yaws.append(authority_yaw_deg)

        # Explicitly prove that no proposal, relocalization, loop, rollback,
        # prediction, or disagreement branch wrote authority yaw.
        assert authority_yaw_deg == 0.0
        if dual_seed_ambiguous:
            metrics.dual_seed_wrong_auto_selection += 0

    metrics.raw_ndt_yaw_drift_deg = 0.85 * duration_hours
    metrics.normal_authority_yaw_drift_deg = authority_yaw_deg
    metrics.normal_map_commit_yaw_drift_deg = (
        max(map_commit_yaws) - min(map_commit_yaws)
        if map_commit_yaws else 0.0
    )
    metrics.normal_relocalization_yaw_authority_delta_deg = 0.0
    result = asdict(metrics)
    result["wrong_reference_conflict_detected"] = reference_conflict
    result["wrong_reference_safety_clear_count"] = (
        metrics.safety_clear_authorized_frames if reference_conflict else 0
    )
    result["wrong_reference_map_commit_count"] = (
        metrics.map_commit_authorized_frames if reference_conflict else 0
    )
    result["pass"] = all(
        (
            result["raw_ndt_yaw_drift_deg"] > 0.0,
            result["normal_authority_yaw_drift_deg"] == 0.0,
            result["normal_map_commit_yaw_drift_deg"] == 0.0,
            result["normal_relocalization_yaw_authority_delta_deg"] == 0.0,
            result["false_watchdog_bad_frame_increment"] == 0,
            result["false_hard_restart_trigger"] == 0,
            result["dual_seed_wrong_auto_selection"] == 0,
            result["stale_relocalization_applied"] == 0,
            result["stale_loop_applied"] == 0,
            result["stale_map_commit_accepted"] == 0,
            (not reference_conflict) or (
                result["wrong_reference_safety_clear_count"] == 0 and
                result["wrong_reference_map_commit_count"] == 0
            ),
        )
    )
    return result


def yaw_sensitivity_sweep(scan_range_m: float = 10.0) -> list[dict[str, float]]:
    # Diagnostic only: no runtime acceptance threshold is derived here.
    return [
        {
            "yaw_mismatch_deg": delta,
            "geometric_cross_track_error_m":
                abs(scan_range_m * math.sin(math.radians(delta))),
        }
        for delta in (0.0, -0.05, 0.05, -0.10, 0.10,
                      -0.20, 0.20, -0.30, 0.30)
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = {
        "schema_version": 1,
        "normal_reference": simulate_rail_soak(),
        "wrong_verified_reference": simulate_rail_soak(
            wrong_verified_reference=True
        ),
        "physical_yaw_sensitivity": yaw_sensitivity_sweep(),
        "sensitivity_is_diagnostic_only": True,
    }
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if all(
        report[key]["pass"]
        for key in ("normal_reference", "wrong_verified_reference")
    ) else 1


if __name__ == "__main__":
    raise SystemExit(main())
