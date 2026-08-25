from pathlib import Path

import csv
import importlib.util


ROOT = Path(__file__).resolve().parents[1]


def _load_analyzer():
    path = (
        ROOT
        / "src/ndt_slam/scripts/analysis/"
        "analyze_integrated_cargo_identity_shadow.py"
    )
    spec = importlib.util.spec_from_file_location("combined_analyzer", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_combined_handoff_enforces_one_four_bag_attempt() -> None:
    runner = (
        ROOT
        / "src/ndt_slam/scripts/validation/run_p0_combined_ubuntu_acceptance.sh"
    ).read_text(encoding="utf-8")
    assert "combined_four_bag_attempt.marker" in runner
    assert "reason=already_attempted" in runner
    assert "COMBINED_FOUR_BAG_ATTEMPT_COUNT=1" in runner
    assert "run_integrated_cargo_identity_shadow_four_bags.sh" in runner
    assert "--yaw-reference" not in runner or "yaw_reference" in runner


def test_replay_harness_freezes_rail_mode_from_reference() -> None:
    runner = (
        ROOT / "src/ndt_slam/scripts/ops/server_monitor_bag_validate.sh"
    ).read_text(encoding="utf-8")
    assert "--yaw-reference" in runner
    assert "'mode': 'RAIL_AUTHORITY'" in runner
    assert "invalid frozen yaw reference" in runner


def test_handoff_keeps_product_and_field_claims_closed() -> None:
    handoff = (
        ROOT / "src/ndt_slam/doc/p0_combined_ubuntu_acceptance.md"
    ).read_text(encoding="utf-8")
    assert "FIELD_READY=NO" in handoff
    assert "no Cargo product takeover" in handoff
    assert "no `systemctl enable/start`" in handoff


def test_runtime_report_exposes_rail_timing_percentiles(tmp_path: Path) -> None:
    analyzer = _load_analyzer()
    sample = tmp_path / "runtime_samples.csv"
    fields = [
        "cpu_percent", "rss_mb", "runtime_fixed_yaw_solver_ms",
        "runtime_rail_pose_fitness_ms",
        "runtime_target_normal_cache_build_ms",
        "runtime_rail_graph_worker_ms", "runtime_average_process_time_ms",
    ]
    with sample.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow(dict(zip(fields, [10, 100, 1, 2, 3, 4, 5])))
        writer.writerow(dict(zip(fields, [20, 200, 2, 3, 4, 5, 6])))
    result = analyzer.analyze_runtime(sample)
    for name in (
        "fixed_yaw_solver_ms", "rail_pose_fitness_ms",
        "target_normal_cache_build_ms", "rail_graph_worker_ms",
        "whole_frame_ms",
    ):
        assert result[f"{name}_p50"] >= 0.0
        assert result[f"{name}_p95"] >= result[f"{name}_p50"]
        assert result[f"{name}_max"] >= result[f"{name}_p95"]
