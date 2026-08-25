import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (
    ROOT
    / "src/ndt_slam/scripts/validation/run_rail_72h_synthetic.py"
)


def _module():
    spec = importlib.util.spec_from_file_location("rail_soak", SCRIPT)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_72h_raw_drift_cannot_enter_authority_map_or_watchdog() -> None:
    result = _module().simulate_rail_soak()
    assert result["pass"]
    assert result["raw_ndt_yaw_drift_deg"] > 60.0
    assert result["normal_authority_yaw_drift_deg"] == 0.0
    assert result["normal_map_commit_yaw_drift_deg"] == 0.0
    assert result["false_watchdog_bad_frame_increment"] == 0
    assert result["false_hard_restart_trigger"] == 0
    assert result["dual_seed_disagreement_count"] > 0
    assert result["dual_seed_wrong_auto_selection"] == 0


def test_wrong_verified_reference_fails_closed_without_self_healing() -> None:
    result = _module().simulate_rail_soak(wrong_verified_reference=True)
    assert result["pass"]
    assert result["wrong_reference_conflict_detected"]
    assert result["wrong_reference_safety_clear_count"] == 0
    assert result["wrong_reference_map_commit_count"] == 0
    assert result["normal_authority_yaw_drift_deg"] == 0.0


def test_physical_skew_sensitivity_is_diagnostic_not_authority() -> None:
    sweep = _module().yaw_sensitivity_sweep()
    assert {item["yaw_mismatch_deg"] for item in sweep} == {
        0.0, -0.05, 0.05, -0.10, 0.10, -0.20, 0.20, -0.30, 0.30
    }
    assert all(item["geometric_cross_track_error_m"] >= 0.0 for item in sweep)
