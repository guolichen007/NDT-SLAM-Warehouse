import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "src/ndt_slam/scripts/analysis/analyze_map_yaw_consistency.py"


def _module():
    spec = importlib.util.spec_from_file_location("map_yaw_audit", SCRIPT)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_axial_180_degree_equivalent_directions_remain_consistent() -> None:
    module = _module()
    assert module.axial_delta_deg(180.0, 0.0) == 0.0
    assert module.axial_delta_deg(-180.0, 0.0) == 0.0
    assert module.axial_delta_deg(179.0, 1.0) == -2.0


def test_orthogonal_warehouse_structure_cannot_fake_map_yaw_skew() -> None:
    module = _module()
    points = []
    for index in range(-20, 21):
        points.append((index * 0.1, 0.0, 0.0))
        points.append((0.0, index * 0.1, 0.0))
    result = module.estimate_axial_orientation(points, reference_deg=0.0)
    assert result["status"] == "INCONCLUSIVE"
    assert result["reason"] == "orthogonal_or_isotropic_structure"


def test_reference_disambiguates_principal_and_orthogonal_axis() -> None:
    module = _module()
    assert module.choose_axial_orientation_deg(90.0, 0.0) == 0.0
    assert module.choose_axial_orientation_deg(2.0, 0.0) == 2.0
