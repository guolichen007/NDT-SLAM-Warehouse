import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NDT_CPP = ROOT / "src" / "ndt_slam" / "src" / "ndt_slam.cpp"
AUTHORITY_CPP = (ROOT / "src" / "ndt_slam" / "src" /
                 "crane_yaw_authority.cpp")
CONFIG = ROOT / "src" / "ndt_slam" / "config" / "crane_yaw_authority.yaml"


class YawAuthorityShadowContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = NDT_CPP.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")

    def test_checked_in_config_cannot_take_runtime_pose(self):
        self.assertRegex(self.config, r"apply_to_runtime_pose:\s*false")
        self.assertRegex(self.config, r"shadow_only:\s*true")
        authority_source = AUTHORITY_CPP.read_text(encoding="utf-8")
        self.assertIn("PRODUCT_MODE_NOT_IMPLEMENTED_IN_SHADOW_BUILD",
                      authority_source)

    def test_shadow_result_never_enters_product_pose_or_ekf(self):
        forbidden = (
            r"current_pose_\s*=\s*last_rail_refinement_",
            r"updateWithNDT\s*\(\s*last_rail_refinement_",
            r"addFrameToMap\s*\([^;]*last_rail_refinement_",
            r"enqueueMapCommitJob\s*\([^;]*last_rail_refinement_",
        )
        for pattern in forbidden:
            self.assertIsNone(re.search(pattern, self.source, re.DOTALL),
                              pattern)

    def test_ndt_average_updates_only_after_align_attempt(self):
        align = self.source.index("ndt_->align(aligned, initial_guess)")
        average = self.source.index("average_ndt_time_ms_ +=", align)
        next_convergence = self.source.index(
            "last_ndt_converged_ = ndt_->hasConverged()", align)
        self.assertLess(align, average)
        self.assertLess(average, next_convergence)

    def test_phase_one_has_no_map_yaw_metadata_writer(self):
        self.assertNotIn("yaw_reference\"", self.source)
        self.assertNotIn("map_uuid", self.config)


if __name__ == "__main__":
    unittest.main()
