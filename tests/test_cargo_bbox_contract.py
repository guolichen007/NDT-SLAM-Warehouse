from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(encoding="utf-8")


def section(start: str, end: str) -> str:
    first = NODE.index(start)
    last = NODE.index(end, first)
    return NODE[first:last]


class CargoBoundingBoxContractTest(unittest.TestCase):
    def test_VerticalExtentUsesConfiguredRobustPercentiles(self):
        body = section(
            "NdtSlamNode::HookCargoDetection "
            "NdtSlamNode::detectCargoAroundOdomAnchor(",
            "uint64_t NdtSlamNode::computeCloudHash(",
        )
        self.assertIn(
            "float z05 = zs[static_cast<int>(n * p_low)];", body
        )
        self.assertIn(
            "float z95 = zs[static_cast<int>(n * p_high)];", body
        )
        self.assertNotIn("float z05 = zs[0];", body)
        self.assertNotIn("float z95 = zs[n - 1];", body)


if __name__ == "__main__":
    unittest.main()
