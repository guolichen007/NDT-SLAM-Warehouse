from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(encoding="utf-8")
FUSION = (
    ROOT / "src/ndt_slam/include/ndt_slam/cargo_geometry_fusion.hpp"
).read_text(encoding="utf-8")
CONFIG = (
    ROOT / "src/ndt_slam/config/live_longterm_mapping.yaml"
).read_text(encoding="utf-8")


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

    def test_BottomEstimatorReusesBoundedDetectorVerticalExtent(self):
        body = section(
            "NdtSlamNode::HookCargoBottomEstimate "
            "NdtSlamNode::estimateCargoBottom(",
            "void NdtSlamNode::publishSelectedCorePoints(",
        )
        self.assertIn("const float z05 = detection.z05;", body)
        self.assertIn("const float z95 = detection.z95;", body)
        self.assertIn(
            "raw_visible_height, odom_anchor_config_.min_size_z,", body
        )
        self.assertIn("odom_anchor_config_.max_size_z);", body)
        self.assertIn("result.bottom_z_base = z95 - visible_height;", body)
        self.assertNotIn("z_values.size() * 0.05", body)
        self.assertNotIn("z_values.size() * 0.95", body)

    def test_PendingShapeRejectsClampedObbAndBoundsAllAxes(self):
        body = section(
            "void NdtSlamNode::updateCargoLiftAndGeometryFusion(",
            "void NdtSlamNode::runPendingCargoAvoidance(",
        )
        self.assertIn(
            "!hook_fixed_cargo_.oriented_footprint_clamped", body
        )
        self.assertIn(
            "const bool weak_shape_evidence = shape.valid &&", body
        )
        self.assertIn(
            "shape.height_m = std::clamp(\n"
            "            measured_height, odom_anchor_config_.min_size_z,\n"
            "            odom_anchor_config_.max_size_z);",
            body,
        )
        self.assertIn(
            "origin.top_z95_map - origin.support_z_map,\n"
            "            odom_anchor_config_.min_size_z,\n"
            "            odom_anchor_config_.max_size_z);",
            body,
        )
        self.assertIn(
            "cargo_lift_origin_result_.origin.length_m,\n"
            "                odom_anchor_config_.max_size_x);",
            body,
        )

    def test_NewEpisodeStartsFromConservativeBoundedShape(self):
        body = section(
            "void NdtSlamNode::updateCargoLiftAndGeometryFusion(",
            "void NdtSlamNode::runPendingCargoAvoidance(",
        )
        self.assertIn("const Eigen::Vector3f configured_seed(", body)
        self.assertIn("measured_size.cwiseMin(configured_seed);", body)
        self.assertNotIn(
            "pending_cargo_shape_continuity_.size_m = measured_size;", body
        )

    def test_ProvisionalRvizMarkerCannotBypassPhysicalBounds(self):
        body = section(
            "visualization_msgs::Marker provisional_marker;",
            "last_anchor_detect_stamp_ = msg->header.stamp;",
        )
        self.assertIn(
            "!hook_fixed_cargo_.oriented_footprint_clamped", body
        )
        self.assertIn("hook_fixed_cargo_.size_visible.z()", body)
        self.assertNotIn("hook_fixed_cargo_.visible_height", body)

    def test_RuntimeHeightAndLengthCapsCloseAlternatePaths(self):
        fusion_config = CONFIG.split("cargo_geometry_fusion:", 1)[1].split(
            "cargo_recognition:", 1
        )[0]
        self.assertIn("float maximum_height_m = 2.00F;", FUSION)
        self.assertIn("maximum_height_m: 2.00", fusion_config)
        self.assertIn("max_size_x: 3.50", CONFIG)
        self.assertIn(
            "cargo_geometry_fusion_config_.maximum_height_m >",
            NODE,
        )
        self.assertIn(
            "cargo_geometry_fusion.maximum_height_m:"
            "exceeds_odom_anchor_max_size_z",
            NODE,
        )
        self.assertNotIn(
            "cargo_geometry_fusion_config_.maximum_height_m = std::min(",
            NODE,
        )


if __name__ == "__main__":
    unittest.main()
