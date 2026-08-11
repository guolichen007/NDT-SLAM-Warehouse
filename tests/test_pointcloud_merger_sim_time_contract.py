import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MERGER = ROOT / "src" / "ndt_slam" / "src" / "PointCloudMerger.cpp"


class PointCloudMergerSimTimeContractTest(unittest.TestCase):
    def test_publish_loop_is_independent_of_sim_time(self):
        source = MERGER.read_text(encoding="utf-8")
        self.assertIn("createWallTimer", source)
        self.assertIn("ros::WallTimerEvent", source)
        self.assertIn("ros::WallTimer timer_", source)
        self.assertNotIn("nh_.createTimer", source)

    def test_output_keeps_sensor_timestamp(self):
        source = MERGER.read_text(encoding="utf-8")
        self.assertIn("output.header.stamp = latest_stamp", source)


if __name__ == "__main__":
    unittest.main()
