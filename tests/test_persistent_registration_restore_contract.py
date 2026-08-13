import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
NODE = ROOT / "src" / "ndt_slam" / "src" / "ndt_slam.cpp"
LOADER = (
    ROOT
    / "src"
    / "ndt_slam"
    / "src"
    / "persistent_registration_loader.cpp"
)


class PersistentRegistrationRestoreContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = NODE.read_text(encoding="utf-8")
        cls.loader = LOADER.read_text(encoding="utf-8")

    def test_restore_finishes_before_workers_start(self):
        for constructor_tail in self.node.split(
            "NdtSlamNode::NdtSlamNode("
        )[1:3]:
            restore = constructor_tail.index(
                "restorePersistentRegistrationTarget();"
            )
            relocalizer = constructor_tail.index("relocalizer_.start();")
            process_thread = constructor_tail.index(
                "std::thread(&NdtSlamNode::processCloudThread, this)"
            )
            self.assertLess(restore, relocalizer)
            self.assertLess(restore, process_thread)

    def test_restored_map_skips_new_map_bootstrap(self):
        restore = self.node.split(
            "bool NdtSlamNode::restorePersistentRegistrationTarget()", 1
        )[1].split(
            "bool NdtSlamNode::publishPersistentDisplayMapFromTiles()", 1
        )[0]
        self.assertIn("global_map_", restore)
        self.assertIn("local_map_", restore)
        self.assertIn("bootstrap_local_map_complete_ = true", restore)
        self.assertIn("bindNdtInputTarget", restore)
        self.assertIn("INVALID_EXISTING_MAP", restore)
        self.assertIn("throw std::runtime_error", restore)

    def test_loader_validates_authoritative_manifest(self):
        for token in (
            '"schema"',
            '"schema_version"',
            '"map_uuid"',
            '"tile_size_m"',
            '"sha256"',
            "MapSessionSnapshot::sha256File",
            "persistent_registration_catalog_mismatch",
        ):
            self.assertIn(token, self.loader)


if __name__ == "__main__":
    unittest.main()
