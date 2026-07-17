import unittest
from pathlib import Path


NODE = (Path(__file__).resolve().parents[1] / "src" / "ndt_slam" / "src" /
        "ndt_slam.cpp").read_text(encoding="utf-8")


class StaticManifestContractTest(unittest.TestCase):
    def test_ImmatureSnapshotDoesNotReplaceLastGoodManifest(self):
        start = NODE.index("bool NdtSlamNode::writePersistentStaticEvidence()")
        end = NODE.index("bool NdtSlamNode::suspendPersistentStaticEvidence", start)
        body = NODE[start:end]
        maturity_gate = body.index("mature_cells < minimum_persisted_cells")
        manifest_commit = body.index("std::rename(temporary.c_str(), manifest_path.c_str())")
        last_good_remove = body.index("static_evidence_manifest.last_good.json")
        self.assertLess(maturity_gate, manifest_commit)
        self.assertLess(manifest_commit, last_good_remove)
        self.assertIn("PENDING_TEMPORAL_MATURITY", body)

    def test_MatureCurrentEpochActivatesManifest(self):
        start = NODE.index("bool NdtSlamNode::writePersistentStaticEvidence()")
        end = NODE.index("bool NdtSlamNode::suspendPersistentStaticEvidence", start)
        body = NODE[start:end]
        self.assertIn('"  \\"mature_cells\\": " << mature_cells', body)
        self.assertIn("activation=MATURE_CURRENT_EPOCH", body)
        self.assertIn("static_evidence_manifest_active_ = true", body)

    def test_ManifestRenameFailureCannotLeaveOldManifestLoadable(self):
        start = NODE.index("bool NdtSlamNode::suspendPersistentStaticEvidence")
        end = NODE.index("void NdtSlamNode::flushDirtyTiles", start)
        body = NODE[start:end]
        marker_commit = body.index("suspension_marker_path.c_str())")
        archive_copy = body.index("boost::filesystem::copy_file")
        self.assertLess(marker_commit, archive_copy)
        self.assertIn("active file remains blocked by suspension marker", body)
        self.assertIn("return suspended", body)

    def test_SuspendedManifestIsNotLoadedAfterRestart(self):
        start = NODE.index("bool NdtSlamNode::loadPersistentStaticEvidence()")
        end = NODE.index("bool NdtSlamNode::writePersistentStaticEvidence()", start)
        body = NODE[start:end]
        marker_gate = body.index("is_regular_file(suspension_marker_path)")
        manifest_parse = body.index("YAML::LoadFile(manifest_path)")
        self.assertLess(marker_gate, manifest_parse)
        self.assertIn("suspension marker active; refusing", body)
        self.assertIn("previous epoch manifest until current epoch matures", body)


if __name__ == "__main__":
    unittest.main()
