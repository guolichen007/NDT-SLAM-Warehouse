from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(encoding="utf-8")
STATIC_TEST = (
    ROOT / "src/ndt_slam/test/static_obstacle_evidence_index_test.cpp"
).read_text(encoding="utf-8")
KEYFRAME_TEST = (
    ROOT / "src/ndt_slam/test/relocalization_scan_context_test.cpp"
).read_text(encoding="utf-8")


def section(start: str, end: str) -> str:
    first = NODE.index(start)
    last = NODE.index(end, first)
    return NODE[first:last]


class MapLoadTransactionContractTest(unittest.TestCase):
    def test_PreparedInstallFailureLeavesOldIdentity(self):
        self.assertIn("PreparedInstallFailureLeavesOldIdentity", STATIC_TEST)
        self.assertIn("prepareSnapshotInstall", STATIC_TEST)

    def test_StaticSuspendFailureBlocksInstall(self):
        body = section(
            "bool NdtSlamNode::installLoadedRuntimeMap(",
            "bool NdtSlamNode::loadMapSessionService(",
        )
        suspension = body.index('suspendPersistentStaticEvidence("load_map")')
        failure_return = body.index("return true;", suspension)
        first_map_swap = body.index("global_map_ =")
        self.assertLess(suspension, failure_return)
        self.assertLess(failure_return, first_map_swap)

    def test_StaticRestoreFailureLeavesOldMap(self):
        stage = section(
            "LoadedRuntimeMapCandidate NdtSlamNode::stageRuntimeMap(",
            "bool NdtSlamNode::loadMapService(",
        )
        install = section(
            "bool NdtSlamNode::installLoadedRuntimeMap(",
            "bool NdtSlamNode::loadMapSessionService(",
        )
        self.assertIn("prepareSnapshotInstall(", stage)
        self.assertNotIn("restoreSnapshotWithoutRevisionIncrement(", install)
        self.assertNotIn("loadSnapshotCandidate(", install)

    def test_KeyframeInstallCandidateValidatedBeforeMutation(self):
        self.assertIn(
            "KeyframeInstallCandidateValidatedBeforeMutation", KEYFRAME_TEST
        )
        self.assertIn("prepareKeyFrameDatabase(", KEYFRAME_TEST)

    def test_SuccessfulInstallChangesAllIdentityFieldsTogether(self):
        body = section(
            "bool NdtSlamNode::installLoadedRuntimeMap(",
            "bool NdtSlamNode::loadMapSessionService(",
        )
        required = (
            "global_map_ =",
            "installPreparedSnapshot(",
            "static_height_field_",
            "loaded_map_session_uuid_.swap(",
            "loaded_map_session_generation_ =",
            "installPreparedKeyFrameDatabase(",
        )
        positions = [body.index(token) for token in required]
        success = body.index("response.success = true;")
        self.assertTrue(all(position < success for position in positions))


if __name__ == "__main__":
    unittest.main()
