from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(encoding="utf-8")


class StartupMapDiagnosticsContractTest(unittest.TestCase):
    def test_required_lifecycle_events_are_present(self):
        for event in (
            "PROCESS_START",
            "REGISTRATION_TARGET_READY",
            "FIRST_CLOUD",
            "FIRST_NDT_BEGIN",
            "FIRST_NDT_RESULT",
            "FIRST_LOCALIZATION_ACCEPTED",
            "RELOCALIZER_FIRST_ACTIVE_SEARCH",
            "MAP_COMMIT_REARM",
            "FIRST_PERSISTENT_WRITE",
        ):
            self.assertIn(f"[StartupMap] {event}", NODE)

    def test_relocalizer_event_follows_successful_submission(self):
        submit = NODE.index("!relocalizer_.submit(std::move(job))")
        event = NODE.index(
            '"[StartupMap] RELOCALIZER_FIRST_ACTIVE_SEARCH', submit
        )
        self.assertLess(submit, event)

    def test_persistent_write_event_follows_atomic_tile_replace(self):
        replace = NODE.index(
            "std::rename(tmp_path.c_str(), filepath.c_str())"
        )
        event = NODE.index('"[StartupMap] FIRST_PERSISTENT_WRITE', replace)
        self.assertLess(replace, event)


if __name__ == "__main__":
    unittest.main()
