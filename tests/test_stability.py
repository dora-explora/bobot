import csv
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from robot.imu import IMUSnapshot
from robot.stability import StabilityScorer


def attitude(roll, pitch, timestamp=0.0, connected=True):
    return IMUSnapshot(
        connected=connected,
        error="",
        timestamp=timestamp,
        roll_delta_degrees=roll,
        pitch_delta_degrees=pitch,
    )


class StabilityScorerTests(unittest.TestCase):
    def test_rules_formula_uses_rms_of_delta_roll_and_pitch(self):
        scorer = StabilityScorer(poll_hz=20.0, log_enabled=False)
        scorer.start(0.0)

        self.assertTrue(scorer.update(attitude(3.0, 4.0), 0.0))
        self.assertTrue(scorer.update(attitude(0.0, 12.0), 0.05))

        expected_rms = math.sqrt((25.0 + 144.0) / 2.0)
        snapshot = scorer.snapshot(0.05)
        self.assertEqual(snapshot.sample_count, 2)
        self.assertAlmostEqual(snapshot.current_tilt_degrees, 12.0)
        self.assertAlmostEqual(snapshot.rms_tilt_degrees, expected_rms)
        self.assertAlmostEqual(snapshot.score_points, -2.5 * expected_rms)

    def test_fractional_poll_rate_limits_sampling(self):
        scorer = StabilityScorer(poll_hz=2.5, log_enabled=False)
        scorer.start(0.0)

        self.assertTrue(scorer.update(attitude(1.0, 0.0), 0.0))
        self.assertFalse(scorer.update(attitude(2.0, 0.0), 0.2))
        self.assertTrue(scorer.update(attitude(3.0, 0.0), 0.4))

        self.assertEqual(scorer.sample_count, 2)
        self.assertEqual(scorer.poll_hz, 2.5)

    def test_toggle_starts_fresh_and_then_finalizes(self):
        scorer = StabilityScorer(log_enabled=False)

        scorer.toggle(1.0)
        scorer.update(attitude(3.0, 4.0), 1.0)
        scorer.toggle(2.0)
        final = scorer.snapshot(5.0)

        self.assertEqual(final.status, "final")
        self.assertEqual(final.sample_count, 1)
        self.assertEqual(final.elapsed_seconds, 1.0)

        scorer.toggle(6.0)
        fresh = scorer.snapshot(6.0)
        self.assertEqual(fresh.status, "running")
        self.assertEqual(fresh.sample_count, 0)

    def test_invalid_imu_sample_is_not_scored(self):
        scorer = StabilityScorer(log_enabled=False)
        scorer.start(0.0)

        self.assertFalse(scorer.update(attitude(None, None, connected=False), 0.0))

        snapshot = scorer.snapshot(0.0)
        self.assertEqual(snapshot.sample_count, 0)
        self.assertIn("waiting for valid", snapshot.error)

    def test_logging_disabled_creates_nothing(self):
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "stability"
            scorer = StabilityScorer(log_enabled=False, log_dir=path)

            scorer.start(0.0)
            scorer.update(attitude(1.0, 2.0), 0.0)
            scorer.stop(1.0)

            self.assertFalse(path.exists())

    def test_optional_csv_contains_each_scored_sample(self):
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "stability"
            scorer = StabilityScorer(
                poll_hz=20.0,
                log_enabled=True,
                log_dir=path,
            )

            scorer.start(0.0)
            scorer.update(attitude(3.0, 4.0, timestamp=10.0), 0.0)
            scorer.update(attitude(0.0, 6.0, timestamp=10.05), 0.05)
            scorer.stop(1.0)

            files = list(path.glob("*.csv"))
            self.assertEqual(len(files), 1)
            with files[0].open(newline="", encoding="utf-8") as log_file:
                rows = list(csv.DictReader(log_file))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["tilt_degrees"], "5.0")
            self.assertAlmostEqual(
                float(rows[1]["stability_score_points"]),
                -2.5 * math.sqrt((25.0 + 36.0) / 2.0),
            )

    def test_csv_failure_remains_visible_but_does_not_stop_scoring(self):
        with TemporaryDirectory() as temporary_directory:
            blocked_path = Path(temporary_directory) / "not-a-directory"
            blocked_path.write_text("file", encoding="utf-8")
            scorer = StabilityScorer(log_enabled=True, log_dir=blocked_path)

            scorer.start(0.0)
            self.assertTrue(scorer.update(attitude(3.0, 4.0), 0.0))

            snapshot = scorer.snapshot(0.0)
            self.assertEqual(snapshot.sample_count, 1)
            self.assertIn("CSV unavailable", snapshot.error)

    def test_future_washboard_state_starts_and_stops_automatically(self):
        scorer = StabilityScorer(log_enabled=False)

        scorer.sync_state("manual", 0.0)
        scorer.sync_state("washboard", 1.0)
        self.assertTrue(scorer.running)
        self.assertEqual(scorer.source, "state:washboard")

        scorer.sync_state("washboard", 2.0)
        self.assertTrue(scorer.running)

        scorer.sync_state("hill_climb", 3.0)
        self.assertFalse(scorer.running)
        self.assertEqual(scorer.status, "final")


if __name__ == "__main__":
    unittest.main()
