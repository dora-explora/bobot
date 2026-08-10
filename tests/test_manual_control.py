import unittest
from unittest.mock import patch

import manual_control


class FakeWatchdog:
    def __init__(self, outputs, initial_pulses, failsafe_pulses, *_args):
        self.outputs = outputs
        self.initial_pulses = initial_pulses
        self.failsafe_pulses = failsafe_pulses
        self.applied = []

    def apply_pulses(self, pulses):
        self.applied.append(pulses)

    def close(self):
        pass


class ManualControlOutputTests(unittest.TestCase):
    def test_output_includes_motor_and_suspension_channels(self):
        with patch.object(manual_control, "Pca9685Watchdog", FakeWatchdog):
            output = manual_control.Pca9685Output()

        output.apply(0.0, 0.0, "raised")

        latest = output.watchdog.applied[-1]
        self.assertIn("front_left", latest)
        if output.suspension_enabled:
            self.assertIn("suspension_front_left", latest)
            self.assertEqual(output.suspension_state, "raised")

    def test_watchdog_uses_configured_suspension_failsafe(self):
        with patch.object(manual_control, "Pca9685Watchdog", FakeWatchdog):
            output = manual_control.Pca9685Output()

        if output.suspension_enabled:
            expected = manual_control.suspension_pulses(
                manual_control.config.SUSPENSION_FAILSAFE_STATE
            )["rear_right"]
            self.assertEqual(
                output.watchdog.failsafe_pulses["suspension_rear_right"],
                expected,
            )


if __name__ == "__main__":
    unittest.main()
