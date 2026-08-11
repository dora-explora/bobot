import unittest
from unittest.mock import patch
import os

from robot import config
from robot.actuators import Pca9685Actuators, suspension_pulses
from robot.controller import ControllerUpdate
from robot.models import DriveCommand
from robot.suspension import SuspensionControl


class SuspensionControlTests(unittest.TestCase):
    def test_millisecond_config_accepts_fractions_and_legacy_microseconds(self):
        with patch.dict(os.environ, {"TEST_SERVO_MS": "1.75"}, clear=False):
            self.assertEqual(config.env_servo_ms("TEST_SERVO_MS", "1.5"), 1.75)
        with patch.dict(os.environ, {"TEST_SERVO_US": "1800"}, clear=False):
            self.assertEqual(
                config.env_servo_ms("MISSING_SERVO_MS", "1.5", "TEST_SERVO_US"),
                1.8,
            )

    def test_each_position_uses_its_per_corner_calibration(self):
        outputs = (
            ("front_left", 12, 1.1, 1.9),
            ("front_right", 13, 1.2, 1.8),
            ("rear_left", 14, 1.3, 1.7),
            ("rear_right", 15, 1.4, 1.6),
        )
        with patch.object(config, "SUSPENSION_OUTPUTS", outputs):
            self.assertEqual(suspension_pulses("bottomed")["front_left"], 1100)
            self.assertEqual(suspension_pulses("bottomed")["rear_right"], 1400)
            self.assertEqual(suspension_pulses("raised")["front_left"], 1900)
            self.assertEqual(suspension_pulses("raised")["rear_right"], 1600)
            front = suspension_pulses("front_bottomed")
            self.assertEqual(front["front_left"], 1100)
            self.assertEqual(front["rear_left"], 1700)
            rear = suspension_pulses("rear_bottomed")
            self.assertEqual(rear["front_right"], 1800)
            self.assertEqual(rear["rear_right"], 1400)

    def test_disabled_actuators_still_report_requested_position(self):
        with patch.object(config, "ENABLE_ACTUATORS", False):
            actuators = Pca9685Actuators()

        actuators.apply(DriveCommand(), "raised")

        self.assertEqual(actuators.suspension_state, "raised")

    def test_bumpers_select_positions_in_manual_mode(self):
        suspension = SuspensionControl("bottomed")

        message = suspension.update(
            "manual",
            ControllerUpdate(right_bumper_pressed=True),
        )
        self.assertEqual(suspension.state, "raised")
        self.assertIn("right bumper", message)

        suspension.update("manual", ControllerUpdate(left_bumper_pressed=True))
        self.assertEqual(suspension.state, "bottomed")

    def test_bumpers_are_ignored_outside_manual_mode(self):
        suspension = SuspensionControl("bottomed")

        message = suspension.update(
            "static",
            ControllerUpdate(right_bumper_pressed=True),
        )

        self.assertEqual(message, "")
        self.assertEqual(suspension.state, "bottomed")

        message = suspension.update(
            "menu",
            ControllerUpdate(front_suspension_pressed=True),
        )
        self.assertEqual(message, "")
        self.assertEqual(suspension.state, "bottomed")

    def test_triggers_select_front_or_rear_bottomed(self):
        suspension = SuspensionControl("raised")

        suspension.update("manual", ControllerUpdate(front_suspension_pressed=True))
        self.assertEqual(suspension.state, "front_bottomed")

        suspension.update("manual", ControllerUpdate(rear_suspension_pressed=True))
        self.assertEqual(suspension.state, "rear_bottomed")

    def test_simultaneous_bumpers_do_not_move_suspension(self):
        suspension = SuspensionControl("bottomed")

        suspension.update(
            "manual",
            ControllerUpdate(
                left_bumper_pressed=True,
                right_bumper_pressed=True,
            ),
        )

        self.assertEqual(suspension.state, "bottomed")


if __name__ == "__main__":
    unittest.main()
