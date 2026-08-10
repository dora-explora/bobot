import unittest
from unittest.mock import patch

from robot import config
from robot.actuators import Pca9685Actuators, suspension_pulses
from robot.controller import ControllerUpdate
from robot.models import DriveCommand
from robot.suspension import SuspensionControl


class SuspensionControlTests(unittest.TestCase):
    def test_each_position_uses_its_per_corner_calibration(self):
        outputs = (
            ("front_left", 12, 1100, 1900),
            ("front_right", 13, 1200, 1800),
            ("rear_left", 14, 1300, 1700),
            ("rear_right", 15, 1400, 1600),
        )
        with patch.object(config, "SUSPENSION_OUTPUTS", outputs):
            self.assertEqual(suspension_pulses("bottomed")["front_left"], 1100)
            self.assertEqual(suspension_pulses("bottomed")["rear_right"], 1400)
            self.assertEqual(suspension_pulses("raised")["front_left"], 1900)
            self.assertEqual(suspension_pulses("raised")["rear_right"], 1600)

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
