import unittest
from unittest.mock import patch

from robot import config
from robot.actuators import climb_output
from robot.climb import ClimbState


class FakeController:
    def __init__(self, value):
        self.value = value

    def climb_axis(self):
        return self.value


class ClimbTests(unittest.TestCase):
    def test_up_runs_only_pinch_and_down_runs_only_winch(self):
        up = ClimbState(FakeController(0.6)).process(None, 0.0)
        self.assertEqual(up.command.pinch, 0.6)
        self.assertEqual(up.command.winch, 0.0)

        down = ClimbState(FakeController(-0.4)).process(None, 0.0)
        self.assertEqual(down.command.pinch, 0.0)
        self.assertEqual(down.command.winch, 0.4)

    def test_center_neutralizes_both(self):
        result = ClimbState(FakeController(0.0)).process(None, 0.0)
        self.assertEqual(result.command.pinch, 0.0)
        self.assertEqual(result.command.winch, 0.0)

    def test_hardware_limit_is_applied_once(self):
        value, pulse = climb_output(0.5, 0.25, 1500, 1700)
        self.assertEqual(value, 0.125)
        self.assertEqual(pulse, 1525)

