import unittest

from robot import config
from robot.actuators import normalize_throttle


class RuntimeThrottleLimitTests(unittest.TestCase):
    def setUp(self):
        self.original_limit = config.THROTTLE_LIMIT
        self.original_step = config.THROTTLE_LIMIT_STEP

    def tearDown(self):
        config.THROTTLE_LIMIT = self.original_limit
        config.THROTTLE_LIMIT_STEP = self.original_step

    def test_adjustment_clamps_and_changes_actuator_output_limit(self):
        config.THROTTLE_LIMIT = 0.95
        config.THROTTLE_LIMIT_STEP = 0.05

        self.assertEqual(config.adjust_throttle_limit(1), 1.0)
        self.assertEqual(config.adjust_throttle_limit(-30), 0.0)

        config.THROTTLE_LIMIT = 0.20
        self.assertEqual(normalize_throttle(0.75), 0.20)
        self.assertEqual(config.adjust_throttle_limit(1), 0.25)
        self.assertEqual(normalize_throttle(0.75), 0.25)


if __name__ == "__main__":
    unittest.main()
