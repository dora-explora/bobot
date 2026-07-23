import unittest

from robot import config
from robot.horizon import HorizonEstimator
from robot.imu import IMUSnapshot


class HorizonEstimatorTests(unittest.TestCase):
    def setUp(self):
        self.original = {
            name: getattr(config, name)
            for name in (
                "HORIZON_BASE_Y_RATIO",
                "HORIZON_PITCH_SIGN",
                "HORIZON_ROLL_SIGN",
                "CAMERA_VERTICAL_FOV_DEG",
                "IMU_MAX_AGE_SECONDS",
            )
        }
        config.HORIZON_BASE_Y_RATIO = 0.25
        config.HORIZON_PITCH_SIGN = 1.0
        config.HORIZON_ROLL_SIGN = 1.0
        config.CAMERA_VERTICAL_FOV_DEG = 40.0
        config.IMU_MAX_AGE_SECONDS = 0.5

    def tearDown(self):
        for name, value in self.original.items():
            setattr(config, name, value)

    def test_disconnected_imu_uses_configured_baseline(self):
        attitude = IMUSnapshot(False, "offline", 10.0)
        horizon = HorizonEstimator().estimate(640, 400, attitude, 10.0)

        self.assertEqual(horizon.center_y, 100)
        self.assertEqual(horizon.left_y, 100)
        self.assertEqual(horizon.right_y, 100)
        self.assertFalse(horizon.confident)
        self.assertEqual(horizon.source, "configured baseline")

    def test_pitch_moves_and_roll_tilts_horizon(self):
        attitude = IMUSnapshot(
            connected=True,
            error="",
            timestamp=10.0,
            roll_delta_degrees=10.0,
            pitch_delta_degrees=5.0,
        )
        horizon = HorizonEstimator().estimate(640, 400, attitude, 10.1)

        self.assertEqual(horizon.center_y, 150)
        self.assertLess(horizon.left_y, horizon.center_y)
        self.assertGreater(horizon.right_y, horizon.center_y)
        self.assertTrue(horizon.confident)
        self.assertEqual(horizon.source, "baseline + IMU")

    def test_stale_imu_does_not_move_horizon(self):
        attitude = IMUSnapshot(
            connected=True,
            error="",
            timestamp=1.0,
            roll_delta_degrees=20.0,
            pitch_delta_degrees=20.0,
        )
        horizon = HorizonEstimator().estimate(640, 400, attitude, 2.0)

        self.assertEqual((horizon.left_y, horizon.right_y), (100, 100))
        self.assertFalse(horizon.confident)


if __name__ == "__main__":
    unittest.main()
