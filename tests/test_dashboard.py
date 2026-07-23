import unittest

from robot.dashboard import TuiDashboard
from robot.horizon import HorizonEstimate
from robot.imu import IMUSnapshot


class DashboardIndicatorTests(unittest.TestCase):
    def test_tilt_indicator_places_marker_away_from_center(self):
        indicator = TuiDashboard._tilt_indicator(20.0, 15.0)

        self.assertEqual(len(indicator), 7)
        self.assertTrue(all(len(line) == 21 for line in indicator))
        marker_y = next(index for index, line in enumerate(indicator) if "O" in line)
        marker_x = indicator[marker_y].index("O")
        self.assertLess(marker_y, 3)
        self.assertGreater(marker_x, 10)

    def test_yaw_indicator_uses_terminal_width(self):
        line = TuiDashboard._yaw_indicator(90.0, 80)

        self.assertLessEqual(len(line), 80)
        self.assertIn("#", line)
        self.assertIn("|", line)

    def test_imu_lines_include_graphs_and_horizon(self):
        snapshot = IMUSnapshot(
            connected=True,
            error="",
            timestamp=1.0,
            roll_degrees=2.0,
            pitch_degrees=3.0,
            yaw_degrees=4.0,
            roll_delta_degrees=1.0,
            pitch_delta_degrees=2.0,
            yaw_delta_degrees=3.0,
            acceleration=(0.0, 0.0, 9.8),
            gyro=(0.1, 0.2, 0.3),
        )
        horizon = HorizonEstimate(100, 120, 110, True, "baseline + IMU")

        lines = TuiDashboard._imu_lines(snapshot, horizon, 80)

        self.assertEqual(lines[0], "[IMU]")
        self.assertTrue(any("relative roll=1.0deg" in line for line in lines))
        self.assertTrue(any("yaw -180" in line for line in lines))
        self.assertTrue(any("horizon=baseline + IMU" in line for line in lines))


if __name__ == "__main__":
    unittest.main()
