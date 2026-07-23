import math
import unittest

from robot.imu import (
    BNO085Service,
    angle_delta_degrees,
    quaternion_to_euler_degrees,
)


def euler_quaternion(roll_degrees, pitch_degrees, yaw_degrees):
    roll = math.radians(roll_degrees) / 2.0
    pitch = math.radians(pitch_degrees) / 2.0
    yaw = math.radians(yaw_degrees) / 2.0
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class FakeSensor:
    def __init__(self, quaternion=(0.0, 0.0, 0.0, 1.0)):
        self.quaternion = quaternion
        self.game_quaternion = quaternion
        self.acceleration = (1.0, 2.0, 3.0)
        self.gyro = (0.1, 0.2, 0.3)
        self.enabled = []

    def enable_feature(self, feature_id, report_interval):
        self.enabled.append((feature_id, report_interval))


def service_for_sensor(sensor):
    return BNO085Service(
        i2c_factory=lambda: object(),
        sensor_factory=lambda i2c, address: sensor,
        report_features=(
            ("rotation-vector", 1),
            ("accelerometer", 2),
            ("gyro", 3),
        ),
        clock=lambda: 12.5,
    )


class QuaternionTests(unittest.TestCase):
    def test_quaternion_conversion_handles_identity_and_each_axis(self):
        cases = (
            ((0.0, 0.0, 0.0, 2.0), (0.0, 0.0, 0.0)),
            (euler_quaternion(90.0, 0.0, 0.0), (90.0, 0.0, 0.0)),
            (euler_quaternion(0.0, 45.0, 0.0), (0.0, 45.0, 0.0)),
            (euler_quaternion(0.0, 0.0, -120.0), (0.0, 0.0, -120.0)),
        )

        for quaternion, expected in cases:
            with self.subTest(expected=expected):
                actual = quaternion_to_euler_degrees(quaternion)
                for value, target in zip(actual, expected):
                    self.assertAlmostEqual(value, target, places=6)

    def test_quaternion_conversion_rejects_zero_magnitude(self):
        with self.assertRaises(ValueError):
            quaternion_to_euler_degrees((0.0, 0.0, 0.0, 0.0))

    def test_yaw_delta_wraps_across_180_degrees(self):
        self.assertAlmostEqual(angle_delta_degrees(-179.0, 179.0), 2.0)
        self.assertAlmostEqual(angle_delta_degrees(179.0, -179.0), -2.0)


class BNO085ServiceTests(unittest.TestCase):
    def test_first_read_sets_baseline_and_later_read_returns_deltas(self):
        sensor = FakeSensor(euler_quaternion(10.0, -5.0, 179.0))
        service = service_for_sensor(sensor)

        first = service.read()
        self.assertTrue(first.connected)
        self.assertEqual(first.timestamp, 12.5)
        self.assertAlmostEqual(first.roll_delta_degrees, 0.0)
        self.assertAlmostEqual(first.pitch_delta_degrees, 0.0)
        self.assertAlmostEqual(first.yaw_delta_degrees, 0.0)
        self.assertEqual(first.acceleration, (1.0, 2.0, 3.0))
        self.assertEqual(first.gyro, (0.1, 0.2, 0.3))

        sensor.quaternion = euler_quaternion(13.0, -1.0, -179.0)
        second = service.read()
        self.assertAlmostEqual(second.roll_delta_degrees, 3.0, places=5)
        self.assertAlmostEqual(second.pitch_delta_degrees, 4.0, places=5)
        self.assertAlmostEqual(second.yaw_delta_degrees, 2.0, places=5)

        self.assertTrue(service.rebaseline())
        third = service.read()
        self.assertAlmostEqual(third.roll_delta_degrees, 0.0, places=5)
        self.assertAlmostEqual(third.pitch_delta_degrees, 0.0, places=5)
        self.assertAlmostEqual(third.yaw_delta_degrees, 0.0, places=5)

    def test_connect_enables_all_available_reports_at_requested_interval(self):
        sensor = FakeSensor()
        service = BNO085Service(
            address=0x4B,
            report_interval_us=25_000,
            i2c_factory=lambda: object(),
            sensor_factory=lambda i2c, address: sensor,
            report_features=(("rotation-vector", 7), ("accelerometer", 8), ("gyro", 9)),
        )

        self.assertTrue(service.connected)
        self.assertEqual(sensor.enabled, [(7, 25_000), (8, 25_000), (9, 25_000)])
        self.assertEqual(
            service.reports_enabled,
            ("rotation-vector", "accelerometer", "gyro"),
        )

    def test_game_rotation_mode_reads_game_quaternion(self):
        sensor = FakeSensor(euler_quaternion(0.0, 0.0, 45.0))
        sensor.quaternion = euler_quaternion(0.0, 0.0, -90.0)
        service = BNO085Service(
            rotation_mode="game",
            i2c_factory=lambda: object(),
            sensor_factory=lambda i2c, address: sensor,
            report_features=(("game-rotation-vector", 1),),
        )

        snapshot = service.read()

        self.assertAlmostEqual(snapshot.yaw_degrees, 45.0)

    def test_unavailable_hardware_returns_status_without_raising(self):
        def unavailable_i2c():
            raise RuntimeError("I2C bus unavailable")

        service = BNO085Service(
            i2c_factory=unavailable_i2c,
            sensor_factory=lambda i2c, address: FakeSensor(),
            report_features=(),
            clock=lambda: 99.0,
        )

        snapshot = service.read()
        self.assertFalse(service.connected)
        self.assertFalse(snapshot.connected)
        self.assertEqual(snapshot.timestamp, 99.0)
        self.assertIn("I2C bus unavailable", snapshot.error)
        self.assertIsNone(snapshot.roll_degrees)

    def test_read_failure_is_reported_without_discarding_other_values(self):
        sensor = FakeSensor()
        sensor.quaternion = None
        service = service_for_sensor(sensor)

        snapshot = service.read()

        self.assertTrue(snapshot.connected)
        self.assertIn("orientation read failed", snapshot.error)
        self.assertIsNone(snapshot.roll_degrees)
        self.assertEqual(snapshot.acceleration, (1.0, 2.0, 3.0))
        self.assertEqual(snapshot.gyro, (0.1, 0.2, 0.3))


if __name__ == "__main__":
    unittest.main()
