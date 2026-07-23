"""Optional BNO085 orientation service with no import-time hardware dependencies."""

from dataclasses import dataclass
import math
import os
import time
from typing import Callable


DEFAULT_I2C_ADDRESS = 0x4B
DEFAULT_REPORT_INTERVAL_US = 10_000


@dataclass(frozen=True)
class IMUSnapshot:
    """One best-effort sensor reading.

    The timestamp uses the service's monotonic clock. Quaternion-derived angles
    are degrees; acceleration is m/s^2 and gyro is radians/second.
    """

    connected: bool
    error: str
    timestamp: float
    roll_degrees: float | None = None
    pitch_degrees: float | None = None
    yaw_degrees: float | None = None
    roll_delta_degrees: float | None = None
    pitch_delta_degrees: float | None = None
    yaw_delta_degrees: float | None = None
    acceleration: tuple[float, float, float] | None = None
    gyro: tuple[float, float, float] | None = None


def wrap_angle_degrees(angle: float) -> float:
    """Wrap an angle to [-180, 180)."""
    angle = float(angle)
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return 0.0 if abs(wrapped) < 1e-12 else wrapped


def angle_delta_degrees(angle: float, baseline: float) -> float:
    """Return the shortest signed angular difference from baseline to angle."""
    return wrap_angle_degrees(float(angle) - float(baseline))


def quaternion_to_euler_degrees(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Convert an (x, y, z, w) quaternion to roll, pitch, and yaw degrees."""
    try:
        x, y, z, w = (float(value) for value in quaternion)
    except (TypeError, ValueError) as error:
        raise ValueError("quaternion must contain four numeric values") from error

    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ValueError("quaternion values must be finite")
    magnitude = math.sqrt(x * x + y * y + z * z + w * w)
    if magnitude <= 1e-12:
        raise ValueError("quaternion magnitude must be nonzero")
    x, y, z, w = (value / magnitude for value in (x, y, z, w))

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)

    angles = tuple(math.degrees(value) for value in (roll, pitch, yaw))
    return tuple(0.0 if abs(value) < 1e-12 else value for value in angles)


class BNO085Service:
    """Best-effort BNO085 reader suitable for a camera/control loop."""

    def __init__(
        self,
        address: int | None = None,
        report_interval_us: int | None = None,
        *,
        i2c_factory: Callable[[], object] | None = None,
        sensor_factory: Callable[[object, int], object] | None = None,
        report_features: tuple[tuple[str, int], ...] | None = None,
        clock: Callable[[], float] = time.monotonic,
        auto_connect: bool = True,
    ):
        self.address = (
            _env_int("IMU_I2C_ADDRESS", DEFAULT_I2C_ADDRESS)
            if address is None
            else int(address)
        )
        self.report_interval_us = (
            _env_int("IMU_REPORT_INTERVAL_US", DEFAULT_REPORT_INTERVAL_US)
            if report_interval_us is None
            else int(report_interval_us)
        )
        if not 0 <= self.address <= 0x7F:
            raise ValueError("IMU I2C address must be between 0x00 and 0x7F")
        if self.report_interval_us <= 0:
            raise ValueError("IMU report interval must be positive")

        self._i2c_factory = i2c_factory or _default_i2c_factory
        self._sensor_factory = sensor_factory or _default_sensor_factory
        self._report_features = report_features
        self._clock = clock
        self._i2c = None
        self._sensor = None
        self._connected = False
        self._connection_warning = ""
        self._error = ""
        self._baseline: tuple[float, float, float] | None = None
        self._last_angles: tuple[float, float, float] | None = None
        self._reports_enabled: tuple[str, ...] = ()

        if auto_connect:
            self.connect()

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def error(self) -> str:
        return self._error

    @property
    def reports_enabled(self) -> tuple[str, ...]:
        return self._reports_enabled

    def connect(self) -> bool:
        """Initialize hardware and reports, returning False instead of raising."""
        self.close()
        self._baseline = None
        self._last_angles = None
        i2c = None
        try:
            i2c = self._i2c_factory()
            sensor = self._sensor_factory(i2c, self.address)
            features = (
                _default_report_features()
                if self._report_features is None
                else self._report_features
            )
        except Exception as error:
            _deinit_i2c(i2c)
            self._set_connection_failure(error)
            return False

        enabled = []
        warnings = []
        for name, feature_id in features:
            try:
                sensor.enable_feature(
                    feature_id,
                    report_interval=self.report_interval_us,
                )
                enabled.append(name)
            except Exception as error:
                warnings.append(name + " report unavailable: " + str(error))

        self._i2c = i2c
        self._sensor = sensor
        self._connected = True
        self._reports_enabled = tuple(enabled)
        self._connection_warning = "; ".join(warnings)
        self._error = self._connection_warning
        return True

    def read(self) -> IMUSnapshot:
        """Read all enabled values and return any failures in the snapshot."""
        timestamp = self._clock()
        if self._sensor is None:
            return IMUSnapshot(
                connected=False,
                error=self._error or "BNO085 is not connected",
                timestamp=timestamp,
            )

        errors = [self._connection_warning] if self._connection_warning else []
        angles = None
        deltas = None
        acceleration = None
        gyro = None

        try:
            quaternion = self._sensor.quaternion
            if quaternion is None:
                raise ValueError("rotation-vector report has no data")
            angles = quaternion_to_euler_degrees(quaternion)
            if self._baseline is None:
                self._baseline = angles
            self._last_angles = angles
            deltas = tuple(
                angle_delta_degrees(value, baseline)
                for value, baseline in zip(angles, self._baseline)
            )
        except Exception as error:
            errors.append("orientation read failed: " + str(error))

        try:
            acceleration = _three_axis_tuple(self._sensor.acceleration, "acceleration")
        except Exception as error:
            errors.append("acceleration read failed: " + str(error))

        try:
            gyro = _three_axis_tuple(self._sensor.gyro, "gyro")
        except Exception as error:
            errors.append("gyro read failed: " + str(error))

        self._error = "; ".join(error for error in errors if error)
        return IMUSnapshot(
            connected=self._connected,
            error=self._error,
            timestamp=timestamp,
            roll_degrees=None if angles is None else angles[0],
            pitch_degrees=None if angles is None else angles[1],
            yaw_degrees=None if angles is None else angles[2],
            roll_delta_degrees=None if deltas is None else deltas[0],
            pitch_delta_degrees=None if deltas is None else deltas[1],
            yaw_delta_degrees=None if deltas is None else deltas[2],
            acceleration=acceleration,
            gyro=gyro,
        )

    def rebaseline(self) -> bool:
        """Make the most recent valid orientation the new zero point."""
        if self._last_angles is None:
            self._baseline = None
            return False
        self._baseline = self._last_angles
        return True

    def close(self) -> None:
        """Release the I2C object when it supports deinitialization."""
        i2c = self._i2c
        self._i2c = None
        self._sensor = None
        self._connected = False
        self._reports_enabled = ()
        close_error = _deinit_i2c(i2c)
        if close_error:
            self._error = close_error

    def _set_connection_failure(self, error: Exception) -> None:
        self._i2c = None
        self._sensor = None
        self._connected = False
        self._reports_enabled = ()
        self._connection_warning = ""
        self._error = "BNO085 connection failed: " + str(error)


def _env_int(name: str, default: int) -> int:
    raw_value = os.environ.get(name)
    return default if raw_value is None else int(raw_value, 0)


def _three_axis_tuple(
    values: tuple[float, float, float] | None,
    label: str,
) -> tuple[float, float, float]:
    if values is None:
        raise ValueError(label + " report has no data")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(label + " must contain three numeric values") from error
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError(label + " must contain three finite values")
    return result


def _default_i2c_factory():
    import board
    import busio

    return busio.I2C(board.SCL, board.SDA, frequency=400_000)


def _default_sensor_factory(i2c, address):
    from adafruit_bno08x.i2c import BNO08X_I2C

    return BNO08X_I2C(i2c, address=address)


def _default_report_features() -> tuple[tuple[str, int], ...]:
    import adafruit_bno08x

    names = (
        ("rotation-vector", "BNO_REPORT_ROTATION_VECTOR"),
        ("accelerometer", "BNO_REPORT_ACCELEROMETER"),
        ("gyro", "BNO_REPORT_GYROSCOPE"),
    )
    return tuple(
        (display_name, getattr(adafruit_bno08x, constant_name))
        for display_name, constant_name in names
        if hasattr(adafruit_bno08x, constant_name)
    )


def _deinit_i2c(i2c) -> str:
    if i2c is None or not hasattr(i2c, "deinit"):
        return ""
    try:
        i2c.deinit()
    except Exception as error:
        return "I2C close failed: " + str(error)
    return ""
