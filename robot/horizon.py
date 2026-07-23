"""IMU-adjusted image horizon used as a conservative object-position gate."""
from dataclasses import dataclass
import math

from robot import config
from robot.vision_common import clamp


@dataclass
class HorizonEstimate:
    left_y: int
    right_y: int
    center_y: int
    confident: bool
    source: str

    def y_at(self, x, frame_width):
        ratio = clamp(x / float(max(1, frame_width - 1)), 0.0, 1.0)
        return self.left_y + (self.right_y - self.left_y) * ratio


class HorizonEstimator:
    def estimate(self, frame_width, frame_height, attitude, now):
        center_y = frame_height * config.HORIZON_BASE_Y_RATIO
        roll = 0.0
        confident = False
        source = "configured baseline"
        if (
            attitude is not None
            and attitude.connected
            and attitude.pitch_delta_degrees is not None
            and attitude.roll_delta_degrees is not None
            and now - attitude.timestamp <= config.IMU_MAX_AGE_SECONDS
        ):
            pixels_per_degree = frame_height / max(1.0, config.CAMERA_VERTICAL_FOV_DEG)
            center_y += (
                attitude.pitch_delta_degrees
                * config.HORIZON_PITCH_SIGN
                * pixels_per_degree
            )
            roll = attitude.roll_delta_degrees * config.HORIZON_ROLL_SIGN
            confident = True
            source = "baseline + IMU"

        half_width = frame_width / 2.0
        rise = math.tan(math.radians(clamp(roll, -60.0, 60.0))) * half_width
        left_y = int(round(clamp(center_y - rise, 0, frame_height - 1)))
        right_y = int(round(clamp(center_y + rise, 0, frame_height - 1)))
        return HorizonEstimate(
            left_y=left_y,
            right_y=right_y,
            center_y=int(round(clamp(center_y, 0, frame_height - 1))),
            confident=confident,
            source=source,
        )
