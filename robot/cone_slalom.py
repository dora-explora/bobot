import cv2
import numpy as np

from robot import config
from robot.models import ConeDetection
from robot.vision_common import clamp, contour_is_triangular, hue_in_range


class ConeSlalom:
    """Detect deep-orange traffic cones independently of ball detection."""

    name = "cone_slalom"

    def detect(self, hsv):
        mask = cv2.inRange(
            hsv,
            np.array([config.CONE_HUE_MIN, config.CONE_SATURATION_MIN, config.CONE_VALUE_MIN]),
            np.array([config.CONE_HUE_MAX, 255, config.CONE_VALUE_MAX]),
        )
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_area = hsv.shape[0] * hsv.shape[1]
        cones = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < frame_area * 0.001:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if not self._is_cone(contour, x, y, width, height, mask, hsv):
                continue
            cones.append(ConeDetection(
                label="orange-cone",
                center_x=x + width // 2,
                center_y=y + height // 2,
                area=area,
                confidence=clamp(area / float(frame_area * 0.12), 0.0, 1.0),
                box=(x, y, width, height),
                contour=contour,
            ))
        return cones

    def _is_cone(self, contour, x, y, width, height, mask, hsv):
        contour_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)
        pixels = hsv[contour_mask == 255]
        if len(pixels) < 30:
            return False
        hue, saturation, brightness = (int(np.median(pixels[:, index])) for index in range(3))
        if not (hue_in_range(hue, config.CONE_HUE_MIN, config.CONE_HUE_MAX)
                and saturation >= config.CONE_SATURATION_MIN
                and config.CONE_VALUE_MIN <= brightness <= config.CONE_VALUE_MAX):
            return False
        if contour_is_triangular(contour):
            return True
        if height < max(12, width * 0.8):
            return False
        roi = mask[y:y + height, x:x + width]
        widths = [self._width_at(roi, start, end) for start, end in ((.10, .30), (.42, .58), (.70, .92))]
        top, middle, bottom = widths
        return top > 0 and middle > 0 and bottom >= top * 1.35 and bottom >= middle * 1.12

    @staticmethod
    def _width_at(mask, start_ratio, end_ratio):
        start, end = int(mask.shape[0] * start_ratio), max(int(mask.shape[0] * end_ratio), int(mask.shape[0] * start_ratio) + 1)
        columns = np.where(np.any(mask[start:end] > 0, axis=0))[0]
        return 0 if len(columns) == 0 else int(columns[-1] - columns[0] + 1)

    def status_lines(self, cones):
        if not cones:
            return ["cones=0 nearest=none"]
        nearest = max(cones, key=lambda cone: (cone.center_y, cone.area))
        return [
            "cones=" + str(len(cones)) + " nearest=" + nearest.label,
            "nearest x=" + str(nearest.center_x) + " y=" + str(nearest.center_y)
            + " area=" + str(int(nearest.area)) + " confidence=" + str(round(nearest.confidence, 2)),
        ]
