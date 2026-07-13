import cv2
import numpy as np

from robot.vision_common import clamp


class BucketDetection:
    """Future scoring state: find the orange tape stripe marking the bucket."""

    name = "bucket"

    def analyze(self, hsv):
        # A deliberately narrow orange range keeps this independent from ball colors.
        mask = cv2.inRange(hsv, np.array([5, 130, 100]), np.array([22, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return {"found": False, "center_x": None, "area": 0.0}
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        x, y, width, height = cv2.boundingRect(contour)
        return {"found": area > hsv.shape[0] * hsv.shape[1] * .002, "center_x": x + width // 2, "area": area,
                "confidence": clamp(area / float(hsv.shape[0] * hsv.shape[1] * .08), 0.0, 1.0)}

    @staticmethod
    def status_lines(result):
        return ["bucket state is not active", "orange stripe=" + ("seen" if result.get("found") else "not seen")]
