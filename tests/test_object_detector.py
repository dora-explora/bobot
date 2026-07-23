import unittest

import cv2
import numpy as np

from robot.horizon import HorizonEstimate
from robot.object_detector import ObjectDetector


class ObjectDetectorTests(unittest.TestCase):
    def setUp(self):
        self.detector = ObjectDetector()
        self.horizon = HorizonEstimate(120, 120, 120, False, "test")

    def detect(self, frame, horizon=None):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return self.detector.detect(
            frame,
            hsv,
            self.horizon if horizon is None else horizon,
        )

    @staticmethod
    def frame(background=(150, 150, 150)):
        return np.full((480, 640, 3), background, dtype=np.uint8)

    def test_gradient_lit_ball_is_proposed_by_shape(self):
        frame = self.frame()
        for radius in range(45, 0, -1):
            ratio = radius / 45.0
            color = (
                int(190 - 70 * ratio),
                int(90 - 40 * ratio),
                int(35 - 10 * ratio),
            )
            cv2.circle(frame, (320, 330), radius, color, -1)

        detections, _ = self.detect(frame)

        balls = [item for item in detections if item.kind == "ball"]
        self.assertEqual(len(balls), 1)
        self.assertGreater(balls[0].ball_score, balls[0].cone_score)

    def test_ball_survives_bright_tan_wood_background(self):
        frame = self.frame((100, 185, 225))
        cv2.circle(frame, (320, 330), 40, (230, 60, 40), -1, cv2.LINE_AA)

        detections, debug = self.detect(frame)

        self.assertEqual(sum(item.kind == "ball" for item in detections), 1)
        self.assertGreaterEqual(debug.rejected_large, 1)

    def test_deep_orange_triangle_is_a_cone(self):
        frame = self.frame()
        points = np.array([[320, 210], [265, 390], [375, 390]], np.int32)
        cv2.fillConvexPoly(frame, points, (0, 100, 225))

        detections, _ = self.detect(frame)

        cones = [item for item in detections if item.kind == "cone"]
        self.assertEqual(len(cones), 1)
        self.assertGreater(cones[0].cone_score, cones[0].ball_score)

    def test_long_yellow_plank_is_not_a_ball(self):
        frame = self.frame()
        cv2.rectangle(frame, (100, 270), (540, 320), (60, 220, 240), -1)

        detections, _ = self.detect(frame)

        self.assertFalse(any(item.kind == "ball" for item in detections))

    def test_ball_candidate_above_horizon_becomes_unknown(self):
        frame = self.frame()
        cv2.circle(frame, (320, 70), 30, (20, 20, 230), -1)
        horizon = HorizonEstimate(160, 160, 160, True, "test")

        detections, debug = self.detect(frame, horizon)

        self.assertFalse(any(item.kind == "ball" for item in detections))
        self.assertEqual(debug.rejected_horizon, 1)
        self.assertTrue(any("horizon" in item.rejection_reason for item in detections))


if __name__ == "__main__":
    unittest.main()
