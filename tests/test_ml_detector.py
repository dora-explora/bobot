from pathlib import Path
from tempfile import TemporaryDirectory
import time
from types import SimpleNamespace
import unittest

import numpy as np

from robot.ml_detector import AsyncMLDetector, OnnxBallConeDetector


class FakeSession:
    def __init__(self, rows):
        self.rows = np.asarray([rows], dtype=np.float32)
        self.feed = None

    def get_inputs(self):
        return [SimpleNamespace(name="images")]

    def run(self, _outputs, feed):
        self.feed = feed
        return [self.rows]


def manifest(size=100):
    return {
        "format": "yolo26-e2e-onnx",
        "input_width": size,
        "input_height": size,
        "classes": {"0": "ball", "1": "cone"},
    }


class OnnxBallConeDetectorTests(unittest.TestCase):
    def test_letterbox_output_is_mapped_back_to_camera_coordinates(self):
        session = FakeSession([[25, 35, 75, 65, 0.9, 0]])
        detector = OnnxBallConeDetector(
            session=session,
            manifest=manifest(),
            model_path="unused.onnx",
        )
        frame = np.zeros((100, 200, 3), dtype=np.uint8)

        detections, debug = detector.detect(frame)

        self.assertEqual(session.feed["images"].shape, (1, 3, 100, 100))
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].kind, "ball")
        self.assertEqual(detections[0].box, (50, 20, 100, 60))
        self.assertEqual((detections[0].center_x, detections[0].center_y), (100, 50))
        self.assertTrue(detections[0].certain)
        self.assertEqual(debug.vision_backend, "ml")

    def test_horizon_rejects_physically_impossible_detection(self):
        session = FakeSession([[40, 5, 60, 25, 0.9, 1]])
        detector = OnnxBallConeDetector(
            session=session,
            manifest=manifest(),
            model_path="unused.onnx",
        )
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        horizon = SimpleNamespace(y_at=lambda _x, _width: 50)

        detections, debug = detector.detect(frame, horizon)

        self.assertEqual(detections, [])
        self.assertEqual(debug.rejected_horizon, 1)


class AsyncMLDetectorTests(unittest.TestCase):
    def test_worker_returns_detection_without_blocking_submitter(self):
        class Detector:
            def detect(self, _frame, _horizon):
                return [], SimpleNamespace(vision_backend="ml")

        worker = AsyncMLDetector(detector_factory=Detector)
        try:
            sequence = worker.submit(np.zeros((4, 4, 3), dtype=np.uint8), time.time())
            deadline = time.time() + 1.0
            result = None
            while time.time() < deadline and result is None:
                result = worker.poll_after(0)
                time.sleep(0.005)

            self.assertIsNotNone(result)
            self.assertEqual(result.sequence, sequence)
            self.assertEqual(result.error, "")
            self.assertTrue(worker.status(time.time())["ready"])
        finally:
            worker.close()


if __name__ == "__main__":
    unittest.main()
