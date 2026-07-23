import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import numpy as np

from robot.capture_state import CaptureState
from robot.controller import ControllerUpdate


class CaptureStateTests(unittest.TestCase):
    def test_capture_press_saves_frame_and_metadata(self):
        frame = np.zeros((24, 32, 3), dtype=np.uint8)
        attitude = SimpleNamespace(
            connected=True,
            roll_degrees=1.0,
            pitch_degrees=2.0,
            yaw_degrees=3.0,
            roll_delta_degrees=0.1,
            pitch_delta_degrees=0.2,
            yaw_delta_degrees=0.3,
        )
        horizon = SimpleNamespace(
            left_y=7,
            center_y=8,
            right_y=9,
            source="imu",
            confident=True,
        )
        with TemporaryDirectory() as temporary_directory:
            state = CaptureState(temporary_directory, "test-session")

            result = state.process(
                frame,
                10.0,
                attitude,
                horizon,
                ControllerUpdate(capture_pressed=True),
            )

            images = list((Path(temporary_directory) / "test-session").glob("*.jpg"))
            self.assertEqual(len(images), 1)
            metadata_path = Path(temporary_directory) / "test-session" / "metadata.jsonl"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8").strip())
            self.assertEqual(metadata["image"], images[0].name)
            self.assertEqual(metadata["width"], 32)
            self.assertEqual(metadata["attitude"]["yaw_degrees"], 3.0)
            self.assertEqual(metadata["horizon"]["center_y"], 8)
            self.assertEqual(result.command.mode, "capture")
            self.assertEqual(state.saved_count, 1)

    def test_no_press_does_not_create_session_directory(self):
        frame = np.zeros((24, 32, 3), dtype=np.uint8)
        with TemporaryDirectory() as temporary_directory:
            state = CaptureState(temporary_directory, "test-session")

            state.process(frame, 10.0, controller_update=ControllerUpdate())

            self.assertFalse((Path(temporary_directory) / "test-session").exists())


if __name__ == "__main__":
    unittest.main()
