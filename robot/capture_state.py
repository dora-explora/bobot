"""Motor-neutral camera dataset capture state."""
import json
from pathlib import Path
import time

import cv2

from robot import config
from robot.models import DriveCommand, StateResult


class CaptureState:
    """Save raw corrected frames and sensor context on explicit button presses."""

    name = "capture"

    def __init__(self, root=None, session=None):
        root = Path(config.CAPTURE_ROOT if root is None else root)
        session = session or config.CAPTURE_SESSION or time.strftime("%Y%m%d-%H%M%S")
        self.session_name = session
        self.session_path = root / session
        self.metadata_path = self.session_path / "metadata.jsonl"
        self.saved_count = 0
        self.last_saved = ""
        self.last_error = ""
        self.last_capture_time = float("-inf")

    def process(
        self,
        frame,
        now,
        attitude=None,
        horizon=None,
        controller_update=None,
    ):
        requested = bool(
            controller_update is not None and controller_update.capture_pressed
        )
        if requested and now - self.last_capture_time >= config.CAPTURE_MIN_INTERVAL:
            self._save(frame, now, attitude, horizon)

        return StateResult(
            command=DriveCommand(mode="capture", reason="capture mode is motor-neutral"),
            state_lines=self.status_lines(),
            attitude=attitude,
            horizon=horizon,
        )

    def status_lines(self):
        lines = [
            "X captures one corrected camera frame; A=manual B=static hold Y=menu",
            "session=" + self.session_name,
            "path=" + str(self.session_path),
            "saved=" + str(self.saved_count)
            + " latest=" + (self.last_saved or "none"),
        ]
        if self.last_error:
            lines.append("error=" + self.last_error)
        return lines

    def _save(self, frame, now, attitude, horizon):
        self.last_capture_time = now
        self.last_error = ""
        try:
            self.session_path.mkdir(parents=True, exist_ok=True)
            timestamp_ns = time.time_ns()
            stem = "frame_" + str(timestamp_ns)
            final_path = self.session_path / (stem + ".jpg")
            temporary_path = self.session_path / (stem + ".tmp.jpg")
            ok = cv2.imwrite(
                str(temporary_path),
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, config.CAPTURE_JPEG_QUALITY],
            )
            if not ok:
                raise OSError("OpenCV failed to encode snapshot")
            temporary_path.replace(final_path)
            metadata = {
                "image": final_path.name,
                "captured_at_unix": now,
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
                "attitude": self._attitude_metadata(attitude),
                "horizon": self._horizon_metadata(horizon),
            }
            with self.metadata_path.open("a", encoding="utf-8") as metadata_file:
                metadata_file.write(json.dumps(metadata, sort_keys=True) + "\n")
            self.saved_count += 1
            self.last_saved = final_path.name
        except (OSError, cv2.error) as error:
            self.last_error = str(error)

    @staticmethod
    def _attitude_metadata(attitude):
        if attitude is None:
            return None
        return {
            name: getattr(attitude, name, None)
            for name in (
                "connected",
                "roll_degrees",
                "pitch_degrees",
                "yaw_degrees",
                "roll_delta_degrees",
                "pitch_delta_degrees",
                "yaw_delta_degrees",
            )
        }

    @staticmethod
    def _horizon_metadata(horizon):
        if horizon is None:
            return None
        return {
            name: getattr(horizon, name, None)
            for name in ("left_y", "center_y", "right_y", "source", "confident")
        }
