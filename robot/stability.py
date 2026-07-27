"""Washboard stability scoring from baseline-relative IMU roll and pitch."""
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import time

from robot import config


STABILITY_MULTIPLIER = -2.5
AUTOMATIC_STATES = frozenset(("washboard", "rough_section"))


@dataclass(frozen=True)
class StabilitySnapshot:
    status: str
    source: str
    current_roll_degrees: float | None
    current_pitch_degrees: float | None
    current_tilt_degrees: float | None
    sample_count: int
    rms_tilt_degrees: float
    score_points: float
    elapsed_seconds: float
    poll_hz: float
    log_enabled: bool
    log_path: str
    error: str


class StabilityScorer:
    """Accumulate the rules-defined washboard RMS tilt and negative score."""

    def __init__(
        self,
        poll_hz=None,
        log_enabled=None,
        log_dir=None,
    ):
        self.poll_hz = (
            config.STABILITY_POLL_HZ if poll_hz is None else float(poll_hz)
        )
        if not math.isfinite(self.poll_hz) or self.poll_hz <= 0.0:
            raise ValueError("stability poll rate must be a positive finite number")
        self.poll_interval = 1.0 / self.poll_hz
        self.log_enabled = (
            config.STABILITY_LOG_ENABLED
            if log_enabled is None
            else bool(log_enabled)
        )
        self.log_dir = Path(
            config.STABILITY_LOG_DIR if log_dir is None else log_dir
        )
        self.status = "idle"
        self.source = "none"
        self.sample_count = 0
        self.sum_tilt_squared = 0.0
        self.current_roll = None
        self.current_pitch = None
        self.current_tilt = None
        self.started_at = None
        self.stopped_at = None
        self.next_sample_at = None
        self.log_path = ""
        self.imu_error = ""
        self.log_error = ""
        self._log_file = None
        self._log_writer = None
        self._was_in_automatic_state = False

    @property
    def running(self):
        return self.status == "running"

    @property
    def rms_tilt(self):
        if self.sample_count == 0:
            return 0.0
        return math.sqrt(self.sum_tilt_squared / self.sample_count)

    @property
    def score(self):
        return STABILITY_MULTIPLIER * self.rms_tilt

    def start(self, now, source="controller"):
        """Reset accumulated data and begin one washboard scoring window."""
        self._close_log()
        self.status = "running"
        self.source = source
        self.sample_count = 0
        self.sum_tilt_squared = 0.0
        self.current_roll = None
        self.current_pitch = None
        self.current_tilt = None
        self.started_at = float(now)
        self.stopped_at = None
        self.next_sample_at = float(now)
        self.log_path = ""
        self.imu_error = ""
        self.log_error = ""
        if self.log_enabled:
            self._open_log()
        return "started fresh washboard stability scoring"

    def stop(self, now):
        """Finalize the current score without discarding it."""
        if not self.running:
            return "washboard stability scoring is not running"
        self.status = "final"
        self.stopped_at = float(now)
        self.next_sample_at = None
        self._close_log()
        return "finalized washboard stability score"

    def toggle(self, now):
        """Controller behavior: start/reset when idle/final, stop when running."""
        if self.running:
            return self.stop(now)
        return self.start(now, source="controller")

    def sync_state(self, state_name, now):
        """Automatically bracket a future dedicated washboard runtime state."""
        in_automatic_state = state_name in AUTOMATIC_STATES
        message = ""
        if in_automatic_state and not self._was_in_automatic_state:
            message = self.start(now, source="state:" + state_name)
        elif not in_automatic_state and self._was_in_automatic_state and self.running:
            message = self.stop(now)
        self._was_in_automatic_state = in_automatic_state
        return message

    def update(self, attitude, now):
        """Sample a valid delta attitude no faster than the configured rate."""
        now = float(now)
        if not self.running or now + 1e-12 < self.next_sample_at:
            return False
        self.next_sample_at = now + self.poll_interval
        if not self._valid_attitude(attitude):
            self.imu_error = "waiting for valid roll/pitch delta from IMU"
            return False

        roll = float(attitude.roll_delta_degrees)
        pitch = float(attitude.pitch_delta_degrees)
        tilt_squared = roll * roll + pitch * pitch
        tilt = math.sqrt(tilt_squared)
        self.current_roll = roll
        self.current_pitch = pitch
        self.current_tilt = tilt
        self.sum_tilt_squared += tilt_squared
        self.sample_count += 1
        self.imu_error = ""
        self._write_log_row(attitude, now)
        return True

    def snapshot(self, now):
        if self.started_at is None:
            elapsed = 0.0
        else:
            end = float(now) if self.running else self.stopped_at
            elapsed = max(0.0, end - self.started_at)
        return StabilitySnapshot(
            status=self.status,
            source=self.source,
            current_roll_degrees=self.current_roll,
            current_pitch_degrees=self.current_pitch,
            current_tilt_degrees=self.current_tilt,
            sample_count=self.sample_count,
            rms_tilt_degrees=self.rms_tilt,
            score_points=self.score,
            elapsed_seconds=elapsed,
            poll_hz=self.poll_hz,
            log_enabled=self.log_enabled,
            log_path=self.log_path,
            error="; ".join(
                error for error in (self.imu_error, self.log_error) if error
            ),
        )

    def close(self, now=None):
        if self.running:
            self.stop(time.time() if now is None else now)
        else:
            self._close_log()

    @staticmethod
    def _valid_attitude(attitude):
        if (
            attitude is None
            or not attitude.connected
            or attitude.roll_delta_degrees is None
            or attitude.pitch_delta_degrees is None
        ):
            return False
        return (
            math.isfinite(attitude.roll_delta_degrees)
            and math.isfinite(attitude.pitch_delta_degrees)
        )

    def _open_log(self):
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            suffix = str(time.time_ns() % 1_000_000_000).zfill(9)
            path = self.log_dir / ("stability_" + timestamp + "_" + suffix + ".csv")
            self._log_file = path.open("x", newline="", encoding="utf-8")
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow([
                "sample",
                "runtime_timestamp",
                "imu_timestamp",
                "elapsed_seconds",
                "roll_delta_degrees",
                "pitch_delta_degrees",
                "tilt_degrees",
                "rms_tilt_degrees",
                "stability_score_points",
            ])
            self._log_file.flush()
            self.log_path = str(path)
        except OSError as error:
            self._close_log()
            self.log_error = "stability CSV unavailable: " + str(error)

    def _write_log_row(self, attitude, now):
        if self._log_writer is None:
            return
        try:
            self._log_writer.writerow([
                self.sample_count,
                now,
                attitude.timestamp,
                now - self.started_at,
                self.current_roll,
                self.current_pitch,
                self.current_tilt,
                self.rms_tilt,
                self.score,
            ])
            self._log_file.flush()
        except OSError as error:
            self.log_error = "stability CSV write failed: " + str(error)
            self._close_log()

    def _close_log(self):
        log_file = self._log_file
        self._log_file = None
        self._log_writer = None
        if log_file is not None:
            try:
                log_file.close()
            except OSError as error:
                self.log_error = "stability CSV close failed: " + str(error)
