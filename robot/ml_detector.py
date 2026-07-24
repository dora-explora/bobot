"""End-to-end YOLO26 ONNX inference and a latest-frame async worker."""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np

from robot import config
from robot.models import DetectionDebug, ObjectDetection
from robot.vision_common import clamp


BALL_COLOR = (64, 230, 64)
CONE_COLOR = (0, 150, 255)


@dataclass
class InferenceResult:
    sequence: int
    captured_at: float
    completed_at: float
    latency_ms: float
    detections: list
    debug: DetectionDebug
    attitude: object = None
    error: str = ""


class OnnxBallConeDetector:
    """Run a fixed-size YOLO26 end-to-end graph without Ultralytics at runtime."""

    def __init__(
        self,
        model_path=None,
        manifest_path=None,
        session=None,
        manifest=None,
    ):
        self.model_path = Path(model_path or config.ML_MODEL_PATH)
        self.manifest_path = Path(manifest_path or config.ML_MODEL_MANIFEST)
        self.manifest = manifest or self._load_manifest()
        self.input_width = int(self.manifest["input_width"])
        self.input_height = int(self.manifest["input_height"])
        self._validate_manifest()
        self.session = session or self._create_session()
        inputs = self.session.get_inputs()
        if len(inputs) != 1:
            raise ValueError("ML model must have exactly one input")
        self.input_name = inputs[0].name

    def detect(self, frame, horizon=None):
        tensor, scale, pad_x, pad_y = self._preprocess(frame)
        outputs = self.session.run(None, {self.input_name: tensor})
        if not outputs:
            raise ValueError("ML model returned no output tensors")
        rows = np.asarray(outputs[0])
        if rows.ndim == 3 and rows.shape[0] == 1:
            rows = rows[0]
        if rows.ndim != 2 or rows.shape[1] != 6:
            raise ValueError(
                "Expected end-to-end output (1,N,6); received " + str(tuple(np.asarray(outputs[0]).shape))
            )

        frame_height, frame_width = frame.shape[:2]
        frame_area = float(frame_height * frame_width)
        debug = DetectionDebug(
            vision_backend="ml",
            vision_status="ready",
        )
        detections = []
        hsv = None
        ordered_rows = sorted(rows, key=lambda row: float(row[4]), reverse=True)
        for row in ordered_rows:
            confidence = float(row[4])
            class_id = int(round(float(row[5])))
            if class_id not in (0, 1):
                continue
            threshold = (
                config.ML_BALL_CONFIDENCE
                if class_id == 0
                else config.ML_CONE_CONFIDENCE
            )
            if not math.isfinite(confidence) or confidence < threshold:
                continue
            debug.candidate_count += 1
            x1 = (float(row[0]) - pad_x) / scale
            y1 = (float(row[1]) - pad_y) / scale
            x2 = (float(row[2]) - pad_x) / scale
            y2 = (float(row[3]) - pad_y) / scale
            x1, x2 = sorted((
                clamp(x1, 0.0, frame_width - 1.0),
                clamp(x2, 0.0, frame_width - 1.0),
            ))
            y1, y2 = sorted((
                clamp(y1, 0.0, frame_height - 1.0),
                clamp(y2, 0.0, frame_height - 1.0),
            ))
            width, height = x2 - x1, y2 - y1
            area = width * height
            area_ratio = area / frame_area
            if area_ratio < config.ML_MIN_BOX_AREA_RATIO:
                debug.rejected_small += 1
                continue
            if area_ratio > config.ML_MAX_BOX_AREA_RATIO:
                debug.rejected_large += 1
                continue
            center_x = int(round((x1 + x2) / 2.0))
            center_y = int(round((y1 + y2) / 2.0))
            kind = "ball" if class_id == 0 else "cone"
            allowance = (
                config.HORIZON_BALL_ALLOWANCE_RATIO
                if kind == "ball"
                else config.HORIZON_CONE_ALLOWANCE_RATIO
            )
            if self._above_horizon(
                center_x,
                center_y,
                horizon,
                frame_width,
                frame_height,
                allowance,
            ):
                debug.rejected_horizon += 1
                continue

            if hsv is None:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hue, color_score = self._color_data(
                hsv,
                int(x1),
                int(y1),
                max(1, int(round(width))),
                max(1, int(round(height))),
            )
            x, y = int(round(x1)), int(round(y1))
            box_width, box_height = (
                max(1, int(round(width))),
                max(1, int(round(height))),
            )
            contour = np.array(
                [
                    [[x, y]],
                    [[x + box_width, y]],
                    [[x + box_width, y + box_height]],
                    [[x, y + box_height]],
                ],
                dtype=np.int32,
            )
            detections.append(
                ObjectDetection(
                    kind=kind,
                    label=kind,
                    center_x=center_x,
                    center_y=center_y,
                    area=area,
                    confidence=confidence,
                    box=(x, y, box_width, box_height),
                    radius=max(1, int(round(max(width, height) / 2.0))),
                    contour=contour,
                    color=BALL_COLOR if kind == "ball" else CONE_COLOR,
                    ball_score=confidence if kind == "ball" else 0.0,
                    cone_score=confidence if kind == "cone" else 0.0,
                    color_score=color_score,
                    hue=hue,
                    certain=confidence >= config.ML_CERTAIN_CONFIDENCE,
                )
            )
            if len(detections) >= config.ML_MAX_DETECTIONS:
                break

        debug.accepted = sum(item.kind == "ball" for item in detections)
        debug.cones = sum(item.kind == "cone" for item in detections)
        debug.certain_count = sum(item.certain for item in detections)
        debug.uncertain_count = len(detections) - debug.certain_count
        return detections, debug

    def _load_manifest(self):
        if not self.manifest_path.exists():
            raise FileNotFoundError("ML model manifest not found: " + str(self.manifest_path))
        return json.loads(self.manifest_path.read_text(encoding="utf-8"))

    def _validate_manifest(self):
        if self.manifest.get("format") != "yolo26-e2e-onnx":
            raise ValueError("ML manifest format must be yolo26-e2e-onnx")
        classes = self.manifest.get("classes")
        if classes != {"0": "ball", "1": "cone"}:
            raise ValueError("ML manifest classes must be 0=ball and 1=cone")
        if self.input_width <= 0 or self.input_height <= 0:
            raise ValueError("ML manifest input dimensions must be positive")
        expected_hash = self.manifest.get("sha256")
        if (
            config.ML_VERIFY_MODEL_HASH
            and expected_hash
            and self.model_path.exists()
            and self._sha256(self.model_path) != expected_hash
        ):
            raise ValueError("ML model checksum does not match its manifest")

    def _create_session(self):
        if not self.model_path.exists():
            raise FileNotFoundError("ML model not found: " + str(self.model_path))
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise RuntimeError(
                "onnxruntime is not installed; install requirements-ml-pi.txt"
            ) from error
        options = ort.SessionOptions()
        options.intra_op_num_threads = config.ML_INTRA_THREADS
        options.inter_op_num_threads = config.ML_INTER_THREADS
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        return ort.InferenceSession(
            str(self.model_path),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )

    def _preprocess(self, frame):
        frame_height, frame_width = frame.shape[:2]
        scale = min(
            self.input_width / float(frame_width),
            self.input_height / float(frame_height),
        )
        resized_width = max(1, int(round(frame_width * scale)))
        resized_height = max(1, int(round(frame_height * scale)))
        resized = cv2.resize(
            frame,
            (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR,
        )
        pad_x = (self.input_width - resized_width) // 2
        pad_y = (self.input_height - resized_height) // 2
        canvas = np.full(
            (self.input_height, self.input_width, 3),
            114,
            dtype=np.uint8,
        )
        canvas[
            pad_y:pad_y + resized_height,
            pad_x:pad_x + resized_width,
        ] = resized
        tensor = canvas[:, :, ::-1].transpose(2, 0, 1)
        tensor = np.ascontiguousarray(tensor, dtype=np.float32) / 255.0
        return tensor[np.newaxis, ...], scale, pad_x, pad_y

    @staticmethod
    def _color_data(hsv, x, y, width, height):
        roi = hsv[
            max(0, y):min(hsv.shape[0], y + height),
            max(0, x):min(hsv.shape[1], x + width),
        ]
        if roi.size == 0:
            return 0.0, 0.0
        pixels = roi.reshape(-1, 3)
        colorful = pixels[pixels[:, 1] >= 25]
        if len(colorful) == 0:
            colorful = pixels
        return (
            float(np.median(colorful[:, 0])),
            float(np.median(colorful[:, 1])) / 255.0,
        )

    @staticmethod
    def _above_horizon(
        center_x,
        center_y,
        horizon,
        frame_width,
        frame_height,
        allowance_ratio,
    ):
        if horizon is None:
            return False
        allowed_y = horizon.y_at(center_x, frame_width) - frame_height * allowance_ratio
        return center_y < allowed_y

    @staticmethod
    def _sha256(path):
        digest = hashlib.sha256()
        with Path(path).open("rb") as model_file:
            for chunk in iter(lambda: model_file.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()


class AsyncMLDetector:
    """Run inference off-loop and replace queued work with the newest frame."""

    def __init__(self, detector_factory=None, clock=None):
        self.detector_factory = detector_factory or OnnxBallConeDetector
        self.clock = clock or time.time
        self.condition = threading.Condition()
        self.pending = None
        self.latest = None
        self.sequence = 0
        self.dropped_frames = 0
        self.error = ""
        self.ready = False
        self.stopped = False
        self.inference_fps = 0.0
        self._last_completed_at = None
        self.thread = threading.Thread(
            target=self._run,
            name="ball-cone-onnx",
            daemon=True,
        )
        self.thread.start()

    def submit(self, frame, captured_at, attitude=None, horizon=None):
        frame_copy = frame.copy()
        with self.condition:
            if self.stopped:
                return 0
            self.sequence += 1
            if self.pending is not None:
                self.dropped_frames += 1
            self.pending = (
                self.sequence,
                frame_copy,
                captured_at,
                attitude,
                horizon,
            )
            self.condition.notify()
            return self.sequence

    def poll_after(self, sequence):
        with self.condition:
            if self.latest is None or self.latest.sequence <= sequence:
                return None
            return self.latest

    def status(self, now):
        with self.condition:
            latest = self.latest
            return {
                "ready": self.ready,
                "error": self.error,
                "dropped_frames": self.dropped_frames,
                "inference_fps": self.inference_fps,
                "latency_ms": 0.0 if latest is None else latest.latency_ms,
                "age_seconds": 0.0 if latest is None else max(0.0, now - latest.captured_at),
                "sequence": 0 if latest is None else latest.sequence,
            }

    def close(self):
        with self.condition:
            self.stopped = True
            self.pending = None
            self.condition.notify_all()
        self.thread.join(timeout=3.0)

    def _run(self):
        try:
            detector = self.detector_factory()
        except BaseException as error:
            with self.condition:
                self.error = type(error).__name__ + ": " + str(error)
                self.ready = False
            return
        with self.condition:
            self.ready = True
        while True:
            with self.condition:
                while self.pending is None and not self.stopped:
                    self.condition.wait()
                if self.stopped:
                    return
                sequence, frame, captured_at, attitude, horizon = self.pending
                self.pending = None
            started = time.perf_counter()
            try:
                detections, debug = detector.detect(frame, horizon)
                error_message = ""
            except BaseException as error:
                detections = []
                debug = DetectionDebug(
                    vision_backend="ml",
                    vision_status="error",
                    vision_error=type(error).__name__ + ": " + str(error),
                )
                error_message = debug.vision_error
            latency_ms = (time.perf_counter() - started) * 1000.0
            completed_at = self.clock()
            result = InferenceResult(
                sequence=sequence,
                captured_at=captured_at,
                completed_at=completed_at,
                latency_ms=latency_ms,
                detections=detections,
                debug=debug,
                attitude=attitude,
                error=error_message,
            )
            with self.condition:
                if self._last_completed_at is not None:
                    instantaneous_fps = 1.0 / max(
                        0.001,
                        completed_at - self._last_completed_at,
                    )
                    self.inference_fps = (
                        instantaneous_fps
                        if self.inference_fps <= 0.0
                        else self.inference_fps * 0.75 + instantaneous_fps * 0.25
                    )
                self._last_completed_at = completed_at
                self.latest = result
                self.error = error_message
