"""Temporal object tracking with color and camera-attitude compensation."""
from dataclasses import dataclass, replace
import math

from robot import config
from robot.vision_common import clamp, hue_distance


def wrapped_angle_delta(current, previous):
    return (current - previous + 180.0) % 360.0 - 180.0


@dataclass
class Track:
    track_id: int
    kind: str
    center_x: float
    center_y: float
    area: float
    hue: float
    ball_score: float
    cone_score: float
    color_score: float
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    hits: int = 1
    misses: int = 0
    last_time: float = 0.0
    last_detection: object = None


class ObjectTracker:
    def __init__(self):
        self.tracks = []
        self.next_track_id = 1
        self.last_attitude = None

    def update(self, detections, attitude, frame_width, frame_height, now, debug):
        existing_tracks = list(self.tracks)
        camera_dx, camera_dy, camera_roll = self._camera_motion(
            attitude,
            frame_width,
            frame_height,
        )
        debug.imu_compensation_x = camera_dx
        debug.imu_compensation_y = camera_dy
        debug.imu_compensation_roll = camera_roll
        predictions = {
            track.track_id: self._predict(
                track,
                camera_dx,
                camera_dy,
                camera_roll,
                frame_width,
                frame_height,
                now,
            )
            for track in existing_tracks
        }

        assignments = self._assign(detections, predictions, frame_width)
        updated = []
        matched_track_ids = set()
        matched_detection_indices = set()
        for detection_index, track in assignments:
            detection = detections[detection_index]
            predicted_x, predicted_y = predictions[track.track_id]
            tracked_detection = self._update_track(
                track,
                detection,
                predicted_x,
                predicted_y,
                now,
            )
            track.last_detection = tracked_detection
            updated.append(tracked_detection)
            matched_track_ids.add(track.track_id)
            matched_detection_indices.add(detection_index)

        for index, detection in enumerate(detections):
            if index in matched_detection_indices:
                continue
            track = self._new_track(detection, now)
            tracked_detection = self._decorate(
                detection,
                track,
                force_uncertain=True,
            )
            track.last_detection = tracked_detection
            updated.append(tracked_detection)

        for track in existing_tracks:
            if track.track_id in matched_track_ids:
                continue
            track.misses += 1
            if track.misses <= config.TRACK_MAX_MISSES:
                predicted_x, predicted_y = predictions[track.track_id]
                track.center_x, track.center_y = predicted_x, predicted_y
                track.last_time = now
                if track.last_detection is not None:
                    updated.append(self._predicted_detection(track))

        self.tracks = [
            track for track in self.tracks
            if track.misses <= config.TRACK_MAX_MISSES
        ]
        debug.tracked_count = len(self.tracks)
        debug.predicted_count = sum(item.predicted for item in updated)
        debug.unknown_count = sum(item.kind == "unknown" for item in updated)
        debug.uncertain_count = sum(item.kind != "unknown" and not item.certain for item in updated)
        debug.certain_count = sum(item.certain for item in updated)
        debug.accepted = sum(item.kind == "ball" for item in updated)
        debug.cones = sum(item.kind == "cone" for item in updated)
        return updated

    def _assign(self, detections, predictions, frame_width):
        maximum_distance = max(30.0, frame_width * config.TRACK_MATCH_RADIUS_RATIO)
        pairs = []
        for detection_index, detection in enumerate(detections):
            for track in self.tracks:
                predicted_x, predicted_y = predictions[track.track_id]
                distance = math.hypot(
                    detection.center_x - predicted_x,
                    detection.center_y - predicted_y,
                )
                if distance > maximum_distance:
                    continue
                color_cost = hue_distance(detection.hue, track.hue) / 90.0
                size_cost = abs(math.log(max(1.0, detection.area) / max(1.0, track.area)))
                class_cost = (
                    0.65
                    if detection.kind != "unknown"
                    and track.kind != "unknown"
                    and detection.kind != track.kind
                    else 0.0
                )
                cost = (
                    distance / maximum_distance
                    + color_cost * config.TRACK_COLOR_WEIGHT
                    + min(size_cost, 2.0) * config.TRACK_SIZE_WEIGHT
                    + class_cost
                )
                pairs.append((cost, detection_index, track))

        assignments = []
        used_detections = set()
        used_tracks = set()
        for cost, detection_index, track in sorted(pairs, key=lambda item: item[0]):
            if cost > 1.35:
                continue
            if detection_index in used_detections or track.track_id in used_tracks:
                continue
            assignments.append((detection_index, track))
            used_detections.add(detection_index)
            used_tracks.add(track.track_id)
        return assignments

    def _update_track(self, track, detection, predicted_x, predicted_y, now):
        position_alpha = clamp(config.TRACK_POSITION_SMOOTHING, 0.0, 1.0)
        new_x = predicted_x * (1.0 - position_alpha) + detection.center_x * position_alpha
        new_y = predicted_y * (1.0 - position_alpha) + detection.center_y * position_alpha
        measured_dx = new_x - track.center_x
        measured_dy = new_y - track.center_y
        velocity_alpha = clamp(config.TRACK_VELOCITY_SMOOTHING, 0.0, 1.0)
        track.velocity_x = track.velocity_x * (1.0 - velocity_alpha) + measured_dx * velocity_alpha
        track.velocity_y = track.velocity_y * (1.0 - velocity_alpha) + measured_dy * velocity_alpha

        score_alpha = clamp(config.TRACK_SCORE_SMOOTHING, 0.0, 1.0)
        track.ball_score = track.ball_score * (1.0 - score_alpha) + detection.ball_score * score_alpha
        track.cone_score = track.cone_score * (1.0 - score_alpha) + detection.cone_score * score_alpha
        track.color_score = track.color_score * (1.0 - score_alpha) + detection.color_score * score_alpha
        track.hue = self._blend_hue(track.hue, detection.hue, score_alpha)
        track.area = track.area * (1.0 - score_alpha) + detection.area * score_alpha
        track.center_x, track.center_y = new_x, new_y
        track.hits += 1
        track.misses = 0
        track.last_time = now
        track.kind = self._tracked_kind(track, detection)
        return self._decorate(detection, track)

    def _new_track(self, detection, now):
        track = Track(
            track_id=self.next_track_id,
            kind=detection.kind,
            center_x=detection.center_x,
            center_y=detection.center_y,
            area=detection.area,
            hue=detection.hue,
            ball_score=detection.ball_score,
            cone_score=detection.cone_score,
            color_score=detection.color_score,
            last_time=now,
        )
        self.next_track_id += 1
        self.tracks.append(track)
        return track

    @staticmethod
    def _tracked_kind(track, detection):
        if detection.rejection_reason.startswith("above ") or "area implausible" in detection.rejection_reason:
            return "unknown"
        score = max(track.ball_score, track.cone_score)
        margin = abs(track.ball_score - track.cone_score)
        if score < config.OBJECT_UNCERTAIN_SCORE or margin < config.OBJECT_CLASS_MARGIN:
            return "unknown"
        return "ball" if track.ball_score >= track.cone_score else "cone"

    @staticmethod
    def _decorate(detection, track, force_uncertain=False):
        score = max(track.ball_score, track.cone_score)
        margin = abs(track.ball_score - track.cone_score)
        certain = (
            not force_uncertain
            and track.kind != "unknown"
            and track.hits >= config.TRACK_CONFIRM_FRAMES
            and score >= config.OBJECT_CERTAIN_SCORE
            and margin >= config.OBJECT_CERTAIN_MARGIN
        )
        color = (
            (64, 230, 64)
            if track.kind == "ball"
            else (0, 150, 255)
            if track.kind == "cone"
            else (0, 220, 220)
        )
        hard_rejection = (
            detection.rejection_reason.startswith("above ")
            or "area implausible" in detection.rejection_reason
        )
        return replace(
            detection,
            kind=track.kind,
            label=track.kind,
            center_x=int(round(track.center_x)),
            center_y=int(round(track.center_y)),
            confidence=score,
            color=color,
            ball_score=track.ball_score,
            cone_score=track.cone_score,
            color_score=track.color_score,
            hue=track.hue,
            certain=certain,
            track_id=track.track_id,
            track_hits=track.hits,
            motion_x=track.velocity_x,
            motion_y=track.velocity_y,
            rejection_reason=(
                detection.rejection_reason
                if track.kind == "unknown" or hard_rejection
                else ""
            ),
        )

    @staticmethod
    def _predict(track, camera_dx, camera_dy, camera_roll, frame_width, frame_height, now):
        delta_time = clamp(now - track.last_time, 0.0, 0.25)
        center_x, center_y = frame_width / 2.0, frame_height / 2.0
        relative_x, relative_y = track.center_x - center_x, track.center_y - center_y
        angle = math.radians(camera_roll)
        rotated_x = relative_x * math.cos(angle) - relative_y * math.sin(angle)
        rotated_y = relative_x * math.sin(angle) + relative_y * math.cos(angle)
        frame_rate_scale = delta_time * 30.0
        return (
            center_x + rotated_x + camera_dx + track.velocity_x * frame_rate_scale,
            center_y + rotated_y + camera_dy + track.velocity_y * frame_rate_scale,
        )

    @staticmethod
    def _predicted_detection(track):
        previous = track.last_detection
        shift_x = int(round(track.center_x - previous.center_x))
        shift_y = int(round(track.center_y - previous.center_y))
        x, y, width, height = previous.box
        return replace(
            previous,
            center_x=int(round(track.center_x)),
            center_y=int(round(track.center_y)),
            box=(x + shift_x, y + shift_y, width, height),
            certain=False,
            predicted=True,
            track_hits=track.hits,
            motion_x=track.velocity_x,
            motion_y=track.velocity_y,
            rejection_reason="track prediction after miss " + str(track.misses),
        )

    def _camera_motion(self, attitude, frame_width, frame_height):
        if (
            attitude is None
            or not attitude.connected
            or attitude.yaw_degrees is None
            or attitude.pitch_degrees is None
            or attitude.roll_degrees is None
        ):
            self.last_attitude = None
            return 0.0, 0.0, 0.0
        if self.last_attitude is None:
            self.last_attitude = attitude
            return 0.0, 0.0, 0.0
        yaw_delta = wrapped_angle_delta(
            attitude.yaw_degrees,
            self.last_attitude.yaw_degrees,
        )
        pitch_delta = wrapped_angle_delta(
            attitude.pitch_degrees,
            self.last_attitude.pitch_degrees,
        )
        roll_delta = wrapped_angle_delta(
            attitude.roll_degrees,
            self.last_attitude.roll_degrees,
        )
        self.last_attitude = attitude
        return (
            yaw_delta * frame_width / max(1.0, config.CAMERA_HORIZONTAL_FOV_DEG)
            * config.IMU_TRACK_YAW_SIGN,
            pitch_delta * frame_height / max(1.0, config.CAMERA_VERTICAL_FOV_DEG)
            * config.IMU_TRACK_PITCH_SIGN,
            roll_delta * config.IMU_TRACK_ROLL_SIGN,
        )

    @staticmethod
    def _blend_hue(first, second, alpha):
        delta = (second - first + 90.0) % 180.0 - 90.0
        return (first + delta * alpha) % 180.0
