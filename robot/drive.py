import numpy as np

from robot import config
from robot.models import DriveCommand, VisionTarget
from robot.vision_common import clamp


def score_target(target, targets, frame_width, frame_height, frame_area):
    distance = clamp(target.center_y / float(frame_height), 0.0, 1.0)
    area = clamp(target.area / float(frame_area * config.CLOSE_BALL_AREA_RATIO), 0.0, 1.0)
    center = 1.0 - clamp(abs(target.center_x - frame_width / 2.0) / (frame_width / 2.0), 0.0, 1.0)
    radius = max(1.0, frame_width * config.TARGET_CLUSTER_RADIUS_RATIO)
    cluster_area, neighbors = target.area, 0
    for other in targets:
        if other is target:
            continue
        distance_between = np.hypot(target.center_x - other.center_x, target.center_y - other.center_y)
        if distance_between <= radius:
            cluster_area += other.area * (1.0 - distance_between / radius)
            neighbors += 1
    cluster = clamp(
        clamp(cluster_area / float(frame_area * config.CLOSE_BALL_AREA_RATIO), 0.0, 1.0) * .65
        + clamp(neighbors / 3.0, 0.0, 1.0) * .35, 0.0, 1.0)
    return {"score": distance * config.TARGET_DISTANCE_WEIGHT + cluster * config.TARGET_CLUSTER_WEIGHT
                     + area * config.TARGET_AREA_WEIGHT + center * config.TARGET_CENTER_WEIGHT,
            "distance": distance, "cluster": cluster, "area": area, "center": center, "neighbors": neighbors}


class TargetStabilizer:
    def __init__(self):
        self.target = None
        self.last_seen = 0.0
        self.acquired = 0.0
        self.switch_candidate = None
        self.switch_frames = 0

    def select(self, targets, frame_width, frame_height, now, debug):
        debug.raw_target_count = len(targets)
        if not targets:
            return self._hold_or_clear(now, debug)
        frame_area = frame_width * frame_height
        scored = sorted(((score_target(target, targets, frame_width, frame_height, frame_area), target) for target in targets),
                        key=lambda item: (item[0]["score"], item[0]["distance"], item[0]["cluster"], item[1].area), reverse=True)
        best_score, best = scored[0]
        matched = self._find_match(targets, frame_width)
        if self.target is None or matched is None:
            return self._lock(best, best_score, now, debug)
        current_score = score_target(matched, targets, frame_width, frame_height, frame_area)
        if best is not matched and self._should_switch(best, best_score, current_score, frame_width):
            return self._lock(best, best_score, now, debug)
        self.switch_candidate, self.switch_frames = None, 0
        return self._lock(matched, current_score, now, debug)

    def _find_match(self, targets, frame_width):
        if self.target is None:
            return None
        radius = max(24.0, frame_width * config.TARGET_LOCK_RADIUS_RATIO)
        candidates = [(np.hypot(target.center_x - self.target.center_x, target.center_y - self.target.center_y), target) for target in targets]
        candidates = [candidate for candidate in candidates if candidate[0] <= radius]
        return min(candidates, default=(None, None), key=lambda item: item[0])[1]

    def _should_switch(self, candidate, candidate_score, current_score, frame_width):
        if candidate_score["score"] < current_score["score"] + config.TARGET_SWITCH_MARGIN:
            self.switch_candidate, self.switch_frames = None, 0
            return False
        radius = max(24.0, frame_width * config.TARGET_LOCK_RADIUS_RATIO)
        same = self.switch_candidate is not None and np.hypot(candidate.center_x - self.switch_candidate.center_x, candidate.center_y - self.switch_candidate.center_y) <= radius
        self.switch_candidate = candidate
        self.switch_frames = self.switch_frames + 1 if same else 1
        return self.switch_frames >= config.TARGET_SWITCH_FRAMES

    def _lock(self, target, priority, now, debug):
        if self.target is None:
            self.acquired = now
            self.target = target
        else:
            alpha = clamp(config.TARGET_SMOOTHING, 0.0, 1.0)
            self.target = VisionTarget(target.label,
                int(round(self.target.center_x * (1 - alpha) + target.center_x * alpha)),
                int(round(self.target.center_y * (1 - alpha) + target.center_y * alpha)),
                self.target.area * (1 - alpha) + target.area * alpha,
                self.target.confidence * (1 - alpha) + target.confidence * alpha,
                target.box, int(round(self.target.radius * (1 - alpha) + target.radius * alpha)), target.color)
        self.last_seen = now
        self._fill_debug(priority, now, False, debug)
        return self.target

    def _hold_or_clear(self, now, debug):
        if self.target is not None and now - self.last_seen <= config.TARGET_HOLD_SECONDS:
            self._fill_debug(None, now, True, debug)
            return self.target
        self.target, self.switch_candidate, self.switch_frames = None, None, 0
        debug.stable_target_locked = False
        debug.stable_target_held = False
        debug.stable_target_label = "none"
        debug.stable_target_age = 0.0
        return None

    def _fill_debug(self, priority, now, held, debug):
        debug.stable_target_locked = self.target is not None
        debug.stable_target_held = held
        debug.stable_target_label = self.target.label if self.target else "none"
        debug.stable_target_age = now - self.acquired
        debug.switch_candidate_frames = self.switch_frames
        if priority:
            debug.priority_score = priority["score"]
            debug.priority_distance = priority["distance"]
            debug.priority_cluster = priority["cluster"]
            debug.priority_area = priority["area"]
            debug.priority_center = priority["center"]
            debug.priority_neighbors = priority["neighbors"]


class DriveStabilizer:
    def __init__(self):
        self.last_steering = 0.0
        self.last_time = 0.0

    def smooth(self, command, now, debug):
        debug.raw_steering = command.steering
        if not self.last_time:
            self.last_time, self.last_steering = now, command.steering
            debug.smoothed_steering = command.steering
            return command
        maximum_change = config.STEERING_SLEW_RATE * max(.001, now - self.last_time)
        delta = command.steering - self.last_steering
        limited = clamp(delta, -maximum_change, maximum_change)
        steering = self.last_steering + limited
        self.last_time, self.last_steering = now, steering
        debug.smoothed_steering = steering
        debug.steering_limited = abs(delta - limited) > .001
        return DriveCommand(steering, command.throttle, command.mode, command.reason)


def ball_seeking_command(target, frame_width, frame_area, last_seen, now):
    if target is None:
        reason = "coasting after recent target" if now - last_seen <= config.LOST_TARGET_TIMEOUT else "no target"
        return DriveCommand(0.0, 0.0, "assist", reason)
    error = (target.center_x - frame_width / 2.0) / (frame_width / 2.0)
    steering = 0.0 if abs(error) < config.STEERING_DEADBAND else clamp(error * config.STEERING_GAIN, -1.0, 1.0)
    if target.area / float(frame_area) >= config.CLOSE_BALL_AREA_RATIO:
        return DriveCommand(steering, 0.0, "assist", "target close")
    return DriveCommand(steering, min(config.MAX_TRIAL_THROTTLE, config.THROTTLE_HARD_LIMIT), "assist", "seeking " + target.label)
