import cv2
import numpy as np

from robot import config
from robot.models import DetectionDebug, VisionTarget
from robot.vision_common import (
    boxes_nearly_duplicate,
    clamp,
    hue_distance,
    hsv_range,
    is_spherical,
    make_mask,
    mask_from_range,
)


KNOWN_SPECS = [
    ("red", 0, (0, 0, 255)), ("orange", 10, (0, 140, 255)),
    ("yellow", 30, (0, 255, 255)), ("green", 60, (0, 255, 0)),
    ("blue", 110, (255, 0, 0)), ("purple", 135, (255, 0, 180)),
    ("pink", 160, (255, 0, 255)),
]


def make_known_colors():
    colors = []
    for name, hue, box_color in KNOWN_SPECS:
        color_range = hsv_range((hue, 150, 160), config.KNOWN_COLOR_HUE_PADDING, 255 - config.KNOWN_COLOR_SATURATION_MIN, 255 - config.KNOWN_COLOR_VALUE_MIN)
        color_range["lower"][2] = max(int(color_range["lower"][2]), config.KNOWN_COLOR_VALUE_MIN)
        if "lower2" in color_range:
            color_range["lower2"][2] = max(int(color_range["lower2"][2]), config.KNOWN_COLOR_VALUE_MIN)
        colors.append({"name": name, "box_color": box_color, "known_ball_color": True,
                       "min_detection_saturation": config.KNOWN_COLOR_SATURATION_MIN, **color_range})
    return colors


def calibrated_colors():
    # These ranges retain the proven samples from the original detector.
    return [
        {"name": "yellow", "lower": np.array([33, 0, 167]), "upper": np.array([54, 130, 255]), "box_color": (167, 233, 209), "min_detection_saturation": 45},
        {"name": "pink", "lower": np.array([147, 27, 160]), "upper": np.array([167, 153, 255]), "box_color": (207, 147, 229)},
        {"name": "blue", "lower": np.array([101, 190, 102]), "upper": np.array([113, 255, 252]), "box_color": (181, 89, 18)},
        {"name": "lime", "lower": np.array([62, 0, 165]), "upper": np.array([83, 136, 255]), "box_color": (183, 229, 161), "min_detection_saturation": 45},
        {"name": "purple", "lower": np.array([121, 97, 96]), "upper": np.array([130, 205, 248]), "box_color": (171, 71, 88)},
        {"name": "red", "lower": np.array([168, 137, 141]), "upper": np.array([177, 231, 255]), "box_color": (103, 64, 210)},
        {"name": "orange", "lower": np.array([0, 80, 181]), "upper": np.array([13, 187, 255]), "box_color": (110, 127, 238)},
        {"name": "lightblue", "lower": np.array([91, 97, 155]), "upper": np.array([104, 220, 255]), "box_color": (224, 184, 76)},
    ]


class AutoCalibrator:
    def __init__(self):
        self.profiles = []
        self.last_update = 0.0
        self.last_candidate_count = 0

    def update(self, hsv, current_time):
        if not config.AUTO_CALIBRATE:
            return self.as_colors()
        if current_time - self.last_update < config.AUTO_CALIBRATION_INTERVAL:
            return self.as_colors()
        self.last_update = current_time
        candidates = self._find_candidates(hsv)
        self.last_candidate_count = len(candidates)
        for candidate in candidates:
            self._merge(candidate, current_time)
        return self.as_colors()

    def _find_candidates(self, hsv):
        area_minimum = hsv.shape[0] * hsv.shape[1] * config.AUTO_CALIBRATION_MIN_AREA_RATIO
        mask = cv2.inRange(hsv, np.array([0, config.AUTO_CALIBRATION_SATURATION_MIN, config.AUTO_CALIBRATION_VALUE_MIN]), np.array([179, 255, 255]))
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.dilate(cv2.erode(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2), None, iterations=1), None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)
            if area < area_minimum or not is_spherical(contour, width, height, mask):
                continue
            contour_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            pixels = hsv[contour_mask == 255]
            if len(pixels) < 30:
                continue
            hue = int(np.median(pixels[:, 0]))
            name = self._classify(hue)
            if name is None:
                continue
            median = tuple(int(np.median(pixels[:, index])) for index in range(3))
            candidates.append({"name": name + "-auto", "hue": hue, "box_color": self._box_color(median),
                               "range": hsv_range(median, config.AUTO_CALIBRATION_HUE_PADDING, config.AUTO_CALIBRATION_SATURATION_PADDING, config.AUTO_CALIBRATION_VALUE_PADDING), "seen": 0.0})
        return candidates

    def _merge(self, candidate, current_time):
        for profile in self.profiles:
            if hue_distance(profile["hue"], candidate["hue"]) <= config.AUTO_CALIBRATION_MERGE_HUE_DISTANCE:
                profile.update(candidate)
                profile["seen"] = current_time
                return
        candidate["seen"] = current_time
        self.profiles.append(candidate)
        self.profiles = sorted(self.profiles, key=lambda profile: profile["seen"], reverse=True)[:config.AUTO_CALIBRATION_MAX_COLORS]

    def as_colors(self):
        return [{"name": profile["name"], "box_color": profile["box_color"], "auto_calibrated": True, **profile["range"]} for profile in self.profiles]

    @staticmethod
    def _classify(hue):
        name, distance = min(((name, hue_distance(hue, known_hue)) for name, known_hue, _ in KNOWN_SPECS), key=lambda item: item[1])
        return name if distance <= config.KNOWN_COLOR_HUE_PADDING + 4 else None

    @staticmethod
    def _box_color(hsv):
        pixel = np.uint8([[[*hsv]]])
        return tuple(int(value) for value in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0][0])


class BallDetector:
    name = "detector"

    def __init__(self):
        self.known_colors = make_known_colors()
        self.calibrated_colors = calibrated_colors()
        self.auto_calibrator = AutoCalibrator()

    def detect(self, hsv, current_time):
        auto_colors = self.auto_calibrator.update(hsv, current_time)
        colors = auto_colors + self.known_colors + self.calibrated_colors
        debug = DetectionDebug(active_colors=len(colors), auto_candidates=self.auto_calibrator.last_candidate_count, auto_profiles=len(auto_colors))
        frame_area = hsv.shape[0] * hsv.shape[1]
        targets = []
        for color in colors:
            debug.masks_checked += 1
            mask = make_mask(hsv, color)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                debug.contours_seen += 1
                area = cv2.contourArea(contour)
                x, y, width, height = cv2.boundingRect(contour)
                center_y = y + height // 2
                minimum = self.minimum_area(center_y, hsv.shape[0], frame_area)
                if area <= minimum:
                    debug.rejected_small += 1
                    continue
                if area >= frame_area * config.MAX_BALL_AREA_RATIO:
                    debug.rejected_large += 1
                    continue
                if not is_spherical(contour, width, height, mask):
                    debug.rejected_shape += 1
                    continue
                targets.append(VisionTarget(color["name"], x + width // 2, center_y, area,
                    clamp(area / float(frame_area * .12), 0.0, 1.0), (x, y, width, height), int(max(width, height) / 2), color["box_color"]))
        raw_count = len(targets)
        targets = self._cull_duplicates(targets)
        debug.rejected_overlap = raw_count - len(targets)
        debug.accepted = len(targets)
        return targets, debug, auto_colors

    @staticmethod
    def minimum_area(center_y, frame_height, frame_area):
        y_ratio = clamp(center_y / float(max(1, frame_height)), 0.0, 1.0)
        scale = clamp(config.MIN_BALL_AREA_TOP_SCALE, 0.0, 1.0)
        return frame_area * config.MIN_BALL_AREA_RATIO * (scale + (1.0 - scale) * y_ratio)

    @staticmethod
    def _cull_duplicates(targets):
        kept = []
        for target in sorted(targets, key=lambda item: item.area, reverse=True):
            if not any(boxes_nearly_duplicate(target.box, other.box) for other in kept):
                kept.append(target)
        return kept
