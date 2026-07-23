"""Color-agnostic candidate generation with explicit ball/cone/unknown scoring."""
import math

import cv2
import numpy as np

from robot import config
from robot.models import DetectionDebug, ObjectDetection
from robot.vision_common import boxes_nearly_duplicate, clamp, hue_in_range


BALL_COLOR = (64, 230, 64)
CONE_COLOR = (0, 150, 255)
UNKNOWN_COLOR = (0, 220, 220)


class ObjectDetector:
    """Propose colorful/locally distinct regions, then classify by shape."""

    def detect(self, frame, hsv, horizon):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        masks = self._candidate_masks(hsv, lab)
        contours = []
        for mask in masks:
            mask_contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            contours.extend(mask_contours)
        frame_height, frame_width = hsv.shape[:2]
        frame_area = frame_height * frame_width
        debug = DetectionDebug(
            masks_checked=len(masks),
            contours_seen=len(contours),
            candidate_count=len(contours),
        )
        detections = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < frame_area * config.OBJECT_MIN_AREA_RATIO:
                debug.rejected_small += 1
                continue
            if area > frame_area * config.OBJECT_MAX_AREA_RATIO:
                debug.rejected_large += 1
                continue

            detections.append(self._classify(contour, hsv, lab))

        detections = self._deduplicate(detections, debug)
        for detection in detections:
            if detection.kind == "ball":
                if not self._ball_area_valid(detection, frame_height, frame_area):
                    detection.kind = "unknown"
                    detection.label = "unknown"
                    detection.certain = False
                    detection.rejection_reason = "ball area implausible for image height"
                    debug.rejected_shape += 1
                elif self._above_horizon(
                    detection,
                    horizon,
                    frame_width,
                    frame_height,
                    config.HORIZON_BALL_ALLOWANCE_RATIO,
                ):
                    detection.kind = "unknown"
                    detection.label = "unknown"
                    detection.certain = False
                    detection.rejection_reason = "above ball horizon"
                    debug.rejected_horizon += 1
            elif detection.kind == "cone" and self._above_horizon(
                detection,
                horizon,
                frame_width,
                frame_height,
                config.HORIZON_CONE_ALLOWANCE_RATIO,
            ):
                detection.kind = "unknown"
                detection.label = "unknown"
                detection.certain = False
                detection.rejection_reason = "above cone horizon"
                debug.rejected_horizon += 1

        debug.unknown_count = sum(item.kind == "unknown" for item in detections)
        debug.uncertain_count = sum(item.kind != "unknown" and not item.certain for item in detections)
        debug.certain_count = sum(item.certain for item in detections)
        debug.accepted = sum(item.kind == "ball" for item in detections)
        debug.cones = sum(item.kind == "cone" for item in detections)
        return detections, debug

    @staticmethod
    def _candidate_masks(hsv, lab):
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        a_channel = lab[:, :, 1].astype(np.float32) - 128.0
        b_channel = lab[:, :, 2].astype(np.float32) - 128.0
        chroma = cv2.magnitude(a_channel, b_channel)
        sigma = max(1.0, config.OBJECT_LOCAL_BLUR_SIGMA)
        blurred_a = cv2.GaussianBlur(a_channel, (0, 0), sigma)
        blurred_b = cv2.GaussianBlur(b_channel, (0, 0), sigma)
        local_delta = cv2.magnitude(a_channel - blurred_a, b_channel - blurred_b)

        valid_value = cv2.inRange(value, config.OBJECT_VALUE_MIN, config.OBJECT_VALUE_MAX)
        saturated = cv2.threshold(
            saturation,
            config.OBJECT_SATURATION_MIN - 1,
            255,
            cv2.THRESH_BINARY,
        )[1]
        chromatic = cv2.threshold(
            chroma,
            config.OBJECT_CHROMA_MIN,
            255,
            cv2.THRESH_BINARY,
        )[1].astype(np.uint8)
        locally_distinct = cv2.threshold(
            local_delta,
            config.OBJECT_LOCAL_CHROMA_DELTA,
            255,
            cv2.THRESH_BINARY,
        )[1].astype(np.uint8)
        global_mask = cv2.bitwise_and(
            valid_value,
            cv2.bitwise_or(saturated, chromatic),
        )
        global_mask = cv2.morphologyEx(
            global_mask,
            cv2.MORPH_CLOSE,
            np.ones((7, 7), np.uint8),
            iterations=2,
        )
        global_mask = cv2.morphologyEx(
            global_mask,
            cv2.MORPH_OPEN,
            np.ones((3, 3), np.uint8),
            iterations=1,
        )

        # Keep local contrast separate. If it were ORed into the global mask, a
        # colorful wood floor could connect to and swallow every object on it.
        local_mask = cv2.bitwise_and(valid_value, locally_distinct)
        local_mask = cv2.dilate(
            local_mask,
            np.ones((3, 3), np.uint8),
            iterations=2,
        )
        local_mask = cv2.morphologyEx(
            local_mask,
            cv2.MORPH_CLOSE,
            np.ones((7, 7), np.uint8),
            iterations=2,
        )
        return global_mask, local_mask

    def _classify(self, contour, hsv, lab):
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        x, y, width, height = cv2.boundingRect(contour)
        (circle_x, circle_y), radius = cv2.minEnclosingCircle(contour)
        circle_area = math.pi * radius * radius
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        circularity = clamp(4.0 * math.pi * area / max(1.0, perimeter * perimeter), 0.0, 1.0)
        circle_fill = clamp(area / max(1.0, circle_area), 0.0, 1.0)
        aspect = min(width, height) / float(max(1, max(width, height)))
        solidity = clamp(area / max(1.0, hull_area), 0.0, 1.0)
        radial = self._radial_consistency(contour, circle_x, circle_y)

        contour_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)
        hsv_pixels = hsv[contour_mask > 0]
        lab_pixels = lab[contour_mask > 0]
        median_hue = float(np.median(hsv_pixels[:, 0])) if len(hsv_pixels) else 0.0
        median_saturation = float(np.median(hsv_pixels[:, 1])) if len(hsv_pixels) else 0.0
        lab_chroma = (
            np.hypot(
                lab_pixels[:, 1].astype(np.float32) - 128.0,
                lab_pixels[:, 2].astype(np.float32) - 128.0,
            )
            if len(lab_pixels)
            else np.array([0.0])
        )
        color_score = clamp(
            median_saturation / 255.0 * 0.55 + float(np.median(lab_chroma)) / 90.0 * 0.45,
            0.0,
            1.0,
        )
        orange_score = float(self._orange_fraction(hsv_pixels))
        taper = self._taper_score(contour_mask[y:y + height, x:x + width])
        approximation = cv2.approxPolyDP(contour, max(1.0, perimeter * config.TRIANGLE_APPROX_EPSILON), True)
        polygonal = len(approximation) <= 4 and cv2.isContourConvex(approximation)
        triangle_score = 1.0 if polygonal else clamp((7 - len(approximation)) / 3.0, 0.0, 1.0)
        vertical = clamp((height / float(max(1, width)) - 0.65) / 1.1, 0.0, 1.0)

        ball_score = clamp(
            circularity * 0.24
            + circle_fill * 0.20
            + radial * 0.17
            + aspect * 0.11
            + solidity * 0.08
            + color_score * 0.10
            + (1.0 - taper) * 0.10
            - triangle_score * 0.16,
            0.0,
            1.0,
        )
        cone_score = clamp(
            taper * 0.34
            + triangle_score * 0.20
            + vertical * 0.10
            + solidity * 0.10
            + orange_score * 0.22
            + (1.0 - circularity) * 0.04
            - radial * 0.08,
            0.0,
            1.0,
        )

        ball_viable = (
            not polygonal
            and aspect >= 0.62
            and circle_fill >= 0.48
            and radial >= 0.36
        )
        cone_viable = orange_score >= 0.20 and taper >= 0.28
        if not ball_viable:
            ball_score = min(ball_score, config.OBJECT_UNCERTAIN_SCORE - 0.01)
        if not cone_viable:
            cone_score = min(cone_score, config.OBJECT_UNCERTAIN_SCORE - 0.01)

        winner = "ball" if ball_score >= cone_score else "cone"
        winner_score = max(ball_score, cone_score)
        margin = abs(ball_score - cone_score)
        if winner_score < config.OBJECT_UNCERTAIN_SCORE or margin < config.OBJECT_CLASS_MARGIN:
            kind = "unknown"
        else:
            kind = winner
        certain = (
            kind != "unknown"
            and winner_score >= config.OBJECT_CERTAIN_SCORE
            and margin >= config.OBJECT_CERTAIN_MARGIN
        )
        return ObjectDetection(
            kind=kind,
            label=kind,
            center_x=int(round(circle_x if kind == "ball" else x + width / 2.0)),
            center_y=int(round(circle_y if kind == "ball" else y + height / 2.0)),
            area=area,
            confidence=winner_score,
            box=(x, y, width, height),
            radius=max(1, int(round(radius))),
            contour=contour,
            color=BALL_COLOR if kind == "ball" else CONE_COLOR if kind == "cone" else UNKNOWN_COLOR,
            ball_score=ball_score,
            cone_score=cone_score,
            color_score=color_score,
            hue=median_hue,
            certain=certain,
            rejection_reason="" if kind != "unknown" else "ambiguous class scores",
        )

    @staticmethod
    def _deduplicate(detections, debug):
        kept = []
        ordered = sorted(
            detections,
            key=lambda item: (item.certain, item.confidence, item.area),
            reverse=True,
        )
        for detection in ordered:
            if any(
                boxes_nearly_duplicate(detection.box, existing.box)
                for existing in kept
            ):
                debug.rejected_overlap += 1
                continue
            kept.append(detection)
        return kept

    @staticmethod
    def _radial_consistency(contour, center_x, center_y):
        points = contour.reshape(-1, 2).astype(np.float32)
        if len(points) < 5:
            return 0.0
        distances = np.hypot(points[:, 0] - center_x, points[:, 1] - center_y)
        mean = float(np.mean(distances))
        if mean <= 0:
            return 0.0
        coefficient = float(np.std(distances)) / mean
        return 1.0 - clamp(coefficient / 0.34, 0.0, 1.0)

    @staticmethod
    def _orange_fraction(pixels):
        if len(pixels) == 0:
            return 0.0
        matches = [
            hue_in_range(pixel[0], config.CONE_HUE_MIN, config.CONE_HUE_MAX)
            and pixel[1] >= config.CONE_SATURATION_MIN
            and config.CONE_VALUE_MIN <= pixel[2] <= config.CONE_VALUE_MAX
            for pixel in pixels
        ]
        return sum(matches) / float(len(matches))

    @classmethod
    def _taper_score(cls, roi):
        if roi.size == 0:
            return 0.0
        top, middle, bottom = [
            cls._width_at(roi, start, end)
            for start, end in ((0.08, 0.30), (0.40, 0.60), (0.72, 0.95))
        ]
        if bottom <= 0 or middle <= 0:
            return 0.0
        top_growth = clamp((bottom - top) / float(bottom) / 0.65, 0.0, 1.0)
        middle_growth = clamp((bottom - middle) / float(bottom) / 0.28, 0.0, 1.0)
        monotonic = 1.0 if top <= middle <= bottom else 0.0
        return top_growth * 0.45 + middle_growth * 0.35 + monotonic * 0.20

    @staticmethod
    def _width_at(mask, start_ratio, end_ratio):
        start = int(mask.shape[0] * start_ratio)
        end = max(start + 1, int(mask.shape[0] * end_ratio))
        columns = np.where(np.any(mask[start:end] > 0, axis=0))[0]
        return 0 if len(columns) == 0 else int(columns[-1] - columns[0] + 1)

    @staticmethod
    def _ball_area_valid(detection, frame_height, frame_area):
        y_ratio = clamp(detection.center_y / float(max(1, frame_height)), 0.0, 1.0)
        minimum_scale = clamp(config.MIN_BALL_AREA_TOP_SCALE, 0.0, 1.0)
        maximum_scale = clamp(config.MAX_BALL_AREA_TOP_SCALE, 0.0, 1.0)
        minimum = frame_area * config.MIN_BALL_AREA_RATIO * (
            minimum_scale + (1.0 - minimum_scale) * y_ratio
        )
        maximum = frame_area * config.MAX_BALL_AREA_RATIO * (
            maximum_scale + (1.0 - maximum_scale) * y_ratio
        )
        return minimum < detection.area < maximum

    @staticmethod
    def _above_horizon(detection, horizon, frame_width, frame_height, allowance_ratio):
        if horizon is None:
            return False
        allowed_y = horizon.y_at(detection.center_x, frame_width) - frame_height * allowance_ratio
        return detection.center_y < allowed_y
