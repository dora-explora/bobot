import cv2
import numpy as np

from robot import config


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def hue_distance(first_hue, second_hue):
    direct = abs(int(first_hue) - int(second_hue))
    return min(direct, 180 - direct)


def hue_in_range(hue, minimum, maximum):
    minimum, maximum = int(minimum) % 180, int(maximum) % 180
    return minimum <= hue <= maximum if minimum <= maximum else hue >= minimum or hue <= maximum


def hsv_range(value, hue_padding, saturation_padding, value_padding):
    hue, saturation, brightness = (int(part) for part in value)
    lower_hue, upper_hue = hue - hue_padding, hue + hue_padding
    low_sv = (max(0, saturation - saturation_padding), max(0, brightness - value_padding))
    high_sv = (min(255, saturation + saturation_padding), min(255, brightness + value_padding))
    result = {"lower": np.array([max(0, lower_hue), *low_sv]), "upper": np.array([min(179, upper_hue), *high_sv])}
    if lower_hue < 0:
        result.update(lower2=np.array([179 + lower_hue, *low_sv]), upper2=np.array([179, *high_sv]))
    elif upper_hue > 179:
        result.update(lower=np.array([lower_hue, *low_sv]), upper=np.array([179, *high_sv]), lower2=np.array([0, *low_sv]), upper2=np.array([upper_hue - 179, *high_sv]))
    return result


def mask_from_range(hsv, color_range):
    mask = cv2.inRange(hsv, color_range["lower"], color_range["upper"])
    if "lower2" in color_range:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, color_range["lower2"], color_range["upper2"]))
    return mask


def make_mask(hsv, color):
    color_range = dict(color)
    color_range["lower"] = color["lower"].copy()
    if "min_detection_saturation" in color:
        color_range["lower"][1] = max(color_range["lower"][1], color["min_detection_saturation"])
    if "lower2" in color:
        color_range["lower2"] = color["lower2"].copy()
        if "min_detection_saturation" in color:
            color_range["lower2"][1] = max(color_range["lower2"][1], color["min_detection_saturation"])
    mask = mask_from_range(hsv, color_range)
    kernel = np.ones((7, 7), np.uint8)
    return cv2.dilate(cv2.erode(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2), None, iterations=1), None, iterations=2)


def contour_is_triangular(contour, perimeter=None):
    perimeter = perimeter or cv2.arcLength(contour, True)
    approximation = cv2.approxPolyDP(contour, max(0.001, config.TRIANGLE_APPROX_EPSILON) * perimeter, True)
    return len(approximation) <= 4 and cv2.isContourConvex(approximation)


def is_spherical(contour, width, height, mask):
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0 or contour_is_triangular(contour, perimeter):
        return False
    area = cv2.contourArea(contour)
    circularity = 4 * np.pi * area / (perimeter * perimeter)
    if not 0.6 <= width / float(max(1, height)) <= 1.4 or circularity < config.MIN_BALL_CIRCULARITY:
        return False
    (x, y), radius = cv2.minEnclosingCircle(contour)
    if radius <= 0:
        return False
    circle_mask = np.zeros_like(mask)
    cv2.circle(circle_mask, (int(x), int(y)), int(radius), 255, -1)
    circle_pixels = cv2.countNonZero(circle_mask)
    if not circle_pixels:
        return False
    circle_fill = area / (np.pi * radius * radius)
    color_fill = cv2.countNonZero(cv2.bitwise_and(mask, circle_mask)) / float(circle_pixels)
    return circle_fill >= config.MIN_BALL_CIRCLE_FILL and color_fill >= 0.45


def box_intersection_area(first, second):
    x1, y1, w1, h1 = first
    x2, y2, w2, h2 = second
    return max(0, min(x1 + w1, x2 + w2) - max(x1, x2)) * max(0, min(y1 + h1, y2 + h2) - max(y1, y2))


def boxes_nearly_duplicate(first, second):
    overlap = box_intersection_area(first, second)
    smallest = min(first[2] * first[3], second[2] * second[3])
    return smallest > 0 and overlap / float(smallest) >= 0.85
