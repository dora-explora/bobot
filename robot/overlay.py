"""Reusable, text-free OpenCV overlays for robot vision output."""

import math

import cv2
import numpy as np


def _canvas_size(image):
    if not isinstance(image, np.ndarray) or image.ndim < 2:
        return None
    height, width = image.shape[:2]
    if height <= 0 or width <= 0:
        return None
    return int(width), int(height)


def _point(value):
    try:
        x, y = float(value[0]), float(value[1])
    except (IndexError, TypeError, ValueError):
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return int(round(x)), int(round(y))


def _points(values):
    try:
        result = [_point(value) for value in values]
    except TypeError:
        return []
    return result if result and all(point is not None for point in result) else []


def _positive_int(value, default=1):
    try:
        number = int(round(float(value)))
    except (TypeError, ValueError):
        return default
    return max(1, number)


def _clipped_line(image, start, end, color, thickness):
    size = _canvas_size(image)
    start, end = _point(start), _point(end)
    if size is None or start is None or end is None:
        return None
    visible, clipped_start, clipped_end = cv2.clipLine(
        (0, 0, size[0], size[1]), start, end
    )
    if not visible:
        return None
    cv2.line(
        image,
        clipped_start,
        clipped_end,
        color,
        _positive_int(thickness),
        cv2.LINE_8,
    )
    return clipped_start, clipped_end


def _dashed_line(
    image,
    start,
    end,
    color,
    thickness,
    dash_length,
    gap_length,
):
    size = _canvas_size(image)
    start, end = _point(start), _point(end)
    if size is None or start is None or end is None:
        return

    visible, start, end = cv2.clipLine((0, 0, size[0], size[1]), start, end)
    if not visible:
        return

    delta_x, delta_y = end[0] - start[0], end[1] - start[1]
    distance = math.hypot(delta_x, delta_y)
    if distance < 1.0:
        cv2.circle(image, start, 0, color, _positive_int(thickness), cv2.LINE_8)
        return

    dash = _positive_int(dash_length, 6)
    gap = _positive_int(gap_length, 4)
    position = 0.0
    while position < distance:
        dash_end = min(distance, position + dash)
        first = (
            int(round(start[0] + delta_x * position / distance)),
            int(round(start[1] + delta_y * position / distance)),
        )
        second = (
            int(round(start[0] + delta_x * dash_end / distance)),
            int(round(start[1] + delta_y * dash_end / distance)),
        )
        cv2.line(
            image,
            first,
            second,
            color,
            _positive_int(thickness),
            cv2.LINE_8,
        )
        position += dash + gap


def draw_solid_circle(image, center, radius, color, thickness=2):
    """Draw a clipped solid circle outline and return ``image``."""
    size = _canvas_size(image)
    center = _point(center)
    try:
        radius = int(round(float(radius)))
    except (TypeError, ValueError):
        return image
    if size is None or center is None or radius <= 0:
        return image
    if (
        center[0] + radius < 0
        or center[1] + radius < 0
        or center[0] - radius >= size[0]
        or center[1] - radius >= size[1]
    ):
        return image
    cv2.circle(
        image,
        center,
        radius,
        color,
        _positive_int(thickness),
        cv2.LINE_8,
    )
    return image


def draw_dashed_circle(
    image,
    center,
    radius,
    color,
    thickness=2,
    dash_length=8,
    gap_length=6,
):
    """Draw a dashed circle outline, using pixel lengths for dashes and gaps."""
    size = _canvas_size(image)
    center = _point(center)
    try:
        radius = int(round(float(radius)))
    except (TypeError, ValueError):
        return image
    if size is None or center is None or radius <= 0:
        return image
    if (
        center[0] + radius < 0
        or center[1] + radius < 0
        or center[0] - radius >= size[0]
        or center[1] - radius >= size[1]
    ):
        return image

    circumference = 2.0 * math.pi * radius
    dash = _positive_int(dash_length, 8)
    gap = _positive_int(gap_length, 6)
    dash_angle = min(359.0, 360.0 * dash / circumference)
    step_angle = 360.0 * (dash + gap) / circumference
    angle = 0.0
    while angle < 360.0:
        cv2.ellipse(
            image,
            center,
            (radius, radius),
            0.0,
            angle,
            min(360.0, angle + dash_angle),
            color,
            _positive_int(thickness),
            cv2.LINE_8,
        )
        angle += step_angle
    return image


def _polygon_segments(points, closed):
    if len(points) < 2:
        return []
    segments = list(zip(points, points[1:]))
    if closed and len(points) > 2:
        segments.append((points[-1], points[0]))
    return segments


def draw_solid_polygon(image, points, color, thickness=2, closed=True):
    """Draw a clipped solid polygon/polyline outline and return ``image``."""
    points = _points(points)
    if _canvas_size(image) is None or not points:
        return image
    if len(points) == 1:
        _clipped_line(image, points[0], points[0], color, thickness)
        return image
    for start, end in _polygon_segments(points, closed):
        _clipped_line(image, start, end, color, thickness)
    return image


def draw_dashed_polygon(
    image,
    points,
    color,
    thickness=2,
    dash_length=8,
    gap_length=6,
    closed=True,
):
    """Draw a clipped dashed polygon/polyline outline and return ``image``."""
    points = _points(points)
    if _canvas_size(image) is None or not points:
        return image
    if len(points) == 1:
        _clipped_line(image, points[0], points[0], color, thickness)
        return image
    for start, end in _polygon_segments(points, closed):
        _dashed_line(
            image,
            start,
            end,
            color,
            thickness,
            dash_length,
            gap_length,
        )
    return image


def draw_solid_triangle(image, points, color, thickness=2):
    """Draw a solid three-point outline; invalid triangles are ignored."""
    triangle = _points(points)
    if len(triangle) != 3:
        return image
    return draw_solid_polygon(image, triangle, color, thickness, closed=True)


def draw_dashed_triangle(
    image,
    points,
    color,
    thickness=2,
    dash_length=8,
    gap_length=6,
):
    """Draw a dashed three-point outline; invalid triangles are ignored."""
    triangle = _points(points)
    if len(triangle) != 3:
        return image
    return draw_dashed_polygon(
        image,
        triangle,
        color,
        thickness,
        dash_length,
        gap_length,
        closed=True,
    )


def draw_motion_arrow(
    image,
    start,
    end,
    color,
    thickness=2,
    tip_length=0.25,
):
    """Draw a clipped arrow from the tracked position toward its prediction."""
    size = _canvas_size(image)
    start, end = _point(start), _point(end)
    if size is None or start is None or end is None or start == end:
        return image
    visible, start, end = cv2.clipLine((0, 0, size[0], size[1]), start, end)
    if not visible or start == end:
        return image
    try:
        tip_length = float(tip_length)
    except (TypeError, ValueError):
        tip_length = 0.25
    tip_length = min(0.5, max(0.05, tip_length))
    cv2.arrowedLine(
        image,
        start,
        end,
        color,
        _positive_int(thickness),
        cv2.LINE_8,
        tipLength=tip_length,
    )
    return image


def draw_horizon_line(
    image,
    horizon_y,
    color,
    roll_degrees=0.0,
    uncertain=False,
    thickness=2,
    dash_length=12,
    gap_length=8,
):
    """Draw the horizon, sloped down to the right for positive camera roll."""
    size = _canvas_size(image)
    if size is None:
        return image
    try:
        horizon_y = float(horizon_y)
        roll_degrees = float(roll_degrees)
    except (TypeError, ValueError):
        return image
    if not math.isfinite(horizon_y) or not math.isfinite(roll_degrees):
        return image

    # Avoid an unbounded slope when an IMU briefly reports near-vertical roll.
    roll_degrees = min(80.0, max(-80.0, roll_degrees))
    half_width = (size[0] - 1) / 2.0
    vertical_offset = math.tan(math.radians(roll_degrees)) * half_width
    start = (0, horizon_y - vertical_offset)
    end = (size[0] - 1, horizon_y + vertical_offset)
    if uncertain:
        _dashed_line(
            image,
            start,
            end,
            color,
            thickness,
            dash_length,
            gap_length,
        )
    else:
        _clipped_line(image, start, end, color, thickness)
    return image
