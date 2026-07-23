import unittest

import numpy as np

from robot.overlay import (
    draw_dashed_circle,
    draw_dashed_polygon,
    draw_dashed_triangle,
    draw_horizon_line,
    draw_motion_arrow,
    draw_solid_circle,
    draw_solid_polygon,
    draw_solid_triangle,
)


COLOR = (40, 180, 250)


def blank(width=120, height=100):
    return np.zeros((height, width, 3), dtype=np.uint8)


def changed_pixels(image):
    return int(np.count_nonzero(np.any(image != 0, axis=2)))


class OverlayTests(unittest.TestCase):
    def test_solid_and_dashed_circles_draw_with_dashed_gaps(self):
        solid = blank()
        dashed = blank()

        draw_solid_circle(solid, (60, 50), 28, COLOR, thickness=1)
        draw_dashed_circle(
            dashed,
            (60, 50),
            28,
            COLOR,
            thickness=1,
            dash_length=7,
            gap_length=7,
        )

        self.assertGreater(changed_pixels(solid), 0)
        self.assertGreater(changed_pixels(dashed), 0)
        self.assertLess(changed_pixels(dashed), changed_pixels(solid))
        solid_outline = np.any(solid != 0, axis=2)
        dashed_outline = np.any(dashed != 0, axis=2)
        self.assertGreater(np.count_nonzero(solid_outline & ~dashed_outline), 0)

    def test_solid_and_dashed_polygons_draw_with_dashed_gaps(self):
        points = [(-15, 80), (35, 10), (110, 20), (135, 85)]
        solid = blank()
        dashed = blank()

        draw_solid_polygon(solid, points, COLOR, thickness=1)
        draw_dashed_polygon(
            dashed,
            points,
            COLOR,
            thickness=1,
            dash_length=6,
            gap_length=6,
        )

        self.assertGreater(changed_pixels(solid), 0)
        self.assertGreater(changed_pixels(dashed), 0)
        self.assertLess(changed_pixels(dashed), changed_pixels(solid))

    def test_triangle_helpers_draw_expected_outlines(self):
        points = [(60, 8), (20, 88), (100, 88)]
        solid = blank()
        dashed = blank()

        draw_solid_triangle(solid, points, COLOR)
        draw_dashed_triangle(dashed, points, COLOR)

        self.assertGreater(changed_pixels(solid), 0)
        self.assertGreater(changed_pixels(dashed), 0)

    def test_motion_arrow_draws_and_clips_to_image(self):
        image = blank()

        draw_motion_arrow(image, (-40, 50), (80, 50), COLOR)

        self.assertGreater(changed_pixels(image), 0)
        self.assertTrue(np.any(image[:, 0] != 0))

    def test_horizon_can_slope_and_be_dashed_when_uncertain(self):
        solid = blank(width=160, height=120)
        uncertain = blank(width=160, height=120)

        draw_horizon_line(
            solid,
            60,
            COLOR,
            roll_degrees=12,
            uncertain=False,
            thickness=1,
        )
        draw_horizon_line(
            uncertain,
            60,
            COLOR,
            roll_degrees=12,
            uncertain=True,
            thickness=1,
            dash_length=8,
            gap_length=8,
        )

        self.assertGreater(changed_pixels(solid), 0)
        self.assertGreater(changed_pixels(uncertain), 0)
        self.assertLess(changed_pixels(uncertain), changed_pixels(solid))
        left_y = np.flatnonzero(np.any(solid[:, 0] != 0, axis=1))[0]
        right_y = np.flatnonzero(np.any(solid[:, -1] != 0, axis=1))[0]
        self.assertNotEqual(left_y, right_y)

    def test_tiny_invalid_and_offscreen_shapes_are_safe(self):
        image = blank()
        before = image.copy()

        draw_solid_circle(image, (20, 20), 0, COLOR)
        draw_dashed_circle(image, (20, 20), -2, COLOR)
        draw_solid_polygon(image, [], COLOR)
        draw_dashed_polygon(image, [(500, 500), (600, 600)], COLOR)
        draw_solid_triangle(image, [(1, 1), (2, 2)], COLOR)
        draw_dashed_triangle(image, None, COLOR)
        draw_motion_arrow(image, (15, 15), (15, 15), COLOR)
        draw_horizon_line(image, float("nan"), COLOR)

        np.testing.assert_array_equal(image, before)


if __name__ == "__main__":
    unittest.main()
