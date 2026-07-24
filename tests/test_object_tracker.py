import unittest

import numpy as np

from robot import config
from robot.imu import IMUSnapshot
from robot.models import DetectionDebug, ObjectDetection
from robot.object_tracker import ObjectTracker


def ball(x, y, hue=60.0):
    return ObjectDetection(
        kind="ball",
        label="ball",
        center_x=x,
        center_y=y,
        area=1800.0,
        confidence=0.82,
        box=(x - 20, y - 20, 40, 40),
        radius=20,
        contour=np.empty((0, 1, 2), dtype=np.int32),
        color=(64, 230, 64),
        ball_score=0.82,
        cone_score=0.12,
        color_score=0.8,
        hue=hue,
        certain=True,
    )


def attitude(yaw, timestamp):
    return IMUSnapshot(
        connected=True,
        error="",
        timestamp=timestamp,
        roll_degrees=0.0,
        pitch_degrees=0.0,
        yaw_degrees=yaw,
        roll_delta_degrees=0.0,
        pitch_delta_degrees=0.0,
        yaw_delta_degrees=yaw,
    )


class ObjectTrackerTests(unittest.TestCase):
    def test_track_must_persist_before_becoming_certain(self):
        tracker = ObjectTracker()
        outputs = []
        for frame_number in range(config.TRACK_CONFIRM_FRAMES):
            outputs = tracker.update(
                [ball(320 + frame_number, 300)],
                None,
                640,
                480,
                1.0 + frame_number / 30.0,
                DetectionDebug(),
            )

        self.assertEqual(len(outputs), 1)
        self.assertEqual(outputs[0].track_id, 1)
        self.assertEqual(outputs[0].track_hits, config.TRACK_CONFIRM_FRAMES)
        self.assertTrue(outputs[0].certain)

    def test_yaw_motion_compensates_large_horizontal_image_shift(self):
        tracker = ObjectTracker()
        first = tracker.update(
            [ball(320, 300)],
            attitude(0.0, 1.0),
            640,
            480,
            1.0,
            DetectionDebug(),
        )[0]
        debug = DetectionDebug()
        second = tracker.update(
            [ball(204, 300)],
            attitude(12.0, 1.1),
            640,
            480,
            1.1,
            debug,
        )[0]

        self.assertEqual(second.track_id, first.track_id)
        self.assertAlmostEqual(debug.imu_compensation_x, -116.36, delta=1.0)
        self.assertAlmostEqual(debug.imu_compensation_y, 0.0)

    def test_color_separates_nearby_crossing_tracks(self):
        tracker = ObjectTracker()
        first = tracker.update(
            [ball(280, 300, 20.0), ball(360, 300, 110.0)],
            None,
            640,
            480,
            1.0,
            DetectionDebug(),
        )
        first_by_hue = {round(item.hue): item.track_id for item in first}
        second = tracker.update(
            [ball(325, 300, 20.0), ball(315, 300, 110.0)],
            None,
            640,
            480,
            1.03,
            DetectionDebug(),
        )
        second_by_hue = {round(item.hue): item.track_id for item in second}

        self.assertEqual(second_by_hue[20], first_by_hue[20])
        self.assertEqual(second_by_hue[110], first_by_hue[110])

    def test_missing_detection_is_rendered_as_uncertain_prediction(self):
        tracker = ObjectTracker()
        first = tracker.update(
            [ball(320, 300)],
            None,
            640,
            480,
            1.0,
            DetectionDebug(),
        )[0]
        debug = DetectionDebug()

        predicted = tracker.update(
            [],
            None,
            640,
            480,
            1.03,
            debug,
        )

        self.assertEqual(len(predicted), 1)
        self.assertEqual(predicted[0].track_id, first.track_id)
        self.assertTrue(predicted[0].predicted)
        self.assertFalse(predicted[0].certain)
        self.assertEqual(debug.predicted_count, 1)

    def test_async_prediction_does_not_consume_miss_allowance(self):
        tracker = ObjectTracker()
        tracker.update(
            [ball(320, 300)],
            attitude(0.0, 1.0),
            640,
            480,
            1.0,
            DetectionDebug(),
        )

        predicted = tracker.predict_only(
            attitude(2.0, 1.1),
            640,
            480,
            1.1,
            DetectionDebug(),
        )

        self.assertEqual(len(predicted), 1)
        self.assertTrue(predicted[0].predicted)
        self.assertEqual(tracker.tracks[0].misses, 0)

    def test_delayed_detection_is_compensated_to_current_attitude(self):
        detections, motion = ObjectTracker.compensate_detections(
            [ball(320, 300)],
            attitude(0.0, 1.0),
            attitude(12.0, 1.2),
            640,
            480,
        )

        self.assertAlmostEqual(motion[0], -116.36, delta=1.0)
        self.assertEqual(detections[0].center_x, 204)
        self.assertEqual(detections[0].box[0], 184)


if __name__ == "__main__":
    unittest.main()
