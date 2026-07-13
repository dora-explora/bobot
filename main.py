"""Primary robot runtime entrypoint.

`detector.py` remains a compatibility launcher, but all new runtime work belongs
here and in the state-specific modules under `robot/`.
"""
import time

import cv2
import numpy as np

from robot import config
from robot.actuators import Pca9685Actuators
from robot.ball_detector import BallDetector
from robot.bucket_detection import BucketDetection
from robot.camera import open_camera
from robot.cone_slalom import ConeSlalom
from robot.controller import ControllerInput
from robot.dashboard import TuiDashboard
from robot.drive import DriveStabilizer, TargetStabilizer, ball_seeking_command
from robot.hill_climb import HillClimb
from robot.models import DriveCommand, StateResult
from robot.rough_section import RoughSection
from robot.vision_common import boxes_nearly_duplicate


class DetectorState:
    """Trial state: detect balls and generate constrained ball-seeking drive."""

    name = "detector"

    def __init__(self):
        self.balls = BallDetector()
        self.cones = ConeSlalom()
        self.targets = TargetStabilizer()
        self.drive = DriveStabilizer()
        self.last_target_seen = 0.0

    def process(self, frame, now):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        targets, debug, auto_colors = self.balls.detect(hsv, now)
        cones = self.cones.detect(hsv)
        targets = self._remove_cones_from_ball_targets(targets, cones, debug)
        best_target = self.targets.select(targets, frame.shape[1], frame.shape[0], now, debug)
        if best_target is not None:
            self.last_target_seen = now
        command = ball_seeking_command(best_target, frame.shape[1], frame.shape[0] * frame.shape[1], self.last_target_seen, now)
        command = self.drive.smooth(command, now, debug)
        debug.cones = len(cones)
        return StateResult(targets=targets, cones=cones, best_target=best_target, command=command, debug=debug,
                           auto_color_names=[color["name"] for color in auto_colors], state_lines=self.cones.status_lines(cones))

    @staticmethod
    def _remove_cones_from_ball_targets(targets, cones, debug):
        kept = [target for target in targets if not any(boxes_nearly_duplicate(target.box, cone.box) for cone in cones)]
        debug.rejected_overlap += len(targets) - len(kept)
        debug.accepted = len(kept)
        return kept


class ManualState:
    """Direct left/right tank drive using the controller's vertical stick axes."""

    name = "manual"

    def __init__(self, controller):
        self.controller = controller

    def process(self, _frame, _now):
        left, right = self.controller.tank_sides()
        return StateResult(
            command=DriveCommand(
                steering=max(-1.0, min(1.0, (left - right) / 2.0)),
                throttle=max(-1.0, min(1.0, (left + right) / 2.0)),
                mode="manual",
                reason="controller tank drive",
                left=left,
                right=right,
            ),
            state_lines=self.controller.debug_lines(),
        )


# Sections are imported above and kept as independent modules. Detector and
# manual are the only currently runnable states.
COURSE_SECTIONS = {
    "detector": DetectorState,
    "manual": ManualState,
    "bucket": BucketDetection,
    "cone_slalom": ConeSlalom,
    "rough_section": RoughSection,
    "hill_climb": HillClimb,
}
ACTIVE_STATE = "detector"


def draw_overlay(frame, result):
    center_x = frame.shape[1] // 2
    cv2.line(frame, (center_x, 0), (center_x, frame.shape[0]), (255, 255, 255), 1)
    for target in result.targets:
        x, y, width, height = target.box
        thickness = 4 if target == result.best_target else 2
        cv2.rectangle(frame, (x, y), (x + width, y + height), target.color, thickness)
        cv2.circle(frame, (target.center_x, target.center_y), 5, target.color, -1)
    for cone in result.cones:
        x, y, width, height = cone.box
        triangle = np.array([[cone.center_x, y], [x, y + height], [x + width, y + height]], dtype=np.int32)
        cv2.drawContours(frame, [cone.contour], -1, cone.color, 2)
        cv2.polylines(frame, [triangle], True, cone.color, 2)
        cv2.drawMarker(frame, (cone.center_x, cone.center_y), cone.color, markerType=cv2.MARKER_TILTED_CROSS, markerSize=12, thickness=2)


def print_telemetry(result):
    target = result.best_target
    debug, command = result.debug, result.command
    summary = "target=none" if target is None else "target=" + target.label + " x=" + str(target.center_x) + " y=" + str(target.center_y) + " area=" + str(int(target.area))
    print("Telemetry:", summary, "cones=" + str(debug.cones), "steering=" + str(round(command.steering, 3)),
          "throttle=" + str(round(command.throttle, 3)), "reason=" + command.reason,
          "left=" + str(None if command.left is None else round(command.left, 3)),
          "right=" + str(None if command.right is None else round(command.right, 3)),
          "stable=" + debug.stable_target_label, "priority=" + str(round(debug.priority_score, 3)))


def run():
    controller = ControllerInput()
    states = {
        "detector": DetectorState(),
        "manual": ManualState(controller),
    }
    active_state = ACTIVE_STATE
    dashboard = TuiDashboard()
    camera = None
    actuators = None
    last_frame_time = 0.0
    last_telemetry = 0.0
    fps = 0.0
    try:
        camera = open_camera(config.FRAME_WIDTH, config.FRAME_HEIGHT)
        actuators = Pca9685Actuators()
        if not config.HEADLESS:
            cv2.namedWindow("Robot Detector")
            print("Visualization window enabled. It accepts no runtime controls.")
        while True:
            ok, frame = camera.read()
            if not ok:
                print("Camera frame read failed; neutralizing outputs.")
                break
            now = time.time()
            if last_frame_time:
                fps = 1.0 / max(.001, now - last_frame_time)
            last_frame_time = now
            controller_update = controller.poll(active_state == "manual")
            if active_state == "detector" and controller_update.start_manual:
                active_state = "manual"
                actuators.neutralize()
                print("Controller A pressed. Entered manual tank-drive mode.")
            elif active_state == "manual" and controller_update.abort_manual:
                actuators.neutralize()
                print("Manual input aborted: " + controller_update.abort_reason + ". Outputs neutralized; exiting.")
                break

            state = states[active_state]
            result = state.process(frame, now)
            actuators.apply(result.command)
            dashboard.draw(frame, active_state, result, actuators, now, fps)
            if not dashboard.enabled and now - last_telemetry >= config.TELEMETRY_INTERVAL:
                print_telemetry(result)
                last_telemetry = now
            if not config.HEADLESS:
                draw_overlay(frame, result)
                cv2.imshow("Robot Detector", frame)
                cv2.waitKey(1)  # Required only to keep the visualization responsive; input is ignored.
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Neutralizing outputs and exiting.")
    finally:
        dashboard.close()
        if actuators is not None:
            actuators.close()
        if camera is not None:
            camera.release()
        controller.close()
        if not config.HEADLESS:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
