"""Primary robot runtime entrypoint.

`detector.py` remains a compatibility launcher, but all new runtime work belongs
here and in the state-specific modules under `robot/`.
"""
import time
import sys
import traceback

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
from robot.mode_control import ModeControl
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
        left_input, right_input = self.controller.tank_sides()
        left = left_input * config.THROTTLE_LIMIT
        right = right_input * config.THROTTLE_LIMIT
        return StateResult(
            command=DriveCommand(
                steering=max(-1.0, min(1.0, (left - right) / 2.0)),
                throttle=max(-1.0, min(1.0, (left + right) / 2.0)),
                mode="manual",
                reason="controller tank drive",
                left=left,
                right=right,
            ),
            state_lines=self.controller.debug_lines() + [
                "scaled motor input left=" + str(round(left, 3))
                + " right=" + str(round(right, 3))
                + " limit=" + str(config.THROTTLE_LIMIT),
            ],
        )


class StaticState:
    """Default safety state: camera and TUI stay active, motor output is neutral."""

    name = "static"

    def __init__(self, controller):
        self.controller = controller

    def process(self, _frame, _now):
        return StateResult(
            command=DriveCommand(mode="static", reason="static mode"),
            state_lines=self.controller.debug_lines() + [
                "motors neutral; A=manual Y=radial menu B=remain static",
            ],
        )


# Sections are imported above and kept as independent modules. Static is the
# default safety state; the controller mode coordinator gates physical output.
COURSE_SECTIONS = {
    "static": StaticState,
    "detector": DetectorState,
    "manual": ManualState,
    "bucket": BucketDetection,
    "cone_slalom": ConeSlalom,
    "rough_section": RoughSection,
    "hill_climb": HillClimb,
}
ACTIVE_STATE = config.ROBOT_START_STATE


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


def print_telemetry(result, mode_control, output_command):
    target = result.best_target
    debug, command = result.debug, result.command
    summary = "target=none" if target is None else "target=" + target.label + " x=" + str(target.center_x) + " y=" + str(target.center_y) + " area=" + str(int(target.area))
    print("Telemetry:", summary, "cones=" + str(debug.cones), "steering=" + str(round(command.steering, 3)),
          "throttle=" + str(round(command.throttle, 3)), "reason=" + command.reason,
          "state=" + mode_control.active_state, "menu=" + str(mode_control.menu_active),
          "output_mode=" + output_command.mode, "output_reason=" + output_command.reason,
          "left=" + str(None if command.left is None else round(command.left, 3)),
          "right=" + str(None if command.right is None else round(command.right, 3)),
          "stable=" + debug.stable_target_label, "priority=" + str(round(debug.priority_score, 3)))


def report_fatal_error(error):
    report = (
        "FATAL ROBOT RUNTIME ERROR\n"
        + time.strftime("%Y-%m-%d %H:%M:%S %z")
        + "\n"
        + "".join(traceback.format_exception(type(error), error, error.__traceback__))
    )
    print(report, file=sys.stderr, flush=True)
    try:
        with open(config.FATAL_ERROR_LOG, "a", encoding="utf-8") as log_file:
            log_file.write(report + "\n")
    except OSError as log_error:
        print("Could not write fatal error log: " + str(log_error), file=sys.stderr, flush=True)


def safe_shutdown(actuators, camera, controller, dashboard):
    """Neutralize first; no cleanup error may prevent the remaining cleanup."""
    if actuators is not None:
        try:
            actuators.neutralize()
        except BaseException as error:
            print("Emergency neutralize failed: " + repr(error), file=sys.stderr, flush=True)
        try:
            actuators.close()
        except BaseException as error:
            print("Actuator watchdog shutdown failed: " + repr(error), file=sys.stderr, flush=True)
    if controller is not None:
        try:
            controller.close()
        except BaseException as error:
            print("Controller shutdown failed: " + repr(error), file=sys.stderr, flush=True)
    if camera is not None:
        try:
            camera.release()
        except BaseException as error:
            print("Camera shutdown failed: " + repr(error), file=sys.stderr, flush=True)
    try:
        dashboard.close()
    except BaseException as error:
        print("TUI shutdown failed: " + repr(error), file=sys.stderr, flush=True)
    if not config.HEADLESS:
        try:
            cv2.destroyAllWindows()
        except BaseException as error:
            print("Display shutdown failed: " + repr(error), file=sys.stderr, flush=True)


def run():
    controller = ControllerInput()
    states = {
        "static": StaticState(controller),
        "detector": DetectorState(),
        "manual": ManualState(controller),
    }
    mode_control = ModeControl(ACTIVE_STATE)
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
                raise RuntimeError("Camera frame read failed")
            now = time.time()
            if last_frame_time:
                fps = 1.0 / max(.001, now - last_frame_time)
            last_frame_time = now
            controller_update = controller.poll()
            decision = mode_control.update(controller_update, controller.right_stick())
            if decision.message and not dashboard.enabled:
                print("Controller: " + decision.message)

            state = states[mode_control.active_state]
            result = state.process(frame, now)
            output_command = mode_control.gate_command(result.command, decision.neutralize_this_frame)
            actuators.apply(output_command)
            dashboard.draw(
                frame,
                mode_control,
                result,
                output_command,
                actuators,
                controller.debug_lines(),
                now,
                fps,
            )
            if not dashboard.enabled and now - last_telemetry >= config.TELEMETRY_INTERVAL:
                print_telemetry(result, mode_control, output_command)
                last_telemetry = now
            if not config.HEADLESS:
                draw_overlay(frame, result)
                cv2.imshow("Robot Detector", frame)
                cv2.waitKey(1)  # Required only to keep the visualization responsive; input is ignored.
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Neutralizing outputs and exiting.")
    except BaseException as error:
        report_fatal_error(error)
    finally:
        safe_shutdown(actuators, camera, controller, dashboard)


if __name__ == "__main__":
    run()
