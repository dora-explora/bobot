"""Primary robot runtime entrypoint.

`detector.py` remains a compatibility launcher, but all new runtime work belongs
here and in the state-specific modules under `robot/`.
"""
import time
import sys
import traceback

import cv2

from robot import config
from robot.actuators import Pca9685Actuators
from robot.bucket_detection import BucketDetection
from robot.camera import open_camera
from robot.cone_slalom import ConeSlalom
from robot.controller import ControllerInput
from robot.dashboard import TuiDashboard
from robot.drive import DriveStabilizer, TargetStabilizer, ball_seeking_command
from robot.hill_climb import HillClimb
from robot.horizon import HorizonEstimator
from robot.imu import BNO085Service
from robot.mode_control import ModeControl
from robot.models import DriveCommand, StateResult
from robot.object_detector import ObjectDetector
from robot.object_tracker import ObjectTracker
from robot.overlay import (
    draw_dashed_circle,
    draw_dashed_polygon,
    draw_dashed_triangle,
    draw_motion_arrow,
    draw_solid_circle,
    draw_solid_polygon,
    draw_solid_triangle,
)
from robot.rough_section import RoughSection


class DetectorState:
    """Trial state: detect balls and generate constrained ball-seeking drive."""

    name = "detector"

    def __init__(self):
        self.objects = ObjectDetector()
        self.tracker = ObjectTracker()
        self.cone_status = ConeSlalom()
        self.targets = TargetStabilizer()
        self.drive = DriveStabilizer()
        self.last_target_seen = 0.0

    def process(self, frame, now, attitude=None, horizon=None):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections, debug = self.objects.detect(frame, hsv, horizon)
        detections = self.tracker.update(
            detections,
            attitude,
            frame.shape[1],
            frame.shape[0],
            now,
            debug,
        )
        targets = [item for item in detections if item.kind == "ball"]
        cones = [item for item in detections if item.kind == "cone"]
        unknowns = [item for item in detections if item.kind == "unknown"]
        confirmed_targets = [target for target in targets if target.certain]
        best_target = self.targets.select(
            confirmed_targets,
            frame.shape[1],
            frame.shape[0],
            now,
            debug,
        )
        if best_target is not None:
            self.last_target_seen = now
        command = ball_seeking_command(best_target, frame.shape[1], frame.shape[0] * frame.shape[1], self.last_target_seen, now)
        command = self.drive.smooth(command, now, debug)
        debug.cones = len(cones)
        return StateResult(
            targets=targets,
            cones=cones,
            unknowns=unknowns,
            best_target=best_target,
            command=command,
            debug=debug,
            state_lines=self.cone_status.status_lines(cones),
            attitude=attitude,
            horizon=horizon,
        )


class ManualState:
    """Direct left/right tank drive using the controller's vertical stick axes."""

    name = "manual"

    def __init__(self, controller):
        self.controller = controller

    def process(self, _frame, _now, attitude=None, horizon=None):
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
            attitude=attitude,
            horizon=horizon,
        )


class StaticState:
    """Default safety state: camera and TUI stay active, motor output is neutral."""

    name = "static"

    def __init__(self, controller):
        self.controller = controller

    def process(self, _frame, _now, attitude=None, horizon=None):
        return StateResult(
            command=DriveCommand(mode="static", reason="static mode"),
            state_lines=self.controller.debug_lines() + [
                "motors neutral; D-pad up/down adjusts limit; A=manual hold Y=radial menu B=remain static",
            ],
            attitude=attitude,
            horizon=horizon,
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
    if result.horizon is not None:
        horizon_points = [
            (0, result.horizon.left_y),
            (frame.shape[1] - 1, result.horizon.right_y),
        ]
        if result.horizon.confident:
            draw_solid_polygon(frame, horizon_points, (255, 255, 0), 2, closed=False)
        else:
            draw_dashed_polygon(frame, horizon_points, (255, 255, 0), 2, closed=False)

    selected_track = getattr(result.best_target, "track_id", 0)
    for detection in result.targets + result.cones + result.unknowns:
        x, y, width, height = detection.box
        selected = selected_track and detection.track_id == selected_track
        thickness = 4 if selected else 2
        if detection.kind == "ball" or (
            detection.kind == "unknown"
            and detection.ball_score >= detection.cone_score
        ):
            draw = draw_solid_circle if detection.certain else draw_dashed_circle
            draw(
                frame,
                (detection.center_x, detection.center_y),
                detection.radius,
                detection.color,
                thickness,
            )
        else:
            triangle = [
                (detection.center_x, y),
                (x, y + height),
                (x + width, y + height),
            ]
            draw = draw_solid_triangle if detection.certain else draw_dashed_triangle
            draw(frame, triangle, detection.color, thickness)

        arrow_end = (
            int(round(detection.center_x + detection.motion_x * config.OVERLAY_MOTION_SCALE)),
            int(round(detection.center_y + detection.motion_y * config.OVERLAY_MOTION_SCALE)),
        )
        draw_motion_arrow(
            frame,
            (detection.center_x, detection.center_y),
            arrow_end,
            detection.color,
            thickness=1,
        )


def print_telemetry(result, mode_control, output_command):
    target = result.best_target
    debug, command = result.debug, result.command
    attitude = result.attitude
    summary = "target=none" if target is None else "target=" + target.label + " x=" + str(target.center_x) + " y=" + str(target.center_y) + " area=" + str(int(target.area))
    print("Telemetry:", summary, "cones=" + str(debug.cones), "steering=" + str(round(command.steering, 3)),
          "throttle=" + str(round(command.throttle, 3)), "reason=" + command.reason,
          "state=" + mode_control.active_state, "menu=" + str(mode_control.menu_active),
          "output_mode=" + output_command.mode, "output_reason=" + output_command.reason,
          "left=" + str(None if command.left is None else round(command.left, 3)),
          "right=" + str(None if command.right is None else round(command.right, 3)),
          "throttle_limit=" + str(round(config.THROTTLE_LIMIT, 3)),
          "stable=" + debug.stable_target_label, "priority=" + str(round(debug.priority_score, 3)),
          "imu=" + ("connected" if attitude is not None and attitude.connected else "offline"),
          "roll=" + _telemetry_angle(attitude, "roll_delta_degrees"),
          "pitch=" + _telemetry_angle(attitude, "pitch_delta_degrees"),
          "yaw=" + _telemetry_angle(attitude, "yaw_delta_degrees"))


def _telemetry_angle(attitude, attribute):
    value = None if attitude is None else getattr(attitude, attribute, None)
    return "n/a" if value is None else str(round(value, 2))


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


def safe_shutdown(actuators, camera, controller, dashboard, imu):
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
    if imu is not None:
        try:
            imu.close()
        except BaseException as error:
            print("IMU shutdown failed: " + repr(error), file=sys.stderr, flush=True)
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
    imu = BNO085Service(
        address=config.IMU_I2C_ADDRESS,
        report_interval_us=config.IMU_REPORT_INTERVAL_US,
        rotation_mode=config.IMU_ROTATION_MODE,
        clock=time.time,
        auto_connect=config.IMU_ENABLED,
    )
    horizon_estimator = HorizonEstimator()
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
    last_imu_connect_attempt = time.time()
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
            attitude = imu.read()
            imu_needs_reconnect = (
                not attitude.connected
                or (
                    attitude.roll_degrees is None
                    and "orientation read failed" in attitude.error
                )
            )
            if (
                config.IMU_ENABLED
                and imu_needs_reconnect
                and now - last_imu_connect_attempt >= config.IMU_RECONNECT_INTERVAL
            ):
                imu.connect()
                last_imu_connect_attempt = now
                attitude = imu.read()
            horizon = horizon_estimator.estimate(
                frame.shape[1],
                frame.shape[0],
                attitude,
                now,
            )
            controller_update = controller.poll()
            if controller_update.throttle_limit_delta:
                previous_limit = config.THROTTLE_LIMIT
                current_limit = config.adjust_throttle_limit(controller_update.throttle_limit_delta)
                if current_limit != previous_limit:
                    direction = "increased" if current_limit > previous_limit else "decreased"
                    mode_control.last_action = (
                        "D-pad " + direction + " throttle limit to "
                        + str(round(current_limit * 100.0)) + "%"
                    )
            menu_stick, menu_stick_source = controller.menu_stick()
            decision = mode_control.update(controller_update, menu_stick, menu_stick_source)
            if decision.message and not dashboard.enabled:
                print("Controller: " + decision.message)

            state = states[mode_control.active_state]
            result = state.process(frame, now, attitude, horizon)
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
        safe_shutdown(actuators, camera, controller, dashboard, imu)


if __name__ == "__main__":
    run()
