import sys

from robot import config
from robot.vision_common import clamp


class TuiDashboard:
    def __init__(self):
        self.enabled = config.TUI_ENABLED and sys.stdout.isatty()
        self.last_draw = 0.0
        self.screen = None
        if not self.enabled:
            return
        try:
            import curses
            self.curses = curses
            self.screen = curses.initscr()
            curses.noecho()
            curses.cbreak()
            self.screen.nodelay(True)
            self.screen.keypad(False)
            try:
                curses.curs_set(0)
            except curses.error:
                pass
        except ImportError:
            self.enabled = False
            print("TUI disabled because curses is unavailable.")

    def close(self):
        if not self.enabled:
            return
        self.curses.nocbreak()
        self.curses.echo()
        self.curses.endwin()
        self.enabled = False

    def draw(self, frame, mode_control, result, output_command, actuators, controller_lines, now, fps):
        if not self.enabled or now - self.last_draw < config.TUI_INTERVAL:
            return
        self.last_draw = now
        height, width = self.screen.getmaxyx()
        lines = self._lines(
            frame,
            mode_control,
            result,
            output_command,
            actuators,
            controller_lines,
            fps,
            width,
            height,
        )
        self.screen.erase()
        for index, line in enumerate(lines[:height - 1]):
            try:
                self.screen.addstr(index, 0, line[:max(0, width - 1)])
            except self.curses.error:
                pass
        self.screen.refresh()

    def _lines(
        self,
        frame,
        mode_control,
        result,
        output_command,
        actuators,
        controller_lines,
        fps,
        width,
        height=None,
    ):
        state_name = mode_control.active_state
        target, debug, command = result.best_target, result.debug, result.command
        compact = height is not None and height < 55
        if compact:
            lines = [
                "Robot Code TUI",
                "[State] active=" + state_name + " menu=" + str(mode_control.menu_active)
                + " available=static,detector,manual,capture",
                "controls A=manual B=static/cancel hold-Y=menu X=capture D-pad=limit",
                "last_action=" + mode_control.last_action,
                "[Status] camera=" + config.CAMERA_BACKEND
                + " frame=" + str(frame.shape[1]) + "x" + str(frame.shape[0])
                + " fps=" + str(round(fps, 1))
                + " headless=" + str(config.HEADLESS),
                "actuators=" + str(config.ENABLE_ACTUATORS)
                + " limit=" + str(round(config.THROTTLE_LIMIT * 100.0)) + "%"
                + " gate=" + mode_control.output_status
                + " ctrl-c=neutralize/exit",
                "",
            ]
        else:
            lines = [
                "Robot Code TUI", "==============", "",
                "[State]", "active=" + state_name + "  menu=" + str(mode_control.menu_active)
                + "  available=static,detector,manual,capture",
                "controls: A=manual  B=static/cancel  hold Y=radial menu  X=capture  D-pad=limit",
                "last_action=" + mode_control.last_action, "",
                "[Status]", "camera=" + config.CAMERA_BACKEND + " frame=" + str(frame.shape[1]) + "x" + str(frame.shape[0])
                + " fps=" + str(round(fps, 1)) + " headless=" + str(config.HEADLESS),
                "actuators=" + str(config.ENABLE_ACTUATORS)
                + " throttle_limit=" + str(round(config.THROTTLE_LIMIT * 100.0)) + "%"
                + " step=" + str(round(config.THROTTLE_LIMIT_STEP * 100.0)) + "%"
                + " ctrl-c=neutralize and exit",
                "output_gate=" + mode_control.output_status, "",
            ]

        if mode_control.menu_active:
            lines.extend(["[Radial Menu]"])
            lines.extend(self._radial_menu(mode_control.menu_selection, width))
            lines.extend([
                "menu stick=" + mode_control.menu_stick_source
                + " x=" + str(round(mode_control.menu_stick[0], 3))
                + " y=" + str(round(mode_control.menu_stick[1], 3)),
                "Release Y to select; B closes and returns to the paused state. Motor output remains neutral.",
                "",
                "[Controller Debug]",
            ])
            lines.extend(controller_lines)
            lines.extend([
                "",
                "[Drive]",
                "planned mode=" + command.mode + " steering=" + str(round(command.steering, 3))
                + " throttle=" + str(round(command.throttle, 3)),
                "output mode=" + output_command.mode + " reason=" + output_command.reason,
                self._motors(actuators),
            ])
            return lines

        if state_name == "manual":
            lines.extend(["[Manual Controller]", "Left/right vertical sticks drive each side. B=static Y=menu."])
            lines.extend(result.state_lines)
        elif state_name == "static":
            lines.extend(["[Static]", "Motor output is neutral by design. A=manual Y=menu."])
            lines.extend(result.state_lines)
        elif state_name == "capture":
            lines.extend(["[Capture]", "Motor output is neutral by design."])
            lines.extend(result.state_lines)
        else:
            lines.extend([
                "[Detector]",
                "throttle=" + ("ENABLED" if mode_control.detector_throttle_enabled else "DISABLED")
                + ("; A=manual" if mode_control.detector_throttle_enabled else "; A=enable detector throttle"),
            ])
            if target is None:
                lines.extend(["selected=none", "stable=" + debug.stable_target_label + " held=" + str(debug.stable_target_held)
                              + " raw_targets=" + str(debug.raw_target_count)])
            else:
                lines.extend(["selected=" + target.label + " x=" + str(target.center_x) + " y=" + str(target.center_y)
                              + " radius=" + str(target.radius)
                              + " track=" + str(target.track_id) + " hits=" + str(target.track_hits),
                              "area=" + str(int(target.area)) + " confidence=" + str(round(target.confidence, 2))
                              + " certain=" + str(target.certain)
                              + " ball_score=" + str(round(target.ball_score, 2))
                              + " cone_score=" + str(round(target.cone_score, 2)),
                              "priority score=" + str(round(debug.priority_score, 3)) + " distance=" + str(round(debug.priority_distance, 3))
                              + " cluster=" + str(round(debug.priority_cluster, 3)) + " neighbors=" + str(debug.priority_neighbors),
                              "stable=" + debug.stable_target_label + " age=" + str(round(debug.stable_target_age, 2))
                              + " switch_frames=" + str(debug.switch_candidate_frames)])
            lines.extend(self._vision_lines(result, compact))
        lines.extend(["", "[Drive]", "planned mode=" + command.mode + " steering=" + str(round(command.steering, 3))
                      + " throttle=" + str(round(command.throttle, 3)), "reason=" + command.reason,
                      "output mode=" + output_command.mode + " reason=" + output_command.reason,
                      "raw=" + str(round(debug.raw_steering, 3)) + " smoothed=" + str(round(debug.smoothed_steering, 3))
                      + " slew_limited=" + str(debug.steering_limited), self._steering_bar(command.steering, width),
                      self._motors(actuators)])
        lines.extend([""])
        lines.extend(self._imu_lines(result.attitude, result.horizon, width, compact))
        if state_name == "detector":
            lines.extend(["", "[Cone Slalom]"])
            lines.extend(result.state_lines)
        if state_name in ("manual", "static", "capture"):
            return lines
        if not compact:
            lines.extend(["", "[Tuning]", "ball area min=" + str(config.MIN_BALL_AREA_RATIO) + " top_scale=" + str(config.MIN_BALL_AREA_TOP_SCALE)
                          + " max=" + str(config.MAX_BALL_AREA_RATIO) + " max_top_scale=" + str(config.MAX_BALL_AREA_TOP_SCALE),
                          "object score uncertain=" + str(config.OBJECT_UNCERTAIN_SCORE)
                          + " certain=" + str(config.OBJECT_CERTAIN_SCORE)
                          + " margin=" + str(config.OBJECT_CLASS_MARGIN)
                          + "/" + str(config.OBJECT_CERTAIN_MARGIN),
                          "cone orange hue=" + str(config.CONE_HUE_MIN) + "-" + str(config.CONE_HUE_MAX)
                          + " sat>=" + str(config.CONE_SATURATION_MIN)
                          + " value=" + str(config.CONE_VALUE_MIN) + "-" + str(config.CONE_VALUE_MAX)])
        return lines

    @classmethod
    def _imu_lines(cls, attitude, horizon, terminal_width, compact=False):
        connected = attitude is not None and attitude.connected
        error = "" if attitude is None else attitude.error
        compact_horizon = (
            ""
            if horizon is None or not compact
            else " horizon=" + str(horizon.left_y)
            + "/" + str(horizon.center_y)
            + "/" + str(horizon.right_y)
        )
        lines = [
            "[IMU]",
            "status=" + ("connected" if connected else "offline")
            + " address=" + hex(config.IMU_I2C_ADDRESS)
            + " rotation=" + config.IMU_ROTATION_MODE
            + compact_horizon
            + (" error=" + error if error else ""),
        ]
        roll = cls._value(attitude, "roll_delta_degrees")
        pitch = cls._value(attitude, "pitch_delta_degrees")
        yaw = cls._value(attitude, "yaw_delta_degrees")
        lines.extend([
            "relative roll=" + cls._format_angle(roll)
            + " pitch=" + cls._format_angle(pitch)
            + " yaw=" + cls._format_angle(yaw),
            "tilt: x=roll y=pitch",
        ])
        lines.extend(cls._tilt_indicator(roll, pitch, height=5 if compact else 7))
        lines.append(cls._yaw_indicator(yaw, terminal_width))
        if compact:
            lines.append(
                cls._vector_line("accel", None if attitude is None else attitude.acceleration)
                + " " + cls._vector_line("gyro", None if attitude is None else attitude.gyro)
            )
        else:
            lines.extend([
                "absolute roll=" + cls._format_angle(cls._value(attitude, "roll_degrees"))
                + " pitch=" + cls._format_angle(cls._value(attitude, "pitch_degrees"))
                + " yaw=" + cls._format_angle(cls._value(attitude, "yaw_degrees")),
                cls._vector_line("accel m/s^2", None if attitude is None else attitude.acceleration),
                cls._vector_line("gyro rad/s", None if attitude is None else attitude.gyro),
            ])
        if not compact:
            if horizon is None:
                lines.append("horizon=unavailable")
            else:
                lines.append(
                    "horizon=" + horizon.source
                    + " left=" + str(horizon.left_y)
                    + " center=" + str(horizon.center_y)
                    + " right=" + str(horizon.right_y)
                )
        return lines

    @staticmethod
    def _value(value, attribute):
        return None if value is None else getattr(value, attribute, None)

    @staticmethod
    def _format_angle(value):
        return "n/a" if value is None else str(round(value, 2)) + "deg"

    @staticmethod
    def _vector_line(label, values):
        if values is None:
            return label + "=n/a"
        return label + "=" + ",".join(str(round(value, 2)) for value in values)

    @staticmethod
    def _tilt_indicator(roll, pitch, height=7):
        width = 21
        canvas = [[" " for _ in range(width)] for _ in range(height)]
        for x in range(width):
            canvas[0][x] = "-"
            canvas[-1][x] = "-"
        for y in range(height):
            canvas[y][0] = "|"
            canvas[y][-1] = "|"
        canvas[0][0] = canvas[0][-1] = "+"
        canvas[-1][0] = canvas[-1][-1] = "+"
        center_x, center_y = width // 2, height // 2
        canvas[center_y][center_x] = "+"
        if roll is not None and pitch is not None:
            angle_range = max(1.0, config.IMU_TUI_TILT_RANGE_DEG)
            marker_x = int(round(center_x + clamp(roll / angle_range, -1.0, 1.0) * (center_x - 1)))
            marker_y = int(round(center_y - clamp(pitch / angle_range, -1.0, 1.0) * (center_y - 1)))
            canvas[marker_y][marker_x] = "O"
        return ["".join(row) for row in canvas]

    @staticmethod
    def _yaw_indicator(yaw, terminal_width):
        prefix, suffix = "yaw -180 ", " +180"
        bar_width = max(15, terminal_width - len(prefix) - len(suffix) - 1)
        marker = None
        if yaw is not None:
            marker = int(round((clamp(yaw, -180.0, 180.0) + 180.0) / 360.0 * (bar_width - 1)))
        bar = ["-"] * bar_width
        bar[bar_width // 2] = "|"
        if marker is not None:
            bar[marker] = "#"
        return prefix + "".join(bar) + suffix

    @staticmethod
    def _candidate_lines(result, limit=5):
        candidates = sorted(
            result.targets + result.cones + result.unknowns,
            key=lambda item: (item.certain, item.confidence, item.area),
            reverse=True,
        )[:limit]
        if not candidates:
            return ["objects=none"]
        lines = ["objects (top " + str(len(candidates)) + "):"]
        for item in candidates:
            lines.append(
                "#" + str(item.track_id) + " " + item.kind
                + (" solid" if item.certain else " dashed")
                + " xy=" + str(item.center_x) + "," + str(item.center_y)
                + " score=" + str(round(item.confidence, 2))
                + " ball/cone=" + str(round(item.ball_score, 2))
                + "/" + str(round(item.cone_score, 2))
                + " hue=" + str(round(item.hue, 1))
                + (" reason=" + item.rejection_reason if item.rejection_reason else "")
            )
        return lines

    @classmethod
    def _vision_lines(cls, result, compact):
        debug = result.debug
        lines = [
            "",
            "[Vision] backend=" + debug.vision_backend
            + " status=" + debug.vision_status
            + (" error=" + debug.vision_error if debug.vision_error else ""),
            "inference=" + str(round(debug.inference_latency_ms, 1)) + "ms"
            + " rate=" + str(round(debug.inference_fps, 1)) + "Hz"
            + " age=" + str(round(debug.inference_age_seconds, 3)) + "s"
            + " dropped=" + str(debug.inference_dropped_frames)
            + " sequence=" + str(debug.inference_sequence),
            "proposals=" + str(debug.candidate_count)
            + " contours=" + str(debug.contours_seen)
            + " duplicate=" + str(debug.rejected_overlap)
            + " tracked=" + str(debug.tracked_count)
            + " predicted=" + str(debug.predicted_count),
            "balls=" + str(debug.accepted) + " cones=" + str(debug.cones)
            + " unknown=" + str(debug.unknown_count)
            + " certain=" + str(debug.certain_count)
            + " uncertain=" + str(debug.uncertain_count),
            "reject small=" + str(debug.rejected_small)
            + " large=" + str(debug.rejected_large)
            + " implausible_ball=" + str(debug.rejected_shape)
            + " horizon=" + str(debug.rejected_horizon),
            "IMU compensation dx=" + str(round(debug.imu_compensation_x, 1))
            + " dy=" + str(round(debug.imu_compensation_y, 1))
            + " roll=" + str(round(debug.imu_compensation_roll, 2)) + "deg",
        ]
        lines.extend(cls._candidate_lines(result, limit=3 if compact else 5))
        return lines

    @staticmethod
    def _radial_menu(selection, terminal_width):
        # Keep the menu compact and centered instead of spanning the terminal.
        # Terminal cells are usually about twice as tall as they are wide, so a
        # 21-by-9 character canvas reads as approximately square.
        width = min(21, max(17, terminal_width - 1))

        def label(name):
            return "[" + name.upper() + "]" if selection == name else name

        def centered(value):
            return value.center(width)

        detector = label("detector")
        static = label("static")
        manual = label("manual")
        capture = label("capture")
        gap = max(1, width - len(static) - len(manual))
        return [
            centered(detector),
            centered(""),
            centered("|"),
            static + (" " * gap) + manual,
            centered("|"),
            centered(capture),
            centered(""),
            centered(""),
            centered("selection=" + selection),
        ]

    @staticmethod
    def _motors(actuators):
        values, pulses = actuators.last_motor_values, actuators.last_motor_pulses_us
        return "motors " + " ".join(name + "=" + str(round(values.get(name, 0.0), 3)) + "@" + str(pulses.get(name, "-")) for name in ("front_left", "front_right", "rear_left", "rear_right"))

    @staticmethod
    def _steering_bar(steering, terminal_width):
        prefix, suffix = "steering L ", " R"
        bar_width = max(11, terminal_width - len(prefix) - len(suffix) - 1)
        marker = int(round((clamp(steering, -1.0, 1.0) + 1.0) * .5 * (bar_width - 1)))
        bar = ["-"] * bar_width
        bar[bar_width // 2] = "|"
        bar[marker] = "#"
        return prefix + "".join(bar) + suffix
