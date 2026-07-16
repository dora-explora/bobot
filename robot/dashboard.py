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
        )
        self.screen.erase()
        for index, line in enumerate(lines[:height - 1]):
            try:
                self.screen.addstr(index, 0, line[:max(0, width - 1)])
            except self.curses.error:
                pass
        self.screen.refresh()

    def _lines(self, frame, mode_control, result, output_command, actuators, controller_lines, fps, width):
        state_name = mode_control.active_state
        target, debug, command = result.best_target, result.debug, result.command
        lines = [
            "Robot Code TUI", "==============", "",
            "[State]", "active=" + state_name + "  menu=" + str(mode_control.menu_active)
            + "  available=static,detector,manual",
            "controls: A=manual/select  B=static/close menu  Y=radial menu",
            "last_action=" + mode_control.last_action, "",
            "[Status]", "camera=" + config.CAMERA_BACKEND + " frame=" + str(frame.shape[1]) + "x" + str(frame.shape[0])
            + " fps=" + str(round(fps, 1)) + " headless=" + str(config.HEADLESS),
            "actuators=" + str(config.ENABLE_ACTUATORS)
            + " throttle_limit=" + str(config.THROTTLE_LIMIT)
            + " ctrl-c=neutralize and exit",
            "output_gate=" + mode_control.output_status, "",
        ]

        if mode_control.menu_active:
            lines.extend(["[Radial Menu]"])
            lines.extend(self._radial_menu(mode_control.menu_selection, width))
            lines.extend([
                "right_stick x=" + str(round(mode_control.right_stick[0], 3))
                + " y=" + str(round(mode_control.right_stick[1], 3)),
                "A selects; B closes and returns to the paused state. Motor output remains neutral.",
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
                              + " radius=" + str(target.radius), "area=" + str(int(target.area)) + " confidence=" + str(round(target.confidence, 2)),
                              "priority score=" + str(round(debug.priority_score, 3)) + " distance=" + str(round(debug.priority_distance, 3))
                              + " cluster=" + str(round(debug.priority_cluster, 3)) + " neighbors=" + str(debug.priority_neighbors),
                              "stable=" + debug.stable_target_label + " age=" + str(round(debug.stable_target_age, 2))
                              + " switch_frames=" + str(debug.switch_candidate_frames)])
        lines.extend(["", "[Drive]", "planned mode=" + command.mode + " steering=" + str(round(command.steering, 3))
                      + " throttle=" + str(round(command.throttle, 3)), "reason=" + command.reason,
                      "output mode=" + output_command.mode + " reason=" + output_command.reason,
                      "raw=" + str(round(debug.raw_steering, 3)) + " smoothed=" + str(round(debug.smoothed_steering, 3))
                      + " slew_limited=" + str(debug.steering_limited), self._steering_bar(command.steering, width),
                      self._motors(actuators)])
        if state_name == "detector":
            lines.extend(["", "[Cone Slalom]"])
            lines.extend(result.state_lines)
        if state_name in ("manual", "static"):
            return lines
        lines.extend(["", "[Vision]", "colors=" + str(debug.active_colors) + " masks=" + str(debug.masks_checked)
                      + " contours=" + str(debug.contours_seen) + " balls=" + str(debug.accepted) + " cones=" + str(debug.cones),
                      "reject small=" + str(debug.rejected_small) + " large=" + str(debug.rejected_large)
                      + " shape=" + str(debug.rejected_shape) + " overlap=" + str(debug.rejected_overlap),
                      "auto candidates=" + str(debug.auto_candidates) + " profiles=" + str(debug.auto_profiles)
                      + " names=" + (", ".join(result.auto_color_names) or "none"), "",
                      "[Tuning]", "ball area min=" + str(config.MIN_BALL_AREA_RATIO) + " top_scale=" + str(config.MIN_BALL_AREA_TOP_SCALE)
                      + " max=" + str(config.MAX_BALL_AREA_RATIO) + " max_top_scale=" + str(config.MAX_BALL_AREA_TOP_SCALE),
                      "shape circularity=" + str(config.MIN_BALL_CIRCULARITY) + " fill=" + str(config.MIN_BALL_CIRCLE_FILL)
                      + " triangle_epsilon=" + str(config.TRIANGLE_APPROX_EPSILON),
                      "cone hsv=" + str(config.CONE_HUE_MIN) + "-" + str(config.CONE_HUE_MAX)
                      + " sat>=" + str(config.CONE_SATURATION_MIN) + " value=" + str(config.CONE_VALUE_MIN) + "-" + str(config.CONE_VALUE_MAX)])
        return lines

    @staticmethod
    def _radial_menu(selection, terminal_width):
        width = max(24, terminal_width - 1)

        def label(name):
            return "[" + name.upper() + "]" if selection == name else name

        def centered(value):
            return value.center(width)

        detector = label("detector")
        static = label("static")
        manual = label("manual")
        gap = max(1, width - len(static) - len(manual))
        return [
            centered(detector),
            centered("/ \\"),
            static + (" " * gap) + manual,
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
