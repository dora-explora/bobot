"""Non-blocking Linux gamepad input for runtime modes and tank drive."""
from dataclasses import dataclass
import math

from robot import config
from robot.vision_common import clamp


@dataclass
class ControllerUpdate:
    a_pressed: bool = False
    b_pressed: bool = False
    y_pressed: bool = False
    y_released: bool = False
    capture_pressed: bool = False
    throttle_limit_delta: int = 0
    controller_lost: bool = False


class ControllerInput:
    """Read one evdev gamepad without ever blocking the camera/control loop."""

    def __init__(self):
        self.device = None
        self.ecodes = None
        self.error = ""
        self.last_event = "none"
        self.axis_values = {}
        self.axis_ranges = {}
        self.supported_axis_codes = []
        self.supported_key_codes = []
        self.disconnected = False
        self._menu_stick_source = "right"
        self._menu_input_sequence = 0
        self._menu_stick_sequence = {"left": 0, "right": 0}
        self._connect()

    def _connect(self):
        try:
            from evdev import InputDevice, ecodes, list_devices
        except ImportError:
            self.error = "python-evdev is not installed"
            return

        self.ecodes = ecodes
        paths = [config.CONTROLLER_DEVICE] if config.CONTROLLER_DEVICE != "auto" else list_devices()
        candidates = []
        for path in paths:
            try:
                device = InputDevice(path)
                capabilities = device.capabilities(absinfo=True)
                keys = capabilities.get(ecodes.EV_KEY, [])
                axes = capabilities.get(ecodes.EV_ABS, [])
                key_codes = [item[0] if isinstance(item, tuple) else item for item in keys]
                axis_codes = [item[0] if isinstance(item, tuple) else item for item in axes]
                if config.CONTROLLER_DEVICE == "auto" and (not axis_codes or not key_codes):
                    device.close()
                    continue
                score = (
                    (10 if config.CONTROLLER_RIGHT_Y_AXIS in axis_codes else 0)
                    + (10 if config.CONTROLLER_LEFT_Y_AXIS in axis_codes else 0)
                    + (3 if config.CONTROLLER_A_BUTTON in key_codes else 0)
                    + (3 if config.CONTROLLER_B_BUTTON in key_codes else 0)
                    + (3 if config.CONTROLLER_Y_BUTTON in key_codes else 0)
                    + (2 if config.CONTROLLER_CAPTURE_BUTTON in key_codes else 0)
                    + min(len(axis_codes), 8)
                )
                candidates.append((score, device, keys, axes, key_codes, axis_codes))
            except Exception as error:
                self.error = "controller open failed: " + str(error)

        if not candidates:
            if not self.error:
                self.error = "no controller-like evdev device found"
            return

        _, device, keys, axes, key_codes, axis_codes = max(candidates, key=lambda item: item[0])
        for _, other, *_ in candidates:
            if other is not device:
                other.close()
        # evdev InputDevice opens its file descriptor with O_NONBLOCK itself.
        # Older releases do not expose a set_blocking() convenience method.
        self.device = device
        self.error = ""
        self.disconnected = False
        self.supported_key_codes = sorted(key_codes)
        self.supported_axis_codes = sorted(axis_codes)
        for item in axes:
            if isinstance(item, tuple):
                code, info = item
                self.axis_ranges[code] = (info.min, info.max)
        self.last_event = "connected " + device.path + " " + device.name

    @property
    def connected(self):
        return self.device is not None

    def poll(self):
        """Return edge-triggered mode button presses without blocking."""
        update = ControllerUpdate()
        if self.device is None:
            return update
        try:
            events = list(self.device.read())
        except BlockingIOError:
            return update
        except OSError as error:
            self.error = "controller disconnected: " + str(error)
            self.device = None
            self.disconnected = True
            update.controller_lost = True
            return update

        for event in events:
            self.last_event = self._describe_event(event)
            if event.type == self.ecodes.EV_KEY:
                if event.code == config.CONTROLLER_Y_BUTTON and event.value == 0:
                    update.y_released = True
                elif event.value != 1:
                    continue
                elif event.code == config.CONTROLLER_A_BUTTON:
                    update.a_pressed = True
                elif event.code == config.CONTROLLER_B_BUTTON:
                    update.b_pressed = True
                elif event.code == config.CONTROLLER_Y_BUTTON:
                    update.y_pressed = True
                elif event.code == config.CONTROLLER_CAPTURE_BUTTON:
                    update.capture_pressed = True
                elif event.code == config.CONTROLLER_DPAD_UP_BUTTON:
                    update.throttle_limit_delta += 1
                elif event.code == config.CONTROLLER_DPAD_DOWN_BUTTON:
                    update.throttle_limit_delta -= 1
            elif event.type == self.ecodes.EV_ABS:
                self.axis_values[event.code] = event.value
                if event.code == config.CONTROLLER_DPAD_Y_AXIS:
                    if event.value < 0:
                        update.throttle_limit_delta += 1
                    elif event.value > 0:
                        update.throttle_limit_delta -= 1
                stick = self._menu_stick_for_axis(event.code)
                if stick is not None:
                    self._menu_input_sequence += 1
                    self._menu_stick_sequence[stick] = self._menu_input_sequence
                    self._menu_stick_source = stick
        return update

    def tank_sides(self):
        return self._vertical_axis(config.CONTROLLER_LEFT_Y_AXIS), self._vertical_axis(config.CONTROLLER_RIGHT_Y_AXIS)

    def right_stick(self):
        """Return the right-stick radial-menu vector for compatibility/debugging."""
        return self._stick_vector(config.CONTROLLER_RIGHT_X_AXIS, config.CONTROLLER_RIGHT_Y_AXIS)

    def left_stick(self):
        """Return the left-stick radial-menu vector for menu selection."""
        return self._stick_vector(config.CONTROLLER_LEFT_X_AXIS, config.CONTROLLER_LEFT_Y_AXIS)

    def menu_stick(self):
        """Return the last moved stick's vector and its source name for the radial menu."""
        source = self._menu_stick_source
        if self._menu_stick_sequence["left"] > self._menu_stick_sequence["right"]:
            source = "left"
        elif self._menu_stick_sequence["right"] > self._menu_stick_sequence["left"]:
            source = "right"
        return (self.left_stick() if source == "left" else self.right_stick()), source

    def _stick_vector(self, x_axis, y_axis):
        x = self._normalized_axis(x_axis)
        y = self._normalized_axis(y_axis, invert=config.CONTROLLER_INVERT_Y)
        magnitude = math.hypot(x, y)
        deadzone = clamp(config.CONTROLLER_MENU_DEADZONE, 0.0, 0.95)
        if magnitude <= deadzone:
            return 0.0, 0.0
        scaled = clamp((magnitude - deadzone) / (1.0 - deadzone), 0.0, 1.0)
        return x / magnitude * scaled, y / magnitude * scaled

    def debug_lines(self):
        left, right = self.tank_sides()
        (menu_x, menu_y), menu_source = self.menu_stick()
        left_menu_x, left_menu_y = self.left_stick()
        right_menu_x, right_menu_y = self.right_stick()
        device = "none" if self.device is None else self.device.path + " " + self.device.name
        return [
            "device=" + device,
            "last_event=" + self.last_event,
            "left_y raw=" + str(self.axis_values.get(config.CONTROLLER_LEFT_Y_AXIS, "n/a")) + " command=" + str(round(left, 3)),
            "right_y raw=" + str(self.axis_values.get(config.CONTROLLER_RIGHT_Y_AXIS, "n/a")) + " command=" + str(round(right, 3)),
            "menu source=" + menu_source + " x=" + str(round(menu_x, 3)) + " y=" + str(round(menu_y, 3)),
            "menu left x=" + str(round(left_menu_x, 3)) + " y=" + str(round(left_menu_y, 3))
            + " right x=" + str(round(right_menu_x, 3)) + " y=" + str(round(right_menu_y, 3)),
            "D-pad up/down adjusts throttle limit by " + str(config.THROTTLE_LIMIT_STEP)
            + " current=" + str(round(config.THROTTLE_LIMIT, 3)),
            "axes Lx/Ly/Rx/Ry=" + "/".join(str(code) for code in self._stick_axes())
            + " buttons A/B/Y/X-capture=" + "/".join(str(code) for code in (
                config.CONTROLLER_A_BUTTON,
                config.CONTROLLER_B_BUTTON,
                config.CONTROLLER_Y_BUTTON,
                config.CONTROLLER_CAPTURE_BUTTON,
            ))
            + " deadzone=" + str(config.CONTROLLER_DEADZONE)
            + " menu_deadzone=" + str(config.CONTROLLER_MENU_DEADZONE),
            "D-pad axis=" + str(config.CONTROLLER_DPAD_Y_AXIS)
            + " buttons up/down=" + str(config.CONTROLLER_DPAD_UP_BUTTON)
            + "/" + str(config.CONTROLLER_DPAD_DOWN_BUTTON),
            "detected axes=" + self._code_list(self.supported_axis_codes, self._axis_name),
            "detected keys=" + self._code_list(self.supported_key_codes, self._key_name),
        ] + (["error=" + self.error] if self.error else [])

    def close(self):
        if self.device is not None:
            self.device.close()
            self.device = None

    def _vertical_axis(self, code):
        normalized = self._normalized_axis(code, invert=config.CONTROLLER_INVERT_Y)
        deadzone = clamp(config.CONTROLLER_DEADZONE, 0.0, 0.95)
        if abs(normalized) <= deadzone:
            return 0.0
        return clamp((abs(normalized) - deadzone) / (1.0 - deadzone), 0.0, 1.0) * (1.0 if normalized > 0 else -1.0)

    def _normalized_axis(self, code, invert=False):
        value = self.axis_values.get(code)
        minimum, maximum = self.axis_ranges.get(code, (-32768, 32767))
        if value is None or maximum <= minimum:
            return 0.0
        normalized = (value - minimum) / float(maximum - minimum) * 2.0 - 1.0
        return -normalized if invert else normalized

    def _stick_axes(self):
        return (
            config.CONTROLLER_LEFT_X_AXIS,
            config.CONTROLLER_LEFT_Y_AXIS,
            config.CONTROLLER_RIGHT_X_AXIS,
            config.CONTROLLER_RIGHT_Y_AXIS,
        )

    @staticmethod
    def _menu_stick_for_axis(code):
        if code in (config.CONTROLLER_LEFT_X_AXIS, config.CONTROLLER_LEFT_Y_AXIS):
            return "left"
        if code in (config.CONTROLLER_RIGHT_X_AXIS, config.CONTROLLER_RIGHT_Y_AXIS):
            return "right"
        return None

    def _describe_event(self, event):
        if event.type == self.ecodes.EV_KEY:
            return "key " + self._key_name(event.code) + " value=" + str(event.value)
        if event.type == self.ecodes.EV_ABS:
            return "axis " + self._axis_name(event.code) + " value=" + str(event.value)
        return "event type=" + str(event.type) + " code=" + str(event.code) + " value=" + str(event.value)

    def _key_name(self, code):
        if self.ecodes is None:
            return str(code)
        return str(self.ecodes.bytype[self.ecodes.EV_KEY].get(code, code))

    def _axis_name(self, code):
        if self.ecodes is None:
            return str(code)
        return str(self.ecodes.bytype[self.ecodes.EV_ABS].get(code, code))

    @staticmethod
    def _code_list(codes, name_for_code):
        if not codes:
            return "none"
        return ", ".join(str(code) + ":" + name_for_code(code) for code in codes[:12])
