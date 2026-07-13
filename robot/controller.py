"""Non-blocking Linux gamepad input for the manual tank-drive state."""
from dataclasses import dataclass

from robot import config
from robot.vision_common import clamp


@dataclass
class ControllerUpdate:
    start_manual: bool = False
    abort_manual: bool = False
    abort_reason: str = ""


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
                    + (5 if config.CONTROLLER_A_BUTTON in key_codes else 0)
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

    def poll(self, manual_active):
        update = ControllerUpdate()
        if self.device is None:
            if manual_active and self.disconnected:
                update.abort_manual = True
                update.abort_reason = self.error or "controller disconnected"
            return update
        try:
            events = list(self.device.read())
        except BlockingIOError:
            return update
        except OSError as error:
            self.error = "controller disconnected: " + str(error)
            self.device = None
            self.disconnected = True
            if manual_active:
                update.abort_manual = True
                update.abort_reason = self.error
            return update

        for event in events:
            self.last_event = self._describe_event(event)
            if event.type == self.ecodes.EV_KEY:
                if event.code == config.CONTROLLER_A_BUTTON:
                    if event.value == 1 and not manual_active:
                        update.start_manual = True
                    continue
                if manual_active and event.value == 1:
                    update.abort_manual = True
                    update.abort_reason = "button " + self._key_name(event.code)
            elif event.type == self.ecodes.EV_ABS:
                self.axis_values[event.code] = event.value
                if manual_active and event.code not in self._stick_axes():
                    update.abort_manual = True
                    update.abort_reason = "non-stick axis " + self._axis_name(event.code)
        return update

    def tank_sides(self):
        return self._vertical_axis(config.CONTROLLER_LEFT_Y_AXIS), self._vertical_axis(config.CONTROLLER_RIGHT_Y_AXIS)

    def debug_lines(self):
        left, right = self.tank_sides()
        device = "none" if self.device is None else self.device.path + " " + self.device.name
        return [
            "device=" + device,
            "last_event=" + self.last_event,
            "left_y raw=" + str(self.axis_values.get(config.CONTROLLER_LEFT_Y_AXIS, "n/a")) + " command=" + str(round(left, 3)),
            "right_y raw=" + str(self.axis_values.get(config.CONTROLLER_RIGHT_Y_AXIS, "n/a")) + " command=" + str(round(right, 3)),
            "axes Lx/Ly/Rx/Ry=" + "/".join(str(code) for code in self._stick_axes())
            + " A_button=" + str(config.CONTROLLER_A_BUTTON) + " deadzone=" + str(config.CONTROLLER_DEADZONE),
            "detected axes=" + self._code_list(self.supported_axis_codes, self._axis_name),
            "detected keys=" + self._code_list(self.supported_key_codes, self._key_name),
        ] + (["error=" + self.error] if self.error else [])

    def close(self):
        if self.device is not None:
            self.device.close()
            self.device = None

    def _vertical_axis(self, code):
        value = self.axis_values.get(code)
        minimum, maximum = self.axis_ranges.get(code, (-32768, 32767))
        if value is None or maximum <= minimum:
            return 0.0
        normalized = (value - minimum) / float(maximum - minimum) * 2.0 - 1.0
        if config.CONTROLLER_INVERT_Y:
            normalized = -normalized
        deadzone = clamp(config.CONTROLLER_DEADZONE, 0.0, 0.95)
        if abs(normalized) <= deadzone:
            return 0.0
        return clamp((abs(normalized) - deadzone) / (1.0 - deadzone), 0.0, 1.0) * (1.0 if normalized > 0 else -1.0)

    def _stick_axes(self):
        return (
            config.CONTROLLER_LEFT_X_AXIS,
            config.CONTROLLER_LEFT_Y_AXIS,
            config.CONTROLLER_RIGHT_X_AXIS,
            config.CONTROLLER_RIGHT_Y_AXIS,
        )

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
