"""Controller-driven runtime state selection and actuator interlocks."""
from dataclasses import dataclass
import math

from robot.models import DriveCommand


MENU_OPTIONS = ("detector", "manual", "static")


@dataclass
class ModeDecision:
    neutralize_this_frame: bool = False
    message: str = ""


class ModeControl:
    """Own state transitions separately from vision and motor command logic."""

    def __init__(self, start_state):
        if start_state not in MENU_OPTIONS:
            raise ValueError("ROBOT_START_STATE must be one of: " + ", ".join(sorted(MENU_OPTIONS)))
        self.active_state = start_state
        self.menu_active = False
        self.menu_selection = start_state
        self.detector_throttle_enabled = False
        self.menu_stick = (0.0, 0.0)
        self.menu_stick_source = "right"
        self.last_action = "startup in " + start_state

    @property
    def output_enabled(self):
        if self.menu_active or self.active_state == "static":
            return False
        if self.active_state == "detector":
            return self.detector_throttle_enabled
        return self.active_state == "manual"

    @property
    def output_status(self):
        if self.menu_active:
            return "disabled while radial menu is open"
        if self.active_state == "static":
            return "disabled by static mode"
        if self.active_state == "detector" and not self.detector_throttle_enabled:
            return "detector throttle disabled; press A to enable"
        return "enabled for " + self.active_state

    def update(self, controller_update, menu_stick, menu_stick_source="right"):
        """Apply one batch of button events and return transition side effects."""
        self.menu_stick = menu_stick
        self.menu_stick_source = menu_stick_source
        if controller_update.controller_lost:
            if self.output_enabled or self.menu_active:
                return self._enter_static("controller disconnected; entered static mode")
            return ModeDecision()

        if self.menu_active:
            selection = self.radial_selection(menu_stick)
            if selection is not None:
                self.menu_selection = selection
            if controller_update.b_pressed:
                self.menu_active = False
                return self._decision(True, "closed radial menu; returned to " + self.active_state)
            if controller_update.a_pressed:
                selected = self.menu_selection
                self.menu_active = False
                self.active_state = selected
                self.detector_throttle_enabled = False
                return self._decision(True, "selected " + selected + " from radial menu")
            return ModeDecision()

        if controller_update.b_pressed:
            return self._enter_static("B pressed; entered static mode")

        if controller_update.y_pressed:
            self.menu_active = True
            self.menu_selection = self.active_state
            selection = self.radial_selection(menu_stick)
            if selection is not None:
                self.menu_selection = selection
            if self.active_state == "detector":
                self.detector_throttle_enabled = False
            return self._decision(True, "opened radial menu; motor output paused")

        if controller_update.a_pressed:
            if self.active_state == "detector" and not self.detector_throttle_enabled:
                self.detector_throttle_enabled = True
                return self._decision(False, "A pressed; detector throttle enabled")
            if self.active_state != "manual":
                self.active_state = "manual"
                self.detector_throttle_enabled = False
                return self._decision(True, "A pressed; entered manual mode")

        return ModeDecision()

    def gate_command(self, planned_command, neutralize_this_frame=False):
        if neutralize_this_frame:
            return DriveCommand(mode="disabled", reason="neutral transition frame")
        if not self.output_enabled:
            return DriveCommand(mode="disabled", reason=self.output_status)
        return planned_command

    @staticmethod
    def radial_selection(menu_stick):
        """Map either controller stick to three evenly spaced radial sectors."""
        x, y = menu_stick
        if math.hypot(x, y) <= 0.0:
            return None
        directions = {
            "detector": (0.0, 1.0),
            "manual": (math.sqrt(3.0) / 2.0, -0.5),
            "static": (-math.sqrt(3.0) / 2.0, -0.5),
        }
        return max(directions, key=lambda name: x * directions[name][0] + y * directions[name][1])

    def _enter_static(self, message):
        self.menu_active = False
        self.active_state = "static"
        self.detector_throttle_enabled = False
        self.menu_selection = "static"
        return self._decision(True, message)

    def _decision(self, neutralize, message):
        self.last_action = message
        return ModeDecision(neutralize_this_frame=neutralize, message=message)
