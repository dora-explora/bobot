"""Manual suspension position control independent of drivetrain commands."""


class SuspensionControl:
    VALID_STATES = ("bottomed", "raised")

    def __init__(self, start_state):
        if start_state not in self.VALID_STATES:
            raise ValueError("suspension start state must be bottomed or raised")
        self.state = start_state
        self.last_action = "startup " + start_state

    def update(self, active_state, controller_update):
        """Use bumper presses only while the manual drive state is active."""
        if active_state != "manual" or controller_update is None:
            return ""
        if controller_update.left_bumper_pressed and controller_update.right_bumper_pressed:
            return ""
        if controller_update.left_bumper_pressed:
            return self._set_state("bottomed", "left bumper")
        if controller_update.right_bumper_pressed:
            return self._set_state("raised", "right bumper")
        return ""

    def _set_state(self, state, source):
        self.state = state
        self.last_action = source + "; suspension " + state
        return self.last_action
