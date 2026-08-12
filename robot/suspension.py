"""Manual suspension position control independent of drivetrain commands."""


class SuspensionControl:
    VALID_STATES = ("bottomed", "raised", "front_bottomed", "rear_bottomed")

    def __init__(self, start_state):
        if start_state not in self.VALID_STATES:
            raise ValueError("invalid suspension start state: " + start_state)
        self.state = start_state
        self.last_action = "startup " + start_state

    def update(self, active_state, controller_update):
        """Use bumper and D-pad presses only in the manual drive state."""
        if active_state != "manual" or controller_update is None:
            return ""
        actions = [
            (controller_update.left_bumper_pressed, "bottomed", "left bumper"),
            (controller_update.right_bumper_pressed, "raised", "right bumper"),
            (controller_update.front_suspension_pressed, "front_bottomed", "D-pad up"),
            (controller_update.rear_suspension_pressed, "rear_bottomed", "D-pad down"),
        ]
        requested = [action for action in actions if action[0]]
        if len(requested) != 1:
            return ""
        _, state, source = requested[0]
        return self._set_state(state, source)

    def _set_state(self, state, source):
        self.state = state
        self.last_action = source + "; suspension " + state
        return self.last_action
