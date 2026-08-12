"""Manual final-climb state using two momentary unidirectional motors."""

from robot import config
from robot.models import DriveCommand, StateResult


class ClimbState:
    name = "climb"

    def __init__(self, controller):
        self.controller = controller

    def process(
        self,
        _frame,
        _now,
        attitude=None,
        horizon=None,
        controller_update=None,
    ):
        stick = self.controller.climb_axis()
        pinch = max(0.0, stick)
        winch = max(0.0, -stick)
        active = "pinch" if pinch else "winch" if winch else "neutral"
        return StateResult(
            command=DriveCommand(
                mode="climb",
                reason="right stick " + active,
                pinch=pinch,
                winch=winch,
            ),
            state_lines=[
                "right stick up=pinch out; down=winch in; center=both neutral",
                "stick=" + str(round(stick, 3))
                + " requested pinch=" + str(round(pinch, 3))
                + " winch=" + str(round(winch, 3)),
                "limits pinch=" + str(config.CLIMB_PINCH_LIMIT)
                + " winch=" + str(config.CLIMB_WINCH_LIMIT)
                + " deadzone=" + str(config.CLIMB_STICK_DEADZONE),
                "B=immediate neutral and static; hold Y=radial menu",
            ],
            attitude=attitude,
            horizon=horizon,
        )
