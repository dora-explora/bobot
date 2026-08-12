import curses
import os
import time

from robot import config
from robot.actuators import Pca9685Watchdog, suspension_pulses


STEERING_CHANNEL = int(os.environ.get("STEERING_CHANNEL", "0"))
THROTTLE_CHANNEL = int(os.environ.get("THROTTLE_CHANNEL", "1"))
MOTOR_FRONT_LEFT_CHANNEL = int(os.environ.get("MOTOR_FRONT_LEFT_CHANNEL", "0"))
MOTOR_FRONT_RIGHT_CHANNEL = int(os.environ.get("MOTOR_FRONT_RIGHT_CHANNEL", "1"))
MOTOR_REAR_LEFT_CHANNEL = int(os.environ.get("MOTOR_REAR_LEFT_CHANNEL", "2"))
MOTOR_REAR_RIGHT_CHANNEL = int(os.environ.get("MOTOR_REAR_RIGHT_CHANNEL", "3"))
MOTOR_FRONT_LEFT_SIGN = float(os.environ.get("MOTOR_FRONT_LEFT_SIGN", "1"))
MOTOR_FRONT_RIGHT_SIGN = float(os.environ.get("MOTOR_FRONT_RIGHT_SIGN", "1"))
MOTOR_REAR_LEFT_SIGN = float(os.environ.get("MOTOR_REAR_LEFT_SIGN", "1"))
MOTOR_REAR_RIGHT_SIGN = float(os.environ.get("MOTOR_REAR_RIGHT_SIGN", "1"))
MANUAL_TURN_MIX = float(os.environ.get("MANUAL_TURN_MIX", "0.20"))
MANUAL_THROTTLE_LIMIT = float(os.environ.get("MANUAL_THROTTLE_LIMIT", "1.0"))
STEERING_CENTER_DEGREES = float(os.environ.get("STEERING_CENTER_DEGREES", "110"))
STEERING_LEFT_DEGREES = float(os.environ.get("STEERING_LEFT_DEGREES", "50"))
STEERING_RIGHT_DEGREES = float(os.environ.get("STEERING_RIGHT_DEGREES", "170"))
STEERING_SERVO_MIN_DEGREES = float(os.environ.get("STEERING_SERVO_MIN_DEGREES", "0"))
STEERING_SERVO_MAX_DEGREES = float(os.environ.get("STEERING_SERVO_MAX_DEGREES", "180"))
STEERING_SERVO_MIN_US = int(os.environ.get("STEERING_SERVO_MIN_US", "500"))
STEERING_SERVO_MAX_US = int(os.environ.get("STEERING_SERVO_MAX_US", "2500"))
THROTTLE_NEUTRAL_US = int(os.environ.get("THROTTLE_NEUTRAL_US", "1500"))
THROTTLE_FORWARD_US = int(os.environ.get("THROTTLE_FORWARD_US", "1600"))
THROTTLE_REVERSE_US = int(os.environ.get("THROTTLE_REVERSE_US", "1400"))
STEERING_STEP = float(os.environ.get("MANUAL_STEERING_STEP", "0.05"))
THROTTLE_STEP = float(os.environ.get("MANUAL_THROTTLE_STEP", "0.02"))
CLIMB_STEP = float(os.environ.get("MANUAL_CLIMB_STEP", "0.05"))
ACTUATOR_WATCHDOG_SECONDS = float(os.environ.get("ACTUATOR_WATCHDOG_SECONDS", "0.25"))
ACTUATOR_STARTUP_TIMEOUT_SECONDS = float(os.environ.get("ACTUATOR_STARTUP_TIMEOUT_SECONDS", "3.0"))

MOTOR_OUTPUTS = (
    ("front_left", MOTOR_FRONT_LEFT_CHANNEL, MOTOR_FRONT_LEFT_SIGN),
    ("front_right", MOTOR_FRONT_RIGHT_CHANNEL, MOTOR_FRONT_RIGHT_SIGN),
    ("rear_left", MOTOR_REAR_LEFT_CHANNEL, MOTOR_REAR_LEFT_SIGN),
    ("rear_right", MOTOR_REAR_RIGHT_CHANNEL, MOTOR_REAR_RIGHT_SIGN),
)
SUSPENSION_OUTPUTS = config.SUSPENSION_OUTPUTS


def pulse_triplet(name, fallback):
    raw_value = os.environ.get(name, ",".join(str(value) for value in fallback))
    try:
        values = tuple(int(value.strip()) for value in raw_value.split(","))
    except ValueError as error:
        raise ValueError(name + " must be reverse,neutral,forward microseconds") from error
    if len(values) != 3 or any(value <= 0 for value in values):
        raise ValueError(name + " must contain three positive values: reverse,neutral,forward")
    return values


SHARED_ESC_US = (THROTTLE_REVERSE_US, THROTTLE_NEUTRAL_US, THROTTLE_FORWARD_US)
MOTOR_ESC_US = {
    "front_left": pulse_triplet("MOTOR_FRONT_LEFT_ESC_US", SHARED_ESC_US),
    "front_right": pulse_triplet("MOTOR_FRONT_RIGHT_ESC_US", SHARED_ESC_US),
    "rear_left": pulse_triplet("MOTOR_REAR_LEFT_ESC_US", SHARED_ESC_US),
    "rear_right": pulse_triplet("MOTOR_REAR_RIGHT_ESC_US", SHARED_ESC_US),
}


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalized_to_pulse(command, minimum_us, center_us, maximum_us):
    command = clamp(command, -1.0, 1.0)

    if command < 0:
        return int(center_us + (center_us - minimum_us) * command)

    return int(center_us + (maximum_us - center_us) * command)


def steering_degrees_to_pulse_us(degrees):
    degrees = clamp(
        degrees,
        min(STEERING_SERVO_MIN_DEGREES, STEERING_SERVO_MAX_DEGREES),
        max(STEERING_SERVO_MIN_DEGREES, STEERING_SERVO_MAX_DEGREES)
    )
    degree_span = STEERING_SERVO_MAX_DEGREES - STEERING_SERVO_MIN_DEGREES

    if degree_span == 0:
        raise ValueError("STEERING_SERVO_MIN_DEGREES and STEERING_SERVO_MAX_DEGREES must differ")

    servo_position = (degrees - STEERING_SERVO_MIN_DEGREES) / degree_span
    pulse_span = STEERING_SERVO_MAX_US - STEERING_SERVO_MIN_US

    return int(round(STEERING_SERVO_MIN_US + servo_position * pulse_span))


def normalized_to_steering_degrees(command):
    command = clamp(command, -1.0, 1.0)

    if command < 0:
        return STEERING_CENTER_DEGREES + (
            STEERING_CENTER_DEGREES - STEERING_LEFT_DEGREES
        ) * command

    return STEERING_CENTER_DEGREES + (
        STEERING_RIGHT_DEGREES - STEERING_CENTER_DEGREES
    ) * command


def normalized_to_steering_pulse(command):
    return steering_degrees_to_pulse_us(normalized_to_steering_degrees(command))


def normalized_to_throttle_pulse(command, esc_us):
    reverse_us, neutral_us, forward_us = esc_us
    return normalized_to_pulse(
        clamp(command, -1.0, 1.0),
        reverse_us,
        neutral_us,
        forward_us
    )


def motor_mix(steering, throttle):
    throttle = clamp(throttle, -abs(MANUAL_THROTTLE_LIMIT), abs(MANUAL_THROTTLE_LIMIT))
    turn = clamp(steering, -1.0, 1.0) * abs(MANUAL_TURN_MIX)
    left = throttle + turn
    right = throttle - turn

    return {
        "front_left": clamp(left * MOTOR_FRONT_LEFT_SIGN, -1.0, 1.0),
        "front_right": clamp(right * MOTOR_FRONT_RIGHT_SIGN, -1.0, 1.0),
        "rear_left": clamp(left * MOTOR_REAR_LEFT_SIGN, -1.0, 1.0),
        "rear_right": clamp(right * MOTOR_REAR_RIGHT_SIGN, -1.0, 1.0),
    }


class Pca9685Output:
    def __init__(self):
        self.last_steering_us = None
        self.last_throttle_us = None
        self.last_motor_values = {}
        self.last_motor_pulses_us = {}
        self.last_climb_values = {"pinch": 0.0, "winch": 0.0}
        self.last_climb_pulses_us = {
            name: neutral_us for name, _, neutral_us, _ in config.CLIMB_OUTPUTS
        }
        self.suspension_enabled = config.SUSPENSION_ENABLED
        self.suspension_state = config.SUSPENSION_START_STATE
        self.last_suspension_pulses_us = suspension_pulses(self.suspension_state)
        motor_neutral_pulses = {
            name: MOTOR_ESC_US[name][1]
            for name, _, _ in MOTOR_OUTPUTS
        }
        outputs = tuple((name, channel) for name, channel, _ in MOTOR_OUTPUTS)
        initial_pulses = dict(motor_neutral_pulses)
        failsafe_pulses = dict(motor_neutral_pulses)
        outputs += tuple((name, channel) for name, channel, _, _ in config.CLIMB_OUTPUTS)
        climb_neutral = {
            name: neutral_us for name, _, neutral_us, _ in config.CLIMB_OUTPUTS
        }
        initial_pulses.update(climb_neutral)
        failsafe_pulses.update(climb_neutral)
        if self.suspension_enabled:
            outputs += tuple(
                ("suspension_" + name, channel)
                for name, channel, _, _ in SUSPENSION_OUTPUTS
            )
            initial_pulses.update(
                ("suspension_" + name, pulse)
                for name, pulse in suspension_pulses(config.SUSPENSION_START_STATE).items()
            )
            failsafe_pulses.update(
                ("suspension_" + name, pulse)
                for name, pulse in suspension_pulses(config.SUSPENSION_FAILSAFE_STATE).items()
            )
        self.watchdog = Pca9685Watchdog(
            outputs,
            initial_pulses,
            failsafe_pulses,
            ACTUATOR_WATCHDOG_SECONDS,
            ACTUATOR_STARTUP_TIMEOUT_SECONDS,
        )

    def apply(self, steering, throttle, suspension_state=None, pinch=0.0, winch=0.0):
        if suspension_state is not None:
            self.suspension_state = suspension_state
            self.last_suspension_pulses_us = suspension_pulses(suspension_state)
        motor_values = motor_mix(steering, throttle)
        motor_pulses_us = {
            name: normalized_to_throttle_pulse(value, MOTOR_ESC_US[name])
            for name, value in motor_values.items()
        }
        self.last_steering_us = None
        self.last_throttle_us = None
        self.last_motor_values = motor_values
        self.last_motor_pulses_us = motor_pulses_us
        requested = {"pinch": pinch, "winch": winch}
        limits = {"pinch": config.CLIMB_PINCH_LIMIT, "winch": config.CLIMB_WINCH_LIMIT}
        self.last_climb_values = {}
        self.last_climb_pulses_us = {}
        for name, _, neutral_us, forward_us in config.CLIMB_OUTPUTS:
            value = clamp(requested[name], 0.0, 1.0) * limits[name]
            self.last_climb_values[name] = value
            self.last_climb_pulses_us[name] = int(round(
                neutral_us + (forward_us - neutral_us) * value
            ))

        pulses = dict(motor_pulses_us)
        pulses.update(self.last_climb_pulses_us)
        if self.suspension_enabled:
            pulses.update(
                ("suspension_" + name, pulse)
                for name, pulse in self.last_suspension_pulses_us.items()
            )
        self.watchdog.apply_pulses(pulses)

    def neutralize(self):
        self.apply(0.0, 0.0)

    def close(self):
        try:
            self.neutralize()
        finally:
            self.watchdog.close()


def steering_bar(steering, width):
    label = "L "
    suffix = " R"
    bar_width = max(11, width - len(label) - len(suffix) - 1)
    center_index = bar_width // 2
    marker_index = int(round((clamp(steering, -1.0, 1.0) + 1.0) * 0.5 * (bar_width - 1)))
    bar = ["-"] * bar_width
    bar[center_index] = "|"
    bar[marker_index] = "#"
    return label + "".join(bar) + suffix


def throttle_bar(throttle, width):
    label = "R "
    suffix = " F"
    bar_width = max(11, width - len(label) - len(suffix) - 1)
    center_index = bar_width // 2
    marker_index = int(round((clamp(throttle, -1.0, 1.0) + 1.0) * 0.5 * (bar_width - 1)))
    bar = ["-"] * bar_width
    bar[center_index] = "|"
    bar[marker_index] = "#"
    return label + "".join(bar) + suffix


def motor_status(output, name):
    value = output.last_motor_values.get(name, 0.0)
    pulse = output.last_motor_pulses_us.get(name, None)
    return str(round(value, 3)) + "@" + str(pulse)


def suspension_status(output, name):
    channel = next(channel for output_name, channel, _, _ in SUSPENSION_OUTPUTS if output_name == name)
    pulse = output.last_suspension_pulses_us.get(name, None)
    pulse_ms = "-" if pulse is None else str(round(pulse / 1000.0, 3)) + "ms"
    return name + "=" + str(channel) + "@" + pulse_ms


def draw(screen, steering, throttle, output, message):
    height, width = screen.getmaxyx()
    lines = [
        "Manual Four-Motor PWM Control",
        "=============================",
        "",
        "[Channels]",
        "FL/FR/RL/RR="
        + str(MOTOR_FRONT_LEFT_CHANNEL)
        + "/"
        + str(MOTOR_FRONT_RIGHT_CHANNEL)
        + "/"
        + str(MOTOR_REAR_LEFT_CHANNEL)
        + "/"
        + str(MOTOR_REAR_RIGHT_CHANNEL),
        "FL=" + motor_status(output, "front_left")
        + " FR=" + motor_status(output, "front_right"),
        "RL=" + motor_status(output, "rear_left")
        + " RR=" + motor_status(output, "rear_right"),
        "",
        "[Turn Mix]",
        "steering=" + str(round(steering, 3))
        + " manual_turn_mix=" + str(MANUAL_TURN_MIX),
        steering_bar(steering, width),
        "",
        "[Throttle]",
        "value=" + str(round(throttle, 3)) + " limit=" + str(MANUAL_THROTTLE_LIMIT),
        "ESC us FL/FR/RL/RR=" + " ".join(
            name + "=" + "/".join(str(value) for value in MOTOR_ESC_US[name])
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        throttle_bar(throttle, width),
        "",
        "[Suspension]",
        "enabled=" + str(output.suspension_enabled)
        + " state=" + output.suspension_state
        + " start=" + config.SUSPENSION_START_STATE
        + " failsafe=" + config.SUSPENSION_FAILSAFE_STATE,
        suspension_status(output, "front_left")
        + " " + suspension_status(output, "front_right"),
        suspension_status(output, "rear_left")
        + " " + suspension_status(output, "rear_right"),
        "",
        "[Climb]",
        "pinch=" + str(round(output.last_climb_values["pinch"], 3))
        + "@" + str(output.last_climb_pulses_us["pinch"])
        + " channel=" + str(config.CLIMB_PINCH_CHANNEL),
        "winch=" + str(round(output.last_climb_values["winch"], 3))
        + "@" + str(output.last_climb_pulses_us["winch"])
        + " channel=" + str(config.CLIMB_WINCH_CHANNEL),
        "limits pinch/winch=" + str(config.CLIMB_PINCH_LIMIT)
        + "/" + str(config.CLIMB_WINCH_LIMIT),
        "",
        "[Keys]",
        "A center turn        S full left    D full right",
        "J/I throttle +step/full forward    L/M throttle -step/full reverse",
        "K neutral throttle   W/E turn -/+ step",
        "B bottom suspension  R raise suspension",
        "U/O/P pinch step/full/neutral   V/N/C winch step/full/neutral",
        "Space all neutral    Q quit",
        "",
        "[Status]",
        message,
    ]
    screen.erase()

    for index, line in enumerate(lines[:height - 1]):
        screen.addstr(index, 0, line[:max(0, width - 1)])

    screen.refresh()


def run(screen, output):
    steering = 0.0
    throttle = 0.0
    pinch = 0.0
    winch = 0.0
    message = "Ready. Wheels should be off the ground."
    output.neutralize()
    draw(screen, steering, throttle, output, message)
    screen.nodelay(True)

    while True:
        draw(screen, steering, throttle, output, message)
        key = screen.getch()

        if key == -1:
            time.sleep(0.03)
            continue

        key = chr(key).lower() if 0 <= key <= 255 else ""

        if key == "q":
            message = "Quitting; neutralizing outputs."
            break

        if key == " ":
            steering = 0.0
            throttle = 0.0
            pinch = 0.0
            winch = 0.0
            message = "All neutral."
        elif key == "a":
            steering = 0.0
            message = "Turn centered."
        elif key == "s":
            steering = -1.0
            message = "Full left tank mix."
        elif key == "d":
            steering = 1.0
            message = "Full right tank mix."
        elif key == "w":
            steering = clamp(steering - STEERING_STEP, -1.0, 1.0)
            message = "Turn stepped left."
        elif key == "e":
            steering = clamp(steering + STEERING_STEP, -1.0, 1.0)
            message = "Turn stepped right."
        elif key == "i":
            throttle = 1.0
            message = "Throttle full forward."
        elif key == "k":
            throttle = 0.0
            message = "Throttle neutral."
        elif key == "m":
            throttle = -1.0
            message = "Throttle full reverse."
        elif key == "j":
            throttle = clamp(throttle + THROTTLE_STEP, -1.0, 1.0)
            message = "Throttle stepped forward."
        elif key == "l":
            throttle = clamp(throttle - THROTTLE_STEP, -1.0, 1.0)
            message = "Throttle stepped reverse."
        elif key == "b":
            output.apply(steering, throttle, "bottomed", pinch, winch)
            message = "Suspension bottomed."
            continue
        elif key == "r":
            output.apply(steering, throttle, "raised", pinch, winch)
            message = "Suspension raised."
            continue
        elif key == "u":
            pinch = clamp(pinch + CLIMB_STEP, 0.0, 1.0)
            message = "Pinch stepped forward."
        elif key == "o":
            pinch = 1.0
            message = "Pinch full configured output."
        elif key == "p":
            pinch = 0.0
            message = "Pinch neutral."
        elif key == "v":
            winch = clamp(winch + CLIMB_STEP, 0.0, 1.0)
            message = "Winch stepped inward."
        elif key == "n":
            winch = 1.0
            message = "Winch full configured output."
        elif key == "c":
            winch = 0.0
            message = "Winch neutral."
        else:
            message = "Unknown key: " + repr(key)

        output.apply(steering, throttle, pinch=pinch, winch=winch)

    output.neutralize()


def main():
    output = Pca9685Output()

    try:
        curses.wrapper(run, output)
    finally:
        output.close()


if __name__ == "__main__":
    main()
