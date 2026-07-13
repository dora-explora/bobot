import curses
import os


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

MOTOR_OUTPUTS = (
    ("front_left", MOTOR_FRONT_LEFT_CHANNEL, MOTOR_FRONT_LEFT_SIGN),
    ("front_right", MOTOR_FRONT_RIGHT_CHANNEL, MOTOR_FRONT_RIGHT_SIGN),
    ("rear_left", MOTOR_REAR_LEFT_CHANNEL, MOTOR_REAR_LEFT_SIGN),
    ("rear_right", MOTOR_REAR_RIGHT_CHANNEL, MOTOR_REAR_RIGHT_SIGN),
)


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


def normalized_to_throttle_pulse(command):
    return normalized_to_pulse(
        clamp(command, -1.0, 1.0),
        THROTTLE_REVERSE_US,
        THROTTLE_NEUTRAL_US,
        THROTTLE_FORWARD_US
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
        import board
        import busio
        from adafruit_pca9685 import PCA9685

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        self.last_steering_us = None
        self.last_throttle_us = None
        self.last_motor_values = {}
        self.last_motor_pulses_us = {}

    def set_pulse_us(self, channel, pulse_us):
        duty_cycle = int(clamp(pulse_us * 50 * 65535 / 1000000, 0, 65535))
        self.pca.channels[channel].duty_cycle = duty_cycle

    def apply(self, steering, throttle):
        motor_values = motor_mix(steering, throttle)
        motor_pulses_us = {
            name: normalized_to_throttle_pulse(value)
            for name, value in motor_values.items()
        }
        self.last_steering_us = None
        self.last_throttle_us = None
        self.last_motor_values = motor_values
        self.last_motor_pulses_us = motor_pulses_us

        for name, channel, _ in MOTOR_OUTPUTS:
            self.set_pulse_us(channel, motor_pulses_us[name])

    def neutralize(self):
        self.apply(0.0, 0.0)

    def close(self):
        self.neutralize()
        self.pca.deinit()


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
        "value=" + str(round(throttle, 3))
        + " limit=" + str(MANUAL_THROTTLE_LIMIT)
        + " reverse/neutral/forward us="
        + str(THROTTLE_REVERSE_US)
        + "/" + str(THROTTLE_NEUTRAL_US)
        + "/" + str(THROTTLE_FORWARD_US),
        throttle_bar(throttle, width),
        "",
        "[Keys]",
        "A center turn        S full left    D full right",
        "J/I throttle +step/full forward    L/M throttle -step/full reverse",
        "K neutral throttle   W/E turn -/+ step",
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
        else:
            message = "Unknown key: " + repr(key)

        output.apply(steering, throttle)

    output.neutralize()


def main():
    output = Pca9685Output()

    try:
        curses.wrapper(run, output)
    finally:
        output.close()


if __name__ == "__main__":
    main()
