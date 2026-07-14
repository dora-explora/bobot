from robot import config
from robot.models import DriveCommand
from robot.vision_common import clamp


def normalized_to_pulse(command, minimum_us, center_us, maximum_us):
    command = clamp(command, -1.0, 1.0)
    return int(center_us + (center_us - minimum_us) * command) if command < 0 else int(center_us + (maximum_us - center_us) * command)


def normalize_throttle(command):
    if not config.ENABLE_THROTTLE:
        return 0.0
    command = clamp(command, -1.0, 1.0)
    if command > 0:
        command = min(command, config.THROTTLE_HARD_LIMIT)
        return min(config.THROTTLE_MIN_ACTIVE, config.THROTTLE_HARD_LIMIT) if 0 < command < config.THROTTLE_MIN_ACTIVE else command
    return max(command, -config.THROTTLE_HARD_LIMIT) if command < 0 and config.THROTTLE_ALLOW_REVERSE else 0.0


def throttle_pulse(command, esc_us):
    reverse_us, neutral_us, forward_us = esc_us
    return normalized_to_pulse(normalize_throttle(command), reverse_us, neutral_us, forward_us)


def motor_mix(command):
    if command.left is not None and command.right is not None:
        requested = {
            "front_left": command.left,
            "front_right": command.right,
            "rear_left": command.left,
            "rear_right": command.right,
        }
        return {name: requested[name] * sign for name, _, sign in config.MOTOR_OUTPUTS}

    turn = clamp(command.steering, -1.0, 1.0) * min(abs(config.MOTOR_STEERING_MIX), config.THROTTLE_HARD_LIMIT)
    left, right = command.throttle + turn, command.throttle - turn
    requested = {"front_left": left, "front_right": right, "rear_left": left, "rear_right": right}
    return {name: requested[name] * sign for name, _, sign in config.MOTOR_OUTPUTS}


class Pca9685Actuators:
    def __init__(self):
        self.enabled = config.ENABLE_ACTUATORS
        self.last_motor_values = {}
        self.last_motor_pulses_us = {}
        if not self.enabled:
            print("Actuators disabled. Set ENABLE_ACTUATORS=true to use PCA9685 outputs.")
            return
        try:
            import board
            import busio
            from adafruit_pca9685 import PCA9685
        except ImportError:
            print("PCA9685 libraries are missing. Install adafruit-blinka and adafruit-circuitpython-pca9685.")
            raise
        self.pca = PCA9685(busio.I2C(board.SCL, board.SDA))
        self.pca.frequency = 50
        print("PCA9685 enabled on channels " + ", ".join(name + "=" + str(channel) for name, channel, _ in config.MOTOR_OUTPUTS))
        print("ESC reverse/neutral/forward us: " + ", ".join(
            name + "=" + "/".join(str(value) for value in config.MOTOR_ESC_US[name])
            for name, _, _ in config.MOTOR_OUTPUTS
        ))
        self.neutralize()

    def apply(self, command):
        requested = motor_mix(command)
        self.last_motor_values = {name: normalize_throttle(value) for name, value in requested.items()}
        self.last_motor_pulses_us = {
            name: throttle_pulse(value, config.MOTOR_ESC_US[name])
            for name, value in requested.items()
        }
        if self.enabled:
            for name, channel, _ in config.MOTOR_OUTPUTS:
                self._set_pulse(channel, self.last_motor_pulses_us[name])

    def _set_pulse(self, channel, pulse_us):
        duty = int(clamp(pulse_us * 50 * 65535 / 1000000, 0, 65535))
        self.pca.channels[channel].duty_cycle = duty

    def neutralize(self):
        self.apply(DriveCommand(reason="neutralize"))

    def close(self):
        self.neutralize()
        if self.enabled:
            self.pca.deinit()
