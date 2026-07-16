"""PCA9685 output with an independent process watchdog.

The PCA9685 latches its last PWM duty cycle. Keeping hardware ownership in a
separate process lets that process return the ESCs to neutral if camera or
vision code blocks, raises, or terminates unexpectedly.
"""
import multiprocessing
import time
import traceback

from robot import config
from robot.models import DriveCommand
from robot.vision_common import clamp


def normalized_to_pulse(command, minimum_us, center_us, maximum_us):
    command = clamp(command, -1.0, 1.0)
    return int(center_us + (center_us - minimum_us) * command) if command < 0 else int(center_us + (maximum_us - center_us) * command)


def normalize_throttle(command):
    command = clamp(command, -1.0, 1.0)
    if command > 0:
        command = min(command, config.THROTTLE_LIMIT)
        return min(config.THROTTLE_MIN_ACTIVE, config.THROTTLE_LIMIT) if 0 < command < config.THROTTLE_MIN_ACTIVE else command
    return max(command, -config.THROTTLE_LIMIT) if command < 0 and config.THROTTLE_ALLOW_REVERSE else 0.0


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

    turn = clamp(command.steering, -1.0, 1.0) * min(abs(config.MOTOR_STEERING_MIX), config.THROTTLE_LIMIT)
    left, right = command.throttle + turn, command.throttle - turn
    requested = {"front_left": left, "front_right": right, "rear_left": left, "rear_right": right}
    return {name: requested[name] * sign for name, _, sign in config.MOTOR_OUTPUTS}


def _duty_cycle_for_pulse(pulse_us):
    return int(clamp(pulse_us * 50 * 65535 / 1000000, 0, 65535))


def _write_watchdog_error(error_report, fatal_error_log):
    try:
        with open(fatal_error_log, "a", encoding="utf-8") as log_file:
            log_file.write("PCA9685 WATCHDOG ERROR\n" + error_report + "\n")
    except OSError:
        pass


def _pca9685_watchdog_worker(connection, motor_outputs, neutral_pulses, watchdog_seconds, fatal_error_log):
    """Own the I2C device and fail to neutral when parent heartbeats stop."""
    pca = None

    def write_pulses(pulses):
        for name, channel, _ in motor_outputs:
            pca.channels[channel].duty_cycle = _duty_cycle_for_pulse(pulses[name])

    try:
        import board
        import busio
        from adafruit_pca9685 import PCA9685

        pca = PCA9685(busio.I2C(board.SCL, board.SDA))
        pca.frequency = 50
        write_pulses(neutral_pulses)
        connection.send(("ready", ""))
        last_heartbeat = time.monotonic()

        while True:
            wait_seconds = max(0.01, watchdog_seconds - (time.monotonic() - last_heartbeat))
            if not connection.poll(wait_seconds):
                write_pulses(neutral_pulses)
                last_heartbeat = time.monotonic()
                continue

            message, payload = connection.recv()
            if message == "pulses":
                write_pulses(payload)
                last_heartbeat = time.monotonic()
            elif message == "shutdown":
                write_pulses(neutral_pulses)
                break
    except BaseException:
        error_report = traceback.format_exc()
        _write_watchdog_error(error_report, fatal_error_log)
        try:
            connection.send(("fatal", error_report))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if pca is not None:
            try:
                write_pulses(neutral_pulses)
            except BaseException:
                pass
            try:
                pca.deinit()
            except BaseException:
                pass
        try:
            connection.close()
        except BaseException:
            pass


class Pca9685Watchdog:
    """Parent-side interface for watchdog-owned PCA9685 output."""

    def __init__(self, motor_outputs, neutral_pulses, watchdog_seconds, startup_timeout_seconds):
        if watchdog_seconds <= 0:
            raise ValueError("ACTUATOR_WATCHDOG_SECONDS must be greater than zero")
        self.connection, worker_connection = multiprocessing.get_context("spawn").Pipe()
        self.process = multiprocessing.get_context("spawn").Process(
            target=_pca9685_watchdog_worker,
            args=(worker_connection, motor_outputs, neutral_pulses, watchdog_seconds, config.FATAL_ERROR_LOG),
            name="pca9685-watchdog",
            daemon=False,
        )
        self.neutral_pulses = neutral_pulses
        self.closed = False
        self.process.start()
        worker_connection.close()
        if not self.connection.poll(startup_timeout_seconds):
            self._terminate_worker()
            raise RuntimeError("PCA9685 watchdog did not become ready before timeout")
        status, detail = self.connection.recv()
        if status != "ready":
            self._terminate_worker()
            raise RuntimeError("PCA9685 watchdog startup failed:\n" + detail)

    def apply_pulses(self, pulses):
        self._check_health()
        try:
            self.connection.send(("pulses", pulses))
        except (BrokenPipeError, EOFError, OSError) as error:
            raise RuntimeError("PCA9685 watchdog connection failed: " + str(error)) from error

    def neutralize(self):
        self.apply_pulses(self.neutral_pulses)

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.process.is_alive():
                self.connection.send(("shutdown", None))
                self.process.join(1.0)
        except (BrokenPipeError, EOFError, OSError):
            pass
        finally:
            if self.process.is_alive():
                self._terminate_worker()
            self.connection.close()

    def _check_health(self):
        if self.connection.poll():
            status, detail = self.connection.recv()
            if status == "fatal":
                raise RuntimeError("PCA9685 watchdog failed:\n" + detail)
        if not self.process.is_alive():
            raise RuntimeError("PCA9685 watchdog stopped unexpectedly")

    def _terminate_worker(self):
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(1.0)


class Pca9685Actuators:
    def __init__(self):
        self.enabled = config.ENABLE_ACTUATORS
        self.last_motor_values = {}
        self.last_motor_pulses_us = {}
        self.watchdog = None
        if not self.enabled:
            print("Actuators disabled. Set ENABLE_ACTUATORS=true to use PCA9685 outputs.")
            return

        neutral_pulses = {
            name: config.MOTOR_ESC_US[name][1]
            for name, _, _ in config.MOTOR_OUTPUTS
        }
        self.watchdog = Pca9685Watchdog(
            config.MOTOR_OUTPUTS,
            neutral_pulses,
            config.ACTUATOR_WATCHDOG_SECONDS,
            config.ACTUATOR_STARTUP_TIMEOUT_SECONDS,
        )
        print("PCA9685 watchdog enabled on channels " + ", ".join(name + "=" + str(channel) for name, channel, _ in config.MOTOR_OUTPUTS))
        print("Actuator watchdog timeout: " + str(config.ACTUATOR_WATCHDOG_SECONDS) + " seconds")
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
            self.watchdog.apply_pulses(self.last_motor_pulses_us)

    def neutralize(self):
        self.apply(DriveCommand(reason="neutralize"))

    def close(self):
        try:
            if self.enabled:
                self.neutralize()
        finally:
            if self.watchdog is not None:
                self.watchdog.close()
