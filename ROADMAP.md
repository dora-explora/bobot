# Autonomous RC-Car Trial Roadmap

## This Week's Trial Goals

1. Detect calibrated multi-colored tennis balls from the Pi camera.
2. Pick the best visible ball target.
3. Compute low-speed tank-drive motor commands from ball steering error.
4. Keep actuator output disabled by default until hardware is ready.

## Electronics Assumptions

- Raspberry Pi 5 is the main computer.
- PCA9685 is connected over I2C.
- PCA9685 channel 0 drives the front-left brushless ESC signal.
- PCA9685 channel 1 drives the front-right brushless ESC signal.
- PCA9685 channel 2 drives the rear-left brushless ESC signal.
- PCA9685 channel 3 drives the rear-right brushless ESC signal.
- The Pi, PCA9685, ESC/BECs, and motor battery must share ground.
- PCA9685 outputs are PWM control signals only. Do not power DC motors from the PCA9685.

## Runtime Modes

Dry-run vision and command telemetry:

```bash
python3 main.py
```

`detector.py` remains a compatibility launcher for existing Pi commands, but
`main.py` is the primary entrypoint from now on.

The terminal dashboard is the main information display. The detector window only shows the camera feed with boxes, dots, and the centerline; it does not show text, and the mask window is disabled.

Disable the terminal dashboard and use plain telemetry prints:

```bash
TUI=false python3 main.py
```

Force headless telemetry:

```bash
HEADLESS=true python3 main.py
```

Enable real PCA9685 actuator output only after bench testing:

```bash
ENABLE_ACTUATORS=true python3 main.py
```

Run manual PWM control for four-ESC bench testing:

```bash
python3 manual_control.py
```

Install controller input support for the new `manual` state:

```bash
sudo apt install -y python3-evdev
```

While running `main.py`, press the controller A button to enter manual tank
drive. The left and right vertical sticks control the matching motor sides.
Any non-stick controller input neutralizes every output and exits the runtime,
including while the robot is in detector mode.
Use `CONTROLLER_DEVICE=/dev/input/eventN` if automatic gamepad discovery picks
the wrong device. The manual TUI temporarily includes raw input diagnostics.
Use `ROBOT_START_STATE=manual python3 main.py` to start in the manual debug
state without first matching the A-button mapping. `START_STATE=manual` is
also accepted as an alias.

Useful tuning variables:

```bash
AUTO_CALIBRATE=true
AUTO_CALIBRATION_INTERVAL=1.0
AUTO_CALIBRATION_SATURATION_MIN=35
AUTO_CALIBRATION_VALUE_MIN=70
MIN_BALL_AREA_RATIO=0.004
MAX_BALL_AREA_RATIO=0.15
TUI=true
TUI_INTERVAL=0.1
KNOWN_COLOR_HUE_PADDING=12
KNOWN_COLOR_SATURATION_MIN=45
KNOWN_COLOR_VALUE_MIN=60
TARGET_LOCK_RADIUS_RATIO=0.18
TARGET_SWITCH_MARGIN=0.18
TARGET_SWITCH_FRAMES=3
TARGET_HOLD_SECONDS=0.35
TARGET_SMOOTHING=0.35
STEERING_SLEW_RATE=2.0
MOTOR_FRONT_LEFT_CHANNEL=0
MOTOR_FRONT_RIGHT_CHANNEL=1
MOTOR_REAR_LEFT_CHANNEL=2
MOTOR_REAR_RIGHT_CHANNEL=3
MOTOR_FRONT_LEFT_SIGN=1
MOTOR_FRONT_RIGHT_SIGN=1
MOTOR_REAR_LEFT_SIGN=1
MOTOR_REAR_RIGHT_SIGN=1
MOTOR_STEERING_MIX=0.08
THROTTLE_NEUTRAL_US=1500
THROTTLE_FORWARD_US=1600
THROTTLE_REVERSE_US=1400
MOTOR_FRONT_LEFT_ESC_US=1400,1500,1600
MOTOR_FRONT_RIGHT_ESC_US=1400,1500,1600
MOTOR_REAR_LEFT_ESC_US=1400,1500,1600
MOTOR_REAR_RIGHT_ESC_US=1400,1500,1600
ACTUATOR_WATCHDOG_SECONDS=0.25
ACTUATOR_STARTUP_TIMEOUT_SECONDS=3.0
FATAL_ERROR_LOG=/tmp/bobot-fatal.log
ENABLE_THROTTLE=false
THROTTLE_HARD_LIMIT=0.12
THROTTLE_MIN_ACTIVE=0.06
THROTTLE_ALLOW_REVERSE=false
MAX_TRIAL_THROTTLE=0.10
MANUAL_STEERING_STEP=0.05
MANUAL_THROTTLE_STEP=0.02
MANUAL_TURN_MIX=0.20
MANUAL_THROTTLE_LIMIT=1.0
STEERING_GAIN=1.25
STEERING_DEADBAND=0.06
CLOSE_BALL_AREA_RATIO=0.18
LOST_TARGET_TIMEOUT=0.5
ROBOT_START_STATE=detector
CONTROLLER_DEVICE=auto
CONTROLLER_A_BUTTON=304
CONTROLLER_LEFT_X_AXIS=0
CONTROLLER_LEFT_Y_AXIS=1
CONTROLLER_RIGHT_X_AXIS=3
CONTROLLER_RIGHT_Y_AXIS=4
CONTROLLER_DEADZONE=0.10
CONTROLLER_INVERT_Y=true
```

Each `MOTOR_*_ESC_US` value is one compact `reverse,neutral,forward` triplet
in microseconds. It overrides the shared `THROTTLE_REVERSE_US`,
`THROTTLE_NEUTRAL_US`, and `THROTTLE_FORWARD_US` fallback for that ESC only.

## Safety Rules

- Test all four ESC channels with wheels off the ground first.
- Keep `ENABLE_ACTUATORS=false` until PWM ranges are confirmed.
- Keep `ENABLE_THROTTLE=false` until the car is lifted or restrained.
- `manual_control.py` can send full forward and reverse; use it only with wheels off the ground.
- Use Ctrl-C to exit; the program neutralizes all four motor channels in its shutdown path.
- Add a physical kill switch before any fast or untethered run.
- PCA9685 output is owned by a separate watchdog process. It receives a fresh
  command every control frame and writes all four calibrated neutral pulses
  after `ACTUATOR_WATCHDOG_SECONDS` without one. This covers a hung or crashed
  camera/control process, including one that can no longer read controller input.
- The watchdog does not cover I2C bus lockup, PCA9685 failure, or Pi/OS power
  failure. A physical e-stop or power/signal gate that disables ESC output is
  still mandatory before anyone is near the moving vehicle.

## Runtime Layout

- `main.py` owns camera lifecycle, actuator lifecycle, mission state selection,
  visualization, and the state-aware TUI dashboard.
- `robot/ball_detector.py` owns multicolor ball detection and auto-calibration.
- `robot/cone_slalom.py` owns cone detection and slalom status.
- `robot/bucket_detection.py`, `robot/rough_section.py`, and
  `robot/hill_climb.py` isolate their future course behaviors from the trial
  detector state.
- `robot/drive.py`, `robot/actuators.py`, `robot/camera.py`, and
  `robot/dashboard.py` provide shared runtime services.
- The only enabled mission state is currently `detector`; the TUI visibly shows
  that state and renders its detector-specific information.

## Course Roadmap

- Ball acquisition: add a compliant roller intake after ball-seeking motion is reliable.
- Bucket scoring: detect the orange tape stripe, align to it, then run a timed shooter/feed sequence.
- Obstacle navigation: classify grey bricks, black PVC pipes, and wooden ramps into simple blocked/free regions.
- Ramp run: align to the ramp centerline, use steady throttle, and reduce steering gain while climbing.
- Final bar positioning: detect the bar, center under it, stop at a calibrated image position, then hand off to the climb mechanism.
