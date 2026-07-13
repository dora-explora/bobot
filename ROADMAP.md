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
python3 detector.py
```

The terminal dashboard is the main information display. The detector window only shows the camera feed with boxes, dots, and the centerline; it does not show text, and the mask window is disabled.

Disable the terminal dashboard and use plain telemetry prints:

```bash
TUI=false python3 detector.py
```

Force headless telemetry:

```bash
HEADLESS=true python3 detector.py
```

Enable real PCA9685 actuator output only after bench testing:

```bash
ENABLE_ACTUATORS=true python3 detector.py
```

Run manual PWM control for four-ESC bench testing:

```bash
python3 manual_control.py
```

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
```

## Safety Rules

- Test all four ESC channels with wheels off the ground first.
- Keep `ENABLE_ACTUATORS=false` until PWM ranges are confirmed.
- Keep `ENABLE_THROTTLE=false` until the car is lifted or restrained.
- `manual_control.py` can send full forward and reverse; use it only with wheels off the ground.
- Use Ctrl-C to exit; the program neutralizes all four motor channels in its shutdown path.
- Add a physical kill switch before any fast or untethered run.

## Course Roadmap

- Ball acquisition: add a compliant roller intake after ball-seeking motion is reliable.
- Bucket scoring: detect the orange tape stripe, align to it, then run a timed shooter/feed sequence.
- Obstacle navigation: classify grey bricks, black PVC pipes, and wooden ramps into simple blocked/free regions.
- Ramp run: align to the ramp centerline, use steady throttle, and reduce steering gain while climbing.
- Final bar positioning: detect the bar, center under it, stop at a calibrated image position, then hand off to the climb mechanism.
