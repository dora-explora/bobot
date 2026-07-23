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

The terminal dashboard is the main information display. The detector window is
text-free: circles represent balls, triangles represent cones, yellow outlines
represent unknown candidates, arrows show tracked motion, and the cyan line is
the IMU-adjusted horizon. Dashed outlines are unconfirmed candidates; solid
outlines have passed temporal confirmation. The mask window remains disabled.

## Pi Dependencies

Use a virtual environment that can see Raspberry Pi OS's camera and OpenCV
packages, then install the two CircuitPython hardware drivers:

```bash
sudo apt install -y python3-venv python3-opencv python3-picamera2 python3-evdev i2c-tools
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-bno08x
```

The BNO085 is expected at `0x4b` and the PCA9685 at `0x40`. Verify both:

```bash
i2cdetect -y 1
```

Adafruit recommends a 400 kHz I2C bus for the BNO085 on Raspberry Pi. Add this
to `/boot/firmware/config.txt` on current Raspberry Pi OS and reboot:

```ini
dtparam=i2c_arm_baudrate=400000
```

The runtime retries a failed IMU connection without stopping camera, controller,
or motor safety handling. `IMU_ROTATION_MODE=game` is the default because its
yaw is not corrected by the magnetometer: it may drift over a long run, but it
avoids sudden heading corrections that are harmful to frame-to-frame tracking.
Set it to `absolute` if field testing shows the magnetometer is stable around
the drive motors.

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

`main.py` starts in static mode, which keeps every motor neutral. A enters
manual tank drive, B enters static mode, and holding Y opens the radial state
menu. Either stick changes the selection, with the most recently moved stick
taking priority; releasing Y confirms the selection and B cancels it. Menu
output is always neutral. The left and right vertical sticks control the
matching motor sides in manual mode. D-pad up/down changes the live global
throttle limit by `THROTTLE_LIMIT_STEP` (default 5%) without restarting; the
TUI displays the current limit. Most controllers expose the D-pad as vertical
axis `17`, but button-code fallbacks are configurable below.

Selecting detector enters it with motor output disabled. Press A once to enable
detector output; pressing A again enters manual mode. Opening the menu from
detector clears this enable latch. Use `CONTROLLER_DEVICE=/dev/input/eventN` if
automatic gamepad discovery picks the wrong device. The manual and menu TUI
views include raw input diagnostics. `ROBOT_START_STATE` and its `START_STATE`
alias select the initial state, but detector still starts with output disabled.

Useful tuning variables:

```bash
PICAMERA2_FLIP_180=true
OBJECT_SATURATION_MIN=32
OBJECT_CHROMA_MIN=20
OBJECT_LOCAL_CHROMA_DELTA=10
OBJECT_LOCAL_BLUR_SIGMA=9
OBJECT_VALUE_MIN=35
OBJECT_VALUE_MAX=255
OBJECT_MIN_AREA_RATIO=0.00035
OBJECT_MAX_AREA_RATIO=0.22
OBJECT_UNCERTAIN_SCORE=0.42
OBJECT_CERTAIN_SCORE=0.67
OBJECT_CLASS_MARGIN=0.09
OBJECT_CERTAIN_MARGIN=0.15
MIN_BALL_AREA_RATIO=0.004
MIN_BALL_AREA_TOP_SCALE=0.25
MAX_BALL_AREA_RATIO=0.15
MAX_BALL_AREA_TOP_SCALE=1.0
TUI=true
TUI_INTERVAL=0.1
TRACK_CONFIRM_FRAMES=3
TRACK_MAX_MISSES=5
TRACK_MATCH_RADIUS_RATIO=0.14
TRACK_SCORE_SMOOTHING=0.40
TRACK_POSITION_SMOOTHING=0.58
TRACK_VELOCITY_SMOOTHING=0.45
TRACK_COLOR_WEIGHT=0.20
TRACK_SIZE_WEIGHT=0.18
CAMERA_HORIZONTAL_FOV_DEG=66
CAMERA_VERTICAL_FOV_DEG=41
IMU_ENABLED=true
IMU_I2C_ADDRESS=0x4b
IMU_REPORT_INTERVAL_US=50000
IMU_ROTATION_MODE=game
IMU_RECONNECT_INTERVAL=2.0
IMU_MAX_AGE_SECONDS=0.50
IMU_TRACK_YAW_SIGN=-1
IMU_TRACK_PITCH_SIGN=1
IMU_TRACK_ROLL_SIGN=1
IMU_TUI_TILT_RANGE_DEG=35
HORIZON_BASE_Y_RATIO=0.34
HORIZON_BALL_ALLOWANCE_RATIO=0.035
HORIZON_CONE_ALLOWANCE_RATIO=0.14
HORIZON_PITCH_SIGN=1
HORIZON_ROLL_SIGN=1
OVERLAY_MOTION_SCALE=3
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
THROTTLE_LIMIT=1.0
THROTTLE_LIMIT_STEP=0.05
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
ROBOT_START_STATE=static
CONTROLLER_DEVICE=auto
CONTROLLER_A_BUTTON=304
CONTROLLER_B_BUTTON=305
CONTROLLER_Y_BUTTON=307
CONTROLLER_LEFT_X_AXIS=0
CONTROLLER_LEFT_Y_AXIS=1
CONTROLLER_RIGHT_X_AXIS=3
CONTROLLER_RIGHT_Y_AXIS=4
CONTROLLER_DPAD_Y_AXIS=17
CONTROLLER_DPAD_UP_BUTTON=544
CONTROLLER_DPAD_DOWN_BUTTON=545
CONTROLLER_DEADZONE=0.10
CONTROLLER_MENU_DEADZONE=0.35
CONTROLLER_INVERT_Y=true
```

Set `HORIZON_BASE_Y_RATIO` while the robot is sitting in its normal level pose;
the first valid IMU reading becomes the zero attitude applied around that
baseline. If the horizon moves opposite the floor when the robot pitches or
rolls, change the corresponding `HORIZON_*_SIGN` between `1` and `-1`. If a
stationary object's predicted image motion goes opposite its observed motion
while turning, invert `IMU_TRACK_YAW_SIGN`; pitch and roll tracking have
equivalent sign controls.

Detection tuning now has three layers. `OBJECT_*` controls broad proposals,
the score and margin values control ball/cone/unknown classification, and
`TRACK_*` controls temporal association and when a dashed candidate becomes
solid. Only solid, currently observed ball tracks can become a new steering
target. A briefly missed locked track is shown as a dashed prediction while the
existing drive stabilizer holds its last command briefly.

Each `MOTOR_*_ESC_US` value is one compact `reverse,neutral,forward` triplet
in microseconds. It overrides the shared `THROTTLE_REVERSE_US`,
`THROTTLE_NEUTRAL_US`, and `THROTTLE_FORWARD_US` fallback for that ESC only.

## Safety Rules

- Test all four ESC channels with wheels off the ground first.
- Keep `ENABLE_ACTUATORS=false` until PWM ranges are confirmed.
- Static mode is the default throttle interlock. A enters manual, B returns to
  static, and holding Y opens the neutral-output radial menu; releasing Y
  confirms its selection.
- Detector output is disabled whenever detector is selected and must be
  explicitly enabled with A.
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
- `robot/object_detector.py` generates color-agnostic candidates and scores
  each one explicitly as ball, cone, or unknown.
- `robot/object_tracker.py` fuses geometry, hue, size, motion, and
  roll/pitch/yaw camera movement over time.
- `robot/imu.py` owns optional BNO085 acquisition and baseline-relative
  orientation.
- `robot/horizon.py` combines a configured image horizon with IMU pitch and
  roll. Ball and cone candidates have separate above-horizon allowances.
- `robot/overlay.py` owns the text-free solid/dashed shape and motion graphics.
- `robot/ball_detector.py` remains only as legacy code; `main.py` no longer
  uses its fixed HSV color profiles or auto-calibration.
- `robot/cone_slalom.py` supplies the current cone-slalom status summary.
- `robot/bucket_detection.py`, `robot/rough_section.py`, and
  `robot/hill_climb.py` isolate their future course behaviors from the trial
  detector state.
- `robot/drive.py`, `robot/actuators.py`, `robot/camera.py`, and
  `robot/dashboard.py` provide shared runtime services.
- The implemented runtime states are `static`, `manual`, and `detector`; future
  course sections remain isolated modules until their behavior is implemented.

## Course Roadmap

- Ball acquisition: add a compliant roller intake after ball-seeking motion is reliable.
- Bucket scoring: detect the orange tape stripe, align to it, then run a timed shooter/feed sequence.
- Obstacle navigation: classify grey bricks, black PVC pipes, and wooden ramps into simple blocked/free regions.
- Ramp run: align to the ramp centerline, use steady throttle, and reduce steering gain while climbing.
- Final bar positioning: detect the bar, center under it, stop at a calibrated image position, then hand off to the climb mechanism.
