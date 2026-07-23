import os


def env_bool(name, default):
    value = os.environ.get(name, default).lower()
    if value not in ("true", "false"):
        raise ValueError(name + " must be true or false")
    return value == "true"


def env_float(name, default):
    return float(os.environ.get(name, default))


def env_int(name, default):
    return int(os.environ.get(name, default))


def env_auto_int(name, default):
    return int(os.environ.get(name, default), 0)


def env_pulse_triplet(name, fallback):
    """Read reverse,neutral,forward microseconds from one compact envvar."""
    raw_value = os.environ.get(name, ",".join(str(value) for value in fallback))
    try:
        values = tuple(int(value.strip()) for value in raw_value.split(","))
    except ValueError as error:
        raise ValueError(name + " must be reverse,neutral,forward microseconds") from error
    if len(values) != 3 or any(value <= 0 for value in values):
        raise ValueError(name + " must contain three positive values: reverse,neutral,forward")
    return values


FRAME_WIDTH = env_int("FRAME_WIDTH", "640")
FRAME_HEIGHT = env_int("FRAME_HEIGHT", "480")
CAMERA_BACKEND = os.environ.get("CAMERA_BACKEND", "auto").lower()
PICAMERA2_SWAP_RED_BLUE = env_bool("PICAMERA2_SWAP_RED_BLUE", "true")
PICAMERA2_FLIP_180 = env_bool("PICAMERA2_FLIP_180", "true")
HEADLESS_REQUEST = os.environ.get("HEADLESS", "auto").lower()
HEADLESS = (
    HEADLESS_REQUEST == "true"
    or (HEADLESS_REQUEST == "auto" and os.name != "nt" and not os.environ.get("DISPLAY"))
)
if HEADLESS_REQUEST not in ("auto", "true", "false"):
    raise ValueError("HEADLESS must be auto, true, or false")

ENABLE_ACTUATORS = env_bool("ENABLE_ACTUATORS", "false")
ACTUATOR_WATCHDOG_SECONDS = env_float("ACTUATOR_WATCHDOG_SECONDS", "0.25")
ACTUATOR_STARTUP_TIMEOUT_SECONDS = env_float("ACTUATOR_STARTUP_TIMEOUT_SECONDS", "3.0")
FATAL_ERROR_LOG = os.environ.get("FATAL_ERROR_LOG", "/tmp/bobot-fatal.log")
THROTTLE_ALLOW_REVERSE = env_bool("THROTTLE_ALLOW_REVERSE", "false")
THROTTLE_NEUTRAL_US = env_int("THROTTLE_NEUTRAL_US", "1500")
THROTTLE_FORWARD_US = env_int("THROTTLE_FORWARD_US", "1600")
THROTTLE_REVERSE_US = env_int("THROTTLE_REVERSE_US", "1400")
THROTTLE_LIMIT = env_float("THROTTLE_LIMIT", "1.0")
if not 0.0 <= THROTTLE_LIMIT <= 1.0:
    raise ValueError("THROTTLE_LIMIT must be between 0.0 and 1.0")
THROTTLE_LIMIT_STEP = env_float("THROTTLE_LIMIT_STEP", "0.05")
if not 0.0 < THROTTLE_LIMIT_STEP <= 1.0:
    raise ValueError("THROTTLE_LIMIT_STEP must be greater than zero and at most 1.0")


def adjust_throttle_limit(direction):
    """Apply one D-pad adjustment and return the updated runtime limit."""
    global THROTTLE_LIMIT
    THROTTLE_LIMIT = min(1.0, max(0.0, THROTTLE_LIMIT + direction * THROTTLE_LIMIT_STEP))
    return THROTTLE_LIMIT


THROTTLE_MIN_ACTIVE = env_float("THROTTLE_MIN_ACTIVE", "0.06")
MAX_TRIAL_THROTTLE = env_float("MAX_TRIAL_THROTTLE", "0.10")
MOTOR_STEERING_MIX = env_float("MOTOR_STEERING_MIX", "0.08")
MOTOR_OUTPUTS = (
    ("front_left", env_int("MOTOR_FRONT_LEFT_CHANNEL", "0"), env_float("MOTOR_FRONT_LEFT_SIGN", "1")),
    ("front_right", env_int("MOTOR_FRONT_RIGHT_CHANNEL", "1"), env_float("MOTOR_FRONT_RIGHT_SIGN", "1")),
    ("rear_left", env_int("MOTOR_REAR_LEFT_CHANNEL", "2"), env_float("MOTOR_REAR_LEFT_SIGN", "1")),
    ("rear_right", env_int("MOTOR_REAR_RIGHT_CHANNEL", "3"), env_float("MOTOR_REAR_RIGHT_SIGN", "1")),
)
SHARED_ESC_US = (THROTTLE_REVERSE_US, THROTTLE_NEUTRAL_US, THROTTLE_FORWARD_US)
MOTOR_ESC_US = {
    "front_left": env_pulse_triplet("MOTOR_FRONT_LEFT_ESC_US", SHARED_ESC_US),
    "front_right": env_pulse_triplet("MOTOR_FRONT_RIGHT_ESC_US", SHARED_ESC_US),
    "rear_left": env_pulse_triplet("MOTOR_REAR_LEFT_ESC_US", SHARED_ESC_US),
    "rear_right": env_pulse_triplet("MOTOR_REAR_RIGHT_ESC_US", SHARED_ESC_US),
}

TUI_ENABLED = env_bool("TUI", "true")
TUI_INTERVAL = env_float("TUI_INTERVAL", "0.10")
TELEMETRY_INTERVAL = env_float("TELEMETRY_INTERVAL", "0.25")

AUTO_CALIBRATE = env_bool("AUTO_CALIBRATE", "true")
AUTO_CALIBRATION_INTERVAL = env_float("AUTO_CALIBRATION_INTERVAL", "1.0")
AUTO_CALIBRATION_MAX_COLORS = env_int("AUTO_CALIBRATION_MAX_COLORS", "8")
AUTO_CALIBRATION_MIN_AREA_RATIO = env_float("AUTO_CALIBRATION_MIN_AREA_RATIO", "0.006")
AUTO_CALIBRATION_SATURATION_MIN = env_int("AUTO_CALIBRATION_SATURATION_MIN", "35")
AUTO_CALIBRATION_VALUE_MIN = env_int("AUTO_CALIBRATION_VALUE_MIN", "70")
AUTO_CALIBRATION_HUE_PADDING = env_int("AUTO_CALIBRATION_HUE_PADDING", "8")
AUTO_CALIBRATION_SATURATION_PADDING = env_int("AUTO_CALIBRATION_SATURATION_PADDING", "45")
AUTO_CALIBRATION_VALUE_PADDING = env_int("AUTO_CALIBRATION_VALUE_PADDING", "45")
AUTO_CALIBRATION_MERGE_HUE_DISTANCE = env_int("AUTO_CALIBRATION_MERGE_HUE_DISTANCE", "10")
KNOWN_COLOR_HUE_PADDING = env_int("KNOWN_COLOR_HUE_PADDING", "12")
KNOWN_COLOR_SATURATION_MIN = env_int("KNOWN_COLOR_SATURATION_MIN", "45")
KNOWN_COLOR_VALUE_MIN = env_int("KNOWN_COLOR_VALUE_MIN", "60")
MIN_BALL_AREA_RATIO = env_float("MIN_BALL_AREA_RATIO", "0.004")
MIN_BALL_AREA_TOP_SCALE = env_float("MIN_BALL_AREA_TOP_SCALE", "0.25")
MAX_BALL_AREA_RATIO = env_float("MAX_BALL_AREA_RATIO", "0.15")
MAX_BALL_AREA_TOP_SCALE = env_float("MAX_BALL_AREA_TOP_SCALE", "1.0")
MIN_BALL_CIRCULARITY = env_float("MIN_BALL_CIRCULARITY", "0.48")
MIN_BALL_CIRCLE_FILL = env_float("MIN_BALL_CIRCLE_FILL", "0.50")
TRIANGLE_APPROX_EPSILON = env_float("TRIANGLE_APPROX_EPSILON", "0.04")

CONE_HUE_MIN = env_int("CONE_HUE_MIN", "3")
CONE_HUE_MAX = env_int("CONE_HUE_MAX", "22")
CONE_SATURATION_MIN = env_int("CONE_SATURATION_MIN", "120")
CONE_VALUE_MIN = env_int("CONE_VALUE_MIN", "80")
CONE_VALUE_MAX = env_int("CONE_VALUE_MAX", "245")

OBJECT_SATURATION_MIN = env_int("OBJECT_SATURATION_MIN", "32")
OBJECT_CHROMA_MIN = env_float("OBJECT_CHROMA_MIN", "20")
OBJECT_LOCAL_CHROMA_DELTA = env_float("OBJECT_LOCAL_CHROMA_DELTA", "10")
OBJECT_LOCAL_BLUR_SIGMA = env_float("OBJECT_LOCAL_BLUR_SIGMA", "9")
OBJECT_VALUE_MIN = env_int("OBJECT_VALUE_MIN", "35")
OBJECT_VALUE_MAX = env_int("OBJECT_VALUE_MAX", "255")
OBJECT_MIN_AREA_RATIO = env_float("OBJECT_MIN_AREA_RATIO", "0.00035")
OBJECT_MAX_AREA_RATIO = env_float("OBJECT_MAX_AREA_RATIO", "0.22")
OBJECT_UNCERTAIN_SCORE = env_float("OBJECT_UNCERTAIN_SCORE", "0.42")
OBJECT_CERTAIN_SCORE = env_float("OBJECT_CERTAIN_SCORE", "0.67")
OBJECT_CLASS_MARGIN = env_float("OBJECT_CLASS_MARGIN", "0.09")
OBJECT_CERTAIN_MARGIN = env_float("OBJECT_CERTAIN_MARGIN", "0.15")

TARGET_DISTANCE_WEIGHT = env_float("TARGET_DISTANCE_WEIGHT", "0.75")
TARGET_CLUSTER_WEIGHT = env_float("TARGET_CLUSTER_WEIGHT", "0.25")
TARGET_AREA_WEIGHT = env_float("TARGET_AREA_WEIGHT", "0.08")
TARGET_CENTER_WEIGHT = env_float("TARGET_CENTER_WEIGHT", "0.03")
TARGET_CLUSTER_RADIUS_RATIO = env_float("TARGET_CLUSTER_RADIUS_RATIO", "0.22")
TARGET_LOCK_RADIUS_RATIO = env_float("TARGET_LOCK_RADIUS_RATIO", "0.18")
TARGET_SWITCH_MARGIN = env_float("TARGET_SWITCH_MARGIN", "0.18")
TARGET_SWITCH_FRAMES = env_int("TARGET_SWITCH_FRAMES", "3")
TARGET_HOLD_SECONDS = env_float("TARGET_HOLD_SECONDS", "0.35")
TARGET_SMOOTHING = env_float("TARGET_SMOOTHING", "0.35")
TRACK_CONFIRM_FRAMES = env_int("TRACK_CONFIRM_FRAMES", "3")
TRACK_MAX_MISSES = env_int("TRACK_MAX_MISSES", "5")
TRACK_MATCH_RADIUS_RATIO = env_float("TRACK_MATCH_RADIUS_RATIO", "0.14")
TRACK_SCORE_SMOOTHING = env_float("TRACK_SCORE_SMOOTHING", "0.40")
TRACK_POSITION_SMOOTHING = env_float("TRACK_POSITION_SMOOTHING", "0.58")
TRACK_VELOCITY_SMOOTHING = env_float("TRACK_VELOCITY_SMOOTHING", "0.45")
TRACK_COLOR_WEIGHT = env_float("TRACK_COLOR_WEIGHT", "0.20")
TRACK_SIZE_WEIGHT = env_float("TRACK_SIZE_WEIGHT", "0.18")
CAMERA_HORIZONTAL_FOV_DEG = env_float("CAMERA_HORIZONTAL_FOV_DEG", "66.0")
CAMERA_VERTICAL_FOV_DEG = env_float("CAMERA_VERTICAL_FOV_DEG", "41.0")
IMU_TRACK_YAW_SIGN = env_float("IMU_TRACK_YAW_SIGN", "-1.0")
IMU_TRACK_PITCH_SIGN = env_float("IMU_TRACK_PITCH_SIGN", "1.0")
IMU_TRACK_ROLL_SIGN = env_float("IMU_TRACK_ROLL_SIGN", "1.0")

IMU_ENABLED = env_bool("IMU_ENABLED", "true")
IMU_I2C_ADDRESS = env_auto_int("IMU_I2C_ADDRESS", "0x4b")
IMU_REPORT_INTERVAL_US = env_int("IMU_REPORT_INTERVAL_US", "50000")
IMU_ROTATION_MODE = os.environ.get("IMU_ROTATION_MODE", "game").lower()
if IMU_ROTATION_MODE not in ("game", "absolute"):
    raise ValueError("IMU_ROTATION_MODE must be game or absolute")
IMU_MAX_AGE_SECONDS = env_float("IMU_MAX_AGE_SECONDS", "0.50")
IMU_RECONNECT_INTERVAL = env_float("IMU_RECONNECT_INTERVAL", "2.0")
IMU_TUI_TILT_RANGE_DEG = env_float("IMU_TUI_TILT_RANGE_DEG", "35.0")
HORIZON_BASE_Y_RATIO = env_float("HORIZON_BASE_Y_RATIO", "0.34")
HORIZON_BALL_ALLOWANCE_RATIO = env_float("HORIZON_BALL_ALLOWANCE_RATIO", "0.035")
HORIZON_CONE_ALLOWANCE_RATIO = env_float("HORIZON_CONE_ALLOWANCE_RATIO", "0.14")
HORIZON_PITCH_SIGN = env_float("HORIZON_PITCH_SIGN", "1.0")
HORIZON_ROLL_SIGN = env_float("HORIZON_ROLL_SIGN", "1.0")
OVERLAY_MOTION_SCALE = env_float("OVERLAY_MOTION_SCALE", "3.0")

STEERING_SLEW_RATE = env_float("STEERING_SLEW_RATE", "2.0")
STEERING_GAIN = env_float("STEERING_GAIN", "1.25")
STEERING_DEADBAND = env_float("STEERING_DEADBAND", "0.06")
CLOSE_BALL_AREA_RATIO = env_float("CLOSE_BALL_AREA_RATIO", "0.18")
LOST_TARGET_TIMEOUT = env_float("LOST_TARGET_TIMEOUT", "0.5")
ROBOT_START_STATE = os.environ.get(
    "ROBOT_START_STATE",
    os.environ.get("START_STATE", "static"),
).lower()

# Linux gamepad configuration. Defaults follow the common evdev Xbox-style map.
CONTROLLER_DEVICE = os.environ.get("CONTROLLER_DEVICE", "auto")
CONTROLLER_A_BUTTON = env_int(
    "CONTROLLER_A_BUTTON",
    os.environ.get("CONTROLLER_THROTTLE_ENABLE_BUTTON", "304"),
)
CONTROLLER_B_BUTTON = env_int("CONTROLLER_B_BUTTON", "305")
CONTROLLER_Y_BUTTON = env_int("CONTROLLER_Y_BUTTON", "307")
CONTROLLER_LEFT_X_AXIS = env_int("CONTROLLER_LEFT_X_AXIS", "0")
CONTROLLER_LEFT_Y_AXIS = env_int("CONTROLLER_LEFT_Y_AXIS", "1")
CONTROLLER_RIGHT_X_AXIS = env_int("CONTROLLER_RIGHT_X_AXIS", "3")
CONTROLLER_RIGHT_Y_AXIS = env_int("CONTROLLER_RIGHT_Y_AXIS", "4")
CONTROLLER_DPAD_Y_AXIS = env_int("CONTROLLER_DPAD_Y_AXIS", "17")
CONTROLLER_DPAD_UP_BUTTON = env_int("CONTROLLER_DPAD_UP_BUTTON", "544")
CONTROLLER_DPAD_DOWN_BUTTON = env_int("CONTROLLER_DPAD_DOWN_BUTTON", "545")
CONTROLLER_DEADZONE = env_float("CONTROLLER_DEADZONE", "0.10")
CONTROLLER_MENU_DEADZONE = env_float("CONTROLLER_MENU_DEADZONE", "0.35")
CONTROLLER_INVERT_Y = env_bool("CONTROLLER_INVERT_Y", "true")
