from dataclasses import dataclass, field


@dataclass
class VisionTarget:
    label: str
    center_x: int
    center_y: int
    area: float
    confidence: float
    box: tuple
    radius: int
    color: tuple


@dataclass
class ConeDetection:
    label: str
    center_x: int
    center_y: int
    area: float
    confidence: float
    box: tuple
    contour: object
    color: tuple = (255, 0, 255)


@dataclass
class DriveCommand:
    steering: float = 0.0
    throttle: float = 0.0
    mode: str = "disabled"
    reason: str = "idle"
    # Direct side commands are used only by the manual tank-drive state.
    # When absent, actuator output is derived from steering and throttle.
    left: float | None = None
    right: float | None = None


@dataclass
class DetectionDebug:
    active_colors: int = 0
    masks_checked: int = 0
    contours_seen: int = 0
    rejected_small: int = 0
    rejected_large: int = 0
    rejected_shape: int = 0
    rejected_samples: int = 0
    rejected_overlap: int = 0
    accepted: int = 0
    cones: int = 0
    auto_candidates: int = 0
    auto_profiles: int = 0
    priority_score: float = 0.0
    priority_distance: float = 0.0
    priority_cluster: float = 0.0
    priority_area: float = 0.0
    priority_center: float = 0.0
    priority_neighbors: int = 0
    raw_target_count: int = 0
    stable_target_locked: bool = False
    stable_target_held: bool = False
    stable_target_label: str = "none"
    stable_target_age: float = 0.0
    switch_candidate_frames: int = 0
    raw_steering: float = 0.0
    smoothed_steering: float = 0.0
    steering_limited: bool = False


@dataclass
class StateResult:
    targets: list = field(default_factory=list)
    cones: list = field(default_factory=list)
    best_target: VisionTarget | None = None
    command: DriveCommand = field(default_factory=DriveCommand)
    debug: DetectionDebug = field(default_factory=DetectionDebug)
    auto_color_names: list = field(default_factory=list)
    state_lines: list = field(default_factory=list)
