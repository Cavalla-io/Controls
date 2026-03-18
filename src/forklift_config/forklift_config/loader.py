"""
Load controls.toml and build typed config objects.
Path: CONTROLS_CONFIG_PATH env, or REPO_ROOT/system/config/controls.toml, else error.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any

try:
    import tomllib
except ImportError:
    tomllib = None  # type: ignore[assignment]

if tomllib is None:
    try:
        import toml as _toml_pkg
    except ImportError as e:
        raise ImportError(
            "TOML support requires Python 3.11+ (tomllib) or the 'toml' package. "
            "Install with: pip install toml"
        ) from e


def _load_toml(path: str) -> dict[str, Any]:
    """Load TOML file. Uses tomllib (3.11+) or toml package."""
    if tomllib is not None:
        with open(path, "rb") as f:
            return tomllib.load(f)
    return _toml_pkg.load(path)


def _resolve_config_path(path: str | None) -> str:
    """Resolve path to controls.toml. Raises FileNotFoundError if not found."""
    if path:
        if os.path.isabs(path) and os.path.isfile(path):
            return path
        if os.path.isfile(path):
            return os.path.abspath(path)
        raise FileNotFoundError(f"Controls config file not found: {path}")

    env_path = os.environ.get("CONTROLS_CONFIG_PATH")
    if env_path:
        if os.path.isfile(env_path):
            return os.path.abspath(env_path)
        raise FileNotFoundError(f"CONTROLS_CONFIG_PATH points to missing file: {env_path}")

    repo_root = os.environ.get("REPO_ROOT")
    if repo_root:
        default = os.path.join(repo_root, "system", "config", "controls.toml")
        if os.path.isfile(default):
            return os.path.abspath(default)

    raise FileNotFoundError(
        "Controls config not found. Set CONTROLS_CONFIG_PATH to system/config/controls.toml "
        "or set REPO_ROOT to the repo root."
    )


# ---------------------------------------------------------------------------
# Dataclasses (immutable config views)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class HardwareCanConfig:
    channel: str
    bitrate: int
    is_mock: bool


@dataclass(frozen=True)
class MBV15Config:
    node_id: int
    drive_deadband: float
    lift_lower_threshold: float
    aux_threshold: float
    max_drive_rpm: int
    max_steer_can: int
    lowering_pwm_min: int
    lowering_pwm_max: int
    pump_rpm_base: int
    pump_rpm_scale: int
    pump_rpm_aux: int
    pump_rpm_max: int


@dataclass(frozen=True)
class DriverRosTopics:
    safe_raw_command: str
    set_preset: str
    fork_position: str


@dataclass(frozen=True)
class DriverFailsafeConfig:
    drive_scale: float
    steer_scale: float
    lift_scale: float
    lower_scale: float
    allow_fork_movement: bool
    max_height_mm: float
    min_height_mm: float
    accel_time_s: float
    decel_time_s: float


@dataclass(frozen=True)
class DriverConfig:
    default_preset: str
    presets_file: str
    presets: dict[str, dict[str, Any]]
    ros_topics: DriverRosTopics
    failsafe: DriverFailsafeConfig


@dataclass(frozen=True)
class SafetyRosTopics:
    teleop_raw_command: str
    safety: str
    auto_lift_effort: str
    target_fork_height: str
    safe_raw_command: str


@dataclass(frozen=True)
class SafetyConfig:
    heartbeat_timeout_sec: float
    command_timeout_sec: float
    auto_timeout_sec: float
    teleop_priority_timeout_sec: float
    teleop_activity_threshold: float
    watchdog_period_sec: float
    unsafe_status_codes: tuple[int, ...]
    ros_topics: SafetyRosTopics


@dataclass(frozen=True)
class ForkHeightPidConfig:
    kp: float
    ki: float
    kd: float
    deadband_mm: float
    integral_max: float


@dataclass(frozen=True)
class ForkHeightRosTopics:
    fork_position: str
    target_fork_height: str
    auto_lift_effort: str


@dataclass(frozen=True)
class ForkHeightConfig:
    target_timeout_sec: float
    control_loop_period_sec: float
    pid: ForkHeightPidConfig
    ros_topics: ForkHeightRosTopics


@dataclass(frozen=True)
class JoyMappingConfig:
    gear_button: int
    throttle_axis: int
    steer_axis: int
    lift_axis: int
    tilt_up_button: int
    tilt_down_button: int
    shift_left_button: int
    shift_right_button: int


@dataclass(frozen=True)
class TeleopRosTopics:
    joy: str
    raw_command: str


@dataclass(frozen=True)
class TeleopConfig:
    lift_deadband: float
    joy_mapping: JoyMappingConfig
    ros_topics: TeleopRosTopics


@dataclass(frozen=True)
class HardwareConfig:
    can: HardwareCanConfig
    mbv15: MBV15Config


@dataclass(frozen=True)
class ControlsConfig:
    config_path: str
    hardware: HardwareConfig
    driver: DriverConfig
    safety: SafetyConfig
    fork_height: ForkHeightConfig
    teleop: TeleopConfig


def _get(d: dict[str, Any], key: str, default: Any) -> Any:
    if key in d and d[key] is not None:
        return d[key]
    return default


def _build_hardware(raw: dict[str, Any]) -> HardwareConfig:
    can = raw.get("can") or {}
    mbv15_raw = raw.get("mbv15") or {}
    return HardwareConfig(
        can=HardwareCanConfig(
            channel=_get(can, "channel", "can0"),
            bitrate=int(_get(can, "bitrate", 250000)),
            is_mock=bool(_get(can, "is_mock", False)),
        ),
        mbv15=MBV15Config(
            node_id=int(_get(mbv15_raw, "node_id", 3)),
            drive_deadband=float(_get(mbv15_raw, "drive_deadband", 0.02)),
            lift_lower_threshold=float(_get(mbv15_raw, "lift_lower_threshold", 0.05)),
            aux_threshold=float(_get(mbv15_raw, "aux_threshold", 0.5)),
            max_drive_rpm=int(_get(mbv15_raw, "max_drive_rpm", 4000)),
            max_steer_can=int(_get(mbv15_raw, "max_steer_can", 9000)),
            lowering_pwm_min=int(_get(mbv15_raw, "lowering_pwm_min", 40)),
            lowering_pwm_max=int(_get(mbv15_raw, "lowering_pwm_max", 200)),
            pump_rpm_base=int(_get(mbv15_raw, "pump_rpm_base", 1000)),
            pump_rpm_scale=int(_get(mbv15_raw, "pump_rpm_scale", 2500)),
            pump_rpm_aux=int(_get(mbv15_raw, "pump_rpm_aux", 2000)),
            pump_rpm_max=int(_get(mbv15_raw, "pump_rpm_max", 5000)),
        ),
    )


def _build_preset_dict(preset_raw: dict[str, Any]) -> dict[str, Any]:
    """Build one preset dict (same keys as failsafe) from a TOML preset table."""
    return {
        "drive_scale": float(_get(preset_raw, "drive_scale", 0.2)),
        "steer_scale": float(_get(preset_raw, "steer_scale", 0.5)),
        "lift_scale": float(_get(preset_raw, "lift_scale", 0.2)),
        "lower_scale": float(_get(preset_raw, "lower_scale", 0.2)),
        "allow_fork_movement": bool(_get(preset_raw, "allow_fork_movement", False)),
        "max_height_mm": float(_get(preset_raw, "max_height_mm", 0)),
        "min_height_mm": float(_get(preset_raw, "min_height_mm", 0)),
        "accel_time_s": float(_get(preset_raw, "accel_time_s", 1.0)),
        "decel_time_s": float(_get(preset_raw, "decel_time_s", 1.0)),
    }


def _build_driver(raw: dict[str, Any]) -> DriverConfig:
    topics = raw.get("ros_topics") or {}
    failsafe = raw.get("failsafe") or {}
    presets_raw = raw.get("presets") or {}
    presets: dict[str, dict[str, Any]] = {}
    for name, table in presets_raw.items():
        if isinstance(table, dict):
            presets[str(name)] = _build_preset_dict(table)
    return DriverConfig(
        default_preset=_get(raw, "default_preset", "default"),
        presets_file=_get(raw, "presets_file", "") or "",
        presets=presets,
        ros_topics=DriverRosTopics(
            safe_raw_command=_get(topics, "safe_raw_command", "/safe/raw_command"),
            set_preset=_get(topics, "set_preset", "/forklift/set_preset"),
            fork_position=_get(topics, "fork_position", "/fork_position"),
        ),
        failsafe=DriverFailsafeConfig(
            drive_scale=float(_get(failsafe, "drive_scale", 0.2)),
            steer_scale=float(_get(failsafe, "steer_scale", 0.5)),
            lift_scale=float(_get(failsafe, "lift_scale", 0.2)),
            lower_scale=float(_get(failsafe, "lower_scale", 0.2)),
            allow_fork_movement=bool(_get(failsafe, "allow_fork_movement", False)),
            max_height_mm=float(_get(failsafe, "max_height_mm", 0)),
            min_height_mm=float(_get(failsafe, "min_height_mm", 0)),
            accel_time_s=float(_get(failsafe, "accel_time_s", 1.0)),
            decel_time_s=float(_get(failsafe, "decel_time_s", 1.0)),
        ),
    )


def _build_safety(raw: dict[str, Any]) -> SafetyConfig:
    topics = raw.get("ros_topics") or {}
    raw_codes = raw.get("unsafe_status_codes")
    if raw_codes is None:
        unsafe_status_codes: tuple[int, ...] = (1, 3)
    else:
        unsafe_status_codes = tuple(int(x) for x in raw_codes)
    return SafetyConfig(
        heartbeat_timeout_sec=float(_get(raw, "heartbeat_timeout_sec", 0.75)),
        command_timeout_sec=float(_get(raw, "command_timeout_sec", 0.5)),
        auto_timeout_sec=float(_get(raw, "auto_timeout_sec", 0.5)),
        teleop_priority_timeout_sec=float(_get(raw, "teleop_priority_timeout_sec", 3.0)),
        teleop_activity_threshold=float(_get(raw, "teleop_activity_threshold", 0.01)),
        watchdog_period_sec=float(_get(raw, "watchdog_period_sec", 0.1)),
        unsafe_status_codes=unsafe_status_codes,
        ros_topics=SafetyRosTopics(
            teleop_raw_command=_get(topics, "teleop_raw_command", "/teleop/raw_command"),
            safety=_get(topics, "safety", "/safety"),
            auto_lift_effort=_get(topics, "auto_lift_effort", "/forklift/auto_lift_effort"),
            target_fork_height=_get(topics, "target_fork_height", "/forklift/target_fork_height"),
            safe_raw_command=_get(topics, "safe_raw_command", "/safe/raw_command"),
        ),
    )


def _build_fork_height(raw: dict[str, Any]) -> ForkHeightConfig:
    pid = raw.get("pid") or {}
    topics = raw.get("ros_topics") or {}
    return ForkHeightConfig(
        target_timeout_sec=float(_get(raw, "target_timeout_sec", 0.5)),
        control_loop_period_sec=float(_get(raw, "control_loop_period_sec", 0.02)),
        pid=ForkHeightPidConfig(
            kp=float(_get(pid, "kp", 0.006)),
            ki=float(_get(pid, "ki", 0.001)),
            kd=float(_get(pid, "kd", 0.0001)),
            deadband_mm=float(_get(pid, "deadband_mm", 5.0)),
            integral_max=float(_get(pid, "integral_max", 500.0)),
        ),
        ros_topics=ForkHeightRosTopics(
            fork_position=_get(topics, "fork_position", "/fork_position"),
            target_fork_height=_get(topics, "target_fork_height", "/forklift/target_fork_height"),
            auto_lift_effort=_get(topics, "auto_lift_effort", "/forklift/auto_lift_effort"),
        ),
    )


def _build_teleop(raw: dict[str, Any]) -> TeleopConfig:
    joy = raw.get("joy_mapping") or {}
    topics = raw.get("ros_topics") or {}
    return TeleopConfig(
        lift_deadband=float(_get(raw, "lift_deadband", 0.1)),
        joy_mapping=JoyMappingConfig(
            gear_button=int(_get(joy, "gear_button", 0)),
            throttle_axis=int(_get(joy, "throttle_axis", 5)),
            steer_axis=int(_get(joy, "steer_axis", 0)),
            lift_axis=int(_get(joy, "lift_axis", 3)),
            tilt_up_button=int(_get(joy, "tilt_up_button", 13)),
            tilt_down_button=int(_get(joy, "tilt_down_button", 12)),
            shift_left_button=int(_get(joy, "shift_left_button", 14)),
            shift_right_button=int(_get(joy, "shift_right_button", 15)),
        ),
        ros_topics=TeleopRosTopics(
            joy=_get(topics, "joy", "/joy"),
            raw_command=_get(topics, "raw_command", "/teleop/raw_command"),
        ),
    )


def load_controls_config(path: str | None = None) -> ControlsConfig:
    """
    Load controls.toml and return typed ControlsConfig.
    Path: optional override; else CONTROLS_CONFIG_PATH; else REPO_ROOT/system/config/controls.toml.
    """
    resolved = _resolve_config_path(path)
    data = _load_toml(resolved)
    hw = data.get("hardware") or {}
    return ControlsConfig(
        config_path=resolved,
        hardware=_build_hardware(hw),
        driver=_build_driver(data.get("driver") or {}),
        safety=_build_safety(data.get("safety") or {}),
        fork_height=_build_fork_height(data.get("fork_height") or {}),
        teleop=_build_teleop(data.get("teleop") or {}),
    )
