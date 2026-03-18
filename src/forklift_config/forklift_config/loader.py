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


def _require(d: dict[str, Any], key: str, section: str) -> Any:
    """Require key to be present and non-None. Raises ValueError if missing."""
    if key not in d or d[key] is None:
        raise ValueError(
            f"Missing required key '{key}' in [{section}]. "
            "Ensure controls.toml defines all required keys."
        )
    return d[key]


def _build_hardware(raw: dict[str, Any]) -> HardwareConfig:
    can = raw.get("can")
    if can is None or not isinstance(can, dict):
        raise ValueError(
            "Missing required section [hardware.can]. "
            "Ensure controls.toml defines all required sections."
        )
    mbv15_raw = raw.get("mbv15")
    if mbv15_raw is None or not isinstance(mbv15_raw, dict):
        raise ValueError(
            "Missing required section [hardware.mbv15]. "
            "Ensure controls.toml defines all required sections."
        )
    return HardwareConfig(
        can=HardwareCanConfig(
            channel=str(_require(can, "channel", "hardware.can")),
            bitrate=int(_require(can, "bitrate", "hardware.can")),
            is_mock=bool(_require(can, "is_mock", "hardware.can")),
        ),
        mbv15=MBV15Config(
            node_id=int(_require(mbv15_raw, "node_id", "hardware.mbv15")),
            drive_deadband=float(_require(mbv15_raw, "drive_deadband", "hardware.mbv15")),
            lift_lower_threshold=float(_require(mbv15_raw, "lift_lower_threshold", "hardware.mbv15")),
            aux_threshold=float(_require(mbv15_raw, "aux_threshold", "hardware.mbv15")),
            max_drive_rpm=int(_require(mbv15_raw, "max_drive_rpm", "hardware.mbv15")),
            max_steer_can=int(_require(mbv15_raw, "max_steer_can", "hardware.mbv15")),
            lowering_pwm_min=int(_require(mbv15_raw, "lowering_pwm_min", "hardware.mbv15")),
            lowering_pwm_max=int(_require(mbv15_raw, "lowering_pwm_max", "hardware.mbv15")),
            pump_rpm_base=int(_require(mbv15_raw, "pump_rpm_base", "hardware.mbv15")),
            pump_rpm_scale=int(_require(mbv15_raw, "pump_rpm_scale", "hardware.mbv15")),
            pump_rpm_aux=int(_require(mbv15_raw, "pump_rpm_aux", "hardware.mbv15")),
            pump_rpm_max=int(_require(mbv15_raw, "pump_rpm_max", "hardware.mbv15")),
        ),
    )


def _build_preset_dict(preset_raw: dict[str, Any], section: str) -> dict[str, Any]:
    """Build one preset dict (same keys as failsafe) from a TOML preset table."""
    return {
        "drive_scale": float(_require(preset_raw, "drive_scale", section)),
        "steer_scale": float(_require(preset_raw, "steer_scale", section)),
        "lift_scale": float(_require(preset_raw, "lift_scale", section)),
        "lower_scale": float(_require(preset_raw, "lower_scale", section)),
        "allow_fork_movement": bool(_require(preset_raw, "allow_fork_movement", section)),
        "max_height_mm": float(_require(preset_raw, "max_height_mm", section)),
        "min_height_mm": float(_require(preset_raw, "min_height_mm", section)),
        "accel_time_s": float(_require(preset_raw, "accel_time_s", section)),
        "decel_time_s": float(_require(preset_raw, "decel_time_s", section)),
    }


def _build_driver(raw: dict[str, Any]) -> DriverConfig:
    topics = raw.get("ros_topics")
    if topics is None or not isinstance(topics, dict):
        raise ValueError(
            "Missing required section [driver.ros_topics]. "
            "Ensure controls.toml defines all required sections."
        )
    failsafe = raw.get("failsafe")
    if failsafe is None or not isinstance(failsafe, dict):
        raise ValueError(
            "Missing required section [driver.failsafe]. "
            "Ensure controls.toml defines all required sections."
        )
    presets_raw = raw.get("presets") or {}
    presets: dict[str, dict[str, Any]] = {}
    for name, table in presets_raw.items():
        if isinstance(table, dict):
            presets[str(name)] = _build_preset_dict(table, f"driver.presets.{name}")
    return DriverConfig(
        default_preset=str(_require(raw, "default_preset", "driver")),
        presets_file=raw.get("presets_file") or "",
        presets=presets,
        ros_topics=DriverRosTopics(
            safe_raw_command=str(_require(topics, "safe_raw_command", "driver.ros_topics")),
            set_preset=str(_require(topics, "set_preset", "driver.ros_topics")),
            fork_position=str(_require(topics, "fork_position", "driver.ros_topics")),
        ),
        failsafe=DriverFailsafeConfig(
            drive_scale=float(_require(failsafe, "drive_scale", "driver.failsafe")),
            steer_scale=float(_require(failsafe, "steer_scale", "driver.failsafe")),
            lift_scale=float(_require(failsafe, "lift_scale", "driver.failsafe")),
            lower_scale=float(_require(failsafe, "lower_scale", "driver.failsafe")),
            allow_fork_movement=bool(_require(failsafe, "allow_fork_movement", "driver.failsafe")),
            max_height_mm=float(_require(failsafe, "max_height_mm", "driver.failsafe")),
            min_height_mm=float(_require(failsafe, "min_height_mm", "driver.failsafe")),
            accel_time_s=float(_require(failsafe, "accel_time_s", "driver.failsafe")),
            decel_time_s=float(_require(failsafe, "decel_time_s", "driver.failsafe")),
        ),
    )


def _build_safety(raw: dict[str, Any]) -> SafetyConfig:
    topics = raw.get("ros_topics")
    if topics is None or not isinstance(topics, dict):
        raise ValueError(
            "Missing required section [safety.ros_topics]. "
            "Ensure controls.toml defines all required sections."
        )
    raw_codes = _require(raw, "unsafe_status_codes", "safety")
    unsafe_status_codes = tuple(int(x) for x in raw_codes)
    return SafetyConfig(
        heartbeat_timeout_sec=float(_require(raw, "heartbeat_timeout_sec", "safety")),
        command_timeout_sec=float(_require(raw, "command_timeout_sec", "safety")),
        auto_timeout_sec=float(_require(raw, "auto_timeout_sec", "safety")),
        teleop_priority_timeout_sec=float(_require(raw, "teleop_priority_timeout_sec", "safety")),
        teleop_activity_threshold=float(_require(raw, "teleop_activity_threshold", "safety")),
        watchdog_period_sec=float(_require(raw, "watchdog_period_sec", "safety")),
        unsafe_status_codes=unsafe_status_codes,
        ros_topics=SafetyRosTopics(
            teleop_raw_command=str(_require(topics, "teleop_raw_command", "safety.ros_topics")),
            safety=str(_require(topics, "safety", "safety.ros_topics")),
            auto_lift_effort=str(_require(topics, "auto_lift_effort", "safety.ros_topics")),
            target_fork_height=str(_require(topics, "target_fork_height", "safety.ros_topics")),
            safe_raw_command=str(_require(topics, "safe_raw_command", "safety.ros_topics")),
        ),
    )


def _build_fork_height(raw: dict[str, Any]) -> ForkHeightConfig:
    pid = raw.get("pid")
    if pid is None or not isinstance(pid, dict):
        raise ValueError(
            "Missing required section [fork_height.pid]. "
            "Ensure controls.toml defines all required sections."
        )
    topics = raw.get("ros_topics")
    if topics is None or not isinstance(topics, dict):
        raise ValueError(
            "Missing required section [fork_height.ros_topics]. "
            "Ensure controls.toml defines all required sections."
        )
    return ForkHeightConfig(
        target_timeout_sec=float(_require(raw, "target_timeout_sec", "fork_height")),
        control_loop_period_sec=float(_require(raw, "control_loop_period_sec", "fork_height")),
        pid=ForkHeightPidConfig(
            kp=float(_require(pid, "kp", "fork_height.pid")),
            ki=float(_require(pid, "ki", "fork_height.pid")),
            kd=float(_require(pid, "kd", "fork_height.pid")),
            deadband_mm=float(_require(pid, "deadband_mm", "fork_height.pid")),
            integral_max=float(_require(pid, "integral_max", "fork_height.pid")),
        ),
        ros_topics=ForkHeightRosTopics(
            fork_position=str(_require(topics, "fork_position", "fork_height.ros_topics")),
            target_fork_height=str(_require(topics, "target_fork_height", "fork_height.ros_topics")),
            auto_lift_effort=str(_require(topics, "auto_lift_effort", "fork_height.ros_topics")),
        ),
    )


def _build_teleop(raw: dict[str, Any]) -> TeleopConfig:
    joy = raw.get("joy_mapping")
    if joy is None or not isinstance(joy, dict):
        raise ValueError(
            "Missing required section [teleop.joy_mapping]. "
            "Ensure controls.toml defines all required sections."
        )
    topics = raw.get("ros_topics")
    if topics is None or not isinstance(topics, dict):
        raise ValueError(
            "Missing required section [teleop.ros_topics]. "
            "Ensure controls.toml defines all required sections."
        )
    return TeleopConfig(
        lift_deadband=float(_require(raw, "lift_deadband", "teleop")),
        joy_mapping=JoyMappingConfig(
            gear_button=int(_require(joy, "gear_button", "teleop.joy_mapping")),
            throttle_axis=int(_require(joy, "throttle_axis", "teleop.joy_mapping")),
            steer_axis=int(_require(joy, "steer_axis", "teleop.joy_mapping")),
            lift_axis=int(_require(joy, "lift_axis", "teleop.joy_mapping")),
            tilt_up_button=int(_require(joy, "tilt_up_button", "teleop.joy_mapping")),
            tilt_down_button=int(_require(joy, "tilt_down_button", "teleop.joy_mapping")),
            shift_left_button=int(_require(joy, "shift_left_button", "teleop.joy_mapping")),
            shift_right_button=int(_require(joy, "shift_right_button", "teleop.joy_mapping")),
        ),
        ros_topics=TeleopRosTopics(
            joy=str(_require(topics, "joy", "teleop.ros_topics")),
            raw_command=str(_require(topics, "raw_command", "teleop.ros_topics")),
        ),
    )


def load_controls_config(path: str | None = None) -> ControlsConfig:
    """
    Load controls.toml and return typed ControlsConfig.
    Path: optional override; else CONTROLS_CONFIG_PATH; else REPO_ROOT/system/config/controls.toml.
    """
    resolved = _resolve_config_path(path)
    data = _load_toml(resolved)
    for section in ("hardware", "driver", "safety", "fork_height", "teleop"):
        if section not in data or data[section] is None:
            raise ValueError(
                f"Missing required section [{section}]. "
                "Ensure controls.toml defines all required sections."
            )
    return ControlsConfig(
        config_path=resolved,
        hardware=_build_hardware(data["hardware"]),
        driver=_build_driver(data["driver"]),
        safety=_build_safety(data["safety"]),
        fork_height=_build_fork_height(data["fork_height"]),
        teleop=_build_teleop(data["teleop"]),
    )
