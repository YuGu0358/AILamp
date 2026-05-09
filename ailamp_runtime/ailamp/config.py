from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import tomllib

from ailamp.paths import project_root


@dataclass(frozen=True)
class SystemConfig:
    project_name: str
    platform: str


@dataclass(frozen=True)
class ControllerConfig:
    model: str
    mpn: str
    storage: str
    system_card: str


@dataclass(frozen=True)
class MotorConfig:
    port: str
    driver_model: str
    servo_model: str
    servo_quantity: int
    fps: int
    ids: dict[str, int]


@dataclass(frozen=True)
class LEDConfig:
    port: str
    controller: str
    panel: str
    count: int
    baudrate: int
    brightness: int
    power_supply: str
    logic_level_shifter: str


@dataclass(frozen=True)
class PowerConfig:
    servo_supply: str
    led_supply: str
    emergency_switch: str
    barrel_adapter: str
    power_connector: str
    power_wire_red: str
    power_wire_black: str
    signal_wire: str


@dataclass(frozen=True)
class BOMItem:
    part: str
    quantity: str


@dataclass(frozen=True)
class CameraConfig:
    model: str
    device: int
    device_path: str
    width: int
    height: int
    fps: int
    pixel_format: str


@dataclass(frozen=True)
class VisionConfig:
    model: str
    pose_model: str
    pose_enabled: bool
    confidence: float
    left_threshold: float
    right_threshold: float
    close_area_ratio: float
    far_depth_m: float


@dataclass(frozen=True)
class AudioConfig:
    input_model: str
    speaker_model: str
    input_device: str
    output_device: str


@dataclass(frozen=True)
class VoiceConfig:
    provider: str
    transport: str
    enabled: bool


@dataclass(frozen=True)
class RuntimeConfig:
    vision_state_file: str
    vision_interval_s: float
    action_cooldown_s: float


@dataclass(frozen=True)
class BirthdayConfig:
    enabled: bool
    month: int
    day: int
    message: str
    motion: str
    rgb: tuple[int, int, int]
    state_file: str
    speech_command: str


@dataclass(frozen=True)
class SimulationConfig:
    model_path: str
    recordings_dir: str
    lock_freejoint: bool


@dataclass(frozen=True)
class HardwareConfig:
    system: SystemConfig
    controller: ControllerConfig
    power: PowerConfig
    motors: MotorConfig
    led: LEDConfig
    camera: CameraConfig
    vision: VisionConfig
    audio: AudioConfig
    voice: VoiceConfig
    runtime: RuntimeConfig
    birthday: BirthdayConfig
    simulation: SimulationConfig
    hardware_bom: dict[str, BOMItem]


def _resolve_config_path(path: str | Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return project_root() / candidate


def load_hardware_config(path: str | Path) -> HardwareConfig:
    config_path = _resolve_config_path(path)
    with config_path.open("rb") as handle:
        raw: dict[str, Any] = tomllib.load(handle)

    birthday_raw = raw["birthday"]

    return HardwareConfig(
        system=SystemConfig(**raw["system"]),
        controller=ControllerConfig(**raw["controller"]),
        power=PowerConfig(**raw["power"]),
        motors=MotorConfig(**raw["motors"]),
        led=LEDConfig(**raw["led"]),
        camera=CameraConfig(**raw["camera"]),
        vision=VisionConfig(**raw["vision"]),
        audio=AudioConfig(**raw["audio"]),
        voice=VoiceConfig(**raw["voice"]),
        runtime=RuntimeConfig(**raw["runtime"]),
        birthday=BirthdayConfig(
            enabled=birthday_raw["enabled"],
            month=birthday_raw["month"],
            day=birthday_raw["day"],
            message=birthday_raw["message"],
            motion=birthday_raw["motion"],
            rgb=tuple(birthday_raw["rgb"]),
            state_file=birthday_raw["state_file"],
            speech_command=birthday_raw["speech_command"],
        ),
        simulation=SimulationConfig(**raw["simulation"]),
        hardware_bom={key: BOMItem(**value) for key, value in raw["hardware_bom"].items()},
    )
