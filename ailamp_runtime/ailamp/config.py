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
        simulation=SimulationConfig(**raw["simulation"]),
        hardware_bom={key: BOMItem(**value) for key, value in raw["hardware_bom"].items()},
    )
