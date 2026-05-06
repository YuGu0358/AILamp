from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os

from ailamp.config import HardwareConfig
from ailamp.paths import resolve_project_path


@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    detail: str

    def format(self) -> str:
        status = "PASS" if self.passed else "FAIL"
        return f"{status} {self.name}: {self.detail}"


def path_exists(path: str | Path) -> bool:
    return Path(path).exists()


def run_static_hardware_checks(config: HardwareConfig) -> list[CheckResult]:
    model_path = resolve_project_path(config.simulation.model_path)
    recordings_dir = resolve_project_path(config.simulation.recordings_dir)

    return [
        CheckResult("config.project", config.system.project_name == "AILamp", config.system.project_name),
        CheckResult("controller.model", "Jetson Orin Nano Super" in config.controller.model, config.controller.model),
        CheckResult("motor.count", config.motors.servo_quantity == 5, str(config.motors.servo_quantity)),
        CheckResult("motor.ids", set(config.motors.ids.values()) == {1, 2, 3, 4, 5}, str(config.motors.ids)),
        CheckResult("led.count", config.led.count == 64, str(config.led.count)),
        CheckResult("simulation.model", model_path.exists(), str(model_path)),
        CheckResult("recordings.dir", recordings_dir.exists(), str(recordings_dir)),
        CheckResult("recording.wake_up", (recordings_dir / "wake_up.csv").exists(), str(recordings_dir / "wake_up.csv")),
    ]


def run_device_presence_checks(config: HardwareConfig) -> list[CheckResult]:
    return [
        CheckResult("servo.port", os.path.exists(config.motors.port), config.motors.port),
        CheckResult("led.port", os.path.exists(config.led.port), config.led.port),
        CheckResult("camera.device", os.path.exists(config.camera.device_path), config.camera.device_path),
    ]

