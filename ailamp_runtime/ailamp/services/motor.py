from __future__ import annotations

import csv
from pathlib import Path
from typing import Optional


class RecordingStore:
    def __init__(self, recordings_dir: str | Path):
        self.recordings_dir = Path(recordings_dir)

    def list_names(self) -> list[str]:
        if not self.recordings_dir.exists():
            return []
        return sorted(path.stem for path in self.recordings_dir.glob("*.csv"))

    def load(self, name: str) -> list[dict[str, float]]:
        path = self.recordings_dir / f"{name}.csv"
        if not path.exists():
            raise FileNotFoundError(path)

        rows: list[dict[str, float]] = []
        with path.open(newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                rows.append({key: float(value) for key, value in row.items() if key != "timestamp"})
        return rows


class MotorService:
    def __init__(self, port: str, lamp_id: str, recordings_dir: str | Path):
        self.port = port
        self.lamp_id = lamp_id
        self.recordings = RecordingStore(recordings_dir)
        self._animation_service: Optional[object] = None

    def connect(self) -> None:
        try:
            from lelamp.service.motors.animation_service import AnimationService  # type: ignore
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Physical ST3215 playback needs the upstream LeLamp runtime. "
                "Clone https://github.com/humancomputerlab/lelamp_runtime next to AILamp "
                "and install it with `python3 -m pip install -e ../lelamp_runtime` on the Jetson."
            ) from exc

        self._animation_service = AnimationService(port=self.port, lamp_id=self.lamp_id)
        self._animation_service.start()

    def close(self) -> None:
        if self._animation_service is not None:
            self._animation_service.stop()
            self._animation_service = None

    def play(self, recording_name: str) -> None:
        if self._animation_service is None:
            raise RuntimeError("MotorService is not connected")
        self._animation_service.dispatch("play", recording_name)
