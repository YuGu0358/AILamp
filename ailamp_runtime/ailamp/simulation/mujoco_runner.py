from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from ailamp.paths import resolve_project_path
from ailamp.services.motor import RecordingStore


LELAMP_ACTUATOR_ORDER = ["2", "1", "3", "4", "5"]
CSV_TO_ACTUATOR = {
    "base_pitch.pos": "2",
    "base_yaw.pos": "1",
    "elbow_pitch.pos": "3",
    "wrist_roll.pos": "4",
    "wrist_pitch.pos": "5",
}


@dataclass(frozen=True)
class ModelInfo:
    model_path: Path
    nq: int
    nv: int
    nu: int
    joints: list[str]
    actuators: list[str]


class MujocoRunner:
    def __init__(self, model_path: str | Path, lock_freejoint: bool = True):
        self.model_path = resolve_project_path(model_path)
        self.lock_freejoint = lock_freejoint
        self.model = None
        self.data = None

    def load(self) -> None:
        import mujoco  # type: ignore

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        if self.lock_freejoint and self.model.nq >= 7:
            self.data.qpos[0:7] = [0, 0, 0, 1, 0, 0, 0]
            self.data.qvel[0:6] = 0
        mujoco.mj_forward(self.model, self.data)

    def info(self) -> ModelInfo:
        if self.model is None:
            self.load()
        joints = [self.model.joint(i).name for i in range(self.model.njnt)]
        actuators = [self.model.actuator(i).name for i in range(self.model.nu)]
        return ModelInfo(self.model_path, self.model.nq, self.model.nv, self.model.nu, joints, actuators)

    def replay_recording(self, recording_name: str, recordings_dir: str | Path, max_frames: Optional[int] = None) -> int:
        if self.model is None or self.data is None:
            self.load()
        import mujoco  # type: ignore

        store = RecordingStore(resolve_project_path(recordings_dir))
        rows = store.load(recording_name)
        frame_count = 0
        for row in rows[:max_frames]:
            for csv_name, actuator_name in CSV_TO_ACTUATOR.items():
                actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                if actuator_id >= 0:
                    self.data.ctrl[actuator_id] = row[csv_name]
            mujoco.mj_step(self.model, self.data)
            frame_count += 1
        return frame_count

    def render(self, output_path: str | Path, width: int = 1280, height: int = 720) -> Path:
        if self.model is None or self.data is None:
            self.load()
        import mujoco  # type: ignore

        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, height)
        renderer = mujoco.Renderer(self.model, width=width, height=height)
        renderer.update_scene(self.data)
        image = renderer.render()

        from PIL import Image  # type: ignore

        output = resolve_project_path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(image).save(output)
        renderer.close()
        return output

