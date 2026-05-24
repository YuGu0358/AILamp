"""Inject one MJCF <key> per motion CSV into ailamp_robot.xml.

After running this, `ailamp sim-viewer` lists each motion as a named keyframe and
they show up in the mujoco simulator's keyframe dropdown for instant pose recall.
"""
from __future__ import annotations

import csv
import math
import re
from pathlib import Path

import mujoco


PROJECT = Path("/sessions/sweet-blissful-dijkstra/mnt/AILamp_for_Claude")
SCENE = PROJECT / "simulation" / "ailamp_scene.xml"
ROBOT_XML = PROJECT / "simulation" / "ailamp_robot.xml"
RECORDINGS_DIR = PROJECT / "ailamp_runtime" / "ailamp" / "recordings"


CSV_TO_JOINT = {
    "base_pitch.pos": "2",
    "base_yaw.pos": "1",
    "elbow_pitch.pos": "3",
    "wrist_roll.pos": "4",
    "wrist_pitch.pos": "5",
}


def load_row(csv_path: Path, fraction: float) -> dict[str, float]:
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    idx = min(len(rows) - 1, max(0, int(len(rows) * fraction)))
    return {k: float(v) for k, v in rows[idx].items() if k != "timestamp"}


def compute_qpos(model, joint_angles_deg: dict[str, float]) -> list[float]:
    """Build a qpos vector for the model with the given joint angles (degrees → radians)."""
    qpos = [0.0] * model.nq
    # Free joint anchor.
    if model.nq >= 7:
        qpos[0:7] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    for csv_name, joint_name in CSV_TO_JOINT.items():
        if csv_name not in joint_angles_deg:
            continue
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            continue
        adr = model.jnt_qposadr[jid]
        low, high = model.jnt_range[jid]
        value_rad = math.radians(joint_angles_deg[csv_name])
        qpos[adr] = max(low, min(high, value_rad))
    # Anchor virtual target out of frame so it doesn't collide with anything visually.
    for j_name, value in (("target_slide_x", 0.0), ("target_slide_y", -1.5)):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
        if jid >= 0:
            adr = model.jnt_qposadr[jid]
            qpos[adr] = value
    return qpos


def format_qpos(qpos: list[float]) -> str:
    return " ".join(f"{v:.6g}" for v in qpos)


def main():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    print(f"Scene loaded — nq={model.nq}")

    csv_files = sorted(RECORDINGS_DIR.glob("*.csv"))
    keys: list[tuple[str, list[float]]] = []
    for csv_path in csv_files:
        name = csv_path.stem
        # Start frame
        start = load_row(csv_path, fraction=0.0)
        keys.append((f"{name}_start", compute_qpos(model, start)))
        # Peak / middle frame
        mid = load_row(csv_path, fraction=0.5)
        keys.append((f"{name}_peak", compute_qpos(model, mid)))
        # End frame
        end = load_row(csv_path, fraction=1.0)
        keys.append((f"{name}_end", compute_qpos(model, end)))
        print(f"  {name}: 3 keyframes (start, peak, end)")

    keyframe_lines = ["  <keyframe>"]
    for name, qpos in keys:
        keyframe_lines.append(f'    <key name="{name}" qpos="{format_qpos(qpos)}" />')
    keyframe_lines.append("  </keyframe>")
    keyframe_xml = "\n".join(keyframe_lines)

    # Read current robot.xml and inject before </mujoco>.
    xml_text = ROBOT_XML.read_text()
    # Remove any pre-existing keyframe block (so this script is idempotent).
    xml_text = re.sub(r"\s*<keyframe>.*?</keyframe>\s*\n", "\n", xml_text, count=1, flags=re.DOTALL)
    new_text = xml_text.replace("</mujoco>", keyframe_xml + "\n</mujoco>")
    ROBOT_XML.write_text(new_text)
    print(f"injected {len(keys)} keyframes into {ROBOT_XML.name}")


if __name__ == "__main__":
    main()
