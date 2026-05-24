"""Render a gallery preview of every motion CSV at its peak / end frame in MuJoCo.

For each recording in ailamp_runtime/ailamp/recordings/*.csv:
  1. Pick a representative frame (midpoint by default — captures the gesture).
  2. Set MuJoCo qpos to those joint angles (CSV degrees → radians).
  3. mj_forward() to compute body world poses.
  4. Render structural meshes via matplotlib 3D (same pipeline as render_mujoco_structural.py).

Output: a 3x4 grid PNG with each motion labeled.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


PROJECT = Path("/sessions/sweet-blissful-dijkstra/mnt/AILamp_for_Claude")
SCENE = PROJECT / "simulation" / "ailamp_scene.xml"
ASSETS = PROJECT / "simulation" / "assets"
RECORDINGS_DIR = PROJECT / "ailamp_runtime" / "ailamp" / "recordings"
OUT_DIR = Path("/sessions/sweet-blissful-dijkstra/mnt/嵌入式台灯/AILamp_README_update/docs/img")
OUT_DIR.mkdir(parents=True, exist_ok=True)


# CSV column → MuJoCo joint name → qpos address (precomputed below).
CSV_TO_JOINT = {
    "base_pitch.pos": "2",
    "base_yaw.pos": "1",
    "elbow_pitch.pos": "3",
    "wrist_roll.pos": "4",
    "wrist_pitch.pos": "5",
}

STRUCTURAL_MESH_PREFIXES = (
    "ailamp_integrated_base_",
    "ailamp_cable_clip_",
    "lamparm__",
    "lamphead",
    "diffuser",
)

MESH_NAME_TO_FILE = {
    "ailamp_integrated_base_shell_mesh":  "ailamp_adapters/AILamp_LampBase_Electronics_Shell.stl",
    "ailamp_integrated_base_cover_mesh":  "ailamp_adapters/AILamp_LampBase_Electronics_Cover.stl",
    "ailamp_cable_clip_6mm_mesh":         "ailamp_adapters/AILamp_Cable_Clip_6mm.stl",
    "ailamp_cable_clip_10mm_mesh":        "ailamp_adapters/AILamp_Cable_Clip_10mm.stl",
}

BODY_COLORS = {
    "ailamp_base_layout_visuals": (0.32, 0.55, 0.85),
    "lamparm__base_elbow":        (0.74, 0.74, 0.74),
    "lamparm__wrist_head":        (0.62, 0.62, 0.62),
    "scs215_v5":                  (0.55, 0.55, 0.55),
    "lamparm__elbow_wrist":       (0.74, 0.74, 0.74),
    "lamparm__wrist_head_2":      (0.62, 0.62, 0.62),
    "diffuser":                   (0.95, 0.85, 0.4),
}
DEFAULT_COLOR = (0.6, 0.6, 0.6)

# Description shown beneath each panel — written by hand so users know what each motion is for.
MOTION_DESCRIPTIONS = {
    "idle":         "Idle — neutral resting pose (looped during long pauses)",
    "scanning":     "Scanning — slow yaw sweep when no person detected",
    "nod":          "Nod — 'hello / acknowledged' gesture",
    "headshake":    "Headshake — disagreement / 'no' gesture",
    "shy":          "Shy — head tilts away and down (PERSON_CLOSE)",
    "curious":      "Curious — head lifts and leans forward (PERSON_FAR)",
    "happy_wiggle": "Happy wiggle — celebration animation (smile detected)",
    "wake_up":      "Wake up — startup sequence from cold",
    "excited":      "Excited — bonus animation, not yet event-mapped",
    "sad":          "Sad — bonus animation, not yet event-mapped",
    "shock":        "Shock — bonus animation, not yet event-mapped",
}


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def find_mesh_file(mesh_name: str) -> Path | None:
    if mesh_name in MESH_NAME_TO_FILE:
        return ASSETS / MESH_NAME_TO_FILE[mesh_name]
    for c in (ASSETS / f"{mesh_name}.stl", ASSETS / "ailamp_adapters" / f"{mesh_name}.stl"):
        if c.exists():
            return c
    matches = list(ASSETS.rglob(f"{mesh_name}.stl"))
    return matches[0] if matches else None


def shade(mesh, rgb, light=(0.4, -0.5, 0.85)):
    normals = mesh.face_normals
    ld = np.array(light, dtype=float); ld /= np.linalg.norm(ld)
    intensity = np.clip(normals @ ld, 0, 1) * 0.6 + 0.4
    colors = np.zeros((len(mesh.faces), 4))
    colors[:, 0:3] = np.array(rgb)[None, :] * intensity[:, None]
    colors[:, 3] = 1.0
    return colors


def load_recording_row(csv_path: Path, fraction: float = 0.5) -> dict[str, float]:
    """Load the joint angles (in degrees) at a fractional point through the recording."""
    with csv_path.open() as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return {}
    idx = min(len(rows) - 1, int(len(rows) * fraction))
    row = rows[idx]
    return {k: float(v) for k, v in row.items() if k != "timestamp"}


def set_pose(model, data, joint_angles_deg: dict[str, float]) -> None:
    """Set qpos from CSV joint angles. Joint angles in CSV are degrees; MuJoCo expects radians."""
    # Always re-anchor the free joint and target.
    if model.nq >= 7:
        data.qpos[0:7] = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    for csv_name, joint_name in CSV_TO_JOINT.items():
        if csv_name not in joint_angles_deg:
            continue
        value_deg = joint_angles_deg[csv_name]
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            continue
        adr = model.jnt_qposadr[jid]
        # Clamp to joint limits to avoid impossible poses.
        low, high = model.jnt_range[jid]
        value_rad = math.radians(value_deg)
        data.qpos[adr] = max(low, min(high, value_rad))
    # Move virtual person target out of frame.
    for j_name, value in (("target_slide_x", 0.0), ("target_slide_y", -3.5)):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
        if jid >= 0:
            adr = model.jnt_qposadr[jid]
            data.qpos[adr] = value
    mujoco.mj_forward(model, data)


def collect_mesh_world_poses(model, data) -> list[tuple[trimesh.Trimesh, tuple[float, float, float]]]:
    items: list[tuple[trimesh.Trimesh, tuple[float, float, float]]] = []
    for gid in range(model.ngeom):
        geom = model.geom(gid)
        if int(geom.type) != int(mujoco.mjtGeom.mjGEOM_MESH):
            continue
        if int(geom.group) == 3:
            continue
        mesh_id = int(geom.dataid)
        if mesh_id < 0:
            continue
        body_id = int(geom.bodyid)
        body_name = model.body(body_id).name
        if body_name == "virtual_person_target":
            continue
        mesh_name = model.mesh(mesh_id).name
        if not any(mesh_name.startswith(p) for p in STRUCTURAL_MESH_PREFIXES):
            continue
        mesh_file = find_mesh_file(mesh_name)
        if mesh_file is None:
            continue

        body_pos = np.array(data.xpos[body_id])
        body_mat = np.array(data.xmat[body_id]).reshape(3, 3)
        geom_local_pos = np.array(geom.pos)
        geom_local_mat = quat_to_mat(np.array(geom.quat))
        world_pos = body_pos + body_mat @ geom_local_pos
        world_mat = body_mat @ geom_local_mat

        mesh = trimesh.load(str(mesh_file), force="mesh")
        scale = np.array(model.mesh_scale[mesh_id])
        vertices = mesh.vertices * scale
        vertices = (world_mat @ vertices.T).T + world_pos
        mesh.vertices = vertices
        items.append((mesh, BODY_COLORS.get(body_name, DEFAULT_COLOR)))
    return items


def main():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    # Discover all motion CSVs.
    csv_files = sorted(RECORDINGS_DIR.glob("*.csv"))
    motions = [(p.stem, p) for p in csv_files]
    print(f"found {len(motions)} motion CSVs: {[m[0] for m in motions]}")

    cols, rows = 4, 3
    fig = plt.figure(figsize=(18, 13.5), facecolor="white")
    fig.suptitle(
        "AILamp v7.3-C.1 — Motion Library Preview (peak frame in MuJoCo)",
        fontsize=15, fontweight="bold", y=0.98,
    )

    for idx, (name, csv_path) in enumerate(motions):
        if idx >= cols * rows:
            break
        ax = fig.add_subplot(rows, cols, idx + 1, projection="3d")

        angles = load_recording_row(csv_path, fraction=0.5)
        set_pose(model, data, angles)
        items = collect_mesh_world_poses(model, data)

        all_verts = np.vstack([m.vertices for m, _ in items])
        mins = all_verts.min(axis=0)
        maxs = all_verts.max(axis=0)
        center = (mins + maxs) / 2
        # Fixed span across all panels so all motions render at the same scale.
        span = 0.30
        for mesh, rgb in items:
            tri = mesh.vertices[mesh.faces]
            poly = Poly3DCollection(tri, facecolors=shade(mesh, rgb), edgecolors="none", linewidths=0)
            ax.add_collection3d(poly)
        ax.set_xlim(center[0] - span, center[0] + span)
        ax.set_ylim(center[1] - span, center[1] + span)
        ax.set_zlim(-0.05, 0.05 + 2 * span)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=15, azim=-55)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.set_zticklabels([])
        ax.grid(True, alpha=0.08)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.pane.fill = False
            axis.pane.set_edgecolor("#e8e8e8")
        desc = MOTION_DESCRIPTIONS.get(name, name.capitalize())
        ax.set_title(f"{name}\n{desc}", fontsize=10, fontweight="bold", pad=3)
        print(f"  [{idx+1:2d}/{len(motions)}] rendered {name} (peak qpos set, {len(items)} meshes)")

    # Hide unused subplot cells.
    used = len(motions)
    for k in range(used, cols * rows):
        ax = fig.add_subplot(rows, cols, k + 1)
        ax.axis("off")

    out = OUT_DIR / "mujoco-motion-library-v7.3-C.1.png"
    plt.tight_layout(rect=[0, 0.02, 1, 0.96])
    plt.savefig(out, dpi=120, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
