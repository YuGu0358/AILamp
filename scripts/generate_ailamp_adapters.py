from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ORIGINAL_PRINT_DIR = ROOT / "3D"
PCB_EDGE_CLEARANCE_MM = 1.5
CABLE_EXIT_EXTRA_WIDTH_MM = 2.5
JETSON_BOARD_MM = (100.0, 80.0)
JETSON_ASSEMBLY_HEIGHT_MM = 29.0
PICO_BOARD_MM = (51.0, 21.0)
SERVO_DRIVER_BOARD_MM = (65.0, 30.0)
SERVO_DRIVER_MOUNT_HOLE_MM = (58.0, 23.0)
PICO_USB_RELIEF_MM = (14.0, 10.0)
ZIP_TIE_SLOT_MM = (38.0, 4.0)
LELAMP_BASE_FOOTPRINT_MM = (160.0, 190.0)
COVER_ARM_PASS_THROUGH_MM = (12.0, 12.0)
BASE_ARM_MOUNT_OUTER_MM = (92.0, 98.0, 0.0)
BASE_ARM_MOUNT_CLEARANCE_MM = (12.0, 12.0)
BASE_ARM_MOUNT_SCREW_POSITIONS_MM = (
    (-36.0, -40.0),
    (-36.0, 40.0),
    (36.0, -40.0),
    (36.0, 40.0),
)
# 4 × M2 self-tap anchor holes for the lamp arm's root, arranged in a Φ20 mm
# bolt circle around the motor shaft axis (every 90°).  This matches the
# stock Feetech STS3215 metal horn bolt pattern: the arm slips over the horn
# and is bolted to it from above.  The motor body sits vertically inside the
# shell cavity, output shaft pointing up through the cover's central Φ12 mm
# pass-through, with the horn flush in the Φ24 mm recess on top of the
# cover collar.
ARM_MOUNT_SCREW_POSITIONS_MM = (
    (0.0, -10.0),
    (0.0, 10.0),
    (-10.0, 0.0),
    (10.0, 0.0),
)
ARM_MOUNT_SCREW_CLEARANCE_RADIUS_MM = 1.25
# Horn recess on TOP of the shell's central boss (NOT on the cover) —
# accepts the STS3215 metal horn (Φ24 × 3 mm typical) flush so the arm's
# underside meets the boss top when bolted on.  The boss is part of the
# shell, the cover is just a flat 6 mm plate with a Φ26 cutout that lets
# the boss stick through.  This matches the original LeLamp design.
HORN_RECESS_RADIUS_MM = 12.0
HORN_RECESS_DEPTH_MM = 3.0
# STS3215 body cradle inside the shell — holds the motor upright with its
# output shaft pointing up through the boss centre.  Real motor body is
# 24.65 × 45.22 × 35.5 mm; cradle interior 24.7 × 45.3 mm gives 0.05 mm
# assembly slack on each side, height 36 mm clears the body with 0.5 mm
# overshoot.  Wall thickness 2 mm.  Z floor of cradle is raised above the
# shell's own bottom plate by MOTOR_BODY_BOTTOM_Z_MM so the motor's shaft
# tip lines up with the boss's horn recess.
MOTOR_CRADLE_INNER_MM = (24.7, 45.3)
MOTOR_CRADLE_WALL_MM = 2.0
MOTOR_CRADLE_HEIGHT_MM = 36.0
MOTOR_BODY_BOTTOM_Z_MM = 7.0
# Shell central boss — Φ24 mm OD × Φ12 mm ID cylinder rising from the top
# of the motor cradle and protruding 7 mm above the shell wall top.  The
# cover (6 mm flat plate) has a Φ26 cutout that this boss sticks through
# so the horn recess on the boss top sits 1 mm above the cover surface.
SHELL_BOSS_OD_MM = 26.0          # leaves 1 mm rim of material around the Φ24 horn recess
SHELL_BOSS_ID_MM = 12.0
SHELL_BOSS_BOTTOM_Z_MM = 43.0    # sits on top of cradle (cradle: z=7 → z=43)
SHELL_BOSS_TOP_Z_MM = 49.0       # extends to 7 mm above shell wall top (42 mm)
# 4 × M2 self-tap holes through the cradle's long walls clamp the STS3215's
# 4 mounting flange holes (real motor: 4 holes in 32 × 22 mm pattern on the
# 45 × 35.5 mm body side faces near the gear-output end).  Our cradle long
# walls are at X = ±14.35 mm and the mount holes go horizontally through
# both walls at the standard STS3215 flange Y positions ±10 mm and Z =
# cradle_top − 5 mm (so the screws engage the upper end of the body where
# Feetech places the screw bosses on real STS3215 servos).
STS3215_FLANGE_HOLES_YZ_MM = (
    (-10.0, 38.0),
    (10.0, 38.0),
)
STS3215_FLANGE_PILOT_RADIUS_MM = 0.85   # Φ1.7 mm pilot for M2 self-tap
BASE_ARM_LINK_BOOT_MM = (74.0, 74.0, 42.0)
BASE_ARM_LINK_BOOT_CLEARANCE_MM = (42.0, 48.0)
INTEGRATED_BASE_OUTER_MM = (190.0, 230.0, 42.0)
COVER_PLATE_THICKNESS_MM = 4.0
COVER_LIP_HEIGHT_MM = 2.0
INTEGRATED_BASE_COVER_PLATE_MM = (
    190.0,
    230.0,
    COVER_PLATE_THICKNESS_MM + COVER_LIP_HEIGHT_MM,
)
INTEGRATED_BASE_COVER_MM = (
    INTEGRATED_BASE_COVER_PLATE_MM[0],
    INTEGRATED_BASE_COVER_PLATE_MM[1],
    COVER_PLATE_THICKNESS_MM + COVER_LIP_HEIGHT_MM,
)
INTEGRATED_BASE_Y_CENTER_MM = 15.0
INTEGRATED_BASE_WALL_MM = 2.5
INTEGRATED_BASE_BOTTOM_MM = 3.0
INTEGRATED_BASE_INTERNAL_HEIGHT_MM = INTEGRATED_BASE_OUTER_MM[2] - INTEGRATED_BASE_BOTTOM_MM
INTEGRATED_BASE_CASE_SCREW_POSITIONS_MM = (
    (-82.0, -88.0),
    (-82.0, 118.0),
    (82.0, -88.0),
    (82.0, 118.0),
)
JETSON_STANDOFF_SPACING_MM = (84.0, 64.0)
JETSON_STANDOFF_HOLE_MM = 3.4
CASE_SCREW_CLEARANCE_MM = 4.0
INTEGRATED_BASE_CORNER_RADIUS_MM = 18.0
INTEGRATED_BASE_COVER_CORNER_RADIUS_MM = 16.0
SMOOTH_CORNER_STEPS = 28
STANDOFF_SEGMENTS = 32


CORE_NAMESPACE = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
FIXED_ZIP_TIMESTAMP = (2026, 1, 1, 0, 0, 0)
ELECTRONICS_SIDE_DECK_WIRING_ALLOWANCE_MM = (26.0, 15.0)
ROUNDED_CORNER_SEGMENTS = 24
COVER_PASS_THROUGH_CORNER_RADIUS_MM = 5.99  # ≈ Φ12 cable hole (12/2 = 6)
BASE_ARM_MOUNT_COLLAR_CORNER_RADIUS_MM = 28.0
BASE_ARM_MOUNT_COLLAR_INNER_RADIUS_MM = 5.99  # ≈ Φ12 circle (unused after collar removed)
BASE_ARM_MOUNT_COLLAR_TOP_DIAMETER_MM = 74.0
BASE_ARM_LINK_BOOT_CORNER_RADIUS_MM = 37.0
BASE_ARM_LINK_BOOT_INNER_CORNER_RADIUS_MM = 6.0
COVER_CASE_SCREW_CLEARANCE_RADIUS_MM = 2.4


def _loose_pcb_pocket(board_mm: tuple[float, float]) -> tuple[float, float]:
    return (
        board_mm[0] + 2.0 * PCB_EDGE_CLEARANCE_MM,
        board_mm[1] + 2.0 * PCB_EDGE_CLEARANCE_MM,
    )


def _loose_cable_channel_width(nominal_width_mm: float) -> float:
    return nominal_width_mm + CABLE_EXIT_EXTRA_WIDTH_MM


@dataclass(frozen=True)
class AdapterSpec:
    name: str
    outer_mm: tuple[float, float, float]
    board_pocket_mm: tuple[float, float]
    description: str


def adapter_specs() -> list[AdapterSpec]:
    electronics_pocket = (
        SERVO_DRIVER_BOARD_MM[0]
        + PICO_BOARD_MM[0]
        + 2.0 * PCB_EDGE_CLEARANCE_MM
        + ELECTRONICS_SIDE_DECK_WIRING_ALLOWANCE_MM[0],
        max(SERVO_DRIVER_BOARD_MM[1], PICO_BOARD_MM[1])
        + 2.0 * PCB_EDGE_CLEARANCE_MM
        + ELECTRONICS_SIDE_DECK_WIRING_ALLOWANCE_MM[1],
    )
    return [
        AdapterSpec(
            "AILamp_LampBase_Electronics_Shell",
            INTEGRATED_BASE_OUTER_MM,
            (
                INTEGRATED_BASE_OUTER_MM[0] - 2.0 * INTEGRATED_BASE_WALL_MM,
                INTEGRATED_BASE_OUTER_MM[1] - 2.0 * INTEGRATED_BASE_WALL_MM,
            ),
            "Replacement LampBase shell for Jetson Nano, servo driver, Pico WH, wiring, and airflow — pure electronics enclosure (motor lives in arm)",
        ),
        AdapterSpec(
            "AILamp_LampBase_Electronics_Cover",
            INTEGRATED_BASE_COVER_MM,
            BASE_ARM_MOUNT_CLEARANCE_MM,
            "Replacement LampBase cover with raised arm-mount collar, screw holes, and service vents",
        ),
        AdapterSpec(
            "AILamp_Base_Arm_Link_Boot",
            BASE_ARM_LINK_BOOT_MM,
            BASE_ARM_LINK_BOOT_CLEARANCE_MM,
            "Moving base-arm link boot that bridges the fixed cover collar to the arm root",
        ),
        AdapterSpec(
            "AILamp_Jetson_Nano_Base_Tray",
            (122.0, 102.0, 12.0),
            _loose_pcb_pocket(JETSON_BOARD_MM),
            "Loose external Jetson Nano tray",
        ),
        AdapterSpec(
            "AILamp_Electronics_Side_Deck",
            (145.0, 48.0, 10.0),
            electronics_pocket,
            "Servo driver and Pico WH side deck",
        ),
        AdapterSpec(
            "AILamp_Cable_Clip_6mm",
            (24.0, 18.0, 8.0),
            (_loose_cable_channel_width(6.0), 8.0),
            "Loose cable clip for USB and signal wires",
        ),
        AdapterSpec(
            "AILamp_Cable_Clip_10mm",
            (30.0, 22.0, 10.0),
            (_loose_cable_channel_width(10.0), 10.0),
            "Loose cable clip for power or servo bundles",
        ),
    ]


class Mesh:
    def __init__(self, name: str) -> None:
        self.name = name
        self._boxes: list[tuple[float, float, float, float, float, float]] = []
        self._cutouts: list[tuple[float, float, float, float, float, float]] = []

    def add_box(
        self,
        cx: float,
        cy: float,
        cz: float,
        sx: float,
        sy: float,
        sz: float,
    ) -> None:
        self._boxes.append(_box_bounds(cx, cy, cz, sx, sy, sz))

    def subtract_box(
        self,
        cx: float,
        cy: float,
        cz: float,
        sx: float,
        sy: float,
        sz: float,
    ) -> None:
        self._cutouts.append(_box_bounds(cx, cy, cz, sx, sy, sz))

    def write_stl(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        vertices, triangles = self._surface_mesh()
        lines = [f"solid {self.name}"]
        for a, b, c in triangles:
            va = vertices[a]
            vb = vertices[b]
            vc = vertices[c]
            normal = _triangle_normal(va, vb, vc)
            lines.extend(
                [
                    f"  facet normal {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}",
                    "    outer loop",
                    f"      vertex {va[0]:.4f} {va[1]:.4f} {va[2]:.4f}",
                    f"      vertex {vb[0]:.4f} {vb[1]:.4f} {vb[2]:.4f}",
                    f"      vertex {vc[0]:.4f} {vc[1]:.4f} {vc[2]:.4f}",
                    "    endloop",
                    "  endfacet",
                ]
            )
        lines.append(f"endsolid {self.name}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def write_3mf(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        vertices, triangles = self._surface_mesh()
        vertices = "\n".join(
            f'          <vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}" />'
            for x, y, z in vertices
        )
        triangles = "\n".join(
            f'          <triangle v1="{a}" v2="{b}" v3="{c}" />'
            for a, b, c in triangles
        )
        model = f"""<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="{CORE_NAMESPACE}">
  <resources>
    <object id="1" type="model">
      <mesh>
        <vertices>
{vertices}
        </vertices>
        <triangles>
{triangles}
        </triangles>
      </mesh>
    </object>
  </resources>
  <build>
    <item objectid="1" />
  </build>
</model>
"""
        content_types = """<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />
</Types>
"""
        rels = """<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" />
</Relationships>
"""
        with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            _write_zip_entry(archive, "[Content_Types].xml", content_types)
            _write_zip_entry(archive, "_rels/.rels", rels)
            _write_zip_entry(archive, "3D/3dmodel.model", model)

    def _surface_mesh(self) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
        if not self._boxes:
            return [], []

        xs = _sorted_boundaries(self._boxes, self._cutouts, 0, 1)
        ys = _sorted_boundaries(self._boxes, self._cutouts, 2, 3)
        zs = _sorted_boundaries(self._boxes, self._cutouts, 4, 5)
        solid_cells: set[tuple[int, int, int]] = set()

        for ix in range(len(xs) - 1):
            cx = (xs[ix] + xs[ix + 1]) / 2.0
            for iy in range(len(ys) - 1):
                cy = (ys[iy] + ys[iy + 1]) / 2.0
                for iz in range(len(zs) - 1):
                    cz = (zs[iz] + zs[iz + 1]) / 2.0
                    if self._is_solid_point(cx, cy, cz):
                        solid_cells.add((ix, iy, iz))

        vertices: list[tuple[float, float, float]] = []
        vertex_index: dict[tuple[float, float, float], int] = {}
        triangles: list[tuple[int, int, int]] = []

        def vertex_id(vertex: tuple[float, float, float]) -> int:
            key = tuple(round(value, 4) for value in vertex)
            if key not in vertex_index:
                vertex_index[key] = len(vertices)
                vertices.append(key)
            return vertex_index[key]

        def add_quad(quad: tuple[tuple[float, float, float], ...]) -> None:
            a, b, c, d = (vertex_id(vertex) for vertex in quad)
            triangles.append((a, b, c))
            triangles.append((a, c, d))

        directions = (
            (-1, 0, 0),
            (1, 0, 0),
            (0, -1, 0),
            (0, 1, 0),
            (0, 0, -1),
            (0, 0, 1),
        )
        for ix, iy, iz in sorted(solid_cells):
            x0, x1 = xs[ix], xs[ix + 1]
            y0, y1 = ys[iy], ys[iy + 1]
            z0, z1 = zs[iz], zs[iz + 1]
            for direction in directions:
                neighbor = (
                    ix + direction[0],
                    iy + direction[1],
                    iz + direction[2],
                )
                if neighbor in solid_cells:
                    continue
                add_quad(_face_quad(direction, x0, x1, y0, y1, z0, z1))

        return vertices, triangles

    def _is_solid_point(self, x: float, y: float, z: float) -> bool:
        inside_box = any(_contains(box, x, y, z) for box in self._boxes)
        inside_cutout = any(_contains(cutout, x, y, z) for cutout in self._cutouts)
        return inside_box and not inside_cutout


class TriangleMesh:
    def __init__(
        self,
        name: str,
        vertices: list[tuple[float, float, float]] | None = None,
        triangles: list[tuple[int, int, int]] | None = None,
    ) -> None:
        self.name = name
        self.vertices = list(vertices or [])
        self.triangles = list(triangles or [])

    def extend(
        self,
        vertices: list[tuple[float, float, float]],
        triangles: list[tuple[int, int, int]],
    ) -> None:
        offset = len(self.vertices)
        self.vertices.extend(vertices)
        self.triangles.extend((a + offset, b + offset, c + offset) for a, b, c in triangles)

    def extend_mesh(self, mesh: Mesh) -> None:
        vertices, triangles = mesh._surface_mesh()
        self.extend(vertices, triangles)

    def extend_triangle_mesh(self, mesh: "TriangleMesh") -> None:
        self.extend(mesh.vertices, mesh.triangles)

    def write_stl(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        lines = [f"solid {self.name}"]
        for a, b, c in self.triangles:
            va = self.vertices[a]
            vb = self.vertices[b]
            vc = self.vertices[c]
            normal = _triangle_normal(va, vb, vc)
            lines.extend(
                [
                    f"  facet normal {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}",
                    "    outer loop",
                    f"      vertex {va[0]:.4f} {va[1]:.4f} {va[2]:.4f}",
                    f"      vertex {vb[0]:.4f} {vb[1]:.4f} {vb[2]:.4f}",
                    f"      vertex {vc[0]:.4f} {vc[1]:.4f} {vc[2]:.4f}",
                    "    endloop",
                    "  endfacet",
                ]
            )
        lines.append(f"endsolid {self.name}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def write_3mf(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        vertices = "\n".join(
            f'          <vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}" />'
            for x, y, z in self.vertices
        )
        triangles = "\n".join(
            f'          <triangle v1="{a}" v2="{b}" v3="{c}" />'
            for a, b, c in self.triangles
        )
        model = f"""<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="{CORE_NAMESPACE}">
  <resources>
    <object id="1" type="model">
      <mesh>
        <vertices>
{vertices}
        </vertices>
        <triangles>
{triangles}
        </triangles>
      </mesh>
    </object>
  </resources>
  <build>
    <item objectid="1" />
  </build>
</model>
"""
        content_types = """<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />
</Types>
"""
        rels = """<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" />
</Relationships>
"""
        with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            _write_zip_entry(archive, "[Content_Types].xml", content_types)
            _write_zip_entry(archive, "_rels/.rels", rels)
            _write_zip_entry(archive, "3D/3dmodel.model", model)


def _box_bounds(
    cx: float,
    cy: float,
    cz: float,
    sx: float,
    sy: float,
    sz: float,
) -> tuple[float, float, float, float, float, float]:
    return (
        cx - sx / 2.0,
        cx + sx / 2.0,
        cy - sy / 2.0,
        cy + sy / 2.0,
        cz - sz / 2.0,
        cz + sz / 2.0,
    )


def _sorted_boundaries(
    boxes: list[tuple[float, float, float, float, float, float]],
    cutouts: list[tuple[float, float, float, float, float, float]],
    low_index: int,
    high_index: int,
) -> list[float]:
    boundaries = set()
    for bounds in boxes + cutouts:
        boundaries.add(round(bounds[low_index], 4))
        boundaries.add(round(bounds[high_index], 4))
    return sorted(boundaries)


def _contains(
    bounds: tuple[float, float, float, float, float, float],
    x: float,
    y: float,
    z: float,
) -> bool:
    x0, x1, y0, y1, z0, z1 = bounds
    return x0 < x < x1 and y0 < y < y1 and z0 < z < z1


def _face_quad(
    direction: tuple[int, int, int],
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
) -> tuple[tuple[float, float, float], ...]:
    if direction == (-1, 0, 0):
        return ((x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0))
    if direction == (1, 0, 0):
        return ((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1))
    if direction == (0, -1, 0):
        return ((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1))
    if direction == (0, 1, 0):
        return ((x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0))
    if direction == (0, 0, -1):
        return ((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0))
    return ((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))


def _triangle_normal(
    va: tuple[float, float, float],
    vb: tuple[float, float, float],
    vc: tuple[float, float, float],
) -> tuple[float, float, float]:
    ux, uy, uz = vb[0] - va[0], vb[1] - va[1], vb[2] - va[2]
    vx, vy, vz = vc[0] - va[0], vc[1] - va[1], vc[2] - va[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = (nx * nx + ny * ny + nz * nz) ** 0.5
    if length == 0.0:
        return (0.0, 0.0, 0.0)
    return (nx / length, ny / length, nz / length)


def _arc_points(
    cx: float,
    cy: float,
    radius: float,
    start_deg: float,
    end_deg: float,
    segments: int,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        angle = math.radians(start_deg + (end_deg - start_deg) * index / segments)
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points


def _rounded_rect_polygon(
    width: float,
    depth: float,
    radius: float,
    segments: int = ROUNDED_CORNER_SEGMENTS,
    cx: float = 0.0,
    cy: float = 0.0,
) -> list[tuple[float, float]]:
    """CCW outline (viewed from +z) of a rounded rectangle centered at (cx, cy)."""
    if radius * 2 > min(width, depth):
        raise ValueError("corner radius too large for given width/depth")
    hw = width / 2.0
    hd = depth / 2.0
    pts: list[tuple[float, float]] = []
    # Each arc returns segments+1 points; drop the last so the next arc's start does not repeat.
    pts.extend(
        _arc_points(cx + hw - radius, cy - hd + radius, radius, 270.0, 360.0, segments)[:-1]
    )
    pts.extend(
        _arc_points(cx + hw - radius, cy + hd - radius, radius, 0.0, 90.0, segments)[:-1]
    )
    pts.extend(
        _arc_points(cx - hw + radius, cy + hd - radius, radius, 90.0, 180.0, segments)[:-1]
    )
    pts.extend(
        _arc_points(cx - hw + radius, cy - hd + radius, radius, 180.0, 270.0, segments)[:-1]
    )
    return pts


def _polygon_centroid(points: list[tuple[float, float]]) -> tuple[float, float]:
    count = len(points)
    return (
        sum(p[0] for p in points) / count,
        sum(p[1] for p in points) / count,
    )


def _circle_polygon(
    cx: float, cy: float, radius: float, segments: int = ROUNDED_CORNER_SEGMENTS
) -> list[tuple[float, float]]:
    """CCW circle polygon."""
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _triangulate_polygon_with_holes(
    outer: list[tuple[float, float]],
    holes: list[list[tuple[float, float]]],
) -> list[tuple[int, int, int]]:
    """Earcut-triangulate a 2D polygon with holes.

    Returns triangle indices into the concatenated vertex list
    ``outer + holes[0] + holes[1] + ...``.
    """
    import mapbox_earcut as earcut

    flat_points: list[tuple[float, float]] = list(outer)
    ring_ends = [len(outer)]
    for hole in holes:
        flat_points.extend(hole)
        ring_ends.append(ring_ends[-1] + len(hole))
    points_array = np.array(flat_points, dtype=np.float64)
    indices = earcut.triangulate_float64(
        points_array, np.array(ring_ends, dtype=np.uint32)
    )
    return [
        (int(indices[i]), int(indices[i + 1]), int(indices[i + 2]))
        for i in range(0, len(indices), 3)
    ]


def _fan_cap(
    name: str,
    polygon: list[tuple[float, float]],
    z: float,
    facing_up: bool,
) -> TriangleMesh:
    """Fan-triangulate a convex polygon at constant z with the given facing normal."""
    centroid = _polygon_centroid(polygon)
    vertices: list[tuple[float, float, float]] = [(centroid[0], centroid[1], z)]
    for x, y in polygon:
        vertices.append((x, y, z))
    triangles: list[tuple[int, int, int]] = []
    n = len(polygon)
    for i in range(n):
        a = 0
        b = 1 + i
        c = 1 + ((i + 1) % n)
        if facing_up:
            triangles.append((a, b, c))
        else:
            triangles.append((a, c, b))
    return TriangleMesh(name, vertices, triangles)


def _holey_cap(
    name: str,
    outer_polygon: list[tuple[float, float]],
    holes: list[list[tuple[float, float]]],
    z: float,
    facing_up: bool,
) -> TriangleMesh:
    """Triangulate an outer polygon (CCW) with one or more holes (CCW) at constant z.

    Holes are reversed to CW before being handed to earcut.  The returned mesh
    uses the same outer CCW coordinates and CW hole coordinates so that
    coordinate-based merging in tests lines up with adjacent walls/strips.
    """
    holes_cw = [list(reversed(hole)) for hole in holes]
    triangles_2d = _triangulate_polygon_with_holes(outer_polygon, holes_cw)
    flat: list[tuple[float, float]] = list(outer_polygon)
    for hole in holes_cw:
        flat.extend(hole)
    vertices: list[tuple[float, float, float]] = [(x, y, z) for x, y in flat]
    if facing_up:
        triangles = list(triangles_2d)
    else:
        triangles = [(c, b, a) for a, b, c in triangles_2d]
    return TriangleMesh(name, vertices, triangles)


def _extruded_wall(
    name: str,
    polygon: list[tuple[float, float]],
    z_bottom: float,
    z_top: float,
    outward: bool = True,
) -> TriangleMesh:
    """Vertical side walls for a CCW polygon extruded along z."""
    n = len(polygon)
    vertices: list[tuple[float, float, float]] = []
    for x, y in polygon:
        vertices.append((x, y, z_bottom))
    for x, y in polygon:
        vertices.append((x, y, z_top))
    triangles: list[tuple[int, int, int]] = []
    for i in range(n):
        j = (i + 1) % n
        if outward:
            triangles.append((i, j, j + n))
            triangles.append((i, j + n, i + n))
        else:
            triangles.append((i, j + n, j))
            triangles.append((i, i + n, j + n))
    return TriangleMesh(name, vertices, triangles)


def _annular_strip(
    name: str,
    outer_polygon: list[tuple[float, float]],
    inner_polygon: list[tuple[float, float]],
    z: float,
    facing_up: bool,
) -> TriangleMesh:
    """Triangle strip between two same-length polygons at a constant z."""
    if len(outer_polygon) != len(inner_polygon):
        raise ValueError("annular strip requires polygons of equal length")
    n = len(outer_polygon)
    vertices: list[tuple[float, float, float]] = []
    for x, y in outer_polygon:
        vertices.append((x, y, z))
    for x, y in inner_polygon:
        vertices.append((x, y, z))
    triangles: list[tuple[int, int, int]] = []
    for i in range(n):
        j = (i + 1) % n
        if facing_up:
            triangles.append((i, j, j + n))
            triangles.append((i, j + n, i + n))
        else:
            triangles.append((i, j + n, j))
            triangles.append((i, i + n, j + n))
    return TriangleMesh(name, vertices, triangles)


def _hollow_rounded_box(
    name: str,
    outer_w: float,
    outer_d: float,
    outer_h: float,
    wall_t: float,
    bottom_t: float,
    outer_radius: float,
    cy_offset: float = 0.0,
    segments: int = ROUNDED_CORNER_SEGMENTS,
) -> TriangleMesh:
    """Closed 2-manifold rounded-rectangle box with a hollow upper cavity (top is open)."""
    inner_radius = max(outer_radius - wall_t, max(wall_t * 0.5, 2.0))
    outer_pts = _rounded_rect_polygon(outer_w, outer_d, outer_radius, segments, cy=cy_offset)
    inner_pts = _rounded_rect_polygon(
        outer_w - 2.0 * wall_t,
        outer_d - 2.0 * wall_t,
        inner_radius,
        segments,
        cy=cy_offset,
    )
    mesh = TriangleMesh(name)
    # Bottom plane (closes the box from below; normal -z).
    mesh.extend_triangle_mesh(_fan_cap(name + "_bottom", outer_pts, 0.0, facing_up=False))
    # Outer side wall from z=0 up to the top edge.
    mesh.extend_triangle_mesh(
        _extruded_wall(name + "_outer_wall", outer_pts, 0.0, outer_h, outward=True)
    )
    # Annular top edge connecting outer and inner perimeters (closes off the wall thickness).
    mesh.extend_triangle_mesh(
        _annular_strip(name + "_top_edge", outer_pts, inner_pts, outer_h, facing_up=True)
    )
    # Inner wall going down from the top edge to the inner floor (faces into the cavity).
    mesh.extend_triangle_mesh(
        _extruded_wall(name + "_inner_wall", inner_pts, bottom_t, outer_h, outward=False)
    )
    # Inner floor (closes the cavity from below; normal +z).
    mesh.extend_triangle_mesh(_fan_cap(name + "_inner_floor", inner_pts, bottom_t, facing_up=True))
    return mesh


def _rounded_plate_with_hole(
    name: str,
    plate_w: float,
    plate_d: float,
    plate_thickness: float,
    outer_radius: float,
    hole_w: float,
    hole_d: float,
    hole_radius: float,
    cy_offset: float = 0.0,
    segments: int = ROUNDED_CORNER_SEGMENTS,
) -> TriangleMesh:
    """Flat rounded plate with a centered rounded-rectangle through-hole (centered at y=0)."""
    outer_pts = _rounded_rect_polygon(plate_w, plate_d, outer_radius, segments, cy=cy_offset)
    hole_pts = _rounded_rect_polygon(hole_w, hole_d, hole_radius, segments)
    mesh = TriangleMesh(name)
    # Bottom annulus (facing -z).
    mesh.extend_triangle_mesh(
        _annular_strip(name + "_bottom", outer_pts, hole_pts, 0.0, facing_up=False)
    )
    # Top annulus (facing +z).
    mesh.extend_triangle_mesh(
        _annular_strip(name + "_top", outer_pts, hole_pts, plate_thickness, facing_up=True)
    )
    # Outer perimeter wall.
    mesh.extend_triangle_mesh(
        _extruded_wall(name + "_outer", outer_pts, 0.0, plate_thickness, outward=True)
    )
    # Inner perimeter wall around the hole (faces into the hole).
    mesh.extend_triangle_mesh(
        _extruded_wall(name + "_hole", hole_pts, 0.0, plate_thickness, outward=False)
    )
    return mesh


def _lofted_wall(
    name: str,
    bottom_polygon: list[tuple[float, float]],
    z_bottom: float,
    top_polygon: list[tuple[float, float]],
    z_top: float,
    outward: bool = True,
) -> TriangleMesh:
    """Vertical-ish walls connecting two same-length CCW polygons at different z."""
    if len(bottom_polygon) != len(top_polygon):
        raise ValueError("lofted wall requires matching polygon lengths")
    n = len(bottom_polygon)
    vertices: list[tuple[float, float, float]] = []
    for x, y in bottom_polygon:
        vertices.append((x, y, z_bottom))
    for x, y in top_polygon:
        vertices.append((x, y, z_top))
    triangles: list[tuple[int, int, int]] = []
    for i in range(n):
        j = (i + 1) % n
        if outward:
            triangles.append((i, j, j + n))
            triangles.append((i, j + n, i + n))
        else:
            triangles.append((i, j + n, j))
            triangles.append((i, i + n, j + n))
    return TriangleMesh(name, vertices, triangles)


def _tapered_rounded_collar(
    name: str,
    bottom_outer_w: float,
    bottom_outer_d: float,
    bottom_outer_radius: float,
    top_outer_w: float,
    top_outer_d: float,
    top_outer_radius: float,
    inner_w: float,
    inner_d: float,
    inner_radius: float,
    height: float,
    z_bottom: float,
    segments: int = ROUNDED_CORNER_SEGMENTS,
    include_inner_wall: bool = True,
    include_bottom_annulus: bool = True,
) -> TriangleMesh:
    """Closed frame whose outer wall lofts from one rounded rectangle to another.

    Use this to taper the cover's arm-mount collar from its rectangular cover
    footprint (92 x 98 mm at the cover plate) down to a circle that matches
    the cylindrical boot diameter (Φ74 mm at the collar top).

    Pass ``include_bottom_annulus=False`` when the collar sits on top of a
    plate that already provides the +z-facing surface around the collar's
    outer perimeter; this avoids the doubled coincident face at ``z_bottom``
    that would otherwise create 4-triangles-per-edge non-manifoldness if a
    boolean subtract later punches holes through the junction.
    """
    bottom_outer = _rounded_rect_polygon(
        bottom_outer_w, bottom_outer_d, bottom_outer_radius, segments
    )
    top_outer = _rounded_rect_polygon(top_outer_w, top_outer_d, top_outer_radius, segments)
    inner_poly = _rounded_rect_polygon(inner_w, inner_d, inner_radius, segments)
    z_top = z_bottom + height

    mesh = TriangleMesh(name)
    if include_bottom_annulus:
        # Bottom annulus from bottom outer to inner perimeter (facing -z).
        mesh.extend_triangle_mesh(
            _annular_strip(name + "_bottom", bottom_outer, inner_poly, z_bottom, facing_up=False)
        )
    # Top annulus from top outer to inner perimeter (facing +z).
    mesh.extend_triangle_mesh(
        _annular_strip(name + "_top", top_outer, inner_poly, z_top, facing_up=True)
    )
    # Outer wall: lofts from bottom_outer to top_outer.
    mesh.extend_triangle_mesh(
        _lofted_wall(name + "_outer", bottom_outer, z_bottom, top_outer, z_top, outward=True)
    )
    if include_inner_wall:
        # Inner wall: straight extrusion of the inner polygon.
        mesh.extend_triangle_mesh(
            _extruded_wall(name + "_inner", inner_poly, z_bottom, z_top, outward=False)
        )
    return mesh


def _rounded_collar(
    name: str,
    outer_w: float,
    outer_d: float,
    inner_w: float,
    inner_d: float,
    height: float,
    z_bottom: float,
    outer_radius: float,
    inner_radius: float,
    segments: int = ROUNDED_CORNER_SEGMENTS,
    cy_offset: float = 0.0,
) -> TriangleMesh:
    """Closed rounded-rectangle frame (collar) of given outer/inner footprint and height."""
    outer_pts = _rounded_rect_polygon(outer_w, outer_d, outer_radius, segments, cy=cy_offset)
    inner_pts = _rounded_rect_polygon(inner_w, inner_d, inner_radius, segments, cy=cy_offset)
    z_top = z_bottom + height
    mesh = TriangleMesh(name)
    # Bottom annulus (facing -z).
    mesh.extend_triangle_mesh(
        _annular_strip(name + "_bot", outer_pts, inner_pts, z_bottom, facing_up=False)
    )
    # Top annulus (facing +z).
    mesh.extend_triangle_mesh(
        _annular_strip(name + "_top", outer_pts, inner_pts, z_top, facing_up=True)
    )
    # Outer wall.
    mesh.extend_triangle_mesh(
        _extruded_wall(name + "_outer", outer_pts, z_bottom, z_top, outward=True)
    )
    # Inner wall (faces into the collar opening).
    mesh.extend_triangle_mesh(
        _extruded_wall(name + "_inner", inner_pts, z_bottom, z_top, outward=False)
    )
    return mesh


def _solid_cylinder_with_hole(
    name: str,
    cx: float,
    cy: float,
    z_bottom: float,
    height: float,
    outer_radius: float,
    inner_radius: float,
    segments: int = STANDOFF_SEGMENTS,
) -> TriangleMesh:
    """Alias for _cylinder_ring_mesh that lets the new builders read cleanly."""
    return _cylinder_ring_mesh(
        name,
        cx,
        cy,
        z_bottom=z_bottom,
        height=height,
        outer_radius=outer_radius,
        inner_radius=inner_radius,
        segments=segments,
    )


def _write_zip_entry(archive: zipfile.ZipFile, filename: str, text: str) -> None:
    info = zipfile.ZipInfo(filename=filename, date_time=FIXED_ZIP_TIMESTAMP)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o644 << 16
    archive.writestr(info, text.encode("utf-8"))


def _unit_to_mm_scale(unit: str | None) -> float:
    if unit == "meter":
        return 1000.0
    if unit in {None, "millimeter"}:
        return 1.0
    if unit == "centimeter":
        return 10.0
    raise ValueError(f"unsupported 3MF unit: {unit}")


def _load_3mf_mesh_mm(path: Path) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    namespace = {"core": CORE_NAMESPACE}
    with zipfile.ZipFile(path) as archive:
        model_name = next(name for name in archive.namelist() if name.endswith(".model"))
        with archive.open(model_name) as model_file:
            model_root = ET.parse(model_file).getroot()

    scale = _unit_to_mm_scale(model_root.attrib.get("unit"))
    vertices = [
        (
            float(vertex.attrib["x"]) * scale,
            float(vertex.attrib["y"]) * scale,
            float(vertex.attrib["z"]) * scale,
        )
        for vertex in model_root.findall(".//core:vertex", namespace)
    ]
    triangles = [
        (
            int(triangle.attrib["v1"]),
            int(triangle.attrib["v2"]),
            int(triangle.attrib["v3"]),
        )
        for triangle in model_root.findall(".//core:triangle", namespace)
    ]
    if not vertices or not triangles:
        raise ValueError(f"no mesh data found in {path}")
    return vertices, triangles


def _scaled_original_print_mesh(
    name: str,
    source_filename: str,
    target_mm: tuple[float, float, float],
) -> TriangleMesh:
    source_vertices, source_triangles = _load_3mf_mesh_mm(ORIGINAL_PRINT_DIR / source_filename)
    xs = [x for x, _y, _z in source_vertices]
    ys = [y for _x, y, _z in source_vertices]
    zs = [z for _x, _y, z in source_vertices]
    min_z = min(zs)
    scale_x = target_mm[0] / (max(xs) - min(xs))
    scale_y = target_mm[1] / (max(ys) - min(ys))
    scale_z = target_mm[2] / (max(zs) - min(zs))

    # Scale X/Y around the lamp arm origin so the original base-to-arm relationship stays aligned.
    scaled_vertices = [
        (x * scale_x, y * scale_y, (z - min_z) * scale_z)
        for x, y, z in source_vertices
    ]
    return TriangleMesh(name, scaled_vertices, source_triangles)


def _base_plate(
    mesh: Mesh,
    outer_mm: tuple[float, float, float],
    thickness: float,
    center_y: float = 0.0,
) -> None:
    mesh.add_box(0.0, center_y, thickness / 2.0, outer_mm[0], outer_mm[1], thickness)


def _edge_rails(
    mesh: Mesh,
    outer_mm: tuple[float, float, float],
    rail_width: float,
    rail_height: float,
    z_bottom: float,
    center_y: float = 0.0,
) -> None:
    sx, sy, _ = outer_mm
    cz = z_bottom + rail_height / 2.0
    mesh.add_box(0.0, center_y + sy / 2.0 - rail_width / 2.0, cz, sx, rail_width, rail_height)
    mesh.add_box(0.0, center_y - sy / 2.0 + rail_width / 2.0, cz, sx, rail_width, rail_height)
    mesh.add_box(-sx / 2.0 + rail_width / 2.0, center_y, cz, rail_width, sy, rail_height)
    mesh.add_box(sx / 2.0 - rail_width / 2.0, center_y, cz, rail_width, sy, rail_height)


def _vent_slots(
    mesh: Mesh,
    x_values: tuple[float, ...],
    cy: float,
    z_center: float,
    sx: float,
    sy: float,
    sz: float,
) -> None:
    for x in x_values:
        mesh.subtract_box(x, cy, z_center, sx, sy, sz)


def _rounded_corner_cutouts(
    mesh: Mesh,
    outer_mm: tuple[float, float, float],
    radius_mm: float,
    z_center: float,
    z_size: float,
    center_y: float = 0.0,
    steps: int = SMOOTH_CORNER_STEPS,
) -> None:
    sx, sy, _sz = outer_mm
    half_x = sx / 2.0
    half_y = sy / 2.0
    strip = radius_mm / steps
    for step in range(steps):
        inward = (step + 0.5) * strip
        circle_dy = radius_mm - inward
        keep_from_center = max(radius_mm * radius_mm - circle_dy * circle_dy, 0.0) ** 0.5
        cut_width = radius_mm - keep_from_center
        if cut_width <= 0.05:
            continue
        for sign_x in (-1.0, 1.0):
            for sign_y in (-1.0, 1.0):
                cx = sign_x * (half_x - cut_width / 2.0)
                cy = center_y + sign_y * (half_y - inward)
                mesh.subtract_box(cx, cy, z_center, cut_width + 0.4, strip + 0.2, z_size)


def _cylinder_ring_mesh(
    name: str,
    cx: float,
    cy: float,
    z_bottom: float,
    height: float,
    outer_radius: float,
    inner_radius: float,
    segments: int = STANDOFF_SEGMENTS,
) -> TriangleMesh:
    if inner_radius <= 0.0 or inner_radius >= outer_radius:
        raise ValueError("inner_radius must be positive and smaller than outer_radius")
    vertices: list[tuple[float, float, float]] = []
    for z in (z_bottom, z_bottom + height):
        for radius in (outer_radius, inner_radius):
            for index in range(segments):
                angle = 2.0 * 3.141592653589793 * index / segments
                vertices.append(
                    (
                        cx + radius * math.cos(angle),
                        cy + radius * math.sin(angle),
                        z,
                    )
                )

    outer_bottom = 0
    inner_bottom = segments
    outer_top = segments * 2
    inner_top = segments * 3
    triangles: list[tuple[int, int, int]] = []
    for index in range(segments):
        next_index = (index + 1) % segments

        ob0 = outer_bottom + index
        ob1 = outer_bottom + next_index
        ib0 = inner_bottom + index
        ib1 = inner_bottom + next_index
        ot0 = outer_top + index
        ot1 = outer_top + next_index
        it0 = inner_top + index
        it1 = inner_top + next_index

        # Outer wall.
        triangles.append((ob0, ob1, ot1))
        triangles.append((ob0, ot1, ot0))
        # Inner wall, reversed so normals face into the screw clearance hole.
        triangles.append((ib0, it0, it1))
        triangles.append((ib0, it1, ib1))
        # Bottom and top annular faces.
        triangles.append((ob0, ib1, ob1))
        triangles.append((ob0, ib0, ib1))
        triangles.append((ot0, ot1, it1))
        triangles.append((ot0, it1, it0))
    return TriangleMesh(name, vertices, triangles)


def _plate_with_square_cutouts(
    mesh: Mesh,
    outer_x: float,
    outer_y: float,
    thickness: float,
    cutouts: tuple[tuple[float, float, float, float], ...],
) -> None:
    x_edges = {-outer_x / 2.0, outer_x / 2.0}
    y_edges = {-outer_y / 2.0, outer_y / 2.0}
    for cx, cy, sx, sy in cutouts:
        x_edges.update((cx - sx / 2.0, cx + sx / 2.0))
        y_edges.update((cy - sy / 2.0, cy + sy / 2.0))

    sorted_x = sorted(x_edges)
    sorted_y = sorted(y_edges)
    for x0, x1 in zip(sorted_x, sorted_x[1:]):
        for y0, y1 in zip(sorted_y, sorted_y[1:]):
            if x1 <= x0 or y1 <= y0:
                continue
            cx = (x0 + x1) / 2.0
            cy = (y0 + y1) / 2.0
            inside_cutout = any(
                abs(cx - cutout_x) < cutout_sx / 2.0
                and abs(cy - cutout_y) < cutout_sy / 2.0
                for cutout_x, cutout_y, cutout_sx, cutout_sy in cutouts
            )
            if not inside_cutout:
                mesh.add_box(cx, cy, thickness / 2.0, x1 - x0, y1 - y0, thickness)


def _integrated_base_internal_features() -> TriangleMesh:
    """Minimal internal layout: four case-screw bosses + Jetson, ST3215, and Pico standoffs.

    All location ribs / cable-spine boxes / power tabs from the previous revision
    have been removed.  Boards are held by their through-hole screws on the
    standoffs; the cavity is otherwise clear so cable routing and airflow are
    unobstructed and the print stays simple.
    """
    features = TriangleMesh("AILamp_LampBase_Electronics_Internal_Features")
    standoff_z = INTEGRATED_BASE_BOTTOM_MM

    # Four case-screw bosses tying the cover to the shell.
    case_boss_top = INTEGRATED_BASE_OUTER_MM[2] - 10.0  # leaves 10 mm clearance below the cover
    for x, y in INTEGRATED_BASE_CASE_SCREW_POSITIONS_MM:
        features.extend_triangle_mesh(
            _cylinder_ring_mesh(
                "case_screw_boss",
                x,
                y,
                z_bottom=standoff_z,
                height=case_boss_top - standoff_z,
                outer_radius=5.0,
                inner_radius=CASE_SCREW_CLEARANCE_MM / 2.0,
            )
        )

    # Jetson Nano standoffs (84 x 64 mm pattern, 8 mm tall, M3 self-tapping).
    jetson_y = -42.0
    for x in (-JETSON_STANDOFF_SPACING_MM[0] / 2.0, JETSON_STANDOFF_SPACING_MM[0] / 2.0):
        for y in (
            jetson_y - JETSON_STANDOFF_SPACING_MM[1] / 2.0,
            jetson_y + JETSON_STANDOFF_SPACING_MM[1] / 2.0,
        ):
            features.extend_triangle_mesh(
                _cylinder_ring_mesh(
                    "jetson_standoff",
                    x,
                    y,
                    z_bottom=standoff_z,
                    height=8.0,
                    outer_radius=4.0,
                    inner_radius=JETSON_STANDOFF_HOLE_MM / 2.0,
                )
            )

    # (Motor cradle + horn boss now built as a manifold3d assembly directly
    # inside build_integrated_lampbase_shell so the M2 flange holes through
    # the cradle walls can be drilled cleanly.)

    # Waveshare ST3215 driver standoffs (58 x 23 mm hole pattern).
    driver_center_x = -42.0
    driver_center_y = 66.0
    for x in (
        driver_center_x - SERVO_DRIVER_MOUNT_HOLE_MM[0] / 2.0,
        driver_center_x + SERVO_DRIVER_MOUNT_HOLE_MM[0] / 2.0,
    ):
        for y in (
            driver_center_y - SERVO_DRIVER_MOUNT_HOLE_MM[1] / 2.0,
            driver_center_y + SERVO_DRIVER_MOUNT_HOLE_MM[1] / 2.0,
        ):
            features.extend_triangle_mesh(
                _cylinder_ring_mesh(
                    "servo_driver_standoff",
                    x,
                    y,
                    z_bottom=standoff_z,
                    height=7.0,
                    outer_radius=3.5,
                    inner_radius=1.7,
                )
            )

    return features


def _merge_duplicate_vertices(
    tm: TriangleMesh, tolerance: float = 1e-4
) -> TriangleMesh:
    """Collapse coincident vertices (rounded to ``tolerance``) into a single index.

    Required before feeding a multi-sub-mesh TriangleMesh into manifold3d, whose
    2-manifold validation requires topologically-shared vertices (same index)
    rather than just coordinate-shared vertices.
    """
    key_to_index: dict[tuple[int, int, int], int] = {}
    new_vertices: list[tuple[float, float, float]] = []
    remap: list[int] = []
    scale = 1.0 / tolerance
    for x, y, z in tm.vertices:
        key = (round(x * scale), round(y * scale), round(z * scale))
        if key not in key_to_index:
            key_to_index[key] = len(new_vertices)
            new_vertices.append((x, y, z))
        remap.append(key_to_index[key])
    new_triangles: list[tuple[int, int, int]] = []
    for a, b, c in tm.triangles:
        ra, rb, rc = remap[a], remap[b], remap[c]
        if ra != rb and rb != rc and rc != ra:
            new_triangles.append((ra, rb, rc))
    return TriangleMesh(tm.name, new_vertices, new_triangles)


def _trianglemesh_to_manifold(tm: TriangleMesh):
    """Convert a TriangleMesh to a manifold3d.Manifold for boolean ops."""
    import manifold3d as m3

    merged = _merge_duplicate_vertices(tm)
    verts = np.asarray(merged.vertices, dtype=np.float32)
    tris = np.asarray(merged.triangles, dtype=np.uint32)
    mesh_obj = m3.Mesh(vert_properties=verts, tri_verts=tris)
    return m3.Manifold(mesh_obj)


def _manifold_to_trianglemesh(name: str, manif) -> TriangleMesh:
    mesh_obj = manif.to_mesh()
    verts_array = np.asarray(mesh_obj.vert_properties)[:, :3]
    tris_array = np.asarray(mesh_obj.tri_verts)
    vertices = [tuple(float(c) for c in v) for v in verts_array]
    triangles = [tuple(int(c) for c in t) for t in tris_array]
    return TriangleMesh(name, vertices, triangles)


def _shell_vent_slot_cutters():
    """Two staggered rows of ventilation slots on every side wall.

    76 vertical slots total (long walls 11 × 2 rows × 2 sides, short walls
    8 × 2 rows × 2 sides), all clear of the 4 case-screw boss positions.
    Each slot is 4 × 13 mm, fully pierces the 2.5 mm wall, and prints with
    zero overhang on FDM because the long axis is vertical.
    """
    import manifold3d as m3

    slot_width = 4.0   # opening width along the wall
    slot_height = 13.0  # vertical (Z) opening height per row
    slot_punch = 8.0   # depth perpendicular to wall (must exceed wall thickness)
    z_centers = (11.5, 30.5)  # two stacked rows: below Jetson, above Jetson
    cy = INTEGRATED_BASE_Y_CENTER_MM             # 15.0 mm

    cutters = []
    half_w_outer = INTEGRATED_BASE_OUTER_MM[0] / 2.0   # 95
    half_d_outer = INTEGRATED_BASE_OUTER_MM[1] / 2.0   # 115

    # Long side walls (perpendicular to X, at x = ±95).
    # 11 columns of slots, span y = cy ± 90, gap to case-screw boss at y = -88/118 ~= 12 mm.
    long_y_count = 11
    long_y_span = 180.0
    long_y_positions = [
        cy + (i - (long_y_count - 1) / 2.0) * (long_y_span / (long_y_count - 1))
        for i in range(long_y_count)
    ]
    for sign_x in (-1.0, 1.0):
        for y_pos in long_y_positions:
            for z_c in z_centers:
                box = m3.Manifold.cube([slot_punch, slot_width, slot_height], center=True)
                box = box.translate([sign_x * half_w_outer, y_pos, z_c])
                cutters.append(box)

    # Front wall (sign_y = +1) — keep both vent rows.
    short_x_count = 8
    short_x_span = 140.0
    short_x_positions = [
        (i - (short_x_count - 1) / 2.0) * (short_x_span / (short_x_count - 1))
        for i in range(short_x_count)
    ]
    for x_pos in short_x_positions:
        for z_c in z_centers:
            box = m3.Manifold.cube([slot_width, slot_punch, slot_height], center=True)
            box = box.translate([x_pos, cy + half_d_outer, z_c])
            cutters.append(box)

    # Back wall (sign_y = -1) — large rectangular opening for the Jetson Nano
    # I/O panel (power barrel / USB-C, microSD edge, 4 × USB 3.0, HDMI,
    # DisplayPort, Gigabit Ethernet).  Replaces the vent slot column on this
    # wall; the opening itself contributes plenty of ventilation.
    io_window_width = 95.0   # spans X[-47.5, 47.5], clears case-screw bosses at x=±82
    io_window_height = 22.0  # spans Z[8, 30], covers PCB (z=11) up through tallest port (~z=27)
    io_window_z_center = 19.0
    io_box = m3.Manifold.cube([io_window_width, slot_punch, io_window_height], center=True)
    io_box = io_box.translate([0.0, cy - half_d_outer, io_window_z_center])
    cutters.append(io_box)

    # Back wall — 2 × Φ7 mm button holes for off-board Power and Reset buttons
    # mounted above the I/O window.  Z = 36 mm (≈6 mm above the I/O window top
    # at z = 30 mm, clear of the upper case-screw bosses at z = 32 mm).  X = ±25
    # mm keeps them away from the I/O window edges and centred between the back
    # wall midline and the case-screw bosses at x = ±82 mm.
    button_radius = 3.5    # Φ7 mm self-cut hole for 12 mm tactile switch nut + recess
    button_z = 36.0
    for sign_x in (-1.0, 1.0):
        btn = m3.Manifold.cylinder(slot_punch, button_radius, button_radius, 32, center=False)
        btn = btn.rotate([90.0, 0.0, 0.0])  # axis aligned with -Y (through back wall)
        btn = btn.translate([sign_x * 25.0, cy - half_d_outer + slot_punch / 2.0, button_z])
        cutters.append(btn)

    # Union all slots into a single Manifold for one boolean subtract.
    combined = cutters[0]
    for c in cutters[1:]:
        combined = combined + c
    return combined


def _cover_arm_mount_screw_cutters():
    """4 vertical Φ2.5 mm cylinders through the full cover thickness for the
    STS3215 horn anchor screws.

    With the cover now a 6 mm flat plate (no collar), the bores run cleanly
    from below the plate's bottom face up through the lip top.  M2 screws
    drop in from above the horn, pass through the horn's clearance holes,
    and self-tap into the cover's PLA — engaging ≈ 4 mm of material which
    is sufficient for the light arm-anchor load.  The arm + motor body
    (housed inside the lamp arm's bottom segment) hangs above the cover
    via the geared horn, exactly matching the original LeLamp design.
    """
    import manifold3d as m3

    plate_thickness = COVER_PLATE_THICKNESS_MM
    lip_height = COVER_LIP_HEIGHT_MM
    z_bot = -1.0
    z_top = plate_thickness + lip_height + 1.0
    cutter_height = z_top - z_bot
    cutters = []
    for cx, cy in ARM_MOUNT_SCREW_POSITIONS_MM:
        bore = m3.Manifold.cylinder(
            cutter_height,
            ARM_MOUNT_SCREW_CLEARANCE_RADIUS_MM,
            ARM_MOUNT_SCREW_CLEARANCE_RADIUS_MM,
            32,
            center=False,
        )
        bore = bore.translate([cx, cy, z_bot])
        cutters.append(bore)
    combined = cutters[0]
    for c in cutters[1:]:
        combined = combined + c
    return combined


def _shell_motor_mount_assembly():
    """Motor cradle + horn boss carved out for shaft / horn recess / screws.

    Returns a manifold3d Manifold sitting at world (0, 0) — to be unioned into
    the shell.  Contains:
      • a hollow rectangular cradle (28.7 × 49.3 outer, 24.7 × 45.3 inner) from
        z = MOTOR_BODY_BOTTOM_Z_MM (7) up to z = SHELL_BOSS_BOTTOM_Z_MM (43),
        sized for the STS3215 body with 0.05 mm slack;
      • a Φ24 boss on top of the cradle, z = 43 → 49, with a Φ12 shaft hole;
      • a Φ24 × 3 mm horn recess on top of the boss (z = 46 → 49);
      • 4 × M2 self-tap pilot holes (Φ1.7 mm) drilled vertically from the horn
        recess floor down through the boss into the cradle interior for arm
        anchor screws, on a Φ20 mm bolt circle;
      • 4 × M2 self-tap pilot holes (Φ1.7 mm) drilled horizontally through both
        long cradle walls for the STS3215's 4 body flange screws.
    """
    import manifold3d as m3

    cradle_iw, cradle_id = MOTOR_CRADLE_INNER_MM
    cradle_wall = MOTOR_CRADLE_WALL_MM
    cradle_ow = cradle_iw + 2.0 * cradle_wall
    cradle_od = cradle_id + 2.0 * cradle_wall
    z_floor = MOTOR_BODY_BOTTOM_Z_MM
    z_cradle_top = SHELL_BOSS_BOTTOM_Z_MM
    z_boss_top = SHELL_BOSS_TOP_Z_MM
    cradle_h = z_cradle_top - z_floor
    boss_h = z_boss_top - z_cradle_top

    # Cradle: outer block minus inner cavity → hollow tube
    outer = m3.Manifold.cube([cradle_ow, cradle_od, cradle_h], center=True)
    outer = outer.translate([0.0, 0.0, z_floor + cradle_h / 2.0])
    inner = m3.Manifold.cube([cradle_iw, cradle_id, cradle_h + 2.0], center=True)
    inner = inner.translate([0.0, 0.0, z_floor + cradle_h / 2.0])
    cradle = outer - inner

    # Boss: Φ24 solid cylinder above the cradle
    boss = m3.Manifold.cylinder(
        boss_h, SHELL_BOSS_OD_MM / 2.0, SHELL_BOSS_OD_MM / 2.0, 64, center=False
    )
    boss = boss.translate([0.0, 0.0, z_cradle_top])

    assembly = cradle + boss

    # Φ12 shaft hole runs through the full boss (and a touch into the cradle)
    shaft = m3.Manifold.cylinder(
        boss_h + 2.0, SHELL_BOSS_ID_MM / 2.0, SHELL_BOSS_ID_MM / 2.0, 64, center=False
    )
    shaft = shaft.translate([0.0, 0.0, z_cradle_top - 1.0])
    assembly = assembly - shaft

    # Horn recess Φ24 × HORN_RECESS_DEPTH at the boss top
    recess = m3.Manifold.cylinder(
        HORN_RECESS_DEPTH_MM + 0.5,
        HORN_RECESS_RADIUS_MM,
        HORN_RECESS_RADIUS_MM,
        64,
        center=False,
    )
    recess = recess.translate([0.0, 0.0, z_boss_top - HORN_RECESS_DEPTH_MM])
    assembly = assembly - recess

    # 4 arm-anchor M2 pilot bores around the shaft on a Φ20 mm bolt circle.
    # Bored from the horn recess floor (z = z_boss_top − HORN_RECESS_DEPTH)
    # down through the boss into the cradle wall material so M2 self-tap
    # gets ~6 mm of engagement.
    anchor_z_bottom = z_cradle_top - 2.0
    anchor_h = z_boss_top - anchor_z_bottom + 0.5
    for cx, cy in ARM_MOUNT_SCREW_POSITIONS_MM:
        anchor = m3.Manifold.cylinder(
            anchor_h,
            ARM_MOUNT_SCREW_CLEARANCE_RADIUS_MM,
            ARM_MOUNT_SCREW_CLEARANCE_RADIUS_MM,
            32,
            center=False,
        )
        anchor = anchor.translate([cx, cy, anchor_z_bottom])
        assembly = assembly - anchor

    # 4 horizontal M2 pilot holes through both cradle long walls for the
    # STS3215 body-flange screws.  Cylinders are oriented along ±X (drilled
    # through the X-facing walls at x = ±cradle_ow/2 ≈ ±14.35 mm).  Generous
    # length (4 × cradle_wall) ensures they punch fully through.
    flange_h = cradle_wall * 4.0
    for sign_x in (-1.0, 1.0):
        for y, z in STS3215_FLANGE_HOLES_YZ_MM:
            flange = m3.Manifold.cylinder(
                flange_h,
                STS3215_FLANGE_PILOT_RADIUS_MM,
                STS3215_FLANGE_PILOT_RADIUS_MM,
                24,
                center=False,
            )
            # Re-orient so cylinder axis is along X.
            flange = flange.rotate([0.0, 90.0, 0.0])
            flange = flange.translate(
                [sign_x * (cradle_ow / 2.0 + cradle_wall) , y, z]
            )
            assembly = assembly - flange
    return assembly


def build_integrated_lampbase_shell() -> TriangleMesh:
    """Native rounded-rectangle hollow shell with side-wall ventilation slots."""
    outer_w, outer_d, outer_h = INTEGRATED_BASE_OUTER_MM
    base_shell = _hollow_rounded_box(
        "AILamp_LampBase_Electronics_Shell",
        outer_w=outer_w,
        outer_d=outer_d,
        outer_h=outer_h,
        wall_t=INTEGRATED_BASE_WALL_MM,
        bottom_t=INTEGRATED_BASE_BOTTOM_MM,
        outer_radius=INTEGRATED_BASE_CORNER_RADIUS_MM,
        cy_offset=INTEGRATED_BASE_Y_CENTER_MM,
        segments=ROUNDED_CORNER_SEGMENTS,
    )
    # Subtract vent slots via manifold3d.  No motor cradle / boss inside the
    # shell — per the original LeLamp design the base yaw motor lives inside
    # the lamp arm's bottom segment, not in the base.  The base is a pure
    # electronics enclosure with horn anchor holes on the cover.
    shell_manif = _trianglemesh_to_manifold(base_shell)
    shell_manif = shell_manif - _shell_vent_slot_cutters()
    vented = _manifold_to_trianglemesh("AILamp_LampBase_Electronics_Shell", shell_manif)
    # Internal organisation (standoffs, ribs, case-screw bosses) sits inside the cavity.
    vented.extend_triangle_mesh(_integrated_base_internal_features())
    vented = _merge_duplicate_vertices(vented)
    return vented


def build_integrated_lampbase_cover() -> TriangleMesh:
    """Smooth rounded cover with an integrated perimeter lip and arm-mount collar.

    The cover is constructed as a single 2-manifold piece (no overlapping
    sub-meshes that would share each perimeter edge between four faces).  The
    outer wall is one continuous vertical band from the plate bottom up to the
    lip top; the plate top, lip inner wall, and lip top form an inset step
    around the perimeter.  The arm-mount collar and four screw bosses are
    added on top as disjoint manifolds.
    """
    plate_w, plate_d, total_plate_height = INTEGRATED_BASE_COVER_PLATE_MM
    plate_thickness = COVER_PLATE_THICKNESS_MM
    lip_height = total_plate_height - plate_thickness
    collar_w, collar_d, collar_h = BASE_ARM_MOUNT_OUTER_MM
    inner_w, inner_d = BASE_ARM_MOUNT_CLEARANCE_MM
    pass_w, pass_d = COVER_ARM_PASS_THROUGH_MM
    cy_offset = INTEGRATED_BASE_Y_CENTER_MM
    plate_radius = INTEGRATED_BASE_COVER_CORNER_RADIUS_MM
    pass_radius = COVER_PASS_THROUGH_CORNER_RADIUS_MM
    collar_outer_radius = BASE_ARM_MOUNT_COLLAR_CORNER_RADIUS_MM
    collar_inner_radius = BASE_ARM_MOUNT_COLLAR_INNER_RADIUS_MM
    segments = ROUNDED_CORNER_SEGMENTS

    lip_inner_w = plate_w - 2.0 * INTEGRATED_BASE_WALL_MM
    lip_inner_d = plate_d - 2.0 * INTEGRATED_BASE_WALL_MM
    lip_inner_radius = max(plate_radius - INTEGRATED_BASE_WALL_MM, 4.0)

    outer_pts = _rounded_rect_polygon(plate_w, plate_d, plate_radius, segments, cy=cy_offset)
    lip_inner_pts = _rounded_rect_polygon(
        lip_inner_w, lip_inner_d, lip_inner_radius, segments, cy=cy_offset
    )
    hole_pts = _rounded_rect_polygon(pass_w, pass_d, pass_radius, segments)
    # Collar bottom outer footprint — used both to subtract from the plate top
    # (so the plate doesn't double-cover the collar floor) and as the bottom
    # rim of the collar wall itself.
    collar_bottom_outer_pts = _rounded_rect_polygon(
        collar_w, collar_d, collar_outer_radius, segments
    )
    case_screw_holes = [
        _circle_polygon(cx, cy, COVER_CASE_SCREW_CLEARANCE_RADIUS_MM, segments=16)
        for cx, cy in INTEGRATED_BASE_CASE_SCREW_POSITIONS_MM
    ]

    cover = TriangleMesh("AILamp_LampBase_Electronics_Cover")
    # Bottom face: outer perimeter minus the central arm hole and the 4 case-screw holes, facing -z.
    cover.extend_triangle_mesh(
        _holey_cap(
            "cover_bottom",
            outer_pts,
            [hole_pts] + case_screw_holes,
            z=0.0,
            facing_up=False,
        )
    )
    # Continuous outer perimeter wall from z=0 up to lip top.
    cover.extend_triangle_mesh(
        _extruded_wall(
            "cover_outer_wall", outer_pts, 0.0, plate_thickness + lip_height, outward=True
        )
    )
    # Central pass-through inner wall — extruded only through the plate body
    # (z = 0 → plate_thickness).  Above the plate the inner area is open
    # because the lip only exists around the perimeter; the shell's horn boss
    # protrudes through this Φ28 cutout from below.
    cover.extend_triangle_mesh(
        _extruded_wall(
            "cover_inner_wall",
            hole_pts,
            0.0,
            plate_thickness,
            outward=False,
        )
    )
    # Each case-screw hole's inner wall (z=0 → z=plate_thickness).
    for index, hole in enumerate(case_screw_holes):
        cover.extend_triangle_mesh(
            _extruded_wall(
                f"cover_case_screw_wall_{index}",
                hole,
                0.0,
                plate_thickness,
                outward=False,
            )
        )
    # Plate top inside the lip: lip-inner perimeter minus the same 5 holes, facing +z.
    cover.extend_triangle_mesh(
        _holey_cap(
            "cover_plate_top",
            lip_inner_pts,
            [hole_pts] + case_screw_holes,
            z=plate_thickness,
            facing_up=True,
        )
    )
    # Lip inner wall (z=plate_thickness → z=plate_thickness+lip_height).
    cover.extend_triangle_mesh(
        _extruded_wall(
            "cover_lip_inner_wall",
            lip_inner_pts,
            plate_thickness,
            plate_thickness + lip_height,
            outward=False,
        )
    )
    # Lip top annulus: outer to lip-inner at the lip top, facing +z.
    cover.extend_triangle_mesh(
        _annular_strip(
            "cover_lip_top",
            outer_pts,
            lip_inner_pts,
            plate_thickness + lip_height,
            facing_up=True,
        )
    )

    # Simple 6 mm flat plate matching original LeLamp lamp_base_cover.stl.
    # The base yaw motor + horn live ABOVE the cover (motor body housed
    # inside the lamp arm's bottom segment).  The arm's horn (Φ24 metal
    # disc) bolts down to the cover via 4 M2 self-tap screws on a Φ20 mm
    # bolt circle; the central Φ12 hole passes the motor cables down into
    # the base where the servo driver lives.
    cover_manif = _trianglemesh_to_manifold(cover)
    cover_manif = cover_manif - _cover_arm_mount_screw_cutters()
    cover = _manifold_to_trianglemesh("AILamp_LampBase_Electronics_Cover", cover_manif)
    cover = _merge_duplicate_vertices(cover)
    return cover


def _cover_horn_recess_cutter():
    """Φ24 × 2 mm cylindrical recess at the centre of the collar top.

    The STS3215 metal horn nests here flush.  Cut from the cover collar's
    top face (z = plate + collar) downward by HORN_RECESS_DEPTH_MM so the
    arm's underside meets the collar top (not the horn) when bolted on.
    """
    import manifold3d as m3

    plate_thickness = COVER_PLATE_THICKNESS_MM
    collar_h = BASE_ARM_MOUNT_OUTER_MM[2]
    z_top = plate_thickness + collar_h
    recess = m3.Manifold.cylinder(
        HORN_RECESS_DEPTH_MM + 0.5,
        HORN_RECESS_RADIUS_MM,
        HORN_RECESS_RADIUS_MM,
        64,
        center=False,
    )
    recess = recess.translate([0.0, 0.0, z_top - HORN_RECESS_DEPTH_MM])
    return recess


def build_base_arm_link_boot() -> TriangleMesh:
    """Rounded-rectangle boot that bridges the cover collar to the arm root.

    Implemented as a single closed rounded tube (outer/inner rounded
    rectangles capped with annular top and bottom faces).  This is smoother
    than the previous box-stack frame and visually matches the new cover
    collar instead of a square box approximation.
    """
    outer_w, outer_d, outer_h = BASE_ARM_LINK_BOOT_MM
    inner_w, inner_d = BASE_ARM_LINK_BOOT_CLEARANCE_MM
    boot = _rounded_collar(
        "AILamp_Base_Arm_Link_Boot",
        outer_w=outer_w,
        outer_d=outer_d,
        inner_w=inner_w,
        inner_d=inner_d,
        height=outer_h,
        z_bottom=0.0,
        outer_radius=BASE_ARM_LINK_BOOT_CORNER_RADIUS_MM,
        inner_radius=BASE_ARM_LINK_BOOT_INNER_CORNER_RADIUS_MM,
        segments=ROUNDED_CORNER_SEGMENTS,
    )
    return boot


def build_jetson_tray() -> Mesh:
    mesh = Mesh("AILamp_Jetson_Nano_Base_Tray")
    outer = (122.0, 102.0, 12.0)
    _base_plate(mesh, outer, 3.0)
    _edge_rails(mesh, outer, 4.0, 9.0, 3.0)
    mesh.subtract_box(0.0, -49.0, 7.5, 44.0, 10.0, 12.0)
    for x in (-42.0, 42.0):
        for y in (-32.0, 32.0):
            mesh.add_box(x, y, 5.0, 8.0, 8.0, 4.0)
            mesh.subtract_box(x, y, 5.0, 3.2, 3.2, 10.0)
    for y in (-42.0, 42.0):
        mesh.subtract_box(0.0, y, 1.5, ZIP_TIE_SLOT_MM[0], ZIP_TIE_SLOT_MM[1], 6.0)
    return mesh


def build_electronics_side_deck() -> Mesh:
    mesh = Mesh("AILamp_Electronics_Side_Deck")
    outer = (145.0, 48.0, 10.0)
    _base_plate(mesh, outer, 3.0)
    mesh.add_box(-36.5, 21.0, 6.5, 72.0, 4.0, 7.0)
    mesh.add_box(-36.5, -21.0, 6.5, 72.0, 4.0, 7.0)
    mesh.add_box(37.0, 21.0, 6.5, 62.0, 4.0, 7.0)
    mesh.add_box(37.0, -21.0, 6.5, 62.0, 4.0, 7.0)
    mesh.add_box(0.0, 0.0, 5.0, 5.0, 42.0, 4.0)
    for x in (-66.0, -7.0, 18.0, 64.0):
        mesh.add_box(x, 0.0, 5.0, 6.0, 8.0, 4.0)
    for x in (
        -36.5 - SERVO_DRIVER_MOUNT_HOLE_MM[0] / 2.0,
        -36.5 + SERVO_DRIVER_MOUNT_HOLE_MM[0] / 2.0,
    ):
        for y in (
            -SERVO_DRIVER_MOUNT_HOLE_MM[1] / 2.0,
            SERVO_DRIVER_MOUNT_HOLE_MM[1] / 2.0,
        ):
            mesh.subtract_box(x, y, 2.0, 3.4, 3.4, 8.0)
    mesh.subtract_box(68.0, -21.0, 6.5, PICO_USB_RELIEF_MM[0], 10.0, 10.0)
    mesh.subtract_box(37.0, 0.0, 1.5, 38.0, 4.0, 6.0)
    return mesh


def build_cable_clip(
    name: str, channel_width_mm: float, outer_mm: tuple[float, float, float]
) -> Mesh:
    mesh = Mesh(name)
    sx, sy, sz = outer_mm
    wall = max(3.0, (sx - channel_width_mm) / 2.0)
    _base_plate(mesh, outer_mm, 2.5)
    mesh.add_box(-sx / 2.0 + wall / 2.0, 0.0, sz / 2.0, wall, sy, sz - 2.5)
    mesh.add_box(sx / 2.0 - wall / 2.0, 0.0, sz / 2.0, wall, sy, sz - 2.5)
    mesh.add_box(0.0, -sy / 2.0 + 2.0, sz / 2.0, sx, 4.0, sz - 2.5)
    mesh.add_box(0.0, sy / 2.0 - 1.5, sz - 1.0, sx, 3.0, 2.0)
    mesh.subtract_box(0.0, 0.0, 1.25, sx - 8.0, 4.0, 5.0)
    return mesh


def generate_all(output_dir: Path | str = Path("3D/AILamp_Adapters")) -> list[Path]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    builders = {
        "AILamp_LampBase_Electronics_Shell": build_integrated_lampbase_shell,
        "AILamp_LampBase_Electronics_Cover": build_integrated_lampbase_cover,
        "AILamp_Base_Arm_Link_Boot": build_base_arm_link_boot,
        "AILamp_Jetson_Nano_Base_Tray": build_jetson_tray,
        "AILamp_Electronics_Side_Deck": build_electronics_side_deck,
        "AILamp_Cable_Clip_6mm": lambda: build_cable_clip(
            "AILamp_Cable_Clip_6mm",
            _loose_cable_channel_width(6.0),
            (24.0, 18.0, 8.0),
        ),
        "AILamp_Cable_Clip_10mm": lambda: build_cable_clip(
            "AILamp_Cable_Clip_10mm",
            _loose_cable_channel_width(10.0),
            (30.0, 22.0, 10.0),
        ),
    }
    expected_files = {
        output / f"{name}{suffix}"
        for name in builders
        for suffix in (".3mf", ".stl")
    }
    for stale_path in output.glob("AILamp_*"):
        if stale_path.suffix in {".3mf", ".stl"} and stale_path not in expected_files:
            stale_path.unlink()

    written: list[Path] = []
    for spec in adapter_specs():
        mesh = builders[spec.name]()
        stl_path = output / f"{spec.name}.stl"
        model_path = output / f"{spec.name}.3mf"
        mesh.write_3mf(model_path)
        mesh.write_stl(stl_path)
        written.extend([model_path, stl_path])
    return written


if __name__ == "__main__":
    for generated_path in generate_all():
        print(generated_path)
