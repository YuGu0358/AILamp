from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import zipfile


PCB_EDGE_CLEARANCE_MM = 1.5
CABLE_EXIT_EXTRA_WIDTH_MM = 2.5
SCREW_HOLE_EXTRA_DIAMETER_MM = 0.4
M12_LENS_RADIAL_CLEARANCE_MM = 1.0
JETSON_BOARD_MM = (100.0, 80.0)
PICO_BOARD_MM = (51.0, 21.0)
SERVO_DRIVER_BOARD_MM = (65.0, 30.0)
NEOMATRIX_BOARD_MM = (71.17, 71.17)
CAMERA_BOARD_MM = (32.0, 32.0)
RESPEAKER_CASE_MM = (86.0, 35.0)


CORE_NAMESPACE = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
FIXED_ZIP_TIMESTAMP = (2026, 1, 1, 0, 0, 0)
ELECTRONICS_SIDE_DECK_WIRING_ALLOWANCE_MM = (26.0, 15.0)


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
            "AILamp_Head_Camera_Mount",
            (48.0, 42.0, 10.0),
            _loose_pcb_pocket(CAMERA_BOARD_MM),
            "Arducam UB0234 reversible head mount",
        ),
        AdapterSpec(
            "AILamp_NeoMatrix_Holder",
            (86.0, 86.0, 8.0),
            _loose_pcb_pocket(NEOMATRIX_BOARD_MM),
            "Loose NeoMatrix holder behind diffuser",
        ),
        AdapterSpec(
            "AILamp_ReSpeaker_External_Mount",
            (101.0, 50.0, 9.0),
            _loose_pcb_pocket(RESPEAKER_CASE_MM),
            "External ReSpeaker XVF3800 mount",
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
        self.vertices: list[tuple[float, float, float]] = []
        self.triangles: list[tuple[int, int, int]] = []

    def add_box(
        self,
        cx: float,
        cy: float,
        cz: float,
        sx: float,
        sy: float,
        sz: float,
    ) -> None:
        x0 = cx - sx / 2.0
        x1 = cx + sx / 2.0
        y0 = cy - sy / 2.0
        y1 = cy + sy / 2.0
        z0 = cz - sz / 2.0
        z1 = cz + sz / 2.0
        first = len(self.vertices)
        self.vertices.extend(
            [
                (x0, y0, z0),
                (x1, y0, z0),
                (x1, y1, z0),
                (x0, y1, z0),
                (x0, y0, z1),
                (x1, y0, z1),
                (x1, y1, z1),
                (x0, y1, z1),
            ]
        )
        faces = [
            (0, 2, 1),
            (0, 3, 2),
            (4, 5, 6),
            (4, 6, 7),
            (0, 1, 5),
            (0, 5, 4),
            (1, 2, 6),
            (1, 6, 5),
            (2, 3, 7),
            (2, 7, 6),
            (3, 0, 4),
            (3, 4, 7),
        ]
        self.triangles.extend(
            (first + a, first + b, first + c) for a, b, c in faces
        )

    def write_stl(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        lines = [f"solid {self.name}"]
        for a, b, c in self.triangles:
            va = self.vertices[a]
            vb = self.vertices[b]
            vc = self.vertices[c]
            lines.extend(
                [
                    "  facet normal 0 0 0",
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


def _write_zip_entry(archive: zipfile.ZipFile, filename: str, text: str) -> None:
    info = zipfile.ZipInfo(filename=filename, date_time=FIXED_ZIP_TIMESTAMP)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o644 << 16
    archive.writestr(info, text.encode("utf-8"))


def _base_plate(mesh: Mesh, outer_mm: tuple[float, float, float], thickness: float) -> None:
    mesh.add_box(0.0, 0.0, thickness / 2.0, outer_mm[0], outer_mm[1], thickness)


def _edge_rails(
    mesh: Mesh,
    outer_mm: tuple[float, float, float],
    rail_width: float,
    rail_height: float,
    z_bottom: float,
) -> None:
    sx, sy, _ = outer_mm
    cz = z_bottom + rail_height / 2.0
    mesh.add_box(0.0, sy / 2.0 - rail_width / 2.0, cz, sx, rail_width, rail_height)
    mesh.add_box(0.0, -sy / 2.0 + rail_width / 2.0, cz, sx, rail_width, rail_height)
    mesh.add_box(-sx / 2.0 + rail_width / 2.0, 0.0, cz, rail_width, sy, rail_height)
    mesh.add_box(sx / 2.0 - rail_width / 2.0, 0.0, cz, rail_width, sy, rail_height)


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


def build_jetson_tray() -> Mesh:
    mesh = Mesh("AILamp_Jetson_Nano_Base_Tray")
    outer = (122.0, 102.0, 12.0)
    _base_plate(mesh, outer, 3.0)
    _edge_rails(mesh, outer, 4.0, 9.0, 3.0)
    for x in (-42.0, 42.0):
        for y in (-32.0, 32.0):
            mesh.add_box(x, y, 5.0, 8.0, 8.0, 4.0)
    mesh.add_box(0.0, -47.0, 6.0, 38.0, 4.0, 6.0)
    return mesh


def build_electronics_side_deck() -> Mesh:
    mesh = Mesh("AILamp_Electronics_Side_Deck")
    outer = (145.0, 48.0, 10.0)
    _base_plate(mesh, outer, 3.0)
    mesh.add_box(-35.0, 21.0, 6.5, 60.0, 4.0, 7.0)
    mesh.add_box(-35.0, -21.0, 6.5, 60.0, 4.0, 7.0)
    mesh.add_box(35.5, 21.0, 6.5, 74.0, 4.0, 7.0)
    mesh.add_box(35.5, -21.0, 6.5, 74.0, 4.0, 7.0)
    mesh.add_box(0.0, 0.0, 5.0, 5.0, 42.0, 4.0)
    for x in (-65.0, -5.0, 18.0, 68.0):
        mesh.add_box(x, 0.0, 5.0, 6.0, 8.0, 4.0)
    return mesh


def build_camera_mount() -> Mesh:
    mesh = Mesh("AILamp_Head_Camera_Mount")
    outer_x, outer_y, _ = (48.0, 42.0, 10.0)
    plate_thickness = 3.0
    screw_clearance = 3.0 + SCREW_HOLE_EXTRA_DIAMETER_MM
    cutouts = (
        (0.0, 0.0, 16.0, 16.0),
        (-14.0, -14.0, screw_clearance, screw_clearance),
        (-14.0, 14.0, screw_clearance, screw_clearance),
        (14.0, -14.0, screw_clearance, screw_clearance),
        (14.0, 14.0, screw_clearance, screw_clearance),
    )
    _plate_with_square_cutouts(mesh, outer_x, outer_y, plate_thickness, cutouts)
    _edge_rails(mesh, (outer_x, outer_y, 10.0), 3.0, 7.0, 3.0)
    # Raised bosses stop short of each 3.4 x 3.4 mm through-gap.
    for x in (-14.0, 14.0):
        for y in (-14.0, 14.0):
            mesh.add_box(x - 3.5, y, 5.5, 2.0, 8.0, 3.0)
            mesh.add_box(x + 3.5, y, 5.5, 2.0, 8.0, 3.0)
            mesh.add_box(x, y - 3.5, 5.5, 5.0, 2.0, 3.0)
            mesh.add_box(x, y + 3.5, 5.5, 5.0, 2.0, 3.0)
    return mesh


def build_neomatrix_holder() -> Mesh:
    mesh = Mesh("AILamp_NeoMatrix_Holder")
    outer = (86.0, 86.0, 8.0)
    _base_plate(mesh, outer, 2.5)
    _edge_rails(mesh, outer, 5.0, 5.5, 2.5)
    for x in (-32.0, 32.0):
        mesh.add_box(x, 0.0, 5.0, 4.0, 70.0, 3.0)
    for y in (-32.0, 32.0):
        mesh.add_box(0.0, y, 5.0, 70.0, 4.0, 3.0)
    return mesh


def build_respeaker_mount() -> Mesh:
    mesh = Mesh("AILamp_ReSpeaker_External_Mount")
    outer = (101.0, 50.0, 9.0)
    _base_plate(mesh, outer, 3.0)
    _edge_rails(mesh, outer, 4.0, 6.0, 3.0)
    mesh.add_box(-43.0, 0.0, 5.5, 5.0, 36.0, 4.0)
    mesh.add_box(43.0, 0.0, 5.5, 5.0, 36.0, 4.0)
    for x in (-30.0, 0.0, 30.0):
        mesh.add_box(x, 21.0, 5.0, 10.0, 4.0, 3.0)
        mesh.add_box(x, -21.0, 5.0, 10.0, 4.0, 3.0)
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
    return mesh


def generate_all(output_dir: Path | str = Path("3D/AILamp_Adapters")) -> list[Path]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    builders = {
        "AILamp_Jetson_Nano_Base_Tray": build_jetson_tray,
        "AILamp_Electronics_Side_Deck": build_electronics_side_deck,
        "AILamp_Head_Camera_Mount": build_camera_mount,
        "AILamp_NeoMatrix_Holder": build_neomatrix_holder,
        "AILamp_ReSpeaker_External_Mount": build_respeaker_mount,
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
