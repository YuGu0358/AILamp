import hashlib
import importlib.util
from pathlib import Path
import sys
import zipfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
GENERATOR_PATH = ROOT / "scripts/generate_ailamp_adapters.py"
SIM_SYNC_PATH = ROOT / "scripts/sync_simulation_adapter_meshes.py"
ORIGINAL_PRINT_FILES = [
    ROOT / "3D/LampArm (Base-Elbow).3mf",
    ROOT / "3D/LampArm (Elbow-Wrist).3mf",
    ROOT / "3D/LampArm (Pitch).3mf",
    ROOT / "3D/LampBase - Cover.3mf",
    ROOT / "3D/LampBase.3mf",
    ROOT / "3D/LampHead - Diffuser.3mf",
    ROOT / "3D/LampHead.3mf",
]


def load_generator():
    spec = importlib.util.spec_from_file_location(
        "generate_ailamp_adapters", GENERATOR_PATH
    )
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_sim_sync():
    spec = importlib.util.spec_from_file_location(
        "sync_simulation_adapter_meshes", SIM_SYNC_PATH
    )
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_3mf_bounds(path):
    vertices, _triangles = parse_3mf_mesh(path)
    xs = [x for x, _y, _z in vertices]
    ys = [y for _x, y, _z in vertices]
    zs = [z for _x, _y, z in vertices]
    return max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)


def parse_3mf_mesh(path):
    namespace = {"core": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
    with zipfile.ZipFile(path) as archive:
        assert set(archive.namelist()) == {
            "[Content_Types].xml",
            "_rels/.rels",
            "3D/3dmodel.model",
        }
        with archive.open("3D/3dmodel.model") as model_file:
            root = ET.parse(model_file).getroot()

    vertices = root.findall(".//core:vertex", namespace)
    triangles = root.findall(".//core:triangle", namespace)
    parsed_vertices = [
        (
            float(vertex.attrib["x"]),
            float(vertex.attrib["y"]),
            float(vertex.attrib["z"]),
        )
        for vertex in vertices
    ]
    parsed_triangles = [
        (
            int(triangle.attrib["v1"]),
            int(triangle.attrib["v2"]),
            int(triangle.attrib["v3"]),
        )
        for triangle in triangles
    ]
    return parsed_vertices, parsed_triangles


def count_non_manifold_edges(vertices, triangles):
    coordinate_to_index = {}
    for vertex in vertices:
        coordinate_to_index.setdefault(tuple(round(value, 4) for value in vertex), len(coordinate_to_index))

    edge_counts = {}
    for triangle in triangles:
        remapped = [coordinate_to_index[tuple(round(vertices[index][axis], 4) for axis in range(3))] for index in triangle]
        for start, end in (
            (remapped[0], remapped[1]),
            (remapped[1], remapped[2]),
            (remapped[2], remapped[0]),
        ):
            edge = tuple(sorted((start, end)))
            edge_counts[edge] = edge_counts.get(edge, 0) + 1

    return sum(1 for count in edge_counts.values() if count != 2)


def assert_3mf_triangle_indices_are_valid(path):
    vertices, triangles = parse_3mf_mesh(path)
    assert vertices
    assert triangles
    vertex_count = len(vertices)
    for triangle in triangles:
        assert len(set(triangle)) == 3
        for index in triangle:
            assert 0 <= index < vertex_count


def assert_3mf_is_two_manifold(path):
    vertices, triangles = parse_3mf_mesh(path)
    assert count_non_manifold_edges(vertices, triangles) == 0


def test_adapter_specs_use_loose_fit_clearances():
    generator = load_generator()

    assert generator.PCB_EDGE_CLEARANCE_MM == 1.5
    assert generator.CABLE_EXIT_EXTRA_WIDTH_MM == 2.5

    specs = {spec.name: spec for spec in generator.adapter_specs()}
    pcb_clearance = generator.PCB_EDGE_CLEARANCE_MM * 2.0
    assert specs["AILamp_Jetson_Nano_Base_Tray"].board_pocket_mm == (
        generator.JETSON_BOARD_MM[0] + pcb_clearance,
        generator.JETSON_BOARD_MM[1] + pcb_clearance,
    )
    assert specs["AILamp_Electronics_Side_Deck"].board_pocket_mm == (145.0, 48.0)
    assert specs["AILamp_LampBase_Electronics_Shell"].outer_mm == (
        190.0,
        230.0,
        42.0,
    )
    assert generator.INTEGRATED_BASE_COVER_PLATE_MM == (190.0, 230.0, 6.0)
    assert generator.INTEGRATED_BASE_COVER_MM == (190.0, 230.0, 6.0)
    assert generator.INTEGRATED_BASE_INTERNAL_HEIGHT_MM >= generator.JETSON_ASSEMBLY_HEIGHT_MM + 10.0
    assert specs["AILamp_LampBase_Electronics_Cover"].board_pocket_mm == (12.0, 12.0)
    assert generator.COVER_ARM_PASS_THROUGH_MM == (12.0, 12.0)
    assert generator.BASE_ARM_MOUNT_OUTER_MM == (92.0, 98.0, 0.0)
    assert generator.BASE_ARM_MOUNT_CLEARANCE_MM == (12.0, 12.0)
    assert generator.ARM_MOUNT_SCREW_POSITIONS_MM == (
        (0.0, -10.0),
        (0.0, 10.0),
        (-10.0, 0.0),
        (10.0, 0.0),
    )
    assert generator.ARM_MOUNT_SCREW_CLEARANCE_RADIUS_MM == 1.25
    assert generator.HORN_RECESS_RADIUS_MM == 12.0
    assert generator.HORN_RECESS_DEPTH_MM == 3.0
    assert generator.MOTOR_CRADLE_INNER_MM == (24.7, 45.3)
    assert generator.MOTOR_CRADLE_HEIGHT_MM == 36.0
    assert generator.SHELL_BOSS_OD_MM == 26.0
    assert generator.SHELL_BOSS_ID_MM == 12.0
    assert generator.SHELL_BOSS_TOP_Z_MM == 49.0
    assert generator.BASE_ARM_MOUNT_SCREW_POSITIONS_MM == (
        (-36.0, -40.0),
        (-36.0, 40.0),
        (36.0, -40.0),
        (36.0, 40.0),
    )
    assert specs["AILamp_Base_Arm_Link_Boot"].outer_mm == (74.0, 74.0, 42.0)
    assert specs["AILamp_Base_Arm_Link_Boot"].board_pocket_mm == (42.0, 48.0)
    assert generator.BASE_ARM_LINK_BOOT_MM == (74.0, 74.0, 42.0)
    assert generator.BASE_ARM_LINK_BOOT_CLEARANCE_MM == (42.0, 48.0)
    assert generator.INTEGRATED_BASE_Y_CENTER_MM == 15.0
    assert generator.INTEGRATED_BASE_CASE_SCREW_POSITIONS_MM == (
        (-82.0, -88.0),
        (-82.0, 118.0),
        (82.0, -88.0),
        (82.0, 118.0),
    )
    assert generator.INTEGRATED_BASE_CORNER_RADIUS_MM == 18.0
    assert generator.INTEGRATED_BASE_COVER_CORNER_RADIUS_MM == 16.0
    assert generator.JETSON_STANDOFF_SPACING_MM == (84.0, 64.0)
    assert generator.CASE_SCREW_CLEARANCE_MM == 4.0
    assert generator.SERVO_DRIVER_MOUNT_HOLE_MM == (58.0, 23.0)
    assert generator.PICO_USB_RELIEF_MM[0] >= 12.0
    assert specs["AILamp_Cable_Clip_6mm"].board_pocket_mm[0] == (
        6.0 + generator.CABLE_EXIT_EXTRA_WIDTH_MM
    )
    assert specs["AILamp_Cable_Clip_10mm"].board_pocket_mm[0] == (
        10.0 + generator.CABLE_EXIT_EXTRA_WIDTH_MM
    )


def test_generator_writes_all_adapter_files_without_touching_originals(tmp_path):
    generator = load_generator()
    expected_adapter_names = {
        "AILamp_LampBase_Electronics_Shell",
        "AILamp_LampBase_Electronics_Cover",
        "AILamp_Base_Arm_Link_Boot",
        "AILamp_Jetson_Nano_Base_Tray",
        "AILamp_Electronics_Side_Deck",
        "AILamp_Cable_Clip_6mm",
        "AILamp_Cable_Clip_10mm",
    }
    specs = {spec.name: spec for spec in generator.adapter_specs()}
    assert set(specs) == expected_adapter_names

    original_hashes = {path: sha256(path) for path in ORIGINAL_PRINT_FILES}
    original_bytes = {path: path.read_bytes() for path in ORIGINAL_PRINT_FILES}
    output_dir = tmp_path / "adapters"
    stale_adapter = output_dir / "AILamp_Head_Camera_Mount.stl"
    output_dir.mkdir()
    stale_adapter.write_text("stale generated adapter\n")
    mutated_originals = []

    try:
        generator.generate_all(output_dir)
    finally:
        for path, snapshot in original_bytes.items():
            current_bytes = path.read_bytes() if path.exists() else None
            if current_bytes != snapshot:
                mutated_originals.append(path)
                path.write_bytes(snapshot)

    assert not mutated_originals, (
        "generate_all mutated original print files: "
        + ", ".join(str(path) for path in mutated_originals)
    )

    for adapter_name in sorted(specs):
        for suffix in (".3mf", ".stl"):
            output_file = output_dir / f"{adapter_name}{suffix}"
            assert output_file.is_file()
            assert output_file.stat().st_size > 1000
    assert not stale_adapter.exists()

    jetson_bounds = parse_3mf_bounds(output_dir / "AILamp_Jetson_Nano_Base_Tray.3mf")
    assert jetson_bounds[0] >= 122.0
    assert jetson_bounds[1] >= 102.0
    assert jetson_bounds[2] >= 4.0

    assert {path: sha256(path) for path in ORIGINAL_PRINT_FILES} == original_hashes


def test_generated_adapter_files_match_specs_and_are_reproducible(tmp_path):
    generator = load_generator()
    first_output_dir = tmp_path / "first"
    second_output_dir = tmp_path / "second"
    specs = {spec.name: spec for spec in generator.adapter_specs()}

    generator.generate_all(first_output_dir)
    generator.generate_all(str(second_output_dir))

    for adapter_name, spec in specs.items():
        first_3mf = first_output_dir / f"{adapter_name}.3mf"
        second_3mf = second_output_dir / f"{adapter_name}.3mf"
        first_stl = first_output_dir / f"{adapter_name}.stl"
        second_stl = second_output_dir / f"{adapter_name}.stl"

        assert first_3mf.read_bytes() == second_3mf.read_bytes()
        assert first_stl.read_bytes() == second_stl.read_bytes()

        assert tuple(round(value, 2) for value in parse_3mf_bounds(first_3mf)) == (
            round(spec.outer_mm[0], 2),
            round(spec.outer_mm[1], 2),
            round(spec.outer_mm[2], 2),
        )
        assert_3mf_triangle_indices_are_valid(first_3mf)
        assert_3mf_is_two_manifold(first_3mf)

        stl_lines = first_stl.read_text().splitlines()
        assert stl_lines[0] == f"solid {adapter_name}"
        assert stl_lines[-1] == f"endsolid {adapter_name}"


def test_checked_in_adapter_files_match_fresh_generation(tmp_path):
    generator = load_generator()
    generated_dir = tmp_path / "generated"

    generator.generate_all(generated_dir)

    for spec in generator.adapter_specs():
        for suffix in (".3mf", ".stl"):
            checked_in = ROOT / "3D/AILamp_Adapters" / f"{spec.name}{suffix}"
            generated = generated_dir / f"{spec.name}{suffix}"
            assert checked_in.read_bytes() == generated.read_bytes()

        checked_in_3mf = ROOT / "3D/AILamp_Adapters" / f"{spec.name}.3mf"
        assert_3mf_is_two_manifold(checked_in_3mf)


def test_simulation_adapter_meshes_match_print_stl_exports(tmp_path):
    generator = load_generator()
    sim_sync = load_sim_sync()
    generated_sim_dir = tmp_path / "simulation_meshes"
    generated_sim_dir.mkdir()
    stale_mesh = generated_sim_dir / "AILamp_Head_Camera_Mount.stl"
    stale_mesh.write_bytes(b"stale mesh")

    sim_sync.sync_adapter_meshes(ROOT / "3D/AILamp_Adapters", generated_sim_dir)
    assert not stale_mesh.exists()

    for spec in generator.adapter_specs():
        checked_in = ROOT / "simulation/assets/ailamp_adapters" / f"{spec.name}.stl"
        generated = generated_sim_dir / f"{spec.name}.stl"
        assert checked_in.read_bytes() == generated.read_bytes()
