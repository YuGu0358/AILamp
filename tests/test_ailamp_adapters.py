import hashlib
import importlib.util
from pathlib import Path
import sys
import zipfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
GENERATOR_PATH = ROOT / "scripts/generate_ailamp_adapters.py"
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


def assert_3mf_triangle_indices_are_valid(path):
    vertices, triangles = parse_3mf_mesh(path)
    assert vertices
    assert triangles
    vertex_count = len(vertices)
    for triangle in triangles:
        assert len(set(triangle)) == 3
        for index in triangle:
            assert 0 <= index < vertex_count


def test_adapter_specs_use_loose_fit_clearances():
    generator = load_generator()

    assert generator.PCB_EDGE_CLEARANCE_MM == 1.5
    assert generator.CABLE_EXIT_EXTRA_WIDTH_MM == 2.5
    assert generator.SCREW_HOLE_EXTRA_DIAMETER_MM == 0.4
    assert generator.M12_LENS_RADIAL_CLEARANCE_MM == 1.0

    specs = {spec.name: spec for spec in generator.adapter_specs()}
    pcb_clearance = generator.PCB_EDGE_CLEARANCE_MM * 2.0
    assert specs["AILamp_Jetson_Nano_Base_Tray"].board_pocket_mm == (
        generator.JETSON_BOARD_MM[0] + pcb_clearance,
        generator.JETSON_BOARD_MM[1] + pcb_clearance,
    )
    assert specs["AILamp_Electronics_Side_Deck"].board_pocket_mm == (145.0, 48.0)
    assert specs["AILamp_Head_Camera_Mount"].board_pocket_mm == (
        generator.CAMERA_BOARD_MM[0] + pcb_clearance,
        generator.CAMERA_BOARD_MM[1] + pcb_clearance,
    )
    assert specs["AILamp_NeoMatrix_Holder"].board_pocket_mm == (
        generator.NEOMATRIX_BOARD_MM[0] + pcb_clearance,
        generator.NEOMATRIX_BOARD_MM[1] + pcb_clearance,
    )
    assert specs["AILamp_ReSpeaker_External_Mount"].board_pocket_mm == (
        generator.RESPEAKER_CASE_MM[0] + pcb_clearance,
        generator.RESPEAKER_CASE_MM[1] + pcb_clearance,
    )
    assert specs["AILamp_Cable_Clip_6mm"].board_pocket_mm[0] == (
        6.0 + generator.CABLE_EXIT_EXTRA_WIDTH_MM
    )
    assert specs["AILamp_Cable_Clip_10mm"].board_pocket_mm[0] == (
        10.0 + generator.CABLE_EXIT_EXTRA_WIDTH_MM
    )


def test_generator_writes_all_adapter_files_without_touching_originals(tmp_path):
    generator = load_generator()
    expected_adapter_names = {
        "AILamp_Jetson_Nano_Base_Tray",
        "AILamp_Electronics_Side_Deck",
        "AILamp_Head_Camera_Mount",
        "AILamp_NeoMatrix_Holder",
        "AILamp_ReSpeaker_External_Mount",
        "AILamp_Cable_Clip_6mm",
        "AILamp_Cable_Clip_10mm",
    }
    specs = {spec.name: spec for spec in generator.adapter_specs()}
    assert set(specs) == expected_adapter_names

    original_hashes = {path: sha256(path) for path in ORIGINAL_PRINT_FILES}
    original_bytes = {path: path.read_bytes() for path in ORIGINAL_PRINT_FILES}
    output_dir = tmp_path / "adapters"
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

    jetson_bounds = parse_3mf_bounds(output_dir / "AILamp_Jetson_Nano_Base_Tray.3mf")
    assert jetson_bounds[0] >= 122.0
    assert jetson_bounds[1] >= 102.0
    assert jetson_bounds[2] >= 4.0

    neo_matrix_bounds = parse_3mf_bounds(output_dir / "AILamp_NeoMatrix_Holder.3mf")
    assert neo_matrix_bounds[0] >= 86.0
    assert neo_matrix_bounds[1] >= 86.0

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
