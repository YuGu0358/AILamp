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
    namespace = {"core": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
    with zipfile.ZipFile(path) as archive:
        with archive.open("3D/3dmodel.model") as model_file:
            root = ET.parse(model_file).getroot()

    vertices = root.findall(".//core:vertex", namespace)
    xs = [float(vertex.attrib["x"]) for vertex in vertices]
    ys = [float(vertex.attrib["y"]) for vertex in vertices]
    zs = [float(vertex.attrib["z"]) for vertex in vertices]
    return max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)


def test_adapter_specs_use_loose_fit_clearances():
    generator = load_generator()

    assert generator.PCB_EDGE_CLEARANCE_MM == 1.5
    assert generator.CABLE_EXIT_EXTRA_WIDTH_MM == 2.5
    assert generator.SCREW_HOLE_EXTRA_DIAMETER_MM == 0.4
    assert generator.M12_LENS_RADIAL_CLEARANCE_MM == 1.0

    specs = {spec.name: spec for spec in generator.adapter_specs()}
    assert specs["AILamp_Jetson_Nano_Base_Tray"].board_pocket_mm == (103.0, 83.0)
    assert specs["AILamp_Electronics_Side_Deck"].board_pocket_mm == (145.0, 48.0)
    assert specs["AILamp_Head_Camera_Mount"].board_pocket_mm == (35.0, 35.0)
    assert specs["AILamp_NeoMatrix_Holder"].board_pocket_mm == (74.17, 74.17)
    assert specs["AILamp_ReSpeaker_External_Mount"].board_pocket_mm == (89.0, 38.0)


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
