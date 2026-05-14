from pathlib import Path

from ailamp.models import VisionEventType


ADAPTER_PRINT_FILES = {
    "AILamp_Jetson_Nano_Base_Tray.3mf",
    "AILamp_Electronics_Side_Deck.3mf",
    "AILamp_Head_Camera_Mount.3mf",
    "AILamp_NeoMatrix_Holder.3mf",
    "AILamp_ReSpeaker_External_Mount.3mf",
    "AILamp_Cable_Clip_6mm.3mf",
    "AILamp_Cable_Clip_10mm.3mf",
}

ADAPTER_VISUAL_NAMES = {
    "ailamp_jetson_tray_visual",
    "ailamp_electronics_deck_visual",
    "ailamp_camera_mount_visual",
    "ailamp_neomatrix_visual",
    "ailamp_respeaker_visual",
}


def test_docs_do_not_reference_missing_motion_names():
    root = Path(__file__).resolve().parents[1]
    docs_text = "\n".join(path.read_text() for path in (root / "docs").rglob("*.md"))

    assert "look_left" not in docs_text
    assert "look_right" not in docs_text


def test_docs_list_all_vision_event_types():
    root = Path(__file__).resolve().parents[1]
    docs_text = "\n".join(path.read_text() for path in (root / "docs").rglob("*.md"))

    for event_type in VisionEventType:
        assert event_type.value in docs_text


def test_assembly_docs_include_adapter_installation_and_fit_rules():
    root = Path(__file__).resolve().parents[1]
    docs_text = "\n".join(
        [
            (root / "docs/en/3-assembly.md").read_text(),
            (root / "docs/zh/3-装配.md").read_text(),
        ]
    )

    for filename in ADAPTER_PRINT_FILES:
        assert filename in docs_text

    assert "Do not cut `LampHead.3mf`" in docs_text
    assert "不要切割 `LampHead.3mf`" in docs_text
    assert "slightly loose" in docs_text
    assert "不要卡死" in docs_text


def test_simulation_docs_list_adapter_visuals():
    root = Path(__file__).resolve().parents[1]
    docs_text = "\n".join(
        [
            (root / "docs/en/5-control-vision-simulation.md").read_text(),
            (root / "docs/zh/5-控制视觉仿真.md").read_text(),
        ]
    )

    for visual_name in ADAPTER_VISUAL_NAMES:
        assert visual_name in docs_text
