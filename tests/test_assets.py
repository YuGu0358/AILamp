from pathlib import Path

from ailamp.services.motor import RecordingStore


def test_required_assets_and_recordings_are_present():
    root = Path(__file__).resolve().parents[1]

    assert (root / "simulation/scene.xml").exists()
    assert (root / "simulation/ailamp_scene.xml").exists()
    assert (root / "simulation/robot.xml").exists()
    assert (root / "simulation/assets/lamp_head.stl").exists()
    assert (root / "3D/LampHead.3mf").exists()
    adapter_dir = root / "3D/AILamp_Adapters"
    assert (adapter_dir / "AILamp_Jetson_Nano_Base_Tray.3mf").exists()
    assert (adapter_dir / "AILamp_Electronics_Side_Deck.3mf").exists()
    assert (adapter_dir / "AILamp_Head_Camera_Mount.3mf").exists()
    assert (adapter_dir / "AILamp_NeoMatrix_Holder.3mf").exists()
    assert (adapter_dir / "AILamp_ReSpeaker_External_Mount.3mf").exists()
    assert (adapter_dir / "AILamp_Cable_Clip_6mm.3mf").exists()
    assert (adapter_dir / "AILamp_Cable_Clip_10mm.3mf").exists()

    recordings = RecordingStore(root / "ailamp_runtime/ailamp/recordings")
    names = recordings.list_names()
    assert {"wake_up", "idle", "nod", "scanning", "shy"}.issubset(set(names))

    wake_up = recordings.load("wake_up")
    assert wake_up
    assert set(wake_up[0]) == {
        "base_yaw.pos",
        "base_pitch.pos",
        "elbow_pitch.pos",
        "wrist_roll.pos",
        "wrist_pitch.pos",
    }
