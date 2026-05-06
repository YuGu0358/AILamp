from pathlib import Path

from ailamp.config import load_hardware_config


def test_loads_exact_hardware_bom_and_ports():
    config = load_hardware_config(Path("config/hardware.toml"))

    assert config.system.project_name == "AILamp"
    assert config.controller.model == "NVIDIA Jetson Orin Nano Super Developer Kit"
    assert config.controller.mpn == "945-13766-0000-000"
    assert config.motors.port == "/dev/ttyACM0"
    assert config.led.port == "/dev/ttyACM1"
    assert config.camera.device_path == "/dev/video0"
    assert config.motors.ids == {
        "base_yaw": 1,
        "base_pitch": 2,
        "elbow_pitch": 3,
        "wrist_roll": 4,
        "wrist_pitch": 5,
    }
    assert config.led.count == 64

