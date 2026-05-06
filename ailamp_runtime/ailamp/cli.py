from __future__ import annotations

import argparse
from pathlib import Path
import sys

from ailamp.config import load_hardware_config
from ailamp.hardware_check import run_device_presence_checks, run_static_hardware_checks
from ailamp.paths import resolve_project_path
from ailamp.services.behavior import BehaviorService
from ailamp.services.led_serial import LEDSerialService
from ailamp.services.motor import RecordingStore
from ailamp.services.vision import classify_person_position
from ailamp.simulation.mujoco_runner import MujocoRunner
from ailamp.simulation.sim_vision import classify_virtual_target


DEFAULT_CONFIG = "config/hardware.toml"


def _config(args):
    return load_hardware_config(args.config)


def hardware_check(args) -> int:
    config = _config(args)
    results = run_static_hardware_checks(config)
    if args.include_devices:
        results.extend(run_device_presence_checks(config))
    for result in results:
        print(result.format())
    return 0 if all(result.passed for result in results) else 1


def led_test(args) -> int:
    config = _config(args)
    service = LEDSerialService(config.led.port, led_count=config.led.count, baudrate=config.led.baudrate)
    service.connect()
    try:
        print("PING =>", service.ping())
        print("BRIGHTNESS =>", service.brightness(config.led.brightness))
        print("SOLID =>", service.solid(*args.color))
    finally:
        service.close()
    return 0


def motor_test(args) -> int:
    config = _config(args)
    store = RecordingStore(resolve_project_path(config.simulation.recordings_dir))
    print("recordings:", ", ".join(store.list_names()))
    print("port:", config.motors.port)
    print("ids:", config.motors.ids)
    return 0


def sim_demo(args) -> int:
    config = _config(args)
    behavior = BehaviorService()
    positions = [
        None,
        (-0.6, 0.0, 1.5),
        (0.0, 0.0, 1.5),
        (0.6, 0.0, 1.5),
        (0.0, 0.0, 0.45),
        (0.0, 0.0, 3.8),
    ]
    for position in positions:
        event = classify_virtual_target(position)
        action = behavior.decide(event)
        print(f"target={position} event={event.event_type.value} motion={action.motion} rgb={action.rgb}")

    if args.render:
        runner = MujocoRunner(config.simulation.model_path, lock_freejoint=config.simulation.lock_freejoint)
        runner.load()
        runner.replay_recording("wake_up", config.simulation.recordings_dir, max_frames=30)
        output = runner.render("outputs/sim_demo.png")
        print("render:", output)
    return 0


def sim_viewer(args) -> int:
    config = _config(args)
    runner = MujocoRunner(config.simulation.model_path, lock_freejoint=config.simulation.lock_freejoint)
    info = runner.info()
    print(f"model={info.model_path}")
    print(f"nq={info.nq} nv={info.nv} nu={info.nu}")
    print("joints:", ", ".join(info.joints))
    print("actuators:", ", ".join(info.actuators))
    if args.render:
        output = runner.render(args.render)
        print("render:", output)
    return 0


def camera_test(args) -> int:
    config = _config(args)
    from ailamp.services.camera import CameraService

    camera = CameraService(config.camera.device, config.camera.width, config.camera.height, config.camera.fps)
    try:
        report = camera.probe()
        print(report)
    finally:
        camera.close()
    return 0


def audio_test(args) -> int:
    config = _config(args)
    from ailamp.services.audio import AudioService

    report = AudioService(config.audio.input_model, config.audio.speaker_model).probe()
    print("input:", report.input_model)
    print("speaker:", report.speaker_model)
    for device in report.devices:
        print(device)
    return 0


def vision_demo(args) -> int:
    config = _config(args)
    from ailamp.services.camera import CameraService
    from ailamp.services.vision import DetectorService

    camera = CameraService(config.camera.device, config.camera.width, config.camera.height, config.camera.fps)
    detector = DetectorService(config.vision.model, config.vision.confidence)
    detector.load()
    try:
        camera.open()
        frame = camera.read()
        if frame is None:
            print("event=no_person confidence=0.00")
            return 1
        bbox = detector.detect_person(frame)
        event = classify_person_position(bbox, (config.camera.width, config.camera.height))
        action = BehaviorService().decide(event)
        print(f"event={event.event_type.value} confidence={event.confidence:.2f} motion={action.motion} rgb={action.rgb}")
    finally:
        camera.close()
    return 0


def agent(args) -> int:
    from ailamp.agent.livekit_agent import run_agent

    run_agent(args.config)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ailamp")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    subparsers = parser.add_subparsers(dest="command", required=True)

    hardware = subparsers.add_parser("hardware-check")
    hardware.add_argument("--include-devices", action="store_true")
    hardware.set_defaults(func=hardware_check)

    led = subparsers.add_parser("led-test")
    led.add_argument("--color", nargs=3, type=int, default=(255, 180, 80))
    led.set_defaults(func=led_test)

    subparsers.add_parser("motor-test").set_defaults(func=motor_test)
    subparsers.add_parser("camera-test").set_defaults(func=camera_test)
    subparsers.add_parser("audio-test").set_defaults(func=audio_test)
    subparsers.add_parser("vision-demo").set_defaults(func=vision_demo)
    subparsers.add_parser("agent").set_defaults(func=agent)

    sim_demo_parser = subparsers.add_parser("sim-demo")
    sim_demo_parser.add_argument("--render", action="store_true")
    sim_demo_parser.set_defaults(func=sim_demo)

    sim_viewer_parser = subparsers.add_parser("sim-viewer")
    sim_viewer_parser.add_argument("--render", default=None)
    sim_viewer_parser.set_defaults(func=sim_viewer)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

