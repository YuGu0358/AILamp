from __future__ import annotations

import argparse
from datetime import date
from pathlib import Path
import sys

from ailamp.config import load_hardware_config
from ailamp.hardware_check import run_device_presence_checks, run_static_hardware_checks
from ailamp.paths import resolve_project_path
from ailamp.services.behavior import BehaviorService
from ailamp.services.birthday import BirthdayService
from ailamp.services.led_serial import LEDSerialService
from ailamp.services.motor import MotorService
from ailamp.services.motor import RecordingStore
from ailamp.services.vision import classify_person_position
from ailamp.services.vision_runtime import VisionRuntime
from ailamp.simulation.mujoco_runner import MujocoRunner
from ailamp.simulation.sim_vision import classify_virtual_target_from_joints


DEFAULT_CONFIG = "config/hardware.toml"


def _config(args):
    return load_hardware_config(args.config)


def hardware_check(args) -> int:
    config = _config(args)
    results = run_static_hardware_checks(config)
    if args.include_devices:
        results.extend(run_device_presence_checks(config))
    if args.failures_only:
        results = [result for result in results if not result.passed]
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
    target_joint_sets = [
        None,
        {"target_slide_x": -0.6, "target_slide_y": -1.5},
        {"target_slide_x": 0.0, "target_slide_y": -1.5},
        {"target_slide_x": 0.6, "target_slide_y": -1.5},
        {"target_slide_x": 0.0, "target_slide_y": -0.45},
        {"target_slide_x": 0.0, "target_slide_y": -3.8},
    ]
    for joints in target_joint_sets:
        event = classify_virtual_target_from_joints(joints)
        action = behavior.decide(event)
        print(f"target_joints={joints} event={event.event_type.value} motion={action.motion} rgb={action.rgb}")

    if args.render:
        runner = MujocoRunner(config.simulation.model_path, lock_freejoint=config.simulation.lock_freejoint)
        runner.load()
        runner.set_target_position(0.0, 1.5)
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

    camera = CameraService(
        config.camera.device_path,
        config.camera.width,
        config.camera.height,
        config.camera.fps,
        config.camera.pixel_format,
    )
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


def birthday_check(args) -> int:
    config = _config(args)
    today = date.fromisoformat(args.today) if args.today else None
    service = BirthdayService(config.birthday)
    status = service.status(today, force=args.force)
    print(
        "birthday="
        f"enabled={status.enabled} date={status.today.isoformat()} "
        f"is_birthday={status.is_birthday} already_played={status.already_played} "
        f"should_play={status.should_play} message={status.message}"
    )
    if not status.should_play:
        return 0

    if args.with_outputs:
        led = LEDSerialService(config.led.port, led_count=config.led.count, baudrate=config.led.baudrate)
        motors = MotorService(
            config.motors.port,
            config.system.project_name.lower(),
            resolve_project_path(config.simulation.recordings_dir),
        )
        led.connect()
        try:
            led.solid(*status.rgb)
            motors.connect()
            try:
                motors.play(status.motion)
            finally:
                motors.close()
        finally:
            led.close()
        print(f"outputs=played motion={status.motion} rgb={status.rgb}")

    if args.speak:
        print(service.speak(status.message))

    if not args.dry_run:
        service.mark_played(status.today)
        print(f"marked_played={status.today.isoformat()}")
    return 0


def vision_demo(args) -> int:
    config = _config(args)
    from ailamp.services.camera import CameraService
    from ailamp.services.vision import DetectorService

    camera = CameraService(
        config.camera.device_path,
        config.camera.width,
        config.camera.height,
        config.camera.fps,
        config.camera.pixel_format,
    )
    detector = DetectorService(config.vision.model, config.vision.confidence)
    detector.load()
    try:
        camera.open()
        frame = camera.read()
        if frame is None:
            print("event=no_person confidence=0.00")
            return 1
        bbox = detector.detect_person(frame)
        event = classify_person_position(
            bbox,
            (config.camera.width, config.camera.height),
            left_threshold=config.vision.left_threshold,
            right_threshold=config.vision.right_threshold,
            close_area_ratio=config.vision.close_area_ratio,
        )
        action = BehaviorService().decide(event)
        print(f"event={event.event_type.value} confidence={event.confidence:.2f} motion={action.motion} rgb={action.rgb}")
    finally:
        camera.close()
    return 0


def vision_loop(args) -> int:
    config = _config(args)
    runtime = VisionRuntime(config)
    try:
        runtime.open(with_outputs=args.with_outputs)
        try:
            for result in runtime.run(
                max_frames=args.frames,
                interval_s=args.interval,
                apply_outputs=args.with_outputs,
            ):
                print(result.format())
        except KeyboardInterrupt:
            return 130
    finally:
        runtime.close()
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
    hardware.add_argument("--failures-only", action="store_true")
    hardware.set_defaults(func=hardware_check)

    led = subparsers.add_parser("led-test")
    led.add_argument("--color", nargs=3, type=int, default=(255, 180, 80))
    led.set_defaults(func=led_test)

    subparsers.add_parser("motor-test").set_defaults(func=motor_test)
    subparsers.add_parser("camera-test").set_defaults(func=camera_test)
    subparsers.add_parser("audio-test").set_defaults(func=audio_test)
    subparsers.add_parser("vision-demo").set_defaults(func=vision_demo)
    vision_loop_parser = subparsers.add_parser("vision-loop")
    vision_loop_parser.add_argument("--frames", type=int, default=None, help="Stop after N frames; omit to run forever")
    vision_loop_parser.add_argument("--interval", type=float, default=None, help="Seconds between frames")
    vision_loop_parser.add_argument("--with-outputs", action="store_true", help="Drive ST3215 motions and Pico LEDs")
    vision_loop_parser.set_defaults(func=vision_loop)
    subparsers.add_parser("agent").set_defaults(func=agent)

    birthday = subparsers.add_parser("birthday-check")
    birthday.add_argument("--today", default=None, help="Override date as YYYY-MM-DD for testing")
    birthday.add_argument("--force", action="store_true", help="Ignore date and already-played checks")
    birthday.add_argument("--dry-run", action="store_true", help="Do not write birthday state")
    birthday.add_argument("--speak", action="store_true", help="Use local speech command when available")
    birthday.add_argument("--with-outputs", action="store_true", help="Play configured lamp motion and LED color")
    birthday.set_defaults(func=birthday_check)

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
