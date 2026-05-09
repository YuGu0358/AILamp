from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import tempfile
import time
from typing import Callable, Iterator

from ailamp.config import HardwareConfig
from ailamp.models import BehaviorAction, BoundingBox, VisionEvent, VisionEventType
from ailamp.paths import resolve_project_path
from ailamp.services.behavior import BehaviorService
from ailamp.services.camera import CameraService
from ailamp.services.decision import AIDecision, DecisionService
from ailamp.services.led_serial import LEDSerialService
from ailamp.services.motor import JointDeltaCommand, MotorService
from ailamp.services.pose_gesture import PoseDetectorService, classify_pose_gesture
from ailamp.services.vision import DetectorService, classify_person_position


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _bbox_to_dict(bbox: BoundingBox | None) -> dict[str, float | str] | None:
    if bbox is None:
        return None
    return {
        "x": bbox.x,
        "y": bbox.y,
        "width": bbox.width,
        "height": bbox.height,
        "confidence": bbox.confidence,
        "label": bbox.label,
    }


def _bbox_from_dict(raw: dict | None) -> BoundingBox | None:
    if not isinstance(raw, dict):
        return None
    return BoundingBox(
        float(raw["x"]),
        float(raw["y"]),
        float(raw["width"]),
        float(raw["height"]),
        float(raw["confidence"]),
        str(raw.get("label", "person")),
    )


def _joint_deltas_to_dict(commands: tuple[JointDeltaCommand, ...]) -> list[dict[str, float | str]]:
    return [{"joint": command.joint, "delta_deg": command.delta_deg} for command in commands]


def _joint_deltas_from_dict(raw: list[dict] | None) -> tuple[JointDeltaCommand, ...]:
    if not isinstance(raw, list):
        return ()
    return tuple(JointDeltaCommand(str(item["joint"]), float(item["delta_deg"])) for item in raw)


def _decision_to_dict(decision: AIDecision | None) -> dict | None:
    if decision is None:
        return None
    return {
        "motion": decision.motion,
        "rgb": list(decision.rgb),
        "reason": decision.reason,
        "joint_deltas": _joint_deltas_to_dict(decision.joint_deltas),
    }


@dataclass(frozen=True)
class VisionSnapshot:
    event: VisionEvent
    action: BehaviorAction
    updated_at: str
    frame_index: int
    decision: AIDecision | None = None

    def to_dict(self) -> dict:
        return {
            "updated_at": self.updated_at,
            "frame_index": self.frame_index,
            "event": {
                "event_type": self.event.event_type.value,
                "confidence": self.event.confidence,
                "bbox": _bbox_to_dict(self.event.bbox),
                "normalized_offset": self.event.normalized_offset,
                "area_ratio": self.event.area_ratio,
            },
            "action": {
                "motion": self.action.motion,
                "rgb": list(self.action.rgb),
                "priority": self.action.priority,
            },
            "decision": _decision_to_dict(self.decision),
        }

    @classmethod
    def from_dict(cls, raw: dict) -> VisionSnapshot:
        event_raw = raw["event"]
        event = VisionEvent(
            event_type=VisionEventType(event_raw["event_type"]),
            confidence=float(event_raw.get("confidence", 0.0)),
            bbox=_bbox_from_dict(event_raw.get("bbox")),
            normalized_offset=float(event_raw.get("normalized_offset", 0.0)),
            area_ratio=float(event_raw.get("area_ratio", 0.0)),
        )
        action_raw = raw["action"]
        action = BehaviorAction(
            event=event,
            motion=str(action_raw["motion"]),
            rgb=tuple(int(value) for value in action_raw["rgb"]),
            priority=int(action_raw.get("priority", 1)),
        )
        decision_raw = raw.get("decision")
        decision = None
        if isinstance(decision_raw, dict):
            decision = AIDecision(
                event=event,
                motion=str(decision_raw["motion"]),
                rgb=tuple(int(value) for value in decision_raw["rgb"]),
                reason=str(decision_raw.get("reason", "")),
                joint_deltas=_joint_deltas_from_dict(decision_raw.get("joint_deltas")),
            )
        return cls(
            event=event,
            action=action,
            updated_at=str(raw["updated_at"]),
            frame_index=int(raw.get("frame_index", 0)),
            decision=decision,
        )


class VisionStateStore:
    def __init__(self, path: str | Path):
        self.path = resolve_project_path(path)

    def write(self, snapshot: VisionSnapshot) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile("w", dir=self.path.parent, prefix=f".{self.path.name}.", suffix=".tmp", delete=False) as handle:
            handle.write(json.dumps(snapshot.to_dict(), indent=2) + "\n")
            temp_path = Path(handle.name)
        temp_path.replace(self.path)

    def read(self) -> VisionSnapshot | None:
        if not self.path.exists():
            return None
        try:
            raw = json.loads(self.path.read_text())
            return VisionSnapshot.from_dict(raw)
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            return None


@dataclass(frozen=True)
class VisionLoopResult:
    snapshot: VisionSnapshot
    applied: bool

    def format(self) -> str:
        action = self.snapshot.action
        event = self.snapshot.event
        decision = self.snapshot.decision
        deltas = ""
        if decision is not None and decision.joint_deltas:
            deltas = " joint_deltas=" + ",".join(f"{command.joint}:{command.delta_deg:+.2f}" for command in decision.joint_deltas)
        return (
            f"frame={self.snapshot.frame_index} event={event.event_type.value} "
            f"confidence={event.confidence:.2f} motion={action.motion} rgb={action.rgb}{deltas} applied={self.applied}"
        )


class VisionRuntime:
    def __init__(
        self,
        config: HardwareConfig,
        *,
        camera: object | None = None,
        detector: object | None = None,
        behavior: BehaviorService | None = None,
        decision_service: DecisionService | None = None,
        state_store: VisionStateStore | None = None,
        led_service: object | None = None,
        motor_service: object | None = None,
        pose_detector: object | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.config = config
        self.camera = camera or CameraService(
            config.camera.device_path,
            config.camera.width,
            config.camera.height,
            config.camera.fps,
            config.camera.pixel_format,
        )
        self.detector = detector or DetectorService(config.vision.model, config.vision.confidence)
        self.pose_detector = pose_detector
        if self.pose_detector is None and detector is None and config.vision.pose_enabled:
            self.pose_detector = PoseDetectorService(config.vision.pose_model, config.vision.confidence)
        self.behavior = behavior or BehaviorService()
        self.decision_service = decision_service or DecisionService(behavior=self.behavior)
        self.state_store = state_store or VisionStateStore(config.runtime.vision_state_file)
        self.led_service = led_service
        self.motor_service = motor_service
        self.clock = clock
        self._frame_index = 0
        self._last_applied_event: VisionEventType | None = None
        self._last_applied_time = -1_000_000.0
        self._last_seen_person = False

    def open(self, *, with_outputs: bool = False) -> None:
        self.camera.open()
        self.detector.load()
        if self.pose_detector is not None:
            self.pose_detector.load()
        if with_outputs:
            if self.led_service is None:
                self.led_service = LEDSerialService(
                    self.config.led.port,
                    self.config.led.count,
                    self.config.led.baudrate,
                )
            if self.motor_service is None:
                self.motor_service = MotorService(
                    self.config.motors.port,
                    self.config.system.project_name.lower(),
                    resolve_project_path(self.config.simulation.recordings_dir),
                )
            self.led_service.connect()
            self.motor_service.connect()

    def close(self) -> None:
        if self.motor_service is not None:
            self.motor_service.close()
        if self.led_service is not None:
            self.led_service.close()
        self.camera.close()

    def step(self, *, apply_outputs: bool = False) -> VisionLoopResult:
        frame = self.camera.read()
        bbox = self.detector.detect_person(frame) if frame is not None else None
        base_event = classify_person_position(
            bbox,
            (self.config.camera.width, self.config.camera.height),
            left_threshold=self.config.vision.left_threshold,
            right_threshold=self.config.vision.right_threshold,
            close_area_ratio=self.config.vision.close_area_ratio,
            far_area_ratio=self.config.vision.far_area_ratio,
        )
        pose_event = None
        if frame is not None and bbox is not None and self.pose_detector is not None:
            pose_event = classify_pose_gesture(self.pose_detector.detect_pose(frame))
        event = self._choose_event(base_event, pose_event)
        decision = self.decision_service.decide(event)
        action = BehaviorAction(event=event, motion=decision.motion, rgb=decision.rgb)
        snapshot = VisionSnapshot(event=event, action=action, updated_at=_utc_now(), frame_index=self._frame_index, decision=decision)
        self.state_store.write(snapshot)
        applied = self._apply_if_needed(decision, apply_outputs)
        self._frame_index += 1
        return VisionLoopResult(snapshot=snapshot, applied=applied)

    def _choose_event(self, base_event: VisionEvent, pose_event: VisionEvent | None) -> VisionEvent:
        if base_event.event_type == VisionEventType.NO_PERSON:
            if self._last_seen_person:
                self._last_seen_person = False
                return VisionEvent(VisionEventType.PERSON_LEFT_SEAT)
            return base_event

        self._last_seen_person = True
        if base_event.event_type == VisionEventType.PERSON_CLOSE:
            return base_event
        if pose_event is not None and pose_event.event_type not in {VisionEventType.NO_PERSON, VisionEventType.PERSON_CENTER}:
            return pose_event
        return base_event

    def run(
        self,
        *,
        max_frames: int | None = None,
        interval_s: float | None = None,
        apply_outputs: bool = False,
    ) -> Iterator[VisionLoopResult]:
        frame_limit = max_frames if max_frames is not None and max_frames > 0 else None
        interval = self.config.runtime.vision_interval_s if interval_s is None else interval_s
        while frame_limit is None or self._frame_index < frame_limit:
            yield self.step(apply_outputs=apply_outputs)
            if frame_limit is None or self._frame_index < frame_limit:
                time.sleep(max(0.0, interval))

    def _apply_if_needed(self, decision: AIDecision, apply_outputs: bool) -> bool:
        if not apply_outputs:
            return False
        if self.led_service is None or self.motor_service is None:
            raise RuntimeError("VisionRuntime outputs are not connected")
        now = self.clock()
        cooldown_elapsed = now - self._last_applied_time >= self.config.runtime.action_cooldown_s
        if decision.event.event_type == self._last_applied_event and not cooldown_elapsed:
            return False
        if decision.joint_deltas:
            self.motor_service.apply_joint_deltas(decision.joint_deltas)
            self.led_service.solid(*decision.rgb)
        else:
            self.motor_service.play(decision.motion)
            self.led_service.solid(*decision.rgb)
        self._last_applied_event = decision.event.event_type
        self._last_applied_time = now
        return True
