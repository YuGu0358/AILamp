from __future__ import annotations

from ailamp.models import BehaviorAction, VisionEvent, VisionEventType


DEFAULT_BEHAVIOR_MAP: dict[VisionEventType, tuple[str, tuple[int, int, int]]] = {
    VisionEventType.NO_PERSON: ("scanning", (30, 30, 80)),
    VisionEventType.PERSON_LEFT: ("look_left", (80, 120, 255)),
    VisionEventType.PERSON_CENTER: ("nod", (255, 180, 80)),
    VisionEventType.PERSON_RIGHT: ("look_right", (80, 120, 255)),
    VisionEventType.PERSON_CLOSE: ("shy", (255, 80, 120)),
    VisionEventType.PERSON_FAR: ("curious", (80, 255, 160)),
}


class BehaviorService:
    def __init__(self, behavior_map: dict[VisionEventType, tuple[str, tuple[int, int, int]]] | None = None):
        self.behavior_map = behavior_map or DEFAULT_BEHAVIOR_MAP

    def decide(self, event: VisionEvent) -> BehaviorAction:
        motion, rgb = self.behavior_map[event.event_type]
        return BehaviorAction(event=event, motion=motion, rgb=rgb)

    def apply(self, event: VisionEvent, motor_service, led_service) -> BehaviorAction:
        action = self.decide(event)
        motor_service.play(action.motion)
        led_service.solid(*action.rgb)
        return action

