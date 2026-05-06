from ailamp.models import VisionEvent, VisionEventType
from ailamp.services.behavior import BehaviorService


def test_behavior_maps_vision_events_to_motion_and_led():
    service = BehaviorService()

    action = service.decide(VisionEvent(VisionEventType.NO_PERSON))
    assert action.motion == "scanning"
    assert action.rgb == (30, 30, 80)

    action = service.decide(VisionEvent(VisionEventType.PERSON_CENTER, confidence=0.9))
    assert action.motion == "nod"
    assert action.rgb == (255, 180, 80)

    action = service.decide(VisionEvent(VisionEventType.PERSON_CLOSE, confidence=0.9))
    assert action.motion == "shy"
    assert action.rgb == (255, 80, 120)

