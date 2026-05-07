from ailamp.models import BoundingBox, VisionEventType
from ailamp.services.vision import classify_person_position
from ailamp.simulation.sim_vision import classify_virtual_target


def test_classifies_camera_person_position():
    frame = (640, 480)

    assert classify_person_position(None, frame).event_type == VisionEventType.NO_PERSON
    assert classify_person_position(BoundingBox(20, 120, 100, 260, 0.8, "person"), frame).event_type == VisionEventType.PERSON_LEFT
    assert classify_person_position(BoundingBox(270, 120, 100, 220, 0.8, "person"), frame).event_type == VisionEventType.PERSON_CENTER
    assert classify_person_position(BoundingBox(520, 120, 100, 220, 0.8, "person"), frame).event_type == VisionEventType.PERSON_RIGHT
    assert classify_person_position(BoundingBox(120, 20, 420, 420, 0.9, "person"), frame).event_type == VisionEventType.PERSON_CLOSE


def test_classifies_virtual_target_events():
    assert classify_virtual_target(None).event_type == VisionEventType.NO_PERSON
    assert classify_virtual_target((-0.6, 0.0, 1.5)).event_type == VisionEventType.PERSON_LEFT
    assert classify_virtual_target((0.0, 0.0, 1.5)).event_type == VisionEventType.PERSON_CENTER
    assert classify_virtual_target((0.6, 0.0, 1.5)).event_type == VisionEventType.PERSON_RIGHT
    assert classify_virtual_target((0.0, 0.0, 0.45)).event_type == VisionEventType.PERSON_CLOSE
    assert classify_virtual_target((0.0, 0.0, 3.8)).event_type == VisionEventType.PERSON_FAR


def test_classifies_camera_person_position_with_custom_thresholds():
    event = classify_person_position(
        BoundingBox(210, 120, 100, 100, 0.8, "person"),
        (640, 480),
        left_threshold=-0.50,
        right_threshold=0.50,
    )

    assert event.event_type == VisionEventType.PERSON_CENTER
