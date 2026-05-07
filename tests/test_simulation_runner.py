import math

import pytest

from ailamp.simulation.mujoco_runner import (
    CSV_TO_ACTUATOR,
    FREEJOINT_QPOS,
    FREEJOINT_QVEL,
    RecordingControlMapper,
)
from ailamp.simulation.sim_vision import classify_virtual_target_from_joints
from ailamp.models import VisionEventType


def test_recording_control_mapper_converts_degrees_to_radians_and_clamps():
    mapper = RecordingControlMapper()
    row = {
        "base_pitch.pos": -60.0,
        "base_yaw.pos": 45.0,
        "elbow_pitch.pos": 120.0,
        "wrist_roll.pos": -30.0,
        "wrist_pitch.pos": 25.0,
    }

    controls = mapper.row_to_controls(row)

    assert set(controls) == set(CSV_TO_ACTUATOR.values())
    assert controls["1"] == pytest.approx(math.radians(45.0))
    assert controls["2"] == pytest.approx(math.radians(-60.0))
    assert controls["3"] <= 0.3215685722854582


def test_freejoint_constants_lock_base_pose():
    assert FREEJOINT_QPOS == (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    assert FREEJOINT_QVEL == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_virtual_vision_can_use_target_slide_joints():
    assert classify_virtual_target_from_joints(None).event_type == VisionEventType.NO_PERSON
    assert classify_virtual_target_from_joints({"target_slide_x": -0.6, "target_slide_y": -1.5}).event_type == VisionEventType.PERSON_LEFT
    assert classify_virtual_target_from_joints({"target_slide_x": 0.6, "target_slide_y": -1.5}).event_type == VisionEventType.PERSON_RIGHT
    assert classify_virtual_target_from_joints({"target_slide_x": 0.0, "target_slide_y": -0.45}).event_type == VisionEventType.PERSON_CLOSE
    assert classify_virtual_target_from_joints({"target_slide_x": 0.0, "target_slide_y": -3.8}).event_type == VisionEventType.PERSON_FAR
