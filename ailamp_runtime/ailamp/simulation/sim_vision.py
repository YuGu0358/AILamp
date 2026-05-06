from __future__ import annotations

from typing import Optional

from ailamp.models import VisionEvent, VisionEventType


def classify_virtual_target(
    target_xyz: Optional[tuple[float, float, float]],
    *,
    left_threshold_m: float = -0.25,
    right_threshold_m: float = 0.25,
    close_depth_m: float = 0.75,
    far_depth_m: float = 3.0,
) -> VisionEvent:
    if target_xyz is None:
        return VisionEvent(VisionEventType.NO_PERSON)

    x, _y, depth = target_xyz
    if depth < close_depth_m:
        return VisionEvent(VisionEventType.PERSON_CLOSE, confidence=1.0)
    if depth > far_depth_m:
        return VisionEvent(VisionEventType.PERSON_FAR, confidence=1.0)
    if x < left_threshold_m:
        return VisionEvent(VisionEventType.PERSON_LEFT, confidence=1.0, normalized_offset=x)
    if x > right_threshold_m:
        return VisionEvent(VisionEventType.PERSON_RIGHT, confidence=1.0, normalized_offset=x)
    return VisionEvent(VisionEventType.PERSON_CENTER, confidence=1.0, normalized_offset=x)

