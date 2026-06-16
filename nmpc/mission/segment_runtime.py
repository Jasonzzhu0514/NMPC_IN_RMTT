"""Mission-level 3D segment guide target adaptation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from nmpc.mission.math_utils import finite_float
from nmpc.segment_guide import (
    SegmentGuideConfig,
    SegmentGuideReference,
    SegmentPoint3D,
    guide_reference,
)


@dataclass(frozen=True)
class GuidedTargets:
    target_x: float | None
    target_y: float | None
    target_z: float | None
    target_yaw: float | None
    reference: SegmentGuideReference | None


def guided_targets(
    controller: Any,
    *,
    current_x: float | None,
    current_y: float | None,
    current_z: float | None,
    target_x: float | None,
    target_y: float | None,
    target_z: float | None,
    target_yaw: float | None,
    preserve_target_yaw: bool = False,
) -> GuidedTargets:
    if not controller.config.segment_guide_enabled:
        controller._segment_start = None
        controller._last_segment_reference = None
        return GuidedTargets(target_x, target_y, target_z, target_yaw, None)

    current = _point(current_x, current_y, current_z)
    goal = _point(target_x, target_y, target_z)
    if current is None or goal is None:
        controller._segment_start = None
        controller._last_segment_reference = None
        return GuidedTargets(target_x, target_y, target_z, target_yaw, None)

    goal_key = (round(goal.x, 4), round(goal.y, 4), round(goal.z, 4))
    if controller._segment_goal_key != goal_key or controller._segment_start is None:
        controller._segment_start = current
        controller._segment_goal_key = goal_key

    reference = guide_reference(
        start=controller._segment_start,
        goal=goal,
        current=current,
        config=SegmentGuideConfig(
            enabled=True,
            lookahead_m=controller.config.segment_guide_lookahead_m,
            terminal_distance_m=controller.config.segment_guide_terminal_distance_m,
            goal_snap_distance_m=controller.config.segment_guide_goal_snap_distance_m,
            yaw_freeze_distance_m=controller.config.segment_guide_yaw_freeze_distance_m,
            soft_boundary_xy_abs_m=_soft_boundary_xy_abs_m(controller),
        ),
    )
    controller._last_segment_reference = reference
    yaw = (
        target_yaw
        if preserve_target_yaw
        else reference.target_yaw
        if reference.target_yaw is not None
        else target_yaw
    )
    return GuidedTargets(
        reference.target.x,
        reference.target.y,
        reference.target.z,
        yaw,
        reference,
    )


def segment_log_values(controller: Any) -> dict[str, object]:
    reference = getattr(controller, "_last_segment_reference", None)
    if reference is None:
        return {}
    return reference.as_log_values()


def segment_throttle_authority(
    controller: Any,
    *,
    default_authority: float,
    pitch_u: float,
    roll_u: float,
) -> float:
    return default_authority


def reset_segment_guide(controller: Any) -> None:
    controller._segment_start = None
    controller._segment_goal_key = None
    controller._last_segment_reference = None


def _point(
    x: float | int | None,
    y: float | int | None,
    z: float | int | None,
) -> SegmentPoint3D | None:
    px = finite_float(x)
    py = finite_float(y)
    pz = finite_float(z)
    if None in (px, py, pz):
        return None
    return SegmentPoint3D(px, py, pz)


def _soft_boundary_xy_abs_m(controller: Any) -> float | None:
    profile = getattr(getattr(controller, "nmpc_flight", None), "controller_config", None)
    if profile is None:
        return None
    try:
        field_limit = float(profile.field_limit)
        safety_margin = float(profile.safety_margin)
    except (TypeError, ValueError):
        return None
    return max(0.0, field_limit - safety_margin - 0.02)
