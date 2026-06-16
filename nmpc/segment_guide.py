"""3D segment reference generation for conservative XYZ NMPC."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SegmentPoint3D:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class SegmentGuideConfig:
    enabled: bool = False
    lookahead_m: float = 0.18
    terminal_distance_m: float = 0.18
    goal_snap_distance_m: float = 0.18
    yaw_freeze_distance_m: float = 0.20
    soft_boundary_xy_abs_m: float | None = None


@dataclass(frozen=True)
class SegmentGuideReference:
    enabled: bool
    start: SegmentPoint3D
    goal: SegmentPoint3D
    target: SegmentPoint3D
    progress_s: float
    target_s: float
    along_m: float
    segment_length_m: float
    cross_track_3d_m: float
    distance_to_goal_3d_m: float
    target_yaw: float | None
    terminal_mode: bool

    def as_log_values(self) -> dict[str, object]:
        return {
            "nmpc_mission_segment_guide_enabled": int(self.enabled),
            "nmpc_mission_segment_start_x": self.start.x,
            "nmpc_mission_segment_start_y": self.start.y,
            "nmpc_mission_segment_start_z": self.start.z,
            "nmpc_mission_segment_goal_x": self.goal.x,
            "nmpc_mission_segment_goal_y": self.goal.y,
            "nmpc_mission_segment_goal_z": self.goal.z,
            "nmpc_mission_segment_ref_x": self.target.x,
            "nmpc_mission_segment_ref_y": self.target.y,
            "nmpc_mission_segment_ref_z": self.target.z,
            "nmpc_mission_segment_progress_s": self.progress_s,
            "nmpc_mission_segment_target_s": self.target_s,
            "nmpc_mission_segment_along_m": self.along_m,
            "nmpc_mission_segment_length_m": self.segment_length_m,
            "nmpc_mission_segment_cross_track_3d_m": self.cross_track_3d_m,
            "nmpc_mission_segment_distance_to_goal_3d_m": self.distance_to_goal_3d_m,
            "nmpc_mission_segment_terminal_mode": int(self.terminal_mode),
            "nmpc_mission_segment_target_yaw": self.target_yaw,
        }


def guide_reference(
    *,
    start: SegmentPoint3D,
    goal: SegmentPoint3D,
    current: SegmentPoint3D,
    config: SegmentGuideConfig,
) -> SegmentGuideReference:
    dx = goal.x - start.x
    dy = goal.y - start.y
    dz = goal.z - start.z
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return _empty_reference(start, goal, current, enabled=config.enabled)

    ux, uy, uz = dx / length, dy / length, dz / length
    wx = current.x - start.x
    wy = current.y - start.y
    wz = current.z - start.z
    along = wx * ux + wy * uy + wz * uz
    progress_s = _clamp(along / length, 0.0, 1.0)
    distance_to_goal = _distance(current, goal)
    terminal_distance = max(0.0, config.terminal_distance_m)
    goal_snap_distance = max(0.0, float(config.goal_snap_distance_m))
    terminal_mode = distance_to_goal <= terminal_distance
    if terminal_mode or distance_to_goal <= goal_snap_distance:
        target_s = 1.0
    else:
        raw_target_s = max(0.0, (along + max(0.0, config.lookahead_m)) / length)
        max_pass_through_s = 1.0 + goal_snap_distance / length
        boundary_s = _xy_boundary_limited_s(start, dx, dy, config.soft_boundary_xy_abs_m)
        target_s = max(progress_s, min(raw_target_s, max_pass_through_s, boundary_s))
    target = _interpolate(start, dx, dy, dz, target_s)
    projected = _interpolate(start, dx, dy, dz, progress_s)
    cross_track = _distance(current, projected)
    return SegmentGuideReference(
        enabled=config.enabled,
        start=start,
        goal=goal,
        target=target,
        progress_s=progress_s,
        target_s=target_s,
        along_m=progress_s * length,
        segment_length_m=length,
        cross_track_3d_m=cross_track,
        distance_to_goal_3d_m=distance_to_goal,
        target_yaw=target_facing_yaw(current, goal, config.yaw_freeze_distance_m),
        terminal_mode=terminal_mode,
    )


def target_facing_yaw(
    current: SegmentPoint3D,
    goal: SegmentPoint3D,
    freeze_distance_m: float,
) -> float | None:
    dx = goal.x - current.x
    dy = goal.y - current.y
    if math.hypot(dx, dy) <= max(0.0, freeze_distance_m):
        return None
    return math.degrees(math.atan2(dy, dx))


def _empty_reference(
    start: SegmentPoint3D,
    goal: SegmentPoint3D,
    current: SegmentPoint3D,
    *,
    enabled: bool,
) -> SegmentGuideReference:
    return SegmentGuideReference(
        enabled=enabled,
        start=start,
        goal=goal,
        target=goal,
        progress_s=1.0,
        target_s=1.0,
        along_m=0.0,
        segment_length_m=0.0,
        cross_track_3d_m=_distance(current, goal),
        distance_to_goal_3d_m=_distance(current, goal),
        target_yaw=None,
        terminal_mode=True,
    )


def _interpolate(
    start: SegmentPoint3D,
    dx: float,
    dy: float,
    dz: float,
    s: float,
) -> SegmentPoint3D:
    return SegmentPoint3D(start.x + dx * s, start.y + dy * s, start.z + dz * s)


def _distance(a: SegmentPoint3D, b: SegmentPoint3D) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _xy_boundary_limited_s(
    start: SegmentPoint3D,
    dx: float,
    dy: float,
    soft_boundary_xy_abs_m: float | None,
) -> float:
    if soft_boundary_xy_abs_m is None:
        return math.inf
    try:
        limit = float(soft_boundary_xy_abs_m)
    except (TypeError, ValueError):
        return math.inf
    if not math.isfinite(limit) or limit <= 0.0:
        return math.inf

    result = math.inf
    for origin, delta in ((start.x, dx), (start.y, dy)):
        if abs(delta) <= 1e-12:
            continue
        if delta > 0.0:
            candidate = (limit - origin) / delta
        else:
            candidate = (-limit - origin) / delta
        if candidate >= 0.0:
            result = min(result, candidate)
    return result


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
