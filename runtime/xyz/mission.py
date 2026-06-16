"""Pure XYZ waypoint runtime helpers for the RMTT NMPC runner."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rmtt_control.pose_source import PoseSample


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    yaw_deg: float = 0.0
    hold_sec: float = 0.8


@dataclass(frozen=True)
class ArrivalThresholds:
    xy_radius: float = 0.10
    z_radius: float = 0.08
    yaw_radius_deg: float = 8.0


@dataclass(frozen=True)
class SafetyBounds:
    field_limit: float = 1.5
    z_min: float = 0.25
    z_max: float = 2.0
    pose_timeout_sec: float = 0.5


@dataclass(frozen=True)
class SafetyResult:
    ok: bool
    reason: str
    pose_age_sec: float | None = None


class WaypointArrivalTracker:
    def __init__(self, thresholds: ArrivalThresholds) -> None:
        self.thresholds = thresholds
        self._arrived_since: float | None = None

    def reset(self) -> None:
        self._arrived_since = None

    def update(self, pose: PoseSample, waypoint: Waypoint, now: float | None = None) -> bool:
        now_f = time.time() if now is None else float(now)
        if arrived(pose, waypoint, self.thresholds):
            self._arrived_since = now_f if self._arrived_since is None else self._arrived_since
            return now_f - self._arrived_since >= max(0.0, waypoint.hold_sec)
        self._arrived_since = None
        return False


def load_waypoints(path: str | Path) -> list[Waypoint]:
    document = json.loads(Path(path).read_text())
    if isinstance(document, dict):
        document = document.get("waypoints", [])
    if not isinstance(document, list):
        raise ValueError("waypoint JSON must be a list or an object with a waypoints list")
    return [waypoint_from_object(item) for item in document]


def waypoint_from_object(item: Any) -> Waypoint:
    if isinstance(item, (list, tuple)):
        if len(item) < 3:
            raise ValueError("waypoint list entries must contain at least x,y,z")
        return Waypoint(
            x=float(item[0]),
            y=float(item[1]),
            z=float(item[2]),
            yaw_deg=float(item[3]) if len(item) > 3 else 0.0,
            hold_sec=float(item[4]) if len(item) > 4 else 0.8,
        )
    if not isinstance(item, dict):
        raise ValueError("waypoint entries must be objects or lists")
    return Waypoint(
        x=float(item["x"]),
        y=float(item["y"]),
        z=float(item["z"]),
        yaw_deg=float(item.get("yaw_deg", item.get("yaw", 0.0))),
        hold_sec=float(item.get("hold_sec", item.get("hold", 0.8))),
    )


def validate_waypoints(waypoints: list[Waypoint], bounds: SafetyBounds) -> None:
    limit = abs(float(bounds.field_limit))
    for index, waypoint in enumerate(waypoints, start=1):
        if abs(waypoint.x) > limit or abs(waypoint.y) > limit:
            raise ValueError(
                "waypoint {0} outside field limit {1}: x={2}, y={3}".format(
                    index,
                    limit,
                    waypoint.x,
                    waypoint.y,
                )
            )
        if waypoint.z < bounds.z_min or waypoint.z > bounds.z_max:
            raise ValueError(
                "waypoint {0} z outside [{1}, {2}]: {3}".format(
                    index,
                    bounds.z_min,
                    bounds.z_max,
                    waypoint.z,
                )
            )


def check_pose_safety(
    pose: PoseSample | None,
    bounds: SafetyBounds,
    *,
    now: float | None = None,
) -> SafetyResult:
    if pose is None:
        return SafetyResult(False, "missing_pose")
    now_f = time.time() if now is None else float(now)
    if pose.timestamp is not None and bounds.pose_timeout_sec > 0.0:
        age = now_f - pose.timestamp
        if age > bounds.pose_timeout_sec:
            return SafetyResult(False, "stale_pose", age)
    else:
        age = None
    limit = abs(float(bounds.field_limit))
    if abs(pose.x) > limit or abs(pose.y) > limit:
        return SafetyResult(False, "xy_boundary", age)
    if pose.z < bounds.z_min:
        return SafetyResult(False, "z_below_min", age)
    if pose.z > bounds.z_max:
        return SafetyResult(False, "z_above_max", age)
    return SafetyResult(True, "ok", age)


def arrived(pose: PoseSample, waypoint: Waypoint, thresholds: ArrivalThresholds) -> bool:
    xy_error = math.hypot(pose.x - waypoint.x, pose.y - waypoint.y)
    z_error = abs(pose.z - waypoint.z)
    yaw_deg = pose_yaw_deg(pose)
    yaw_error = abs(((waypoint.yaw_deg - yaw_deg + 180.0) % 360.0) - 180.0)
    return (
        xy_error <= thresholds.xy_radius
        and z_error <= thresholds.z_radius
        and yaw_error <= thresholds.yaw_radius_deg
    )


def pose_yaw_deg(pose: PoseSample) -> float:
    return math.degrees(pose.yaw) if abs(pose.yaw) <= 6.5 else pose.yaw
