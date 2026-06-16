from __future__ import annotations

from typing import Any

from nmpc.flight.math_utils import (
    normalized_to_absolute_stick,
    signed_angle_delta,
    smooth_velocity,
)
from nmpc.flight.types import NmpcFlightControlResult, Sample


def update_velocity_estimates(controller: Any, sample: Sample, dt: float) -> None:
    previous = controller._previous_sample
    raw_vx = (sample.x - previous.x) / dt
    raw_vy = (sample.y - previous.y) / dt
    raw_vz = (
        (sample.z - previous.z) / dt
        if sample.z is not None and previous.z is not None
        else None
    )
    raw_yaw_rate = (
        signed_angle_delta(sample.yaw, previous.yaw) / dt
        if sample.yaw is not None and previous.yaw is not None
        else None
    )
    controller._previous_sample = sample
    alpha = controller.nmpc_flight_config.velocity_smoothing_alpha
    controller._vx = smooth_velocity(controller._vx, raw_vx, alpha)
    controller._vy = smooth_velocity(controller._vy, raw_vy, alpha)
    if raw_vz is not None:
        controller._vz = smooth_velocity(controller._vz, raw_vz, alpha)
    if raw_yaw_rate is not None:
        controller._yaw_rate = smooth_velocity(controller._yaw_rate, raw_yaw_rate, alpha)


def position_3d_unavailable_reason(
    controller: Any,
    *,
    z: float | None,
    target_z: float | None,
    yaw: float | None,
    target_yaw: float | None,
) -> str | None:
    if not controller.nmpc_flight_config.position_3d_enabled:
        return "position_3d_disabled"
    if z is None:
        return "position_3d_missing_z"
    if target_z is None:
        return "position_3d_missing_target_z"
    if controller._vz is None:
        return "position_3d_missing_vz"
    if yaw is None:
        return "position_4d_missing_yaw"
    if target_yaw is None:
        return "position_4d_missing_target_yaw"
    if controller._yaw_rate is None:
        return "position_4d_missing_yaw_rate"
    return None


def neutral(controller: Any, **kwargs: Any) -> NmpcFlightControlResult:
    return NmpcFlightControlResult.neutral(neutral=controller.neutral, **kwargs)


def target_state(
    profile: str,
    target_x: float | None,
    target_y: float | None,
    target_z: float | None,
    target_yaw: float | None,
    x: float | None,
    y: float | None,
    z: float | None,
    yaw: float | None,
) -> dict[str, float | str | None]:
    return {
        "profile": profile,
        "target_x": target_x,
        "target_y": target_y,
        "target_z": target_z,
        "target_yaw": target_yaw,
        "state_x": x,
        "state_y": y,
        "state_z": z,
        "state_yaw": yaw,
    }


def absolute(controller: Any, value: float) -> int:
    return normalized_to_absolute_stick(
        value,
        neutral=controller.neutral,
        stick_min=controller.stick_min,
        stick_max=controller.stick_max,
    )
