from __future__ import annotations

from typing import Any

from nmpc.flight_controller import NmpcFlightControlResult
from nmpc.mission.math_utils import clamp, finite_float, z_distance
from nmpc.mission.segment_runtime import (
    segment_log_values,
    segment_throttle_authority,
)
from nmpc.mission.types import (
    ACTIVE_AXES,
    NmpcMissionControlDecision,
    PidValues,
)


def ok_decision(
    controller: Any,
    *,
    timestamp: float,
    current_x: float | None,
    current_y: float | None,
    current_z: float | None,
    current_yaw: float | None,
    target_x: float | None,
    target_y: float | None,
    target_z: float | None,
    target_yaw: float | None,
    original_target_x: float | None = None,
    original_target_y: float | None = None,
    original_target_z: float | None = None,
    original_target_yaw: float | None = None,
    distance_xy: float | None,
    controller_result: NmpcFlightControlResult,
    pid_values: PidValues,
    final_target: bool = False,
) -> NmpcMissionControlDecision:
    authority = max(0.0, min(1.0, float(controller.config.max_abs_u)))
    yaw_authority = max(0.0, min(1.0, float(controller.config.yaw_max_abs_u)))
    trim = controller._update_trim(
        timestamp=timestamp,
        current_x=current_x,
        current_y=current_y,
        target_x=target_x,
        target_y=target_y,
        current_yaw=current_yaw,
    )
    body_xy = controller._body_compensated_xy(
        controller_result,
        current_yaw,
        trim_world_roll_u=trim["world_roll_u"],
        trim_world_pitch_u=trim["world_pitch_u"],
    )
    if body_xy is None:
        controller._reset_trim()
        controller._reset_near_min_effective()
        controller._heading_gate_active = False
        controller._reset_last_enabled_decision()
        return fallback(controller, "yaw_missing_fallback", controller_result, pid_values)

    total_authority = max(authority, min(1.0, float(controller.config.trim_total_max_abs_u)))
    near_fine_authority = controller._near_fine_authority(controller_result)
    if near_fine_authority is not None:
        total_authority = min(total_authority, near_fine_authority)
    base_roll_u = clamp(body_xy[0] - body_xy[2], -authority, authority)
    base_pitch_u = clamp(body_xy[1] - body_xy[3], -authority, authority)
    roll_u = clamp(base_roll_u + body_xy[2], -total_authority, total_authority)
    pitch_u = clamp(base_pitch_u + body_xy[3], -total_authority, total_authority)
    heading_error = finite_float(controller_result.yaw_error)
    heading_gate_active = controller._update_heading_gate(
        heading_error_deg=heading_error,
        distance_xy=distance_xy,
    )
    heading_gate_xy_scale = 1.0
    if heading_gate_active:
        heading_gate_xy_scale = clamp(float(controller.config.heading_gate_xy_scale), 0.0, 1.0)
        roll_u = clamp(roll_u * heading_gate_xy_scale, -total_authority, total_authority)
        pitch_u = clamp(pitch_u * heading_gate_xy_scale, -total_authority, total_authority)
        controller._reset_trim()
        controller._reset_near_min_effective()
    arrival_brake = controller._arrival_brake_xy(
        controller_result=controller_result,
        current_x=current_x,
        current_y=current_y,
        target_x=target_x,
        target_y=target_y,
        distance=distance_xy,
        roll_u=roll_u,
        pitch_u=pitch_u,
    )
    roll_u = arrival_brake["roll_u"]
    pitch_u = arrival_brake["pitch_u"]
    if arrival_brake["active"]:
        controller._reset_near_min_effective()

    near_min_effective, roll_u, pitch_u = _near_min_xy_values(
        controller,
        timestamp=timestamp,
        current_x=current_x,
        current_y=current_y,
        current_yaw=current_yaw,
        target_x=target_x,
        target_y=target_y,
        distance_xy=distance_xy,
        roll_u=roll_u,
        pitch_u=pitch_u,
        total_authority=total_authority,
        near_fine_authority=near_fine_authority,
        heading_gate_active=heading_gate_active,
        arrival_brake_active=bool(arrival_brake["active"]),
    )
    roll_u, pitch_u = _final_terminal_xy_capped_values(
        controller,
        final_target=final_target,
        roll_u=roll_u,
        pitch_u=pitch_u,
    )
    throttle_authority = segment_throttle_authority(
        controller,
        default_authority=authority,
        pitch_u=pitch_u,
        roll_u=roll_u,
    )
    throttle_u = clamp(controller_result.throttle_u, -throttle_authority, throttle_authority)
    yaw_u = clamp(controller_result.yaw_u, -yaw_authority, yaw_authority)
    z_near_min_effective = controller._z_near_min_effective(
        throttle_u=throttle_u,
        distance=z_distance(current_z, target_z),
        total_authority=total_authority,
    )
    throttle_u = z_near_min_effective["throttle_u"]
    throttle_rate_limit, throttle_u = _throttle_rate_limited_values(
        controller,
        throttle_u=throttle_u,
    )
    yaw_rate_limit, yaw_u = _yaw_rate_limited_values(
        controller,
        yaw_u=yaw_u,
    )
    decision = _build_ok_decision(
        controller,
        controller_result=controller_result,
        authority=authority,
        yaw_authority=yaw_authority,
        roll_u=roll_u,
        pitch_u=pitch_u,
        throttle_u=throttle_u,
        yaw_u=yaw_u,
        heading_gate_active=heading_gate_active,
        heading_error=heading_error,
        heading_gate_xy_scale=heading_gate_xy_scale,
        arrival_brake=arrival_brake,
        trim=trim,
        body_xy=body_xy,
        near_min_effective=near_min_effective,
        z_near_min_effective=z_near_min_effective,
        throttle_rate_limit=throttle_rate_limit,
        yaw_rate_limit=yaw_rate_limit,
        pid_values=pid_values,
    )
    controller._remember_enabled_decision(
        decision,
        timestamp=timestamp,
        target_x=original_target_x if original_target_x is not None else target_x,
        target_y=original_target_y if original_target_y is not None else target_y,
        target_z=original_target_z if original_target_z is not None else target_z,
        target_yaw=original_target_yaw if original_target_yaw is not None else target_yaw,
    )
    return decision


def fallback(
    controller: Any,
    reason: str,
    nmpc_flight: NmpcFlightControlResult,
    pid_values: PidValues,
) -> NmpcMissionControlDecision:
    controller._last_limited_throttle_u = 0.0
    controller._last_limited_yaw_u = 0.0
    return controller._fallback_decision(
        reason=reason,
        nmpc_flight=nmpc_flight,
        pid_roll=pid_values.roll,
        pid_pitch=pid_values.pitch,
        pid_throttle=pid_values.throttle,
        pid_yaw=pid_values.yaw,
        pid_roll_u=pid_values.roll_u,
        pid_pitch_u=pid_values.pitch_u,
        pid_throttle_u=pid_values.throttle_u,
        pid_yaw_u=pid_values.yaw_u,
    )


def _final_terminal_xy_capped_values(
    controller: Any,
    *,
    final_target: bool,
    roll_u: float,
    pitch_u: float,
) -> tuple[float, float]:
    if not final_target:
        return roll_u, pitch_u
    if not bool(getattr(controller.config, "final_terminal_xy_cap_enabled", False)):
        return roll_u, pitch_u
    reference = getattr(controller, "_last_segment_reference", None)
    if reference is None or not bool(getattr(reference, "terminal_mode", False)):
        return roll_u, pitch_u
    cap = max(0.0, min(1.0, float(getattr(controller.config, "final_terminal_xy_max_abs_u", 0.0))))
    return clamp(roll_u, -cap, cap), clamp(pitch_u, -cap, cap)


def _near_min_xy_values(
    controller: Any,
    *,
    timestamp: float,
    current_x: float | None,
    current_y: float | None,
    current_yaw: float | None,
    target_x: float | None,
    target_y: float | None,
    distance_xy: float | None,
    roll_u: float,
    pitch_u: float,
    total_authority: float,
    near_fine_authority: float | None,
    heading_gate_active: bool,
    arrival_brake_active: bool,
) -> tuple[dict[str, float | bool | None], float, float]:
    near_total_authority = total_authority
    if controller.config.near_min_effective_adaptive_enabled and near_fine_authority is None:
        near_total_authority = max(
            near_total_authority,
            min(1.0, max(0.0, float(controller.config.near_min_effective_adaptive_max_u))),
        )
    if heading_gate_active or arrival_brake_active:
        near_min_effective = controller._near_min_effective_inactive_values(
            roll_u=roll_u,
            pitch_u=pitch_u,
            distance=distance_xy,
        )
    else:
        near_min_effective = controller._near_min_effective_xy(
            timestamp=timestamp,
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            target_x=target_x,
            target_y=target_y,
            roll_u=roll_u,
            pitch_u=pitch_u,
            distance=distance_xy,
            total_authority=near_total_authority,
        )
    roll_u = near_min_effective["roll_u"]
    pitch_u = near_min_effective["pitch_u"]
    if near_fine_authority is not None:
        roll_u = clamp(roll_u, -near_fine_authority, near_fine_authority)
        pitch_u = clamp(pitch_u, -near_fine_authority, near_fine_authority)
    return near_min_effective, roll_u, pitch_u


def _build_ok_decision(
    controller: Any,
    *,
    controller_result: NmpcFlightControlResult,
    authority: float,
    yaw_authority: float,
    roll_u: float,
    pitch_u: float,
    throttle_u: float,
    yaw_u: float,
    heading_gate_active: bool,
    heading_error: float | None,
    heading_gate_xy_scale: float,
    arrival_brake: dict[str, float | bool | None],
    trim: dict[str, float | bool | None],
    body_xy: tuple[float, float, float, float],
    near_min_effective: dict[str, float | bool | None],
    z_near_min_effective: dict[str, float | bool | None],
    throttle_rate_limit: dict[str, float | bool | None],
    yaw_rate_limit: dict[str, float | bool | None],
    pid_values: PidValues,
) -> NmpcMissionControlDecision:
    return NmpcMissionControlDecision(
        enabled=True,
        reason="ok",
        nmpc_flight=controller_result,
        authority_u=authority,
        axes=ACTIVE_AXES,
        roll=controller._absolute(roll_u),
        pitch=controller._absolute(pitch_u),
        throttle=controller._absolute(throttle_u),
        yaw=controller._absolute(yaw_u),
        roll_u=roll_u,
        pitch_u=pitch_u,
        throttle_u=throttle_u,
        yaw_u=yaw_u,
        yaw_authority_u=yaw_authority,
        heading_gate_active=heading_gate_active,
        heading_error_deg=heading_error,
        heading_gate_xy_scale=heading_gate_xy_scale,
        arrival_brake_active=bool(arrival_brake["active"]),
        arrival_brake_scale=float(arrival_brake["scale"]),
        arrival_brake_time_to_target_sec=arrival_brake["time_to_target_sec"],
        arrival_brake_predicted_crossing=bool(arrival_brake["predicted_crossing"]),
        pid_roll=pid_values.roll,
        pid_pitch=pid_values.pitch,
        pid_throttle=pid_values.throttle,
        pid_yaw=pid_values.yaw,
        pid_roll_u=pid_values.roll_u,
        pid_pitch_u=pid_values.pitch_u,
        pid_throttle_u=pid_values.throttle_u,
        pid_yaw_u=pid_values.yaw_u,
        trim_enabled=trim["enabled"],
        trim_distance=trim["distance"],
        trim_world_pitch_u=trim["world_pitch_u"],
        trim_world_roll_u=trim["world_roll_u"],
        trim_body_pitch_u=body_xy[3],
        trim_body_roll_u=body_xy[2],
        near_min_effective_enabled=near_min_effective["enabled"],
        near_min_effective_distance=near_min_effective["distance"],
        near_min_effective_before_pitch_u=near_min_effective["before_pitch_u"],
        near_min_effective_before_roll_u=near_min_effective["before_roll_u"],
        near_min_effective_after_pitch_u=near_min_effective["after_pitch_u"],
        near_min_effective_after_roll_u=near_min_effective["after_roll_u"],
        near_min_effective_floor_u=near_min_effective["floor_u"],
        near_min_effective_stall_sec=near_min_effective["stall_sec"],
        near_min_effective_progress_mps=near_min_effective["progress_mps"],
        near_min_effective_adaptive_bumps=int(near_min_effective["adaptive_bumps"]),
        z_near_min_effective_enabled=z_near_min_effective["enabled"],
        z_near_min_effective_distance=z_near_min_effective["distance"],
        z_near_min_effective_before_throttle_u=z_near_min_effective["before_throttle_u"],
        z_near_min_effective_after_throttle_u=z_near_min_effective["after_throttle_u"],
        throttle_rate_limit_enabled=bool(throttle_rate_limit["enabled"]),
        throttle_rate_limit_before_u=throttle_rate_limit["before_u"],
        throttle_rate_limit_after_u=throttle_rate_limit["after_u"],
        throttle_rate_limit_max_delta_u=throttle_rate_limit["max_delta_u"],
        yaw_rate_limit_enabled=bool(yaw_rate_limit["enabled"]),
        yaw_rate_limit_before_u=yaw_rate_limit["before_u"],
        yaw_rate_limit_after_u=yaw_rate_limit["after_u"],
        yaw_rate_limit_max_delta_u=yaw_rate_limit["max_delta_u"],
        segment_values=segment_log_values(controller),
    )


def _throttle_rate_limited_values(
    controller: Any,
    *,
    throttle_u: float,
) -> tuple[dict[str, float | bool | None], float]:
    return _rate_limited_values(
        controller,
        value_u=throttle_u,
        enabled_attr="throttle_rate_limit_enabled",
        max_delta_attr="throttle_rate_limit_u_per_step",
        last_attr="_last_limited_throttle_u",
    )


def _yaw_rate_limited_values(
    controller: Any,
    *,
    yaw_u: float,
) -> tuple[dict[str, float | bool | None], float]:
    return _rate_limited_values(
        controller,
        value_u=yaw_u,
        enabled_attr="yaw_rate_limit_enabled",
        max_delta_attr="yaw_rate_limit_u_per_step",
        last_attr="_last_limited_yaw_u",
    )


def _rate_limited_values(
    controller: Any,
    *,
    value_u: float,
    enabled_attr: str,
    max_delta_attr: str,
    last_attr: str,
) -> tuple[dict[str, float | bool | None], float]:
    before = float(value_u)
    if not bool(getattr(controller.config, enabled_attr, False)):
        setattr(controller, last_attr, before)
        return _rate_limit_values(False, before, before, None), before

    max_delta = finite_float(getattr(controller.config, max_delta_attr, None))
    if max_delta is None or max_delta <= 0.0:
        setattr(controller, last_attr, before)
        return _rate_limit_values(False, before, before, max_delta), before

    previous = finite_float(getattr(controller, last_attr, 0.0))
    if previous is None:
        previous = 0.0
    after = clamp(before, previous - max_delta, previous + max_delta)
    setattr(controller, last_attr, after)
    return _rate_limit_values(True, before, after, max_delta), after


def _rate_limit_values(
    enabled: bool,
    before_u: float,
    after_u: float,
    max_delta_u: float | None,
) -> dict[str, float | bool | None]:
    return {
        "enabled": enabled,
        "before_u": before_u,
        "after_u": after_u,
        "max_delta_u": max_delta_u,
    }
