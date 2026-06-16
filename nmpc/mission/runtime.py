from __future__ import annotations

from dataclasses import replace
from typing import Any

from nmpc.flight_controller import NmpcFlightControlResult
from nmpc.mission.math_utils import (
    absolute_or_neutral,
    OsdFreshness,
    osd_freshness,
    xy_distance,
)
from nmpc.mission.runtime_decision import (
    fallback,
    ok_decision,
)
from nmpc.mission.segment_runtime import guided_targets
from nmpc.mission.segment_runtime import reset_segment_guide
from nmpc.mission.types import NmpcMissionControlDecision, PidValues


NMPC_ACTIVE_PHASES = {"MOVE-XYZ", "TASK", "ALIGN", "VERTICAL"}


def compute_mission_controller(
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
    phase: str,
    pid_roll: int | float | None,
    pid_pitch: int | float | None,
    pid_throttle: int | float | None,
    pid_yaw: int | float | None,
    plane_state: str | None = None,
    osd_source: Any | None = None,
    preserve_target_yaw: bool = False,
    final_target: bool = False,
) -> NmpcMissionControlDecision:
    pid_roll_i = absolute_or_neutral(pid_roll, controller.neutral, controller.stick_min, controller.stick_max)
    pid_pitch_i = absolute_or_neutral(pid_pitch, controller.neutral, controller.stick_min, controller.stick_max)
    pid_throttle_i = absolute_or_neutral(pid_throttle, controller.neutral, controller.stick_min, controller.stick_max)
    pid_yaw_i = absolute_or_neutral(pid_yaw, controller.neutral, controller.stick_min, controller.stick_max)
    pid_roll_u = controller._normalized(pid_roll_i)
    pid_pitch_u = controller._normalized(pid_pitch_i)
    pid_throttle_u = controller._normalized(pid_throttle_i)
    pid_yaw_u = controller._normalized(pid_yaw_i)
    pid_values = PidValues(
        pid_roll_i,
        pid_pitch_i,
        pid_throttle_i,
        pid_yaw_i,
        pid_roll_u,
        pid_pitch_u,
        pid_throttle_u,
        pid_yaw_u,
    )

    phase_key = phase.strip().upper()
    if phase_key not in NMPC_ACTIVE_PHASES:
        return _reset_and_fallback(
            controller,
            reason="phase_not_active_fallback",
            core_reason="phase_not_active",
            profile="yaw_priority",
            pid_values=pid_values,
            current_x=current_x,
            current_y=current_y,
            current_z=current_z,
            current_yaw=current_yaw,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_yaw=target_yaw,
        )

    freshness = _current_osd_freshness(controller, osd_source)
    if controller.config.require_osd and freshness.missing:
        return _reset_and_fallback(
            controller,
            reason="osd_missing_fallback",
            core_reason="osd_missing",
            profile="move_xyz_priority",
            pid_values=pid_values,
            osd_freshness=freshness,
            current_x=current_x,
            current_y=current_y,
            current_z=current_z,
            current_yaw=current_yaw,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_yaw=target_yaw,
        )

    original_target_x = target_x
    original_target_y = target_y
    original_target_z = target_z
    original_target_yaw = target_yaw
    guided = guided_targets(
        controller,
        current_x=current_x,
        current_y=current_y,
        current_z=current_z,
        target_x=target_x,
        target_y=target_y,
        target_z=target_z,
        target_yaw=target_yaw,
        preserve_target_yaw=preserve_target_yaw,
    )
    target_x = guided.target_x
    target_y = guided.target_y
    target_z = guided.target_z
    target_yaw = guided.target_yaw
    distance_xy = xy_distance(current_x, current_y, target_x, target_y)
    capture_distance = max(0.0, float(controller.config.capture_distance_m))
    if (
        phase_key == "MOVE-XYZ"
        and distance_xy is not None
        and capture_distance > 0.0
        and distance_xy <= capture_distance
    ):
        return _reset_and_fallback(
            controller,
            reason="capture_distance_fallback",
            core_reason="capture_distance",
            profile="move_xyz_priority",
            pid_values=pid_values,
            current_x=current_x,
            current_y=current_y,
            current_z=current_z,
            current_yaw=current_yaw,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_yaw=target_yaw,
        )

    controller_result = controller.nmpc_flight.compute(
        timestamp=timestamp,
        current_x=current_x,
        current_y=current_y,
        current_z=current_z,
        current_yaw=current_yaw,
        target_x=target_x,
        target_y=target_y,
        target_z=target_z,
        target_yaw=target_yaw,
        pid_roll_u=pid_roll_u,
        pid_pitch_u=pid_pitch_u,
        pid_throttle_u=pid_throttle_u,
        pid_yaw_u=pid_yaw_u,
        phase=phase,
    )
    controller_result = _with_osd_diagnostics(controller_result, freshness)
    if not controller_result.enabled or controller_result.reason != "ok":
        held = controller._held_sample_dt_decision(
            timestamp=timestamp,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_yaw=target_yaw,
            nmpc_flight=controller_result,
            pid_roll=pid_roll_i,
            pid_pitch=pid_pitch_i,
            pid_throttle=pid_throttle_i,
            pid_yaw=pid_yaw_i,
            pid_roll_u=pid_roll_u,
            pid_pitch_u=pid_pitch_u,
            pid_throttle_u=pid_throttle_u,
            pid_yaw_u=pid_yaw_u,
        )
        if held is not None:
            return held
        controller._reset_near_min_effective()
        controller._heading_gate_active = False
        controller._reset_last_enabled_decision()
        return fallback(controller, f"{controller_result.reason}_fallback", controller_result, pid_values)
    if controller.config.fallback_on_boundary and controller_result.boundary_active:
        controller._reset_near_min_effective()
        controller._heading_gate_active = False
        controller._reset_last_enabled_decision()
        return fallback(controller, "boundary_fallback", controller_result, pid_values)

    return ok_decision(
        controller,
        timestamp=timestamp,
        current_x=current_x,
        current_y=current_y,
        current_z=current_z,
        current_yaw=current_yaw,
        target_x=target_x,
        target_y=target_y,
        target_z=target_z,
        target_yaw=target_yaw,
        original_target_x=original_target_x,
        original_target_y=original_target_y,
        original_target_z=original_target_z,
        original_target_yaw=original_target_yaw,
        distance_xy=distance_xy,
        controller_result=controller_result,
        pid_values=pid_values,
        final_target=final_target,
    )


def _reset_and_fallback(
    controller: Any,
    *,
    reason: str,
    core_reason: str,
    profile: str,
    pid_values: PidValues,
    osd_freshness: OsdFreshness | None = None,
    current_x: float | None,
    current_y: float | None,
    current_z: float | None,
    current_yaw: float | None,
    target_x: float | None,
    target_y: float | None,
    target_z: float | None,
    target_yaw: float | None,
) -> NmpcMissionControlDecision:
    controller.nmpc_flight.reset()
    controller._reset_trim()
    controller._reset_near_min_effective()
    controller._heading_gate_active = False
    controller._reset_last_enabled_decision()
    reset_segment_guide(controller)
    controller_result = controller._neutral_core(
        reason=core_reason,
        state_x=current_x,
        state_y=current_y,
        state_z=current_z,
        state_yaw=current_yaw,
        target_x=target_x,
        target_y=target_y,
        target_z=target_z,
        target_yaw=target_yaw,
        profile=profile,
    )
    controller_result = _with_osd_diagnostics(controller_result, osd_freshness)
    return fallback(controller, reason, controller_result, pid_values)


def _current_osd_freshness(controller: Any, osd_source: Any | None) -> OsdFreshness:
    return osd_freshness(
        osd_source,
        monotonic_now=controller._monotonic_provider(),
        max_age_sec=controller.config.max_osd_age_sec,
    )


def _with_osd_diagnostics(
    result: NmpcFlightControlResult,
    freshness: OsdFreshness | None,
) -> NmpcFlightControlResult:
    if freshness is None:
        return result
    return replace(
        result,
        osd_age_sec=freshness.age_sec,
        osd_last_msg_monotonic=freshness.last_msg_monotonic,
        osd_control_monotonic_now=freshness.monotonic_now,
        osd_max_age_sec=freshness.max_age_sec,
        osd_frequency_hz=freshness.frequency_hz,
        osd_missing_reason=freshness.reason,
    )
