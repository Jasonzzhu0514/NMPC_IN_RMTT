from __future__ import annotations

import math
import time
from typing import Any

from nmpc.flight.math_utils import (
    boundary_active,
    clamp,
    command_saturated,
    delta,
    finite_float,
    profile_for_phase,
    signed_angle_delta,
)
from nmpc.flight.model import profile_json
from nmpc.flight.runtime_state import (
    absolute,
    neutral,
    position_3d_unavailable_reason,
    target_state,
    update_velocity_estimates,
)
from nmpc.flight.types import NmpcFlightControlResult, Sample
from nmpc.position_3d_types import Position3DTarget
from simulation.dji_velocity_plant import PlantState


def compute_flight_controller(
    controller: Any,
    *,
    timestamp: float,
    current_x: float | None,
    current_y: float | None,
    current_z: float | None = None,
    current_yaw: float | None = None,
    target_x: float | None,
    target_y: float | None,
    target_z: float | None = None,
    target_yaw: float | None = None,
    pid_roll_u: float | None = None,
    pid_pitch_u: float | None = None,
    pid_throttle_u: float | None = None,
    pid_yaw_u: float | None = None,
    phase: str | None = None,
) -> NmpcFlightControlResult:
    timestamp_f = finite_float(timestamp)
    x = finite_float(current_x)
    y = finite_float(current_y)
    z = finite_float(current_z)
    yaw = finite_float(current_yaw)
    target_x_f = finite_float(target_x)
    target_y_f = finite_float(target_y)
    target_z_f = finite_float(target_z)
    target_yaw_f = finite_float(target_yaw)
    profile = profile_for_phase(phase)
    yaw_error = (
        signed_angle_delta(target_yaw_f, yaw)
        if target_yaw_f is not None and yaw is not None
        else None
    )
    if timestamp_f is None or x is None or y is None or target_x_f is None or target_y_f is None:
        controller.reset()
        return neutral(
            controller,
            reason="invalid_sample",
            **target_state(profile, target_x_f, target_y_f, target_z_f, target_yaw_f, x, y, z, yaw),
            yaw_error=yaw_error,
        )

    sample = Sample(timestamp=timestamp_f, x=x, y=y, z=z, yaw=yaw)
    if controller._previous_sample is None:
        controller._previous_sample = sample
        return neutral(
            controller,
            reason="warming_up",
            **target_state(profile, target_x_f, target_y_f, target_z_f, target_yaw_f, x, y, z, yaw),
            yaw_error=yaw_error,
        )

    dt = sample.timestamp - controller._previous_sample.timestamp
    if not math.isfinite(dt) or dt < controller.nmpc_flight_config.min_sample_dt_sec:
        return neutral(
            controller,
            reason="sample_dt_too_small",
            **target_state(profile, target_x_f, target_y_f, target_z_f, target_yaw_f, x, y, z, yaw),
            state_vx=controller._vx,
            state_vy=controller._vy,
            state_vz=controller._vz,
            state_yaw_rate=controller._yaw_rate,
            yaw_error=yaw_error,
        )

    if dt > controller.nmpc_flight_config.max_sample_gap_sec:
        controller._previous_sample = sample
        controller._vx = None
        controller._vy = None
        controller._vz = None
        controller._yaw_rate = None
        controller.reset_position_3d_controllers()
        controller._last_throttle_u = 0.0
        controller._last_yaw_u = 0.0
        return neutral(
            controller,
            reason="sample_gap",
            **target_state(profile, target_x_f, target_y_f, target_z_f, target_yaw_f, x, y, z, yaw),
            yaw_error=yaw_error,
        )

    update_velocity_estimates(controller, sample, dt)

    unavailable_reason = position_3d_unavailable_reason(
        controller,
        z=z,
        target_z=target_z_f,
        yaw=yaw,
        target_yaw=target_yaw_f,
    )
    if unavailable_reason is not None:
        controller.reset_position_3d_controllers()
        return neutral(
            controller,
            reason=unavailable_reason,
            **target_state(profile, target_x_f, target_y_f, target_z_f, target_yaw_f, x, y, z, yaw),
            state_vx=controller._vx,
            state_vy=controller._vy,
            state_vz=controller._vz,
            state_yaw_rate=controller._yaw_rate,
            yaw_error=yaw_error,
        )

    start = time.perf_counter()
    position_3d_controller = controller.position_3d_controller_for_profile(profile)
    try:
        command, debug = position_3d_controller.compute(
            PlantState(
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                vx=controller._vx,
                vy=controller._vy,
                vz=controller._vz,
                yaw_rate=controller._yaw_rate,
            ),
            Position3DTarget(target_x_f, target_y_f, target_z_f, target_yaw_f),
            timestamp=timestamp_f,
        )
        throttle_u = command.throttle
        yaw_u = command.yaw
    except Exception as exc:  # noqa: BLE001 - NMPC controller failure must not affect flight.
        controller.reset_position_3d_controllers()
        return neutral(
            controller,
            reason=f"solver_error:{type(exc).__name__}",
            **target_state(profile, target_x_f, target_y_f, target_z_f, target_yaw_f, x, y, z, yaw),
            state_vx=controller._vx,
            state_vy=controller._vy,
            state_vz=controller._vz,
            state_yaw_rate=controller._yaw_rate,
            yaw_error=yaw_error,
        )
    solve_time_ms = (time.perf_counter() - start) * 1000.0

    pitch_u = clamp(command.pitch, -1.0, 1.0)
    roll_u = clamp(command.roll, -1.0, 1.0)
    throttle_u = clamp(throttle_u, -1.0, 1.0)
    yaw_u = clamp(yaw_u, -1.0, 1.0)
    controller._last_throttle_u = throttle_u
    controller._last_yaw_u = yaw_u
    total_candidate_count = debug.candidate_count
    total_cost = debug.cost

    return NmpcFlightControlResult(
        enabled=True,
        reason="ok",
        profile=profile,
        target_x=target_x_f,
        target_y=target_y_f,
        target_z=target_z_f,
        target_yaw=target_yaw_f,
        state_x=x,
        state_y=y,
        state_z=z,
        state_yaw=yaw,
        state_vx=controller._vx,
        state_vy=controller._vy,
        state_vz=controller._vz,
        state_yaw_rate=controller._yaw_rate,
        yaw_error=yaw_error,
        pitch_u=pitch_u,
        roll_u=roll_u,
        throttle_u=throttle_u,
        yaw_u=yaw_u,
        pitch_absolute=absolute(controller, pitch_u),
        roll_absolute=absolute(controller, roll_u),
        throttle_absolute=absolute(controller, throttle_u),
        yaw_absolute=absolute(controller, yaw_u),
        delta_pitch_u=delta(pitch_u, pid_pitch_u),
        delta_roll_u=delta(roll_u, pid_roll_u),
        delta_throttle_u=delta(throttle_u, pid_throttle_u),
        delta_yaw_u=delta(yaw_u, pid_yaw_u),
        predicted_terminal_x=debug.predicted_terminal_x,
        predicted_terminal_y=debug.predicted_terminal_y,
        predicted_terminal_z=debug.predicted_terminal_z,
        predicted_terminal_vz=debug.predicted_terminal_vz,
        predicted_terminal_yaw=debug.predicted_terminal_yaw,
        predicted_terminal_yaw_error=debug.predicted_terminal_yaw_error,
        predicted_terminal_yaw_rate=debug.predicted_terminal_yaw_rate,
        predicted_terminal_speed=debug.predicted_terminal_speed,
        solve_time_ms=solve_time_ms,
        candidate_count=total_candidate_count,
        position_3d_candidate_profile=debug.candidate_profile,
        position_3d_candidate_level_count=debug.candidate_level_count,
        position_3d_profile_json=profile_json(controller.controller_config),
        cost=total_cost,
        command_saturated=(
            command_saturated(
                pitch_u,
                roll_u,
                max_abs_u=position_3d_controller.config.max_abs_xy_u,
            )
            or abs(throttle_u)
            >= position_3d_controller.config.max_abs_z_u - 1e-9
            or abs(yaw_u) >= position_3d_controller.config.max_abs_yaw_u - 1e-9
        ),
        boundary_active=boundary_active(
            x,
            y,
            debug.predicted_terminal_x,
            debug.predicted_terminal_y,
            field_limit=controller.controller_config.field_limit,
            safety_margin=controller.controller_config.safety_margin,
            z=z,
            terminal_z=debug.predicted_terminal_z,
            z_max_m=controller.nmpc_flight_config.boundary_z_max_m,
        ),
    )
