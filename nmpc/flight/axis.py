from __future__ import annotations

from nmpc.flight.math_utils import (
    clamp,
    signed_angle_delta,
    wrap_degrees,
)
from nmpc.flight.types import AxisControllerConfig, AxisControllerDebug


def compute_axis_command(
    *,
    position: float | None,
    velocity: float | None,
    target: float | None,
    model,
    config: AxisControllerConfig,
    previous_u: float,
    angular: bool = False,
) -> tuple[float, AxisControllerDebug | None]:
    if position is None or velocity is None or target is None or model is None:
        return 0.0, None

    error = signed_angle_delta(target, position) if angular else target - position
    if abs(error) <= config.stop_radius and abs(velocity) <= config.stop_speed:
        return 0.0, AxisControllerDebug(
            cost=0.0,
            candidate_count=1,
            predicted_terminal_position=position,
            predicted_terminal_velocity=velocity,
        )

    best_u = 0.0
    best_cost = float("inf")
    best_position = position
    best_velocity = velocity
    candidates = axis_candidates(config.max_abs_u)
    for u in candidates:
        predicted_position, predicted_velocity, cost = rollout_axis(
            position=position,
            velocity=velocity,
            target=target,
            model=model,
            command_u=u,
            previous_u=previous_u,
            config=config,
            angular=angular,
        )
        if cost < best_cost:
            best_cost = cost
            best_u = u
            best_position = predicted_position
            best_velocity = predicted_velocity
    return best_u, AxisControllerDebug(
        cost=best_cost,
        candidate_count=len(candidates),
        predicted_terminal_position=best_position,
        predicted_terminal_velocity=best_velocity,
    )


def rollout_axis(
    *,
    position: float,
    velocity: float,
    target: float,
    model,
    command_u: float,
    previous_u: float,
    config: AxisControllerConfig,
    angular: bool,
) -> tuple[float, float, float]:
    current_position = position
    current_velocity = velocity
    cost = config.control_weight * command_u * command_u
    delta_u = command_u - previous_u
    cost += config.control_delta_weight * delta_u * delta_u
    for step in range(config.horizon_steps):
        current_velocity = step_axis_velocity(current_velocity, model, command_u, config.dt)
        current_position += current_velocity * config.dt
        if angular:
            current_position = wrap_degrees(current_position)
        terminal = step == config.horizon_steps - 1
        error = (
            signed_angle_delta(target, current_position)
            if angular
            else target - current_position
        )
        pos_weight = (
            config.terminal_position_weight if terminal else config.position_weight
        )
        vel_weight = (
            config.terminal_velocity_weight if terminal else config.velocity_weight
        )
        cost += pos_weight * error * error + vel_weight * current_velocity * current_velocity
    return current_position, current_velocity, cost


def step_axis_velocity(velocity: float, model, u: float, dt: float) -> float:
    target_velocity = clamp(model.K * u, -model.vmax, model.vmax)
    accel = (target_velocity - velocity) / model.tau
    accel = clamp(accel, -model.amax, model.amax)
    return velocity + accel * dt


def axis_candidates(max_abs_u: float) -> tuple[float, ...]:
    max_abs_u = abs(float(max_abs_u))
    return (
        -max_abs_u,
        -0.75 * max_abs_u,
        -0.5 * max_abs_u,
        -0.25 * max_abs_u,
        0.0,
        0.25 * max_abs_u,
        0.5 * max_abs_u,
        0.75 * max_abs_u,
        max_abs_u,
    )
