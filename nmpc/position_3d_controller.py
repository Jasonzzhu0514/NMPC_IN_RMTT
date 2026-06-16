"""Lightweight 3D shooting NMPC for XYZ guide tracking."""

from __future__ import annotations

import math
from dataclasses import dataclass

from nmpc.flight.axis import step_axis_velocity
from nmpc.flight.math_utils import signed_angle_delta, wrap_degrees
from nmpc.position_3d_types import (
    Position3DCommand,
    Position3DConfig,
    Position3DDebug,
    Position3DTarget,
)
from models.dji_velocity_model import DJIVelocityModel
from simulation.dji_velocity_plant import PlantState


@dataclass(frozen=True)
class _ControlSequence:
    first: Position3DCommand
    second: Position3DCommand | None = None

    def at_step(self, step: int, split_step: int) -> Position3DCommand:
        if self.second is not None and step >= split_step:
            return self.second
        return self.first


class Position3DController:
    def __init__(self, model: DJIVelocityModel, config: Position3DConfig | None = None) -> None:
        model.require_axes(("pitch", "roll", "throttle", "yaw"))
        self.pitch_model = model.axis("pitch")
        self.roll_model = model.axis("roll")
        self.throttle_model = model.axis("throttle")
        self.yaw_model = model.axis("yaw")
        self.config = config or Position3DConfig()
        self._last_command = Position3DCommand(0.0, 0.0, 0.0, 0.0)
        self._candidates = self._build_candidates(self.config)
        self._command_history: list[tuple[float, Position3DCommand]] = []

    def reset(self) -> None:
        self._last_command = Position3DCommand(0.0, 0.0, 0.0, 0.0)
        self._command_history = []

    def record_applied_command(self, timestamp: float, command: Position3DCommand) -> None:
        timestamp_f = _finite_float(timestamp)
        if timestamp_f is None:
            return
        if self._command_history and timestamp_f < self._command_history[-1][0]:
            self._command_history = []
        if not self._command_history:
            self._command_history.append(
                (timestamp_f - self._history_window_sec(), Position3DCommand(0.0, 0.0, 0.0, 0.0))
            )
        self._command_history.append((timestamp_f, command))
        self._last_command = command
        self._trim_command_history(timestamp_f)

    def compute(
        self,
        state: PlantState,
        target: Position3DTarget,
        *,
        timestamp: float | None = None,
    ) -> tuple[Position3DCommand, Position3DDebug]:
        start_time = _finite_float(timestamp)
        if (
            _distance3(state, target) <= self.config.stop_radius
            and _speed3(state) <= self.config.stop_speed
            and abs(signed_angle_delta(target.yaw, state.yaw)) <= self.config.yaw_stop_radius
            and abs(state.yaw_rate) <= self.config.yaw_stop_speed
        ):
            command = Position3DCommand(0.0, 0.0, 0.0, 0.0)
            self._remember_command(start_time, command)
            return command, Position3DDebug(
                cost=0.0,
                candidate_count=1,
                candidate_profile="stopped",
                candidate_level_count=1,
                predicted_terminal_x=state.x,
                predicted_terminal_y=state.y,
                predicted_terminal_z=state.z,
                predicted_terminal_vx=state.vx,
                predicted_terminal_vy=state.vy,
                predicted_terminal_vz=state.vz,
                predicted_terminal_yaw=state.yaw,
                predicted_terminal_yaw_error=signed_angle_delta(target.yaw, state.yaw),
                predicted_terminal_yaw_rate=state.yaw_rate,
                predicted_terminal_speed=_speed3(state),
            )

        best_command: Position3DCommand | None = None
        best_cost = float("inf")
        best_terminal = state
        candidate_count = 0
        for command in self._candidate_commands():
            candidate_count += 1
            predicted, cost = self._rollout(state, target, command, start_time=start_time)
            if cost < best_cost:
                best_cost = cost
                best_command = command
                best_terminal = predicted

        assert best_command is not None
        best_terminal, best_cost = self._rollout_axis_sequence(
            state,
            target,
            best_command,
            start_time=start_time,
        )
        best_command, best_terminal, best_cost, xy_extra_count = self._refine_xy_candidate(
            state,
            target,
            best_command,
            best_terminal,
            best_cost,
            start_time=start_time,
        )
        best_command, best_terminal, best_cost, throttle_extra_count = self._refine_throttle_candidate(
            state,
            target,
            best_command,
            best_terminal,
            best_cost,
            start_time=start_time,
        )
        best_command, best_terminal, best_cost, yaw_extra_count = self._refine_yaw_candidate(
            state,
            target,
            best_command,
            best_terminal,
            best_cost,
            start_time=start_time,
        )
        candidate_count += xy_extra_count + throttle_extra_count + yaw_extra_count
        self._remember_command(start_time, best_command)
        return best_command, Position3DDebug(
            cost=best_cost,
            candidate_count=candidate_count,
            candidate_profile="position_4d",
            candidate_level_count=len(_levels(self.config.max_abs_xy_u, self.config.xy_levels)),
            predicted_terminal_x=best_terminal.x,
            predicted_terminal_y=best_terminal.y,
            predicted_terminal_z=best_terminal.z,
            predicted_terminal_vx=best_terminal.vx,
            predicted_terminal_vy=best_terminal.vy,
            predicted_terminal_vz=best_terminal.vz,
            predicted_terminal_yaw=best_terminal.yaw,
            predicted_terminal_yaw_error=signed_angle_delta(target.yaw, best_terminal.yaw),
            predicted_terminal_yaw_rate=best_terminal.yaw_rate,
            predicted_terminal_speed=_speed3(best_terminal),
        )

    def _rollout(
        self,
        state: PlantState,
        target: Position3DTarget,
        command: Position3DCommand,
        *,
        start_time: float | None,
    ) -> tuple[PlantState, float]:
        sequence = _ControlSequence(command)
        return self._rollout_sequence(state, target, sequence, start_time=start_time)

    def _rollout_sequence(
        self,
        state: PlantState,
        target: Position3DTarget,
        sequence: _ControlSequence,
        *,
        start_time: float | None,
    ) -> tuple[PlantState, float]:
        current = state
        split_step = self._sequence_split_step()
        cost = self._control_cost(sequence.first, state=state, target=target)
        cost += self._sequence_cost(sequence)
        for step in range(self.config.horizon_steps):
            current = self._step(
                current,
                sequence.at_step(step, split_step),
                step=step,
                start_time=start_time,
            )
            terminal = step == self.config.horizon_steps - 1
            cost += self._state_cost(current, target, terminal=terminal)
        return current, cost

    def _refine_xy_candidate(
        self,
        state: PlantState,
        target: Position3DTarget,
        best_command: Position3DCommand,
        best_terminal: PlantState,
        best_cost: float,
        start_time: float | None,
    ) -> tuple[Position3DCommand, PlantState, float, int]:
        if not self.config.xy_fine_enabled:
            return best_command, best_terminal, best_cost, 0
        if _distance3(state, target) > max(0.0, float(self.config.xy_fine_radius_m)):
            return best_command, best_terminal, best_cost, 0
        extra_count = 0
        for pitch in _step_levels(self.config.xy_fine_max_abs_u, self.config.xy_fine_step_u):
            for roll in _step_levels(self.config.xy_fine_max_abs_u, self.config.xy_fine_step_u):
                if abs(pitch - best_command.pitch) < 1e-12 and abs(roll - best_command.roll) < 1e-12:
                    continue
                extra_count += 1
                command = Position3DCommand(
                    pitch,
                    roll,
                    best_command.throttle,
                    best_command.yaw,
                )
                predicted, cost = self._rollout(state, target, command, start_time=start_time)
                if cost < best_cost:
                    best_command = command
                    best_terminal = predicted
                    best_cost = cost
        return best_command, best_terminal, best_cost, extra_count

    def _refine_throttle_candidate(
        self,
        state: PlantState,
        target: Position3DTarget,
        best_command: Position3DCommand,
        best_terminal: PlantState,
        best_cost: float,
        start_time: float | None,
    ) -> tuple[Position3DCommand, PlantState, float, int]:
        if not self.config.throttle_fine_enabled:
            return best_command, best_terminal, best_cost, 0
        extra_count = 0
        for throttle in self._throttle_candidate_levels(step_u=self.config.throttle_fine_step_u):
            if abs(throttle - best_command.throttle) < 1e-12:
                continue
            extra_count += 1
            command = Position3DCommand(
                best_command.pitch,
                best_command.roll,
                throttle,
                best_command.yaw,
            )
            predicted, cost = self._rollout_axis_sequence(state, target, command, start_time=start_time)
            if cost < best_cost:
                best_command = command
                best_terminal = predicted
                best_cost = cost
        return best_command, best_terminal, best_cost, extra_count

    def _refine_yaw_candidate(
        self,
        state: PlantState,
        target: Position3DTarget,
        best_command: Position3DCommand,
        best_terminal: PlantState,
        best_cost: float,
        start_time: float | None,
    ) -> tuple[Position3DCommand, PlantState, float, int]:
        if not self.config.yaw_fine_enabled:
            return best_command, best_terminal, best_cost, 0
        if abs(signed_angle_delta(target.yaw, state.yaw)) > self.config.yaw_fine_error_deg:
            return best_command, best_terminal, best_cost, 0
        extra_count = 0
        for yaw in self._yaw_candidate_levels(
            level_count=self.config.yaw_fine_levels,
            step_u=self.config.yaw_fine_step_u,
        ):
            if abs(yaw - best_command.yaw) < 1e-12:
                continue
            extra_count += 1
            command = Position3DCommand(
                best_command.pitch,
                best_command.roll,
                best_command.throttle,
                yaw,
            )
            predicted, cost = self._rollout_axis_sequence(state, target, command, start_time=start_time)
            if cost < best_cost:
                best_command = command
                best_terminal = predicted
                best_cost = cost
        return best_command, best_terminal, best_cost, extra_count

    def _rollout_axis_sequence(
        self,
        state: PlantState,
        target: Position3DTarget,
        first_command: Position3DCommand,
        *,
        start_time: float | None,
    ) -> tuple[PlantState, float]:
        if not self.config.z_sequence_enabled and not self.config.yaw_sequence_enabled:
            return self._rollout(state, target, first_command, start_time=start_time)
        best_terminal = state
        best_cost = float("inf")
        for second_throttle in self._second_throttle_candidate_levels(first_command.throttle):
            for second_yaw in self._second_yaw_candidate_levels(first_command.yaw):
                sequence = _ControlSequence(
                    first=first_command,
                    second=Position3DCommand(
                        first_command.pitch,
                        first_command.roll,
                        second_throttle,
                        second_yaw,
                    ),
                )
                predicted, cost = self._rollout_sequence(
                    state,
                    target,
                    sequence,
                    start_time=start_time,
                )
                if cost < best_cost:
                    best_terminal = predicted
                    best_cost = cost
        return best_terminal, best_cost

    def _step(
        self,
        state: PlantState,
        command: Position3DCommand,
        *,
        step: int,
        start_time: float | None,
    ) -> PlantState:
        prediction_time = None if start_time is None else start_time + step * self.config.dt
        vx = step_axis_velocity(
            state.vx,
            self.pitch_model,
            self._delayed_u("pitch", command, prediction_time=prediction_time, start_time=start_time),
            self.config.dt,
        )
        vy = step_axis_velocity(
            state.vy,
            self.roll_model,
            self._delayed_u("roll", command, prediction_time=prediction_time, start_time=start_time),
            self.config.dt,
        )
        vz = step_axis_velocity(
            state.vz,
            self.throttle_model,
            self._delayed_u("throttle", command, prediction_time=prediction_time, start_time=start_time),
            self.config.dt,
        )
        yaw_rate = step_axis_velocity(
            state.yaw_rate,
            self.yaw_model,
            self._delayed_u("yaw", command, prediction_time=prediction_time, start_time=start_time),
            self.config.dt,
        )
        return PlantState(
            x=state.x + vx * self.config.dt,
            y=state.y + vy * self.config.dt,
            z=state.z + vz * self.config.dt,
            yaw=wrap_degrees(state.yaw + yaw_rate * self.config.dt),
            vx=vx,
            vy=vy,
            vz=vz,
            yaw_rate=yaw_rate,
        )

    def _remember_command(self, timestamp: float | None, command: Position3DCommand) -> None:
        if timestamp is None:
            self._last_command = command
            return
        self.record_applied_command(timestamp, command)

    def _delayed_u(
        self,
        axis: str,
        command: Position3DCommand,
        *,
        prediction_time: float | None,
        start_time: float | None,
    ) -> float:
        candidate_u = float(getattr(command, axis))
        if prediction_time is None or start_time is None:
            return candidate_u
        model = getattr(self, f"{axis}_model")
        effective_time = prediction_time - max(0.0, float(model.Td))
        if effective_time >= start_time:
            return candidate_u
        return self._history_u(axis, effective_time)

    def _history_u(self, axis: str, sample_time: float) -> float:
        if not self._command_history:
            return 0.0
        if sample_time <= self._command_history[0][0]:
            return float(getattr(self._command_history[0][1], axis))
        if sample_time >= self._command_history[-1][0]:
            return float(getattr(self._command_history[-1][1], axis))
        for index in range(1, len(self._command_history)):
            _, left_command = self._command_history[index - 1]
            right_t, right_command = self._command_history[index]
            if sample_time > right_t:
                continue
            if abs(sample_time - right_t) <= 1e-12:
                return float(getattr(right_command, axis))
            return float(getattr(left_command, axis))
        return float(getattr(self._command_history[-1][1], axis))

    def _history_window_sec(self) -> float:
        max_delay = max(
            self.pitch_model.Td,
            self.roll_model.Td,
            self.throttle_model.Td,
            self.yaw_model.Td,
        )
        return max(0.0, float(max_delay)) + self.config.horizon_steps * self.config.dt + self.config.dt

    def _trim_command_history(self, timestamp: float) -> None:
        cutoff = timestamp - self._history_window_sec()
        while len(self._command_history) > 2 and self._command_history[1][0] < cutoff:
            self._command_history.pop(0)

    def _state_cost(self, state: PlantState, target: Position3DTarget, *, terminal: bool) -> float:
        dx = state.x - target.x
        dy = state.y - target.y
        dz = state.z - target.z
        dyaw = signed_angle_delta(target.yaw, state.yaw)
        xy_weight = self.config.terminal_position_weight_xy if terminal else self.config.position_weight_xy
        xy_error = math.hypot(dx, dy)
        near_precision_radius = max(0.0, float(self.config.near_target_precision_radius_m))
        precision_cost = 0.0
        if terminal and xy_error <= near_precision_radius:
            xy_weight = max(xy_weight, float(self.config.near_target_precision_weight_xy))
            precision_target = max(0.0, float(self.config.near_target_precision_target_m))
            precision_excess = max(0.0, xy_error - precision_target)
            precision_cost = (
                max(0.0, float(self.config.near_target_precision_hinge_weight_xy))
                * precision_excess
                * precision_excess
            )
        z_weight = self.config.terminal_position_weight_z if terminal else self.config.position_weight_z
        yaw_weight = (
            self.config.terminal_position_weight_yaw
            if terminal
            else self.config.position_weight_yaw
        )
        xy_velocity_weight = (
            self.config.terminal_velocity_weight_xy if terminal else self.config.velocity_weight_xy
        )
        z_velocity_weight = (
            self.config.terminal_velocity_weight_z if terminal else self.config.velocity_weight_z
        )
        yaw_velocity_weight = (
            self.config.terminal_velocity_weight_yaw
            if terminal
            else self.config.velocity_weight_yaw
        )
        return (
            xy_weight * (dx * dx + dy * dy)
            + z_weight * dz * dz
            + precision_cost
            + yaw_weight * dyaw * dyaw
            + xy_velocity_weight * (state.vx * state.vx + state.vy * state.vy)
            + z_velocity_weight * state.vz * state.vz
            + yaw_velocity_weight * state.yaw_rate * state.yaw_rate
        )

    def _control_cost(self, command: Position3DCommand, *, state: PlantState, target: Position3DTarget) -> float:
        du_pitch = command.pitch - self._last_command.pitch
        du_roll = command.roll - self._last_command.roll
        du_throttle = command.throttle - self._last_command.throttle
        du_yaw = command.yaw - self._last_command.yaw
        return (
            self.config.control_weight_xy * (command.pitch * command.pitch + command.roll * command.roll)
            + self.config.control_weight_z * command.throttle * command.throttle
            + self.config.control_weight_yaw * command.yaw * command.yaw
            + self.config.control_delta_weight_xy * (du_pitch * du_pitch + du_roll * du_roll)
            + self.config.control_delta_weight_z * du_throttle * du_throttle
            + self.config.control_delta_weight_yaw * du_yaw * du_yaw
            + self._sign_flip_cost(command)
            + self._progress_coast_cost(command, state=state, target=target)
            + self._yaw_progress_coast_cost(command, state=state, target=target)
        )

    def _sequence_cost(self, sequence: _ControlSequence) -> float:
        if sequence.second is None:
            return 0.0
        cost = 0.0
        if self.config.z_sequence_enabled:
            d_throttle = sequence.second.throttle - sequence.first.throttle
            cost += (
                max(0.0, float(self.config.throttle_sequence_delta_weight))
                * d_throttle
                * d_throttle
            )
        if self.config.yaw_sequence_enabled:
            d_yaw = sequence.second.yaw - sequence.first.yaw
            cost += max(0.0, float(self.config.yaw_sequence_delta_weight)) * d_yaw * d_yaw
        return cost

    def _sign_flip_cost(self, command: Position3DCommand) -> float:
        cost = 0.0
        if is_sign_flip(command.pitch, self._last_command.pitch):
            cost += self.config.sign_flip_weight_xy
        if is_sign_flip(command.roll, self._last_command.roll):
            cost += self.config.sign_flip_weight_xy
        if is_sign_flip(command.throttle, self._last_command.throttle):
            cost += self.config.sign_flip_weight_z
        if is_sign_flip(command.yaw, self._last_command.yaw):
            cost += self.config.sign_flip_weight_yaw
        return cost

    def _progress_coast_cost(
        self,
        command: Position3DCommand,
        *,
        state: PlantState,
        target: Position3DTarget,
    ) -> float:
        weight = max(0.0, float(self.config.progress_coast_penalty_weight))
        min_u = max(0.0, float(self.config.progress_coast_penalty_min_u))
        distance_threshold = max(0.0, float(self.config.progress_coast_penalty_distance_m))
        if weight <= 0.0 or min_u <= 0.0 or distance_threshold <= 0.0:
            return 0.0
        distance_to_target = _distance3(state, target)
        if distance_to_target <= distance_threshold:
            return 0.0
        ux = (target.x - state.x) / distance_to_target
        uy = (target.y - state.y) / distance_to_target
        uz = (target.z - state.z) / distance_to_target
        progress_u = command.pitch * ux + command.roll * uy + command.throttle * uz
        deficit = max(0.0, min_u - progress_u)
        return weight * deficit * deficit

    def _yaw_progress_coast_cost(
        self,
        command: Position3DCommand,
        *,
        state: PlantState,
        target: Position3DTarget,
    ) -> float:
        weight = max(0.0, float(self.config.yaw_progress_coast_penalty_weight))
        min_u = max(0.0, float(self.config.yaw_progress_coast_penalty_min_u))
        error_threshold = max(0.0, float(self.config.yaw_progress_coast_penalty_error_deg))
        if weight <= 0.0 or min_u <= 0.0 or error_threshold <= 0.0:
            return 0.0
        yaw_error = signed_angle_delta(target.yaw, state.yaw)
        if abs(yaw_error) <= error_threshold:
            return 0.0
        desired_sign = -1.0 if yaw_error > 0.0 else 1.0
        progress_u = command.yaw * desired_sign
        deficit = max(0.0, min_u - progress_u)
        scaled_deficit = deficit * abs(yaw_error)
        return weight * scaled_deficit * scaled_deficit

    @staticmethod
    def _build_candidates(config: Position3DConfig) -> tuple[Position3DCommand, ...]:
        xy_levels = _levels(config.max_abs_xy_u, config.xy_levels)
        z_levels = _levels(config.max_abs_z_u, config.z_levels)
        yaw_levels = _levels(config.max_abs_yaw_u, config.yaw_levels)
        return tuple(
            Position3DCommand(pitch, roll, throttle, yaw)
            for pitch in xy_levels
            for roll in xy_levels
            for throttle in z_levels
            for yaw in yaw_levels
        )

    def _candidate_commands(self) -> tuple[Position3DCommand, ...]:
        throttle_levels = self._throttle_candidate_levels()
        yaw_levels = self._yaw_candidate_levels(level_count=self.config.yaw_levels)
        return tuple(
            Position3DCommand(command.pitch, command.roll, throttle, yaw)
            for command in self._candidates
            if abs(command.throttle) <= 1e-12
            if abs(command.yaw) <= 1e-12
            for throttle in throttle_levels
            for yaw in yaw_levels
        )

    def _sequence_split_step(self) -> int:
        if self.config.horizon_steps <= 1:
            return 1
        configured = int(self.config.sequence_split_step)
        return min(max(1, configured), self.config.horizon_steps - 1)

    def _second_throttle_candidate_levels(self, first_throttle: float) -> tuple[float, ...]:
        if not self.config.z_sequence_enabled:
            return (round(float(first_throttle), 12),)
        step = max(0.0, float(self.config.throttle_sequence_step_u))
        levels = self._throttle_candidate_levels(step_u=step) if step > 0.0 else self._throttle_candidate_levels()
        return _constrain_levels_around(
            levels,
            center=first_throttle,
            max_abs=self.config.max_abs_z_u,
            max_delta=self.config.throttle_max_delta_u_per_step,
        )

    def _second_yaw_candidate_levels(self, first_yaw: float) -> tuple[float, ...]:
        if not self.config.yaw_sequence_enabled:
            return (round(float(first_yaw), 12),)
        step = max(0.0, float(self.config.yaw_sequence_step_u))
        levels = (
            self._yaw_candidate_levels(level_count=self.config.yaw_fine_levels, step_u=step)
            if step > 0.0
            else self._yaw_candidate_levels(level_count=self.config.yaw_levels)
        )
        return _constrain_levels_around(
            levels,
            center=first_yaw,
            max_abs=self.config.max_abs_yaw_u,
            max_delta=self.config.yaw_max_delta_u_per_step,
        )

    def _throttle_candidate_levels(self, *, step_u: float | None = None) -> tuple[float, ...]:
        levels = (
            _levels(self.config.max_abs_z_u, self.config.z_levels)
            if step_u is None
            else _step_levels(self.config.max_abs_z_u, step_u)
        )
        max_delta = max(0.0, float(self.config.throttle_max_delta_u_per_step))
        if max_delta <= 0.0:
            return levels
        previous = float(self._last_command.throttle)
        low = max(-abs(float(self.config.max_abs_z_u)), previous - max_delta)
        high = min(abs(float(self.config.max_abs_z_u)), previous + max_delta)
        constrained = {round(level, 12) for level in levels if low - 1e-12 <= level <= high + 1e-12}
        constrained.add(round(max(low, min(high, previous)), 12))
        constrained.add(round(low, 12))
        constrained.add(round(high, 12))
        return tuple(sorted(constrained))

    def _yaw_candidate_levels(self, *, level_count: int, step_u: float | None = None) -> tuple[float, ...]:
        levels = (
            _levels(self.config.max_abs_yaw_u, level_count)
            if step_u is None
            else _step_levels(self.config.max_abs_yaw_u, step_u)
        )
        max_delta = max(0.0, float(self.config.yaw_max_delta_u_per_step))
        if max_delta <= 0.0:
            return levels
        previous = float(self._last_command.yaw)
        low = max(-abs(float(self.config.max_abs_yaw_u)), previous - max_delta)
        high = min(abs(float(self.config.max_abs_yaw_u)), previous + max_delta)
        constrained = {round(level, 12) for level in levels if low - 1e-12 <= level <= high + 1e-12}
        constrained.add(round(max(low, min(high, previous)), 12))
        constrained.add(round(low, 12))
        constrained.add(round(high, 12))
        return tuple(sorted(constrained))


def _levels(max_abs_u: float, count: int) -> tuple[float, ...]:
    max_abs = abs(float(max_abs_u))
    count = max(3, int(count))
    if count % 2 == 0:
        count += 1
    midpoint = count // 2
    return tuple(round((index - midpoint) / midpoint * max_abs, 12) for index in range(count))


def _step_levels(max_abs_u: float, step_u: float) -> tuple[float, ...]:
    max_abs = abs(float(max_abs_u))
    step = abs(float(step_u))
    if max_abs <= 0.0 or step <= 0.0:
        return (0.0,)
    count = max(1, int(math.floor(max_abs / step)))
    values = {0.0}
    for index in range(1, count + 1):
        value = min(max_abs, round(index * step, 12))
        values.add(value)
        values.add(-value)
    values.add(round(max_abs, 12))
    values.add(round(-max_abs, 12))
    return tuple(sorted(values))


def _constrain_levels_around(
    levels: tuple[float, ...],
    *,
    center: float,
    max_abs: float,
    max_delta: float,
) -> tuple[float, ...]:
    max_abs_f = abs(float(max_abs))
    max_delta_f = max(0.0, float(max_delta))
    center_f = max(-max_abs_f, min(max_abs_f, float(center)))
    if max_delta_f <= 0.0:
        return levels
    low = max(-max_abs_f, center_f - max_delta_f)
    high = min(max_abs_f, center_f + max_delta_f)
    constrained = {round(level, 12) for level in levels if low - 1e-12 <= level <= high + 1e-12}
    constrained.add(round(center_f, 12))
    constrained.add(round(low, 12))
    constrained.add(round(high, 12))
    return tuple(sorted(constrained))


def is_sign_flip(current: float, previous: float) -> bool:
    eps = 0.015
    if abs(current) < eps or abs(previous) < eps:
        return False
    return current * previous < 0.0


def _finite_float(value: float | int | None) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def _distance3(state: PlantState, target: Position3DTarget) -> float:
    return math.sqrt((state.x - target.x) ** 2 + (state.y - target.y) ** 2 + (state.z - target.z) ** 2)


def _speed3(state: PlantState) -> float:
    return math.sqrt(state.vx * state.vx + state.vy * state.vy + state.vz * state.vz)
