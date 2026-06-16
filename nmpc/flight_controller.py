"""NMPC controller computation for active control.

This module is side-effect free: it computes NMPC stick recommendations from observed state and returns loggable values.

"""

from __future__ import annotations

from pathlib import Path

from nmpc.flight.axis import (
    axis_candidates,
    compute_axis_command,
    rollout_axis,
    step_axis_velocity,
)
from nmpc.flight.math_utils import (
    boundary_active,
    clamp,
    command_saturated,
    delta,
    finite_float,
    normalized_to_absolute_stick,
    profile_for_phase,
    signed_angle_delta,
    smooth_velocity,
)
from nmpc.flight.model import (
    conservative_xy_model,
    profile_json,
)
from nmpc.flight.runtime import compute_flight_controller
from nmpc.flight.types import (
    AxisControllerConfig,
    AxisControllerDebug,
    NmpcFlightControlResult,
    NmpcFlightControllerConfig,
    Sample,
)
from nmpc.position_3d_controller import Position3DController
from nmpc.position_3d_types import Position3DConfig, Position3DProfile
from models.dji_velocity_model import (
    DEFAULT_MODEL_PATH,
    DJIVelocityModel,
    load_velocity_model,
)


_Sample = Sample
_compute_axis_command = compute_axis_command
_rollout_axis = rollout_axis
_step_axis_velocity = step_axis_velocity
_axis_candidates = axis_candidates
_profile_json = profile_json
_conservative_xy_model = conservative_xy_model
_profile_for_phase = profile_for_phase
_smooth_velocity = smooth_velocity
_finite_float = finite_float
_delta = delta
_signed_angle_delta = signed_angle_delta
_command_saturated = command_saturated
_boundary_active = boundary_active
_clamp = clamp


class NmpcFlightController:
    def __init__(
        self,
        *,
        model: DJIVelocityModel | None = None,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        controller_config: Position3DProfile | None = None,
        nmpc_flight_config: NmpcFlightControllerConfig | None = None,
        neutral: int,
        stick_min: int,
        stick_max: int,
    ) -> None:
        loaded_model = model or load_velocity_model(model_path)
        self.controller_config = controller_config or Position3DProfile()
        self.nmpc_flight_config = nmpc_flight_config or NmpcFlightControllerConfig()
        self.model = _conservative_xy_model(loaded_model, self.nmpc_flight_config)
        self.yaw_axis_config = AxisControllerConfig(
            max_abs_u=0.55,
            stop_radius=4.0,
            stop_speed=4.0,
            position_weight=9.0,
            terminal_position_weight=65.0,
            velocity_weight=1.2,
            terminal_velocity_weight=9.0,
            control_weight=0.06,
            control_delta_weight=0.18,
        )
        self.yaw_priority_config = AxisControllerConfig(
            max_abs_u=0.55,
            stop_radius=2.0,
            stop_speed=3.0,
            position_weight=18.0,
            terminal_position_weight=130.0,
            velocity_weight=1.0,
            terminal_velocity_weight=10.0,
            control_weight=0.08,
            control_delta_weight=0.25,
        )
        self.position_3d_controller = Position3DController(
            self.model,
            self._position_3d_config(self.controller_config, self.yaw_axis_config),
        )
        self.yaw_priority_position_3d_controller = Position3DController(
            self.model,
            self._position_3d_config(self.controller_config, self.yaw_priority_config),
        )
        self.task_hold_position_3d_controller = Position3DController(
            self.model,
            self._task_hold_position_3d_config(self.controller_config, self.yaw_priority_config),
        )
        self.neutral = int(neutral)
        self.stick_min = int(stick_min)
        self.stick_max = int(stick_max)
        self._previous_sample: _Sample | None = None
        self._vx: float | None = None
        self._vy: float | None = None
        self._vz: float | None = None
        self._yaw_rate: float | None = None
        self._last_throttle_u = 0.0
        self._last_yaw_u = 0.0

    def update_position_3d_profile(self, config: Position3DProfile) -> None:
        self.controller_config = config
        self.position_3d_controller = Position3DController(
            self.model,
            self._position_3d_config(config, self.yaw_axis_config),
        )
        self.yaw_priority_position_3d_controller = Position3DController(
            self.model,
            self._position_3d_config(config, self.yaw_priority_config),
        )
        self.task_hold_position_3d_controller = Position3DController(
            self.model,
            self._task_hold_position_3d_config(config, self.yaw_priority_config),
        )
        self.reset()

    def reset(self) -> None:
        self.reset_position_3d_controllers()
        self._previous_sample = None
        self._vx = None
        self._vy = None
        self._vz = None
        self._yaw_rate = None
        self._last_throttle_u = 0.0
        self._last_yaw_u = 0.0

    def reset_position_3d_controllers(self) -> None:
        self.position_3d_controller.reset()
        self.yaw_priority_position_3d_controller.reset()
        self.task_hold_position_3d_controller.reset()

    def position_3d_controller_for_profile(self, profile: str) -> Position3DController:
        if profile == "task_hold":
            return self.task_hold_position_3d_controller
        if profile == "yaw_priority":
            return self.yaw_priority_position_3d_controller
        return self.position_3d_controller

    def _task_hold_position_3d_config(
        self,
        config: Position3DProfile,
        yaw_config: AxisControllerConfig,
    ) -> Position3DConfig:
        base = self._position_3d_config(config, yaw_config)
        return Position3DConfig(
            **{
                **base.__dict__,
                "xy_fine_enabled": self.nmpc_flight_config.position_3d_task_xy_fine_enabled,
                "xy_fine_radius_m": self.nmpc_flight_config.position_3d_task_xy_fine_radius_m,
                "xy_fine_max_abs_u": self.nmpc_flight_config.position_3d_task_xy_fine_max_abs_u,
                "xy_fine_step_u": self.nmpc_flight_config.position_3d_task_xy_fine_step_u,
                "position_weight_xy": config.position_weight * 1.25,
                "terminal_position_weight_xy": config.terminal_position_weight * 1.5,
                "velocity_weight_xy": config.velocity_weight * 0.6,
                "terminal_velocity_weight_xy": config.terminal_velocity_weight * 0.7,
                "control_delta_weight_xy": config.control_delta_weight * 0.65,
                "progress_coast_penalty_distance_m": max(
                    base.progress_coast_penalty_distance_m,
                    self.nmpc_flight_config.position_3d_task_xy_fine_radius_m,
                ),
            }
        )

    def _position_3d_config(
        self,
        config: Position3DProfile,
        yaw_config: AxisControllerConfig,
    ) -> Position3DConfig:
        return Position3DConfig(
            dt=config.dt,
            horizon_steps=config.horizon_steps,
            max_abs_xy_u=self.nmpc_flight_config.position_3d_max_abs_xy_u,
            max_abs_z_u=self.nmpc_flight_config.position_3d_max_abs_z_u,
            max_abs_yaw_u=min(
                max(0.0, float(self.nmpc_flight_config.position_3d_max_abs_yaw_u)),
                max(0.0, float(yaw_config.max_abs_u)),
            ),
            throttle_max_delta_u_per_step=(
                self.nmpc_flight_config.position_3d_throttle_max_delta_u_per_step
            ),
            yaw_max_delta_u_per_step=self.nmpc_flight_config.position_3d_yaw_max_delta_u_per_step,
            xy_levels=self.nmpc_flight_config.position_3d_xy_levels,
            z_levels=self.nmpc_flight_config.position_3d_z_levels,
            yaw_levels=self.nmpc_flight_config.position_3d_yaw_levels,
            yaw_fine_enabled=self.nmpc_flight_config.position_3d_yaw_fine_enabled,
            yaw_fine_error_deg=self.nmpc_flight_config.position_3d_yaw_fine_error_deg,
            yaw_fine_levels=self.nmpc_flight_config.position_3d_yaw_fine_levels,
            throttle_fine_enabled=self.nmpc_flight_config.position_3d_throttle_fine_enabled,
            throttle_fine_step_u=self.nmpc_flight_config.position_3d_throttle_fine_step_u,
            yaw_fine_step_u=self.nmpc_flight_config.position_3d_yaw_fine_step_u,
            z_sequence_enabled=self.nmpc_flight_config.position_3d_z_sequence_enabled,
            yaw_sequence_enabled=self.nmpc_flight_config.position_3d_yaw_sequence_enabled,
            sequence_split_step=self.nmpc_flight_config.position_3d_sequence_split_step,
            throttle_sequence_step_u=self.nmpc_flight_config.position_3d_throttle_sequence_step_u,
            yaw_sequence_step_u=self.nmpc_flight_config.position_3d_yaw_sequence_step_u,
            throttle_sequence_delta_weight=(
                self.nmpc_flight_config.position_3d_throttle_sequence_delta_weight
            ),
            yaw_sequence_delta_weight=self.nmpc_flight_config.position_3d_yaw_sequence_delta_weight,
            xy_fine_enabled=config.near_fine_candidate_enabled,
            xy_fine_radius_m=min(
                max(0.0, float(config.near_fine_candidate_radius)),
                max(0.0, float(config.stop_radius) + 0.13),
            ),
            xy_fine_max_abs_u=min(
                max(0.0, float(config.near_fine_candidate_max_abs_u)),
                max(0.0, float(self.nmpc_flight_config.position_3d_max_abs_xy_u)),
            ),
            xy_fine_step_u=config.near_fine_candidate_step_u,
            stop_radius=config.stop_radius,
            stop_speed=config.stop_speed,
            yaw_stop_radius=yaw_config.stop_radius,
            yaw_stop_speed=yaw_config.stop_speed,
            position_weight_xy=config.position_weight,
            position_weight_yaw=yaw_config.position_weight,
            terminal_position_weight_xy=config.terminal_position_weight,
            terminal_position_weight_yaw=yaw_config.terminal_position_weight,
            near_target_precision_radius_m=min(
                max(0.0, float(config.near_target_radius)),
                max(0.0, float(config.stop_radius) + 0.13),
            ),
            near_target_precision_target_m=max(0.0, float(config.stop_radius) + 0.08),
            near_target_precision_weight_xy=max(
                config.terminal_position_weight * 2.2,
                config.near_target_velocity_weight,
            ),
            near_target_precision_hinge_weight_xy=max(1000.0, config.terminal_position_weight * 25.0),
            velocity_weight_xy=config.velocity_weight,
            velocity_weight_yaw=yaw_config.velocity_weight,
            terminal_velocity_weight_xy=config.terminal_velocity_weight,
            terminal_velocity_weight_yaw=yaw_config.terminal_velocity_weight,
            control_weight_xy=config.control_weight,
            control_weight_yaw=yaw_config.control_weight,
            control_delta_weight_xy=config.control_delta_weight,
            control_delta_weight_z=self.nmpc_flight_config.position_3d_control_delta_weight_z,
            control_delta_weight_yaw=yaw_config.control_delta_weight,
            sign_flip_weight_xy=config.near_fine_sign_flip_weight,
            progress_coast_penalty_weight=self.nmpc_flight_config.position_3d_progress_coast_penalty_weight,
            progress_coast_penalty_distance_m=(
                self.nmpc_flight_config.position_3d_progress_coast_penalty_distance_m
            ),
            progress_coast_penalty_min_u=self.nmpc_flight_config.position_3d_progress_coast_penalty_min_u,
            yaw_progress_coast_penalty_weight=(
                self.nmpc_flight_config.position_3d_yaw_progress_coast_penalty_weight
            ),
            yaw_progress_coast_penalty_error_deg=(
                self.nmpc_flight_config.position_3d_yaw_progress_coast_penalty_error_deg
            ),
            yaw_progress_coast_penalty_min_u=(
                self.nmpc_flight_config.position_3d_yaw_progress_coast_penalty_min_u
            ),
        )

    def compute(
        self,
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
        return compute_flight_controller(
            self,
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
