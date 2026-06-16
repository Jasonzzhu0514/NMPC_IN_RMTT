from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class NmpcFlightControllerConfig:
    min_sample_dt_sec: float = 0.02
    max_sample_gap_sec: float = 0.75
    velocity_smoothing_alpha: float = 0.35
    xy_conservative_model_enabled: bool = True
    xy_tau_scale: float = 1.20
    xy_delay_add_sec: float = 0.08
    xy_amax_scale: float = 0.70
    boundary_z_max_m: float = 2.1
    position_3d_enabled: bool = True
    position_3d_max_abs_xy_u: float = 0.20
    position_3d_max_abs_z_u: float = 0.16
    position_3d_max_abs_yaw_u: float = 0.45
    position_3d_throttle_max_delta_u_per_step: float = 0.04
    position_3d_yaw_max_delta_u_per_step: float = 0.15
    position_3d_xy_levels: int = 5
    position_3d_z_levels: int = 5
    position_3d_yaw_levels: int = 7
    position_3d_yaw_fine_enabled: bool = True
    position_3d_yaw_fine_error_deg: float = 12.0
    position_3d_yaw_fine_levels: int = 17
    position_3d_throttle_fine_enabled: bool = True
    position_3d_throttle_fine_step_u: float = 0.01
    position_3d_yaw_fine_step_u: float = 0.05
    position_3d_z_sequence_enabled: bool = False
    position_3d_yaw_sequence_enabled: bool = False
    position_3d_sequence_split_step: int = 4
    position_3d_throttle_sequence_step_u: float = 0.01
    position_3d_yaw_sequence_step_u: float = 0.05
    position_3d_throttle_sequence_delta_weight: float = 10.0
    position_3d_yaw_sequence_delta_weight: float = 1.0
    position_3d_task_xy_fine_enabled: bool = True
    position_3d_task_xy_fine_radius_m: float = 0.18
    position_3d_task_xy_fine_max_abs_u: float = 0.08
    position_3d_task_xy_fine_step_u: float = 0.02
    position_3d_control_delta_weight_z: float = 8.0
    position_3d_progress_coast_penalty_weight: float = 0.0
    position_3d_progress_coast_penalty_distance_m: float = 0.18
    position_3d_progress_coast_penalty_min_u: float = 0.06
    position_3d_yaw_progress_coast_penalty_weight: float = 0.0
    position_3d_yaw_progress_coast_penalty_error_deg: float = 4.0
    position_3d_yaw_progress_coast_penalty_min_u: float = 0.10


@dataclass(frozen=True)
class AxisControllerConfig:
    dt: float = 0.12
    horizon_steps: int = 16
    max_abs_u: float = 0.24
    stop_radius: float = 0.05
    stop_speed: float = 0.05
    position_weight: float = 13.0
    terminal_position_weight: float = 95.0
    velocity_weight: float = 4.0
    terminal_velocity_weight: float = 48.0
    control_weight: float = 0.12
    control_delta_weight: float = 0.60


@dataclass(frozen=True)
class AxisControllerDebug:
    cost: float
    candidate_count: int
    predicted_terminal_position: float
    predicted_terminal_velocity: float


@dataclass(frozen=True)
class NmpcFlightControlResult:
    enabled: bool
    reason: str
    profile: str
    target_x: float | None
    target_y: float | None
    target_z: float | None
    target_yaw: float | None
    state_x: float | None
    state_y: float | None
    state_z: float | None
    state_yaw: float | None
    state_vx: float | None
    state_vy: float | None
    state_vz: float | None
    state_yaw_rate: float | None
    yaw_error: float | None
    pitch_u: float
    roll_u: float
    throttle_u: float
    yaw_u: float
    pitch_absolute: int
    roll_absolute: int
    throttle_absolute: int
    yaw_absolute: int
    delta_pitch_u: float | None
    delta_roll_u: float | None
    delta_throttle_u: float | None
    delta_yaw_u: float | None
    predicted_terminal_x: float | None
    predicted_terminal_y: float | None
    predicted_terminal_z: float | None
    predicted_terminal_vz: float | None
    predicted_terminal_yaw: float | None
    predicted_terminal_yaw_error: float | None
    predicted_terminal_yaw_rate: float | None
    predicted_terminal_speed: float | None
    solve_time_ms: float | None
    candidate_count: int
    position_3d_candidate_profile: str | None
    position_3d_candidate_level_count: int | None
    position_3d_profile_json: str | None
    cost: float | None
    command_saturated: bool
    boundary_active: bool
    osd_age_sec: float | None = None
    osd_last_msg_monotonic: float | None = None
    osd_control_monotonic_now: float | None = None
    osd_max_age_sec: float | None = None
    osd_frequency_hz: float | None = None
    osd_missing_reason: str | None = None

    @classmethod
    def neutral(
        cls,
        *,
        reason: str,
        neutral: int,
        target_x: float | None = None,
        target_y: float | None = None,
        target_z: float | None = None,
        target_yaw: float | None = None,
        state_x: float | None = None,
        state_y: float | None = None,
        state_z: float | None = None,
        state_yaw: float | None = None,
        state_vx: float | None = None,
        state_vy: float | None = None,
        state_vz: float | None = None,
        state_yaw_rate: float | None = None,
        yaw_error: float | None = None,
        profile: str = "xy",
    ) -> "NmpcFlightControlResult":
        return cls(
            enabled=False,
            reason=reason,
            profile=profile,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_yaw=target_yaw,
            state_x=state_x,
            state_y=state_y,
            state_z=state_z,
            state_yaw=state_yaw,
            state_vx=state_vx,
            state_vy=state_vy,
            state_vz=state_vz,
            state_yaw_rate=state_yaw_rate,
            yaw_error=yaw_error,
            pitch_u=0.0,
            roll_u=0.0,
            throttle_u=0.0,
            yaw_u=0.0,
            pitch_absolute=neutral,
            roll_absolute=neutral,
            throttle_absolute=neutral,
            yaw_absolute=neutral,
            delta_pitch_u=None,
            delta_roll_u=None,
            delta_throttle_u=None,
            delta_yaw_u=None,
            predicted_terminal_x=None,
            predicted_terminal_y=None,
            predicted_terminal_z=None,
            predicted_terminal_vz=None,
            predicted_terminal_yaw=None,
            predicted_terminal_yaw_error=None,
            predicted_terminal_yaw_rate=None,
            predicted_terminal_speed=None,
            solve_time_ms=None,
            candidate_count=0,
            position_3d_candidate_profile=None,
            position_3d_candidate_level_count=None,
            position_3d_profile_json=None,
            cost=None,
            command_saturated=False,
            boundary_active=False,
            osd_age_sec=None,
            osd_last_msg_monotonic=None,
            osd_control_monotonic_now=None,
            osd_max_age_sec=None,
            osd_frequency_hz=None,
            osd_missing_reason=None,
        )

    def as_log_values(self) -> dict[str, object]:
        return {
            "nmpc_flight_enabled": int(self.enabled),
            "nmpc_flight_reason": self.reason,
            "nmpc_flight_profile": self.profile,
            "nmpc_solve_time_ms": self.solve_time_ms,
            "nmpc_candidate_count": self.candidate_count,
            "nmpc_position_3d_candidate_profile": self.position_3d_candidate_profile,
            "nmpc_position_3d_candidate_level_count": self.position_3d_candidate_level_count,
            "nmpc_position_3d_profile": self.position_3d_profile_json,
            "nmpc_cost": self.cost,
            "nmpc_state_x": self.state_x,
            "nmpc_state_y": self.state_y,
            "nmpc_state_z": self.state_z,
            "nmpc_state_yaw": self.state_yaw,
            "nmpc_state_vx": self.state_vx,
            "nmpc_state_vy": self.state_vy,
            "nmpc_state_vz": self.state_vz,
            "nmpc_state_yaw_rate": self.state_yaw_rate,
            "nmpc_target_x": self.target_x,
            "nmpc_target_y": self.target_y,
            "nmpc_target_z": self.target_z,
            "nmpc_target_yaw": self.target_yaw,
            "nmpc_yaw_error": self.yaw_error,
            "nmpc_roll_u": self.roll_u,
            "nmpc_pitch_u": self.pitch_u,
            "nmpc_throttle_u": self.throttle_u,
            "nmpc_yaw_u": self.yaw_u,
            "nmpc_roll_absolute": self.roll_absolute,
            "nmpc_pitch_absolute": self.pitch_absolute,
            "nmpc_throttle_absolute": self.throttle_absolute,
            "nmpc_yaw_absolute": self.yaw_absolute,
            "nmpc_delta_roll_u": self.delta_roll_u,
            "nmpc_delta_pitch_u": self.delta_pitch_u,
            "nmpc_delta_throttle_u": self.delta_throttle_u,
            "nmpc_delta_yaw_u": self.delta_yaw_u,
            "nmpc_predicted_terminal_x": self.predicted_terminal_x,
            "nmpc_predicted_terminal_y": self.predicted_terminal_y,
            "nmpc_predicted_terminal_z": self.predicted_terminal_z,
            "nmpc_predicted_terminal_vz": self.predicted_terminal_vz,
            "nmpc_predicted_terminal_yaw": self.predicted_terminal_yaw,
            "nmpc_predicted_terminal_yaw_error": self.predicted_terminal_yaw_error,
            "nmpc_predicted_terminal_yaw_rate": self.predicted_terminal_yaw_rate,
            "nmpc_predicted_terminal_speed": self.predicted_terminal_speed,
            "nmpc_command_saturated": int(self.command_saturated),
            "nmpc_boundary_active": int(self.boundary_active),
            "nmpc_osd_age_sec": self.osd_age_sec,
            "nmpc_osd_last_msg_monotonic": self.osd_last_msg_monotonic,
            "nmpc_osd_control_monotonic_now": self.osd_control_monotonic_now,
            "nmpc_osd_max_age_sec": self.osd_max_age_sec,
            "nmpc_osd_frequency_hz": self.osd_frequency_hz,
            "nmpc_osd_missing_reason": self.osd_missing_reason,
        }


@dataclass(frozen=True)
class Sample:
    timestamp: float
    x: float
    y: float
    z: float | None = None
    yaw: float | None = None
