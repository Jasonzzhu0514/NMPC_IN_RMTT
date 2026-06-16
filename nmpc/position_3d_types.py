from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Position3DProfile:
    dt: float = 0.12
    horizon_steps: int = 16
    first_segment_steps: int = 4
    max_abs_u: float = 0.24
    field_limit: float = 1.5
    safety_margin: float = 0.15
    stop_radius: float = 0.05
    stop_speed: float = 0.05
    position_weight: float = 16.0
    terminal_position_weight: float = 120.0
    velocity_weight: float = 10.0
    terminal_velocity_weight: float = 140.0
    near_target_radius: float = 0.24
    near_target_velocity_weight: float = 130.0
    near_fine_candidate_enabled: bool = True
    near_fine_candidate_radius: float = 0.35
    near_fine_candidate_step_u: float = 0.02
    near_fine_candidate_max_abs_u: float = 0.08
    closing_velocity_weight: float = 14.0
    overshoot_weight: float = 220.0
    overshoot_velocity_weight: float = 90.0
    control_weight: float = 0.20
    control_delta_weight: float = 1.4
    near_fine_control_delta_weight: float = 3.2
    near_fine_sign_flip_weight: float = 0.08
    near_fine_coast_penalty_weight: float = 0.0
    near_fine_coast_penalty_distance_m: float = 0.0
    near_fine_coast_penalty_min_u: float = 0.0
    tail_control_delta_weight: float = 0.12
    boundary_weight: float = 800.0


@dataclass(frozen=True)
class Position3DTarget:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class Position3DConfig:
    dt: float = 0.12
    horizon_steps: int = 14
    max_abs_xy_u: float = 0.18
    max_abs_z_u: float = 0.12
    max_abs_yaw_u: float = 0.55
    throttle_max_delta_u_per_step: float = 0.04
    yaw_max_delta_u_per_step: float = 0.15
    xy_levels: int = 5
    z_levels: int = 5
    yaw_levels: int = 7
    yaw_fine_enabled: bool = True
    yaw_fine_error_deg: float = 12.0
    yaw_fine_levels: int = 17
    throttle_fine_enabled: bool = True
    throttle_fine_step_u: float = 0.01
    yaw_fine_step_u: float = 0.05
    z_sequence_enabled: bool = False
    yaw_sequence_enabled: bool = True
    sequence_split_step: int = 4
    throttle_sequence_step_u: float = 0.01
    yaw_sequence_step_u: float = 0.05
    throttle_sequence_delta_weight: float = 10.0
    yaw_sequence_delta_weight: float = 1.0
    xy_fine_enabled: bool = False
    xy_fine_radius_m: float = 0.18
    xy_fine_max_abs_u: float = 0.08
    xy_fine_step_u: float = 0.02
    stop_radius: float = 0.06
    stop_speed: float = 0.05
    yaw_stop_radius: float = 4.0
    yaw_stop_speed: float = 4.0
    position_weight_xy: float = 16.0
    position_weight_z: float = 14.0
    position_weight_yaw: float = 9.0
    terminal_position_weight_xy: float = 120.0
    terminal_position_weight_z: float = 95.0
    terminal_position_weight_yaw: float = 45.0
    near_target_precision_radius_m: float = 0.22
    near_target_precision_target_m: float = 0.13
    near_target_precision_weight_xy: float = 260.0
    near_target_precision_hinge_weight_xy: float = 3000.0
    velocity_weight_xy: float = 10.0
    velocity_weight_z: float = 8.0
    velocity_weight_yaw: float = 1.2
    terminal_velocity_weight_xy: float = 170.0
    terminal_velocity_weight_z: float = 120.0
    terminal_velocity_weight_yaw: float = 9.0
    control_weight_xy: float = 0.18
    control_weight_z: float = 0.14
    control_weight_yaw: float = 0.06
    control_delta_weight_xy: float = 3.2
    control_delta_weight_z: float = 3.0
    control_delta_weight_yaw: float = 0.18
    sign_flip_weight_xy: float = 1.0
    sign_flip_weight_z: float = 1.0
    sign_flip_weight_yaw: float = 0.0
    progress_coast_penalty_weight: float = 18.0
    progress_coast_penalty_distance_m: float = 0.18
    progress_coast_penalty_min_u: float = 0.06
    yaw_progress_coast_penalty_weight: float = 7000.0
    yaw_progress_coast_penalty_error_deg: float = 4.0
    yaw_progress_coast_penalty_min_u: float = 0.10


@dataclass(frozen=True)
class Position3DCommand:
    pitch: float
    roll: float
    throttle: float
    yaw: float


@dataclass(frozen=True)
class Position3DDebug:
    cost: float
    candidate_count: int
    candidate_profile: str
    candidate_level_count: int
    predicted_terminal_x: float
    predicted_terminal_y: float
    predicted_terminal_z: float
    predicted_terminal_vx: float
    predicted_terminal_vy: float
    predicted_terminal_vz: float
    predicted_terminal_yaw: float
    predicted_terminal_yaw_error: float
    predicted_terminal_yaw_rate: float
    predicted_terminal_speed: float
