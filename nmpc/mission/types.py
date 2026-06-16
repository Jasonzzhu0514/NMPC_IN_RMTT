from __future__ import annotations

from dataclasses import dataclass, field

from nmpc.flight_controller import NmpcFlightControlResult


ACTIVE_AXES = "roll,pitch,throttle,yaw"


@dataclass(frozen=True)
class PidValues:
    roll: int
    pitch: int
    throttle: int
    yaw: int
    roll_u: float | None
    pitch_u: float | None
    throttle_u: float | None
    yaw_u: float | None


@dataclass(frozen=True)
class NmpcMissionControllerConfig:
    max_abs_u: float = 0.08
    yaw_max_abs_u: float = 0.50
    require_osd: bool = False
    max_osd_age_sec: float = 0.5
    fallback_on_boundary: bool = True
    boundary_weight: float = 800.0
    capture_distance_m: float = 0.0
    heading_gate_enabled: bool = False
    heading_gate_enter_deg: float = 45.0
    heading_gate_clear_deg: float = 30.0
    heading_gate_xy_scale: float = 0.0
    heading_gate_min_distance_m: float = 0.20
    segment_guide_enabled: bool = False
    segment_guide_lookahead_m: float = 0.18
    segment_guide_terminal_distance_m: float = 0.18
    segment_guide_goal_snap_distance_m: float = 0.18
    segment_guide_yaw_freeze_distance_m: float = 0.20
    throttle_rate_limit_enabled: bool = False
    throttle_rate_limit_u_per_step: float = 0.04
    yaw_rate_limit_enabled: bool = False
    yaw_rate_limit_u_per_step: float = 0.15
    final_terminal_xy_cap_enabled: bool = True
    final_terminal_xy_max_abs_u: float = 0.10
    sample_dt_hold_enabled: bool = True
    sample_dt_hold_max_age_sec: float = 0.20
    arrival_brake_enabled: bool = False
    arrival_brake_start_m: float = 0.35
    arrival_brake_clear_m: float = 0.16
    arrival_brake_lookahead_sec: float = 0.90
    arrival_brake_min_scale: float = 0.0
    trim_enabled: bool = False
    trim_start_distance_m: float = 0.35
    trim_clear_distance_m: float = 0.16
    trim_leak: float = 0.995
    trim_gain_u_per_m_s: float = 0.05
    trim_max_abs_u: float = 0.06
    trim_total_max_abs_u: float = 0.14
    near_min_effective_enabled: bool = False
    near_min_effective_start_m: float = 0.30
    near_min_effective_clear_m: float = 0.16
    near_min_effective_u: float = 0.13
    near_min_effective_adaptive_enabled: bool = False
    near_min_effective_adaptive_max_u: float = 0.18
    near_min_effective_adaptive_step_u: float = 0.01
    near_min_effective_adaptive_stall_sec: float = 0.8
    near_min_effective_progress_mps: float = 0.015
    z_near_min_effective_enabled: bool = False
    z_near_min_effective_start_m: float = 0.14
    z_near_min_effective_clear_m: float = 0.10
    z_near_min_effective_u: float = 0.10


@dataclass(frozen=True)
class NmpcMissionControlDecision:
    enabled: bool
    reason: str
    nmpc_flight: NmpcFlightControlResult
    authority_u: float
    axes: str
    roll: int
    pitch: int
    throttle: int
    yaw: int
    roll_u: float | None
    pitch_u: float | None
    throttle_u: float | None
    yaw_u: float | None
    yaw_authority_u: float
    pid_roll: int
    pid_pitch: int
    pid_throttle: int
    pid_yaw: int
    pid_roll_u: float | None
    pid_pitch_u: float | None
    pid_throttle_u: float | None
    pid_yaw_u: float | None
    heading_gate_active: bool = False
    heading_error_deg: float | None = None
    heading_gate_xy_scale: float = 1.0
    arrival_brake_active: bool = False
    arrival_brake_scale: float = 1.0
    arrival_brake_time_to_target_sec: float | None = None
    arrival_brake_predicted_crossing: bool = False
    trim_enabled: bool = False
    trim_distance: float | None = None
    trim_world_pitch_u: float = 0.0
    trim_world_roll_u: float = 0.0
    trim_body_pitch_u: float = 0.0
    trim_body_roll_u: float = 0.0
    near_min_effective_enabled: bool = False
    near_min_effective_distance: float | None = None
    near_min_effective_before_pitch_u: float | None = None
    near_min_effective_before_roll_u: float | None = None
    near_min_effective_after_pitch_u: float | None = None
    near_min_effective_after_roll_u: float | None = None
    near_min_effective_floor_u: float | None = None
    near_min_effective_stall_sec: float | None = None
    near_min_effective_progress_mps: float | None = None
    near_min_effective_adaptive_bumps: int = 0
    z_near_min_effective_enabled: bool = False
    z_near_min_effective_distance: float | None = None
    z_near_min_effective_before_throttle_u: float | None = None
    z_near_min_effective_after_throttle_u: float | None = None
    throttle_rate_limit_enabled: bool = False
    throttle_rate_limit_before_u: float | None = None
    throttle_rate_limit_after_u: float | None = None
    throttle_rate_limit_max_delta_u: float | None = None
    yaw_rate_limit_enabled: bool = False
    yaw_rate_limit_before_u: float | None = None
    yaw_rate_limit_after_u: float | None = None
    yaw_rate_limit_max_delta_u: float | None = None
    segment_values: dict[str, object] = field(default_factory=dict)

    def as_log_values(self) -> dict[str, object]:
        values = self.nmpc_flight.as_log_values()
        values.update(
            {
                "nmpc_mission_enabled": int(self.enabled),
                "nmpc_mission_reason": self.reason,
                "nmpc_mission_authority_u": self.authority_u,
                "nmpc_mission_yaw_authority_u": self.yaw_authority_u,
                "nmpc_mission_axes": self.axes,
                "nmpc_mission_roll_u": self.roll_u,
                "nmpc_mission_pitch_u": self.pitch_u,
                "nmpc_mission_throttle_u": self.throttle_u,
                "nmpc_mission_yaw_u": self.yaw_u,
                "nmpc_mission_heading_gate_active": int(self.heading_gate_active),
                "nmpc_mission_heading_error_deg": self.heading_error_deg,
                "nmpc_mission_heading_gate_xy_scale": self.heading_gate_xy_scale,
                "nmpc_mission_arrival_brake_active": int(self.arrival_brake_active),
                "nmpc_mission_arrival_brake_scale": self.arrival_brake_scale,
                "nmpc_mission_arrival_brake_time_to_target_sec": self.arrival_brake_time_to_target_sec,
                "nmpc_mission_arrival_brake_predicted_crossing": int(self.arrival_brake_predicted_crossing),
                "nmpc_mission_roll_absolute": self.roll,
                "nmpc_mission_pitch_absolute": self.pitch,
                "nmpc_mission_throttle_absolute": self.throttle,
                "nmpc_mission_yaw_absolute": self.yaw,
                "nmpc_mission_trim_enabled": int(self.trim_enabled),
                "nmpc_mission_trim_distance": self.trim_distance,
                "nmpc_mission_trim_world_pitch_u": self.trim_world_pitch_u,
                "nmpc_mission_trim_world_roll_u": self.trim_world_roll_u,
                "nmpc_mission_trim_body_pitch_u": self.trim_body_pitch_u,
                "nmpc_mission_trim_body_roll_u": self.trim_body_roll_u,
                "nmpc_mission_near_min_effective_enabled": int(self.near_min_effective_enabled),
                "nmpc_mission_near_min_effective_distance": self.near_min_effective_distance,
                "nmpc_mission_near_min_effective_before_pitch_u": self.near_min_effective_before_pitch_u,
                "nmpc_mission_near_min_effective_before_roll_u": self.near_min_effective_before_roll_u,
                "nmpc_mission_near_min_effective_after_pitch_u": self.near_min_effective_after_pitch_u,
                "nmpc_mission_near_min_effective_after_roll_u": self.near_min_effective_after_roll_u,
                "nmpc_mission_near_min_effective_floor_u": self.near_min_effective_floor_u,
                "nmpc_mission_near_min_effective_stall_sec": self.near_min_effective_stall_sec,
                "nmpc_mission_near_min_effective_progress_mps": self.near_min_effective_progress_mps,
                "nmpc_mission_near_min_effective_adaptive_bumps": self.near_min_effective_adaptive_bumps,
                "nmpc_mission_z_near_min_effective_enabled": int(self.z_near_min_effective_enabled),
                "nmpc_mission_z_near_min_effective_distance": self.z_near_min_effective_distance,
                "nmpc_mission_z_near_min_effective_before_throttle_u": self.z_near_min_effective_before_throttle_u,
                "nmpc_mission_z_near_min_effective_after_throttle_u": self.z_near_min_effective_after_throttle_u,
                "nmpc_mission_throttle_rate_limit_enabled": int(self.throttle_rate_limit_enabled),
                "nmpc_mission_throttle_rate_limit_before_u": self.throttle_rate_limit_before_u,
                "nmpc_mission_throttle_rate_limit_after_u": self.throttle_rate_limit_after_u,
                "nmpc_mission_throttle_rate_limit_max_delta_u": self.throttle_rate_limit_max_delta_u,
                "nmpc_mission_yaw_rate_limit_enabled": int(self.yaw_rate_limit_enabled),
                "nmpc_mission_yaw_rate_limit_before_u": self.yaw_rate_limit_before_u,
                "nmpc_mission_yaw_rate_limit_after_u": self.yaw_rate_limit_after_u,
                "nmpc_mission_yaw_rate_limit_max_delta_u": self.yaw_rate_limit_max_delta_u,
            }
        )
        values.update(self.segment_values)
        return values
