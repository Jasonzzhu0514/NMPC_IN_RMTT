"""Small-authority NMPC active gate for XYZ missions."""

from __future__ import annotations

from dataclasses import replace
import math
import time
from typing import Any, Callable

from nmpc.flight_controller import (
    NmpcFlightController, NmpcFlightControlResult, normalized_to_absolute_stick,
)
from nmpc.mission.math_utils import finite_float
from nmpc.mission.segment_runtime import reset_segment_guide
from nmpc.mission.runtime import compute_mission_controller
from nmpc.mission.strategies import NmpcMissionStrategies
from nmpc.mission.types import (
    ACTIVE_AXES, NmpcMissionControlDecision, NmpcMissionControllerConfig,
)
from nmpc.position_3d_types import Position3DProfile


_finite_float = finite_float


def normalized_stick(
    value: int | float | None,
    *,
    neutral: int,
    stick_min: int,
    stick_max: int,
) -> float | None:
    if value is None:
        return None
    span = stick_max - neutral if value >= neutral else neutral - stick_min
    if span <= 0:
        return None
    return max(-1.0, min(1.0, (float(value) - neutral) / span))


class NmpcMissionController(NmpcMissionStrategies):
    """Gate NMPC controller recommendations into conservative MOVE-XYZ stick commands."""

    def __init__(
        self,
        *,
        neutral: int,
        stick_min: int,
        stick_max: int,
        config: NmpcMissionControllerConfig | None = None,
        nmpc_flight: NmpcFlightController | None = None,
        monotonic_provider: Callable[[], float] = time.monotonic,
    ) -> None:
        self.neutral = int(neutral)
        self.stick_min = int(stick_min)
        self.stick_max = int(stick_max)
        self.config = config or NmpcMissionControllerConfig()
        self.nmpc_flight = nmpc_flight or NmpcFlightController(
            neutral=self.neutral,
            stick_min=self.stick_min,
            stick_max=self.stick_max,
            controller_config=Position3DProfile(
                max_abs_u=max(0.0, min(1.0, float(self.config.max_abs_u))),
                boundary_weight=max(0.0, float(self.config.boundary_weight)),
            ),
        )
        self._monotonic_provider = monotonic_provider
        self._trim_world_pitch_u = 0.0
        self._trim_world_roll_u = 0.0
        self._trim_target_key: tuple[float, float] | None = None
        self._trim_last_timestamp: float | None = None
        self._near_min_target_key: tuple[float, float] | None = None
        self._near_min_last_timestamp: float | None = None
        self._near_min_last_distance: float | None = None
        self._near_min_floor_u = max(0.0, float(self.config.near_min_effective_u))
        self._near_min_stall_sec = 0.0
        self._near_min_progress_mps: float | None = None
        self._near_min_adaptive_bumps = 0
        self._heading_gate_active = False
        self._last_enabled_decision: NmpcMissionControlDecision | None = None
        self._last_enabled_timestamp: float | None = None
        self._last_enabled_target_key: tuple[float | None, float | None, float | None, float | None] | None = None
        self._last_limited_throttle_u = 0.0
        self._last_limited_yaw_u = 0.0
        self._segment_start = None
        self._segment_goal_key = None
        self._last_segment_reference = None

    def reset(self) -> None:
        self.nmpc_flight.reset()
        self._reset_trim()
        self._reset_near_min_effective()
        self._heading_gate_active = False
        self._reset_last_enabled_decision()
        self._last_limited_throttle_u = 0.0
        self._last_limited_yaw_u = 0.0
        reset_segment_guide(self)

    def update_position_3d_profile(self, config: Position3DProfile) -> None:
        max_abs_u = max(0.0, min(1.0, float(config.max_abs_u)))
        self.config = replace(self.config, max_abs_u=max_abs_u)
        self.nmpc_flight.update_position_3d_profile(config)
        self.reset()

    def compute_for_runtime(self, runtime: Any, *, osd_source: Any | None = None) -> NmpcMissionControlDecision:
        target_z = runtime.ctx.current_target_z
        if getattr(runtime.state, "phase", "") == "move_xyz":
            target_z = runtime.ctx.move_target_z
        target_z = runtime.nmpc_target_z if runtime.nmpc_target_z is not None else target_z
        return self.compute(
            timestamp=runtime.current_time,
            current_x=runtime.current_x,
            current_y=runtime.current_y,
            current_z=runtime.current_z,
            current_yaw=runtime.current_yaw,
            target_x=runtime.target_x,
            target_y=runtime.target_y,
            target_z=target_z,
            target_yaw=runtime.yaw_target,
            phase="MOVE-XYZ",
            plane_state=getattr(runtime.state, "plane_state", None),
            pid_roll=runtime.roll,
            pid_pitch=runtime.pitch,
            pid_throttle=runtime.throttle_command,
            pid_yaw=runtime.yaw,
            osd_source=osd_source,
            preserve_target_yaw=runtime.nmpc_preserve_target_yaw,
            final_target=runtime.nmpc_final_target,
        )

    def compute(
        self,
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
        return compute_mission_controller(
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
            phase=phase,
            pid_roll=pid_roll,
            pid_pitch=pid_pitch,
            pid_throttle=pid_throttle,
            pid_yaw=pid_yaw,
            plane_state=plane_state,
            osd_source=osd_source,
            preserve_target_yaw=preserve_target_yaw,
            final_target=final_target,
        )

    def _held_sample_dt_decision(
        self,
        *,
        timestamp: float,
        target_x: float | None,
        target_y: float | None,
        target_z: float | None,
        target_yaw: float | None,
        nmpc_flight: NmpcFlightControlResult,
        pid_roll: int,
        pid_pitch: int,
        pid_throttle: int,
        pid_yaw: int,
        pid_roll_u: float | None,
        pid_pitch_u: float | None,
        pid_throttle_u: float | None,
        pid_yaw_u: float | None,
    ) -> NmpcMissionControlDecision | None:
        if (
            not self.config.sample_dt_hold_enabled
            or nmpc_flight.reason != "sample_dt_too_small"
            or self._last_enabled_decision is None
            or self._last_enabled_timestamp is None
        ):
            return None
        timestamp_f = _finite_float(timestamp)
        if timestamp_f is None:
            return None
        age = timestamp_f - self._last_enabled_timestamp
        max_age = max(0.0, float(self.config.sample_dt_hold_max_age_sec))
        if not math.isfinite(age) or age < 0.0 or age > max_age:
            return None
        if self._last_enabled_target_key != self._target_key(target_x, target_y, target_z, target_yaw):
            return None
        held = replace(
            self._last_enabled_decision,
            reason="sample_dt_hold",
            nmpc_flight=nmpc_flight,
            pid_roll=pid_roll,
            pid_pitch=pid_pitch,
            pid_throttle=pid_throttle,
            pid_yaw=pid_yaw,
            pid_roll_u=pid_roll_u,
            pid_pitch_u=pid_pitch_u,
            pid_throttle_u=pid_throttle_u,
            pid_yaw_u=pid_yaw_u,
            heading_error_deg=_finite_float(nmpc_flight.yaw_error),
        )
        return held

    def _remember_enabled_decision(
        self,
        decision: NmpcMissionControlDecision,
        *,
        timestamp: float,
        target_x: float | None,
        target_y: float | None,
        target_z: float | None,
        target_yaw: float | None,
    ) -> None:
        timestamp_f = _finite_float(timestamp)
        if timestamp_f is None or not decision.enabled:
            return
        self._last_enabled_decision = decision
        self._last_enabled_timestamp = timestamp_f
        self._last_enabled_target_key = self._target_key(target_x, target_y, target_z, target_yaw)

    def _reset_last_enabled_decision(self) -> None:
        self._last_enabled_decision = None
        self._last_enabled_timestamp = None
        self._last_enabled_target_key = None

    def _target_key(
        self,
        target_x: float | None,
        target_y: float | None,
        target_z: float | None,
        target_yaw: float | None,
    ) -> tuple[float | None, float | None, float | None, float | None]:
        return tuple(
            None if value is None else round(value, 4)
            for value in (
                _finite_float(target_x),
                _finite_float(target_y),
                _finite_float(target_z),
                _finite_float(target_yaw),
            )
        )

    def _fallback_decision(
        self,
        *,
        reason: str,
        nmpc_flight: NmpcFlightControlResult,
        pid_roll: int,
        pid_pitch: int,
        pid_throttle: int,
        pid_yaw: int,
        pid_roll_u: float | None,
        pid_pitch_u: float | None,
        pid_throttle_u: float | None,
        pid_yaw_u: float | None,
    ) -> NmpcMissionControlDecision:
        neutral_u = self._normalized(self.neutral)
        return NmpcMissionControlDecision(
            enabled=False,
            reason=reason,
            nmpc_flight=nmpc_flight,
            authority_u=max(0.0, min(1.0, float(self.config.max_abs_u))),
            yaw_authority_u=max(0.0, min(1.0, float(self.config.yaw_max_abs_u))),
            axes=ACTIVE_AXES,
            roll=self.neutral,
            pitch=self.neutral,
            throttle=self.neutral,
            yaw=self.neutral,
            roll_u=neutral_u,
            pitch_u=neutral_u,
            throttle_u=neutral_u,
            yaw_u=neutral_u,
            heading_gate_active=False,
            heading_error_deg=_finite_float(nmpc_flight.yaw_error),
            heading_gate_xy_scale=1.0,
            pid_roll=pid_roll,
            pid_pitch=pid_pitch,
            pid_throttle=pid_throttle,
            pid_yaw=pid_yaw,
            pid_roll_u=pid_roll_u,
            pid_pitch_u=pid_pitch_u,
            pid_throttle_u=pid_throttle_u,
            pid_yaw_u=pid_yaw_u,
            trim_enabled=False,
            trim_distance=None,
        )

    def _neutral_core(self, **kwargs: Any) -> NmpcFlightControlResult:
        return NmpcFlightControlResult.neutral(neutral=self.neutral, **kwargs)

    def _normalized(self, value: int | float | None) -> float | None:
        return normalized_stick(
            value,
            neutral=self.neutral,
            stick_min=self.stick_min,
            stick_max=self.stick_max,
        )

    def _absolute(self, value: float) -> int:
        return normalized_to_absolute_stick(
            value, neutral=self.neutral, stick_min=self.stick_min, stick_max=self.stick_max,
        )
