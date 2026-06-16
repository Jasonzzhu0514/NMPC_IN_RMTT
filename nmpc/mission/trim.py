from __future__ import annotations

import math

from nmpc.flight_controller import NmpcFlightControlResult
from nmpc.mission.math_utils import (
    clamp as _clamp,
    finite_float as _finite_float,
    sign as _sign,
)


class NmpcMissionTrimMixin:
    def _body_compensated_xy(
        self,
        nmpc_flight: NmpcFlightControlResult,
        current_yaw: float | None,
        *,
        trim_world_roll_u: float = 0.0,
        trim_world_pitch_u: float = 0.0,
    ) -> tuple[float, float, float, float] | None:
        yaw = _finite_float(current_yaw)
        if yaw is None:
            return None
        pitch_model = self.nmpc_flight.model.axes.get("pitch")
        roll_model = self.nmpc_flight.model.axes.get("roll")
        if pitch_model is None or roll_model is None or pitch_model.K == 0.0 or roll_model.K == 0.0:
            return None
        trim_world_vx = pitch_model.K * trim_world_pitch_u
        trim_world_vy = roll_model.K * trim_world_roll_u
        world_vx = pitch_model.K * nmpc_flight.pitch_u + trim_world_vx
        world_vy = roll_model.K * nmpc_flight.roll_u + trim_world_vy
        yaw_rad = math.radians(yaw)
        body_forward_v = math.cos(yaw_rad) * world_vx + math.sin(yaw_rad) * world_vy
        body_left_v = -math.sin(yaw_rad) * world_vx + math.cos(yaw_rad) * world_vy
        trim_body_forward_v = math.cos(yaw_rad) * trim_world_vx + math.sin(yaw_rad) * trim_world_vy
        trim_body_left_v = -math.sin(yaw_rad) * trim_world_vx + math.cos(yaw_rad) * trim_world_vy
        pitch_u = body_forward_v / pitch_model.K
        roll_u = body_left_v / roll_model.K
        trim_body_pitch_u = trim_body_forward_v / pitch_model.K
        trim_body_roll_u = trim_body_left_v / roll_model.K
        return roll_u, pitch_u, trim_body_roll_u, trim_body_pitch_u

    def _update_trim(
        self,
        *,
        timestamp: float,
        current_x: float | None,
        current_y: float | None,
        target_x: float | None,
        target_y: float | None,
        current_yaw: float | None,
    ) -> dict[str, float | bool | None]:
        x = _finite_float(current_x)
        y = _finite_float(current_y)
        tx = _finite_float(target_x)
        ty = _finite_float(target_y)
        yaw = _finite_float(current_yaw)
        timestamp_f = _finite_float(timestamp)
        if None in (x, y, tx, ty, yaw, timestamp_f) or not self.config.trim_enabled:
            self._reset_trim()
            return self._trim_values(enabled=False, distance=None)

        target_key = (round(tx, 4), round(ty, 4))
        if self._trim_target_key != target_key:
            self._reset_trim()
            self._trim_target_key = target_key

        distance = math.hypot(tx - x, ty - y)
        if distance <= max(0.0, self.config.trim_clear_distance_m):
            self._reset_trim(target_key=target_key)
            return self._trim_values(enabled=False, distance=distance)
        if distance > max(0.0, self.config.trim_start_distance_m):
            self._leak_trim()
            self._trim_last_timestamp = timestamp_f
            return self._trim_values(enabled=False, distance=distance)

        dt = 0.0 if self._trim_last_timestamp is None else timestamp_f - self._trim_last_timestamp
        self._trim_last_timestamp = timestamp_f
        if not math.isfinite(dt) or dt <= 0.0 or dt > 0.5:
            self._leak_trim()
            return self._trim_values(enabled=True, distance=distance)

        self._leak_trim()
        pitch_model = self.nmpc_flight.model.axes.get("pitch")
        roll_model = self.nmpc_flight.model.axes.get("roll")
        if pitch_model is None or roll_model is None or pitch_model.K == 0.0 or roll_model.K == 0.0:
            self._reset_trim(target_key=target_key)
            return self._trim_values(enabled=False, distance=distance)
        gain = max(0.0, float(self.config.trim_gain_u_per_m_s))
        max_abs = max(0.0, float(self.config.trim_max_abs_u))
        self._trim_world_pitch_u = _clamp(
            self._trim_world_pitch_u + (tx - x) * gain * dt,
            -max_abs,
            max_abs,
        )
        self._trim_world_roll_u = _clamp(
            self._trim_world_roll_u + (ty - y) * gain * dt * _sign(roll_model.K),
            -max_abs,
            max_abs,
        )
        return self._trim_values(enabled=True, distance=distance)

    def _trim_values(self, *, enabled: bool, distance: float | None) -> dict[str, float | bool | None]:
        return {
            "enabled": enabled,
            "distance": distance,
            "world_pitch_u": self._trim_world_pitch_u,
            "world_roll_u": self._trim_world_roll_u,
        }

    def _reset_trim(self, *, target_key: tuple[float, float] | None = None) -> None:
        self._trim_world_pitch_u = 0.0
        self._trim_world_roll_u = 0.0
        self._trim_target_key = target_key
        self._trim_last_timestamp = None

    def _leak_trim(self) -> None:
        leak = _clamp(float(self.config.trim_leak), 0.0, 1.0)
        self._trim_world_pitch_u *= leak
        self._trim_world_roll_u *= leak
