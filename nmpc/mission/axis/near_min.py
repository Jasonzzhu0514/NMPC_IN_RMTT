from __future__ import annotations

import math

from nmpc.mission.math_utils import (
    finite_float as _finite_float,
)
from nmpc.mission.axis.z_near_min import NmpcMissionZNearMinMixin


class NmpcMissionNearMinMixin(NmpcMissionZNearMinMixin):
    def _near_min_effective_xy(
        self,
        *,
        timestamp: float,
        current_x: float | None,
        current_y: float | None,
        current_yaw: float | None,
        target_x: float | None,
        target_y: float | None,
        roll_u: float,
        pitch_u: float,
        distance: float | None,
        total_authority: float,
    ) -> dict[str, float | bool | None]:
        result = self._near_min_effective_inactive_values(
            roll_u=roll_u,
            pitch_u=pitch_u,
            distance=distance,
        )
        distance_f = _finite_float(distance)
        if distance_f is None or not self.config.near_min_effective_enabled:
            self._reset_near_min_effective()
            return result
        clear = max(0.0, float(self.config.near_min_effective_clear_m))
        start = max(clear, float(self.config.near_min_effective_start_m))
        if distance_f <= clear or distance_f > start:
            self._reset_near_min_effective()
            return result

        adaptive = self._update_near_min_effective_adaptive(
            timestamp=timestamp,
            target_x=target_x,
            target_y=target_y,
            distance=distance_f,
        )
        min_abs = adaptive["floor_u"]
        max_abs = max(0.0, min(1.0, float(total_authority)))
        target_abs = min(min_abs, max_abs)
        result.update(
            {
                "floor_u": target_abs,
                "stall_sec": adaptive["stall_sec"],
                "progress_mps": adaptive["progress_mps"],
                "adaptive_bumps": adaptive["adaptive_bumps"],
            }
        )
        if target_abs <= 0.0:
            return result
        progress_mps = _finite_float(adaptive["progress_mps"])
        progress_threshold = max(0.0, float(self.config.near_min_effective_progress_mps))
        if progress_mps is not None and progress_mps >= progress_threshold:
            return result
        roll_u, pitch_u, changed = self._raise_xy_vector_min_abs(
            roll_u=roll_u,
            pitch_u=pitch_u,
            target_abs=target_abs,
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            target_x=target_x,
            target_y=target_y,
        )
        if not changed:
            return result
        result.update(
            {
                "enabled": True,
                "roll_u": roll_u,
                "pitch_u": pitch_u,
                "after_pitch_u": pitch_u,
                "after_roll_u": roll_u,
            }
        )
        return result

    def _near_min_effective_inactive_values(
        self,
        *,
        roll_u: float,
        pitch_u: float,
        distance: float | None,
    ) -> dict[str, float | bool | None]:
        return {
            "enabled": False,
            "distance": distance,
            "roll_u": roll_u,
            "pitch_u": pitch_u,
            "before_pitch_u": pitch_u,
            "before_roll_u": roll_u,
            "after_pitch_u": pitch_u,
            "after_roll_u": roll_u,
            "floor_u": None,
            "stall_sec": None,
            "progress_mps": None,
            "adaptive_bumps": 0,
        }

    def _raise_xy_vector_min_abs(
        self,
        *,
        roll_u: float,
        pitch_u: float,
        target_abs: float,
        current_x: float | None,
        current_y: float | None,
        current_yaw: float | None,
        target_x: float | None,
        target_y: float | None,
    ) -> tuple[float, float, bool]:
        norm = math.hypot(roll_u, pitch_u)
        if norm >= target_abs:
            return roll_u, pitch_u, False
        if norm > 1e-9:
            scale = target_abs / norm
            return roll_u * scale, pitch_u * scale, True

        target_direction = self._target_direction_body_xy(
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            target_x=target_x,
            target_y=target_y,
        )
        if target_direction is None:
            return roll_u, pitch_u, False
        target_roll_u, target_pitch_u = target_direction
        target_norm = math.hypot(target_roll_u, target_pitch_u)
        if target_norm <= 1e-9:
            return roll_u, pitch_u, False
        scale = target_abs / target_norm
        return target_roll_u * scale, target_pitch_u * scale, True

    def _target_direction_body_xy(
        self,
        *,
        current_x: float | None,
        current_y: float | None,
        current_yaw: float | None,
        target_x: float | None,
        target_y: float | None,
    ) -> tuple[float, float] | None:
        x = _finite_float(current_x)
        y = _finite_float(current_y)
        tx = _finite_float(target_x)
        ty = _finite_float(target_y)
        yaw = _finite_float(current_yaw)
        if None in (x, y, tx, ty, yaw):
            return None
        dx = tx - x
        dy = ty - y
        distance = math.hypot(dx, dy)
        if distance <= 1e-9:
            return None
        pitch_model = self.nmpc_flight.model.axes.get("pitch")
        roll_model = self.nmpc_flight.model.axes.get("roll")
        if pitch_model is None or roll_model is None or pitch_model.K == 0.0 or roll_model.K == 0.0:
            return None
        world_vx = dx / distance
        world_vy = dy / distance
        yaw_rad = math.radians(yaw)
        body_forward_v = math.cos(yaw_rad) * world_vx + math.sin(yaw_rad) * world_vy
        body_left_v = -math.sin(yaw_rad) * world_vx + math.cos(yaw_rad) * world_vy
        pitch_u = body_forward_v / pitch_model.K
        roll_u = body_left_v / roll_model.K
        return roll_u, pitch_u

    def _update_near_min_effective_adaptive(
        self,
        *,
        timestamp: float,
        target_x: float | None,
        target_y: float | None,
        distance: float,
    ) -> dict[str, float | int | None]:
        base_u = max(0.0, float(self.config.near_min_effective_u))
        timestamp_f = _finite_float(timestamp)
        tx = _finite_float(target_x)
        ty = _finite_float(target_y)
        if timestamp_f is None or tx is None or ty is None:
            self._reset_near_min_effective()
            return self._near_min_effective_adaptive_values(base_u)

        target_key = (round(tx, 4), round(ty, 4))
        if self._near_min_target_key != target_key:
            self._reset_near_min_effective()
            self._near_min_target_key = target_key

        if not self.config.near_min_effective_adaptive_enabled:
            self._near_min_floor_u = base_u
            self._near_min_last_timestamp = timestamp_f
            self._near_min_last_distance = distance
            self._near_min_stall_sec = 0.0
            self._near_min_progress_mps = None
            self._near_min_adaptive_bumps = 0
            return self._near_min_effective_adaptive_values(base_u)

        self._near_min_floor_u = max(self._near_min_floor_u, base_u)
        dt = (
            0.0
            if self._near_min_last_timestamp is None
            else max(0.0, timestamp_f - self._near_min_last_timestamp)
        )
        progress = (
            None
            if self._near_min_last_distance is None or dt <= 1e-6
            else (self._near_min_last_distance - distance) / dt
        )
        progress_threshold = max(0.0, float(self.config.near_min_effective_progress_mps))
        if progress is not None and progress < progress_threshold:
            self._near_min_stall_sec += dt
        else:
            self._near_min_stall_sec = 0.0
        stall_threshold = max(0.0, float(self.config.near_min_effective_adaptive_stall_sec))
        if self._near_min_stall_sec >= stall_threshold:
            self._near_min_floor_u = min(
                max(0.0, float(self.config.near_min_effective_adaptive_max_u)),
                self._near_min_floor_u + max(0.0, float(self.config.near_min_effective_adaptive_step_u)),
            )
            self._near_min_stall_sec = 0.0
            self._near_min_adaptive_bumps += 1
        self._near_min_last_timestamp = timestamp_f
        self._near_min_last_distance = distance
        self._near_min_progress_mps = progress
        return self._near_min_effective_adaptive_values(self._near_min_floor_u)

    def _near_min_effective_adaptive_values(self, floor_u: float) -> dict[str, float | int | None]:
        return {
            "floor_u": floor_u,
            "stall_sec": self._near_min_stall_sec,
            "progress_mps": self._near_min_progress_mps,
            "adaptive_bumps": self._near_min_adaptive_bumps,
        }

    def _reset_near_min_effective(self) -> None:
        self._near_min_target_key = None
        self._near_min_last_timestamp = None
        self._near_min_last_distance = None
        self._near_min_floor_u = max(0.0, float(self.config.near_min_effective_u))
        self._near_min_stall_sec = 0.0
        self._near_min_progress_mps = None
        self._near_min_adaptive_bumps = 0
