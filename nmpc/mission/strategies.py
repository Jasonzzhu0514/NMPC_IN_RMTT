from __future__ import annotations

from nmpc.flight_controller import NmpcFlightControlResult
from nmpc.mission.math_utils import (
    clamp as _clamp,
    closing_speed_toward_target as _closing_speed_toward_target,
    finite_float as _finite_float,
    predicted_terminal_crosses_target as _predicted_terminal_crosses_target,
)
from nmpc.mission.axis.near_min import NmpcMissionNearMinMixin
from nmpc.mission.trim import NmpcMissionTrimMixin


class NmpcMissionStrategies(NmpcMissionTrimMixin, NmpcMissionNearMinMixin):
    def _near_fine_authority(self, nmpc_flight: NmpcFlightControlResult) -> float | None:
        if nmpc_flight.position_3d_candidate_profile != "near_fine":
            return None
        config = self.nmpc_flight.controller_config
        if not config.near_fine_candidate_enabled:
            return None
        return max(
            0.0,
            min(
                max(0.0, float(config.max_abs_u)),
                max(0.0, float(config.near_fine_candidate_max_abs_u)),
            ),
        )

    def _arrival_brake_xy(
        self,
        *,
        controller_result: NmpcFlightControlResult,
        current_x: float | None,
        current_y: float | None,
        target_x: float | None,
        target_y: float | None,
        distance: float | None,
        roll_u: float,
        pitch_u: float,
    ) -> dict[str, float | bool | None]:
        result = {
            "active": False,
            "scale": 1.0,
            "time_to_target_sec": None,
            "predicted_crossing": False,
            "roll_u": roll_u,
            "pitch_u": pitch_u,
        }
        if not self.config.arrival_brake_enabled:
            return result
        x = _finite_float(current_x)
        y = _finite_float(current_y)
        tx = _finite_float(target_x)
        ty = _finite_float(target_y)
        distance_f = _finite_float(distance)
        if None in (x, y, tx, ty, distance_f) or distance_f <= 1e-9:
            return result
        clear = max(0.0, float(self.config.arrival_brake_clear_m))
        start = max(clear, float(self.config.arrival_brake_start_m))
        if distance_f > start:
            return result

        dx = tx - x
        dy = ty - y
        ux = dx / distance_f
        uy = dy / distance_f
        closing_speed = _closing_speed_toward_target(
            state_vx=controller_result.state_vx,
            state_vy=controller_result.state_vy,
            ux=ux,
            uy=uy,
        )
        time_to_target = (
            distance_f / closing_speed
            if closing_speed is not None and closing_speed > 1e-6
            else None
        )
        predicted_crossing = _predicted_terminal_crosses_target(
            predicted_x=controller_result.predicted_terminal_x,
            predicted_y=controller_result.predicted_terminal_y,
            target_x=tx,
            target_y=ty,
            ux=ux,
            uy=uy,
        )
        lookahead = max(0.0, float(self.config.arrival_brake_lookahead_sec))
        progress_threshold = max(0.0, float(self.config.near_min_effective_progress_mps))
        is_closing = closing_speed is not None and closing_speed > progress_threshold
        should_brake = distance_f <= clear or (
            is_closing
            and (
                predicted_crossing
                or (time_to_target is not None and time_to_target <= lookahead)
            )
        )
        result.update(
            {
                "time_to_target_sec": time_to_target,
                "predicted_crossing": predicted_crossing,
            }
        )
        if not should_brake:
            return result

        min_scale = _clamp(float(self.config.arrival_brake_min_scale), 0.0, 1.0)
        if distance_f <= clear:
            scale = min_scale
        else:
            scale = min_scale + (1.0 - min_scale) * (distance_f - clear) / max(1e-9, start - clear)
        scale = _clamp(scale, min_scale, 1.0)
        result.update(
            {
                "active": True,
                "scale": scale,
                "roll_u": roll_u * scale,
                "pitch_u": pitch_u * scale,
            }
        )
        return result

    def _update_heading_gate(
        self,
        *,
        heading_error_deg: float | None,
        distance_xy: float | None,
    ) -> bool:
        if not self.config.heading_gate_enabled:
            self._heading_gate_active = False
            return False
        distance_f = _finite_float(distance_xy)
        min_distance = max(0.0, float(self.config.heading_gate_min_distance_m))
        if distance_f is None or distance_f <= min_distance:
            self._heading_gate_active = False
            return False
        error = _finite_float(heading_error_deg)
        if error is None:
            self._heading_gate_active = False
            return False
        abs_error = abs(error)
        enter = max(0.0, float(self.config.heading_gate_enter_deg))
        clear = min(enter, max(0.0, float(self.config.heading_gate_clear_deg)))
        if self._heading_gate_active:
            self._heading_gate_active = abs_error > clear
        else:
            self._heading_gate_active = abs_error >= enter
        return self._heading_gate_active
