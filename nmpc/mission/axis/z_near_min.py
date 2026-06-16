from __future__ import annotations

from nmpc.mission.math_utils import (
    finite_float as _finite_float,
    raise_axis_min_abs as _raise_axis_min_abs,
)


class NmpcMissionZNearMinMixin:
    def _z_near_min_effective(
        self,
        *,
        throttle_u: float,
        distance: float | None,
        total_authority: float,
    ) -> dict[str, float | bool | None]:
        result = {
            "enabled": False,
            "distance": distance,
            "throttle_u": throttle_u,
            "before_throttle_u": throttle_u,
            "after_throttle_u": throttle_u,
        }
        distance_f = _finite_float(distance)
        if distance_f is None or not self.config.z_near_min_effective_enabled:
            return result
        clear = max(0.0, float(self.config.z_near_min_effective_clear_m))
        start = max(clear, float(self.config.z_near_min_effective_start_m))
        if distance_f <= clear or distance_f > start:
            return result

        min_abs = max(0.0, float(self.config.z_near_min_effective_u))
        max_abs = max(0.0, min(1.0, float(total_authority)))
        target_abs = min(min_abs, max_abs)
        if target_abs <= 0.0:
            return result
        throttle_u, changed = _raise_axis_min_abs(throttle_u, target_abs)
        if not changed:
            return result
        result.update(
            {
                "enabled": True,
                "throttle_u": throttle_u,
                "after_throttle_u": throttle_u,
            }
        )
        return result
