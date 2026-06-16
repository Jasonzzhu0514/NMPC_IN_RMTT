from __future__ import annotations

import math


def normalized_to_absolute_stick(
    u: float,
    *,
    neutral: int,
    stick_min: int,
    stick_max: int,
) -> int:
    u = clamp(float(u), -1.0, 1.0)
    span = (stick_max - neutral) if u >= 0.0 else (neutral - stick_min)
    value = neutral + u * span
    return int(round(clamp(value, stick_min, stick_max)))


def profile_for_phase(phase: str | None) -> str:
    normalized = (phase or "").strip().upper()
    if normalized == "TASK":
        return "task_hold"
    if normalized in {"MOVE-XYZ", "ALIGN", "VERTICAL"}:
        return "yaw_priority"
    return "move_xyz_priority"


def smooth_velocity(previous: float | None, raw: float, alpha: float) -> float:
    raw = float(raw)
    if previous is None:
        return raw
    alpha = clamp(alpha, 0.0, 1.0)
    return alpha * raw + (1.0 - alpha) * previous


def finite_float(value: float | int | None) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def delta(value: float, reference: float | None) -> float | None:
    reference_f = finite_float(reference)
    if reference_f is None:
        return None
    return value - reference_f


def signed_angle_delta(target: float, current: float) -> float:
    return ((target - current + 180.0) % 360.0) - 180.0


def wrap_degrees(value: float) -> float:
    return ((value + 180.0) % 360.0) - 180.0


def command_saturated(pitch_u: float, roll_u: float, *, max_abs_u: float) -> bool:
    threshold = max(0.0, abs(max_abs_u) - 1e-9)
    return abs(pitch_u) >= threshold or abs(roll_u) >= threshold


def boundary_active(
    x: float,
    y: float,
    terminal_x: float,
    terminal_y: float,
    *,
    field_limit: float,
    safety_margin: float,
    z: float | None = None,
    terminal_z: float | None = None,
    z_max_m: float | None = None,
) -> bool:
    soft_limit = max(0.0, field_limit - safety_margin)
    xy_active = (
        abs(x) >= soft_limit
        or abs(y) >= soft_limit
        or abs(terminal_x) >= soft_limit
        or abs(terminal_y) >= soft_limit
    )
    if xy_active:
        return True
    z_limit = finite_float(z_max_m)
    if z_limit is None or z_limit <= 0.0:
        return False
    current_z = finite_float(z)
    predicted_z = finite_float(terminal_z)
    return (
        current_z is not None
        and current_z >= z_limit
    ) or (
        predicted_z is not None
        and predicted_z >= z_limit
    )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
