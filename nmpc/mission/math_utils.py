from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True)
class OsdFreshness:
    missing: bool
    reason: str | None
    age_sec: float | None
    last_msg_monotonic: float | None
    monotonic_now: float
    max_age_sec: float
    frequency_hz: float | None


def osd_freshness(
    osd_source: Any | None,
    *,
    monotonic_now: float,
    max_age_sec: float,
) -> OsdFreshness:
    max_age = max(0.0, float(max_age_sec))
    frequency = _finite_call(osd_source, "get_osd_frequency")
    if osd_source is None:
        return OsdFreshness(
            missing=True,
            reason="no_osd_source",
            age_sec=None,
            last_msg_monotonic=None,
            monotonic_now=monotonic_now,
            max_age_sec=max_age,
            frequency_hz=frequency,
        )
    last_osd = call_optional(osd_source, "get_last_osd_msg_monotonic")
    if last_osd is None:
        return OsdFreshness(
            missing=True,
            reason="no_last_osd_monotonic",
            age_sec=None,
            last_msg_monotonic=None,
            monotonic_now=monotonic_now,
            max_age_sec=max_age,
            frequency_hz=frequency,
        )
    try:
        last_osd_f = float(last_osd)
        age = monotonic_now - last_osd_f
    except (TypeError, ValueError):
        return OsdFreshness(
            missing=True,
            reason="invalid_last_osd_monotonic",
            age_sec=None,
            last_msg_monotonic=None,
            monotonic_now=monotonic_now,
            max_age_sec=max_age,
            frequency_hz=frequency,
        )
    if not math.isfinite(age) or not math.isfinite(last_osd_f):
        return OsdFreshness(
            missing=True,
            reason="nonfinite_osd_age",
            age_sec=age,
            last_msg_monotonic=last_osd_f,
            monotonic_now=monotonic_now,
            max_age_sec=max_age,
            frequency_hz=frequency,
        )
    if age < 0.0:
        return OsdFreshness(
            missing=True,
            reason="osd_timestamp_in_future",
            age_sec=age,
            last_msg_monotonic=last_osd_f,
            monotonic_now=monotonic_now,
            max_age_sec=max_age,
            frequency_hz=frequency,
        )
    if age > max_age:
        return OsdFreshness(
            missing=True,
            reason="osd_stale",
            age_sec=age,
            last_msg_monotonic=last_osd_f,
            monotonic_now=monotonic_now,
            max_age_sec=max_age,
            frequency_hz=frequency,
        )
    return OsdFreshness(
        missing=False,
        reason=None,
        age_sec=age,
        last_msg_monotonic=last_osd_f,
        monotonic_now=monotonic_now,
        max_age_sec=max_age,
        frequency_hz=frequency,
    )


def missing_osd_frame(
    osd_source: Any | None,
    *,
    monotonic_now: float,
    max_age_sec: float,
) -> bool:
    return osd_freshness(
        osd_source,
        monotonic_now=monotonic_now,
        max_age_sec=max_age_sec,
    ).missing


def call_optional(obj: Any, name: str) -> Any:
    func = getattr(obj, name, None)
    if not callable(func):
        return None
    try:
        return func()
    except Exception:
        return None


def _finite_call(obj: Any, name: str) -> float | None:
    value = call_optional(obj, name)
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def absolute_or_neutral(
    value: int | float | None,
    neutral: int,
    stick_min: int,
    stick_max: int,
) -> int:
    if value is None:
        return neutral
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return neutral
    if not math.isfinite(numeric):
        return neutral
    return int(round(clamp(numeric, stick_min, stick_max)))


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


def sign(value: float) -> float:
    return 1.0 if value >= 0.0 else -1.0


def raise_axis_min_abs(value: float, target_abs: float) -> tuple[float, bool]:
    abs_value = abs(value)
    if abs_value <= 1e-9 or abs_value >= target_abs:
        return value, False
    return math.copysign(target_abs, value), True


def xy_distance(
    current_x: float | int | None,
    current_y: float | int | None,
    target_x: float | int | None,
    target_y: float | int | None,
) -> float | None:
    x = finite_float(current_x)
    y = finite_float(current_y)
    tx = finite_float(target_x)
    ty = finite_float(target_y)
    if None in (x, y, tx, ty):
        return None
    return math.hypot(tx - x, ty - y)


def z_distance(
    current_z: float | int | None,
    target_z: float | int | None,
) -> float | None:
    z = finite_float(current_z)
    tz = finite_float(target_z)
    if z is None or tz is None:
        return None
    return abs(tz - z)


def closing_speed_toward_target(
    *,
    state_vx: float | int | None,
    state_vy: float | int | None,
    ux: float,
    uy: float,
) -> float | None:
    vx = finite_float(state_vx)
    vy = finite_float(state_vy)
    if vx is None or vy is None:
        return None
    return vx * ux + vy * uy


def predicted_terminal_crosses_target(
    *,
    predicted_x: float | int | None,
    predicted_y: float | int | None,
    target_x: float,
    target_y: float,
    ux: float,
    uy: float,
) -> bool:
    px = finite_float(predicted_x)
    py = finite_float(predicted_y)
    if px is None or py is None:
        return False
    remaining = (target_x - px) * ux + (target_y - py) * uy
    return remaining < 0.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))
