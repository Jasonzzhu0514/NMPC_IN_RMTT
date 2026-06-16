from __future__ import annotations

import math


def score_prediction(actual: list[float], predicted: list[float]) -> dict[str, float]:
    if len(actual) != len(predicted) or not actual:
        raise ValueError("actual and predicted series must be non-empty and equal length.")
    residuals = [target - estimate for target, estimate in zip(actual, predicted)]
    rmse = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    span = max(actual) - min(actual)
    nrmse = rmse / max(abs(span), 1e-9)
    mean_actual = sum(actual) / len(actual)
    ss_res = sum(value * value for value in residuals)
    ss_tot = sum((value - mean_actual) ** 2 for value in actual)
    r2 = 1.0 - ss_res / max(ss_tot, 1e-9)
    residual_var = variance(residuals)
    actual_var = variance(actual)
    vaf = 1.0 - residual_var / max(actual_var, 1e-9)
    return {"rmse": rmse, "nrmse": nrmse, "r2": r2, "vaf": vaf}


def unwrap_degrees(values: list[float]) -> list[float]:
    if not values:
        return []
    unwrapped = [values[0]]
    previous_raw = values[0]
    offset = 0.0
    for raw in values[1:]:
        delta = raw - previous_raw
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        unwrapped.append(raw + offset)
        previous_raw = raw
    return unwrapped


def derivative(t: list[float], values: list[float]) -> list[float]:
    if len(values) < 2:
        return [0.0 for _ in values]
    result = []
    for index in range(len(values)):
        if index == 0:
            dt = t[1] - t[0]
            dy = values[1] - values[0]
        elif index == len(values) - 1:
            dt = t[-1] - t[-2]
            dy = values[-1] - values[-2]
        else:
            dt = t[index + 1] - t[index - 1]
            dy = values[index + 1] - values[index - 1]
        result.append(dy / max(dt, 1e-9))
    return result


def moving_average(values: list[float], *, window: int) -> list[float]:
    if window <= 1 or len(values) < window:
        return values
    half = window // 2
    averaged = []
    for index in range(len(values)):
        start = max(0, index - half)
        end = min(len(values), index + half + 1)
        averaged.append(sum(values[start:end]) / (end - start))
    return averaged


def delayed_input(t: list[float], u: list[float], delay: float) -> list[float]:
    return [interpolate(t, u, sample_time - delay) for sample_time in t]


def interpolate(t: list[float], values: list[float], sample_time: float) -> float:
    if sample_time <= t[0]:
        return values[0]
    if sample_time >= t[-1]:
        return values[-1]
    low = 0
    high = len(t) - 1
    while high - low > 1:
        mid = (low + high) // 2
        if t[mid] <= sample_time:
            low = mid
        else:
            high = mid
    ratio = (sample_time - t[low]) / max(t[high] - t[low], 1e-9)
    return values[low] + ratio * (values[high] - values[low])


def abs_slopes(t: list[float], values: list[float]) -> list[float]:
    return [
        abs((values[index] - values[index - 1]) / max(t[index] - t[index - 1], 1e-9))
        for index in range(1, len(values))
    ]


def percentile(values: list[float], percent: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * clamp(percent, 0.0, 100.0) / 100.0
    low = math.floor(index)
    high = math.ceil(index)
    if low == high:
        return ordered[int(index)]
    ratio = index - low
    return ordered[low] * (1.0 - ratio) + ordered[high] * ratio


def variance(values: list[float]) -> float:
    if not values:
        return 0.0
    mean = sum(values) / len(values)
    return sum((value - mean) ** 2 for value in values) / len(values)


def linspace(start: float, stop: float, count: int) -> list[float]:
    if count <= 1:
        return [start]
    step = (stop - start) / (count - 1)
    return [start + step * index for index in range(count)]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
