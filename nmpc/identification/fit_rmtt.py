from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

from nmpc.identification.single_axis.fit_math import (
    abs_slopes,
    clamp,
    delayed_input,
    derivative,
    linspace,
    moving_average,
    percentile,
    score_prediction,
    unwrap_degrees,
)
from nmpc.identification.single_axis.fit_models import FitResult, ModelParams, SampleSeries


STICK_COLUMNS = {
    "roll": "roll",
    "pitch": "pitch",
    "throttle": "throttle",
    "yaw": "yaw",
}
POSITION_COLUMNS = {
    "roll": "y",
    "pitch": "x",
    "throttle": "z",
    "yaw": "yaw_pose",
}
UNIT_BY_AXIS = {
    "roll": "m_s",
    "pitch": "m_s",
    "throttle": "m_s",
    "yaw": "deg_s",
}


@dataclass(frozen=True)
class RmttFitConfig:
    transient_skip_sec: float = 0.25
    min_samples: int = 12
    smoothing_window_linear: int = 7
    smoothing_window_yaw: int = 5
    td_upper_bound: float = 0.50
    tau_lower_bound: float = 0.05
    tau_upper_bound: float = 1.80
    td_grid_count: int = 31
    tau_grid_count: int = 50


def load_rmtt_identification_csvs(
    paths: Iterable[str | Path],
    *,
    axis: str,
    config: RmttFitConfig | None = None,
) -> SampleSeries:
    axis = _validate_axis(axis)
    cfg = config or RmttFitConfig()
    rows: list[tuple[float, float, float, str]] = []
    for path in paths:
        rows.extend(_load_rows(Path(path), axis=axis, config=cfg))
    if len(rows) < cfg.min_samples:
        raise ValueError(f"Not enough valid {axis} samples: {len(rows)} < {cfg.min_samples}")

    rows.sort(key=lambda item: item[0])
    start_time = rows[0][0]
    t = [row[0] - start_time for row in rows]
    u = [row[1] / 100.0 for row in rows]
    position = [row[2] for row in rows]
    segment_starts = _segment_starts(rows)
    if axis == "yaw":
        position = unwrap_degrees([math.degrees(value) if abs(value) <= 6.5 else value for value in position])
        y = moving_average(derivative(t, position), window=cfg.smoothing_window_yaw)
        response_name = "yaw_rate_deg_s"
    else:
        y = moving_average(derivative(t, position), window=cfg.smoothing_window_linear)
        response_name = f"{POSITION_COLUMNS[axis]}_velocity"

    return SampleSeries(
        axis=axis,
        response_name=response_name,
        source=",".join(str(Path(path)) for path in paths),
        t=t,
        u=u,
        y=y,
        metadata={
            "axis": axis,
            "fit_sample_count": float(len(t)),
            "fit_segment_count": float(len(segment_starts)),
            "transient_skip_sec": cfg.transient_skip_sec,
            "td_upper_bound": cfg.td_upper_bound,
            "tau_lower_bound": cfg.tau_lower_bound,
            "tau_upper_bound": cfg.tau_upper_bound,
            "input": "RMTT rc stick normalized from [-100,100] to [-1,1]",
        },
        segment_starts=segment_starts,
    )


def fit_fopdt(series: SampleSeries, *, config: RmttFitConfig | None = None) -> FitResult:
    cfg = config or RmttFitConfig()
    max_abs_y = max(max(abs(value) for value in series.y), 1e-6)
    vmax_guess = max_abs_y * 1.25
    amax_guess = max(percentile(abs_slopes(series.t, series.y), 95.0), 1e-6)
    segment_initials = _segment_initials(series)
    zero_initials = _zero_segment_initials(series)
    best: FitResult | None = None

    for Td in linspace(0.0, max(0.0, cfg.td_upper_bound), max(2, cfg.td_grid_count)):
        for tau in linspace(
            max(1e-6, cfg.tau_lower_bound),
            max(cfg.tau_lower_bound, cfg.tau_upper_bound),
            max(2, cfg.tau_grid_count),
        ):
            baseline = simulate_fopdt(
                series.t,
                series.u,
                ModelParams(K=0.0, tau=tau, Td=Td, vmax=9999.0, amax=9999.0),
                initial=0.0,
                segment_starts=series.segment_starts,
                initial_by_segment=segment_initials,
            )
            unit = simulate_fopdt(
                series.t,
                series.u,
                ModelParams(K=1.0, tau=tau, Td=Td, vmax=9999.0, amax=9999.0),
                initial=0.0,
                segment_starts=series.segment_starts,
                initial_by_segment=zero_initials,
            )
            denom = sum(value * value for value in unit)
            if denom <= 1e-9:
                continue
            K = sum((target - base) * basis for target, base, basis in zip(series.y, baseline, unit)) / denom
            params = ModelParams(
                K=K,
                tau=tau,
                Td=Td,
                vmax=max(abs(K), vmax_guess),
                amax=amax_guess,
            )
            predicted = simulate_fopdt(
                series.t,
                series.u,
                params,
                initial=series.y[0],
                segment_starts=series.segment_starts,
                initial_by_segment=segment_initials,
            )
            metrics = score_prediction(series.y, predicted)
            result = FitResult(params=params, predicted=predicted, **metrics)
            if best is None or result.rmse < best.rmse:
                best = result
    if best is None:
        raise ValueError("Unable to fit FOPDT model from the provided data.")
    return best


def simulate_fopdt(
    t: list[float],
    u: list[float],
    params: ModelParams,
    *,
    initial: float = 0.0,
    segment_starts: list[int] | None = None,
    initial_by_segment: dict[int, float] | None = None,
) -> list[float]:
    if not t:
        return []
    starts = _normalized_segment_starts(segment_starts, sample_count=len(t))
    delayed = _segmented_delayed_input(t, u, params.Td, starts=starts)
    start_set = set(starts)
    initial_by_segment = initial_by_segment or {}
    y = [float(initial_by_segment.get(0, initial))]
    for index in range(1, len(t)):
        if index in start_set:
            y.append(float(initial_by_segment.get(index, initial)))
            continue
        dt = max(0.0, t[index] - t[index - 1])
        target = clamp(params.K * delayed[index - 1], -params.vmax, params.vmax)
        accel = (target - y[-1]) / max(params.tau, 1e-6)
        accel = clamp(accel, -params.amax, params.amax)
        y.append(y[-1] + accel * dt)
    return y


def build_axis_model_document(
    *,
    axis: str,
    series: SampleSeries,
    fit: FitResult,
    source_paths: list[str | Path],
    validation: dict | None = None,
    validation_paths: list[str | Path] | None = None,
    note: str = "",
) -> dict:
    axis = _validate_axis(axis)
    return {
        "K": fit.params.K,
        "tau": fit.params.tau,
        "Td": fit.params.Td,
        "vmax": fit.params.vmax,
        "amax": fit.params.amax,
        "response": series.response_name,
        "unit": UNIT_BY_AXIS[axis],
        "fit": {
            "rmse": fit.rmse,
            "nrmse": fit.nrmse,
            "r2": fit.r2,
            "vaf": fit.vaf,
            "sample_count": len(series.t),
            "source_csv": [str(Path(path)) for path in source_paths],
            "note": note,
        },
        "validation": _validation_document(validation or {}, validation_paths or []),
        "fit_metadata": series.metadata or {},
    }


def _validation_document(metrics: dict, paths: list[str | Path]) -> dict:
    document = dict(metrics)
    if paths:
        document["source_csv"] = [str(Path(path)) for path in paths]
        document["independent"] = True
    elif document:
        document["independent"] = False
    return document


def build_full_model_document(axes: dict[str, dict], *, note: str = "") -> dict:
    created_at = datetime.now().astimezone().isoformat()
    return {
        "metadata": {
            "platform": "RoboMaster TT",
            "mode": "identify",
            "created_at": created_at,
            "stick_range": [-100, 100],
            "input_normalization": "u = rc / 100",
            "note": note,
        },
        "axes": axes,
    }


def write_model_json(path: str | Path, document: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as file:
        json.dump(document, file, indent=2, sort_keys=True)
        file.write("\n")


def write_comparison_csv(path: str | Path, series: SampleSeries, predicted: list[float]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["t", "u", "actual", "predicted", "residual"])
        for t, u, actual, estimate in zip(series.t, series.u, series.y, predicted):
            writer.writerow([t, u, actual, estimate, actual - estimate])


def _load_rows(path: Path, *, axis: str, config: RmttFitConfig) -> list[tuple[float, float, float, str]]:
    rows: list[tuple[float, float, float, str]] = []
    with path.open() as file:
        reader = csv.DictReader(file)
        for row in reader:
            if row.get("axis", "").strip().lower() != axis:
                continue
            if row.get("signal_kind", "").strip().lower() == "recenter":
                continue
            if row.get("safety_ok", "1").strip() in {"0", "false", "False"}:
                continue
            t = _parse_float(row.get("elapsed")) or _parse_float(row.get("wall_time"))
            u = _parse_float(row.get(f"requested_{STICK_COLUMNS[axis]}"))
            if u is None:
                u = _parse_float(row.get(STICK_COLUMNS[axis]))
            position = _parse_float(row.get(POSITION_COLUMNS[axis]))
            signal_elapsed = _signal_elapsed(row)
            if t is None or u is None or position is None:
                continue
            if signal_elapsed is not None and signal_elapsed < config.transient_skip_sec:
                continue
            if row.get("step_name", "").strip().lower() == "center":
                continue
            segment = "|".join(
                (
                    str(path),
                    row.get("signal_kind", ""),
                    row.get("step_name", ""),
                    row.get("step_index", ""),
                    row.get("command_offset", ""),
                )
            )
            rows.append((t, u, position, segment))
    return rows


def _signal_elapsed(row: dict[str, str]) -> float | None:
    return _parse_float(row.get("step_elapsed"))


def _parse_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def _segment_starts(rows: list[tuple[float, float, float, str]]) -> list[int]:
    starts: list[int] = []
    previous = None
    for index, row in enumerate(rows):
        key = row[3]
        if index == 0 or key != previous:
            starts.append(index)
        previous = key
    return starts


def _segmented_delayed_input(
    t: list[float],
    u: list[float],
    delay: float,
    *,
    starts: list[int],
) -> list[float]:
    delayed = [0.0 for _ in t]
    ends = starts[1:] + [len(t)]
    for start, end in zip(starts, ends):
        delayed[start:end] = delayed_input(t[start:end], u[start:end], delay)
    return delayed


def _normalized_segment_starts(segment_starts: list[int] | None, *, sample_count: int) -> list[int]:
    starts = sorted(set(segment_starts or [0]))
    if 0 not in starts:
        starts.insert(0, 0)
    return [index for index in starts if 0 <= index < sample_count]


def _segment_initials(series: SampleSeries) -> dict[int, float]:
    starts = series.segment_starts or [0]
    return {index: series.y[index] for index in starts if 0 <= index < len(series.y)}


def _zero_segment_initials(series: SampleSeries) -> dict[int, float]:
    starts = series.segment_starts or [0]
    return {index: 0.0 for index in starts if 0 <= index < len(series.y)}


def _validate_axis(axis: str) -> str:
    key = axis.strip().lower()
    if key not in STICK_COLUMNS:
        raise ValueError("axis must be one of: {0}".format(", ".join(sorted(STICK_COLUMNS))))
    return key
