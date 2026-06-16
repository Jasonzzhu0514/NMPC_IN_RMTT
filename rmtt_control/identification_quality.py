#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CSV health checks for RMTT stick-to-motion identification runs."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path


AXES = ("pitch", "roll", "throttle", "yaw")
REQUESTED_COLUMNS = {
    "roll": "requested_roll",
    "pitch": "requested_pitch",
    "throttle": "requested_throttle",
    "yaw": "requested_yaw",
}
APPLIED_COLUMNS = {
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


@dataclass(frozen=True)
class CsvQualityThresholds:
    min_rows: int = 30
    min_signed_samples: int = 10
    min_position_span: float = 0.05
    min_yaw_span_deg: float = 5.0
    max_safety_fail_ratio: float = 0.0


@dataclass(frozen=True)
class CsvAxisQuality:
    axis: str
    path: Path
    ok: bool
    warnings: tuple[str, ...]
    failures: tuple[str, ...]
    rows: int
    safe_rows: int
    positive_samples: int
    negative_samples: int
    position_span: float


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check RMTT identification CSV health before fitting.")
    parser.add_argument("--min-rows", type=int, default=30)
    parser.add_argument("--min-signed-samples", type=int, default=10)
    parser.add_argument("--min-position-span", type=float, default=0.05)
    parser.add_argument("--min-yaw-span-deg", type=float, default=5.0)
    parser.add_argument("--max-safety-fail-ratio", type=float, default=0.0)
    for axis in AXES:
        parser.add_argument(f"--{axis}-csv", action="append", default=[])
    args = parser.parse_args(argv)

    thresholds = CsvQualityThresholds(
        min_rows=args.min_rows,
        min_signed_samples=args.min_signed_samples,
        min_position_span=args.min_position_span,
        min_yaw_span_deg=args.min_yaw_span_deg,
        max_safety_fail_ratio=args.max_safety_fail_ratio,
    )
    results: list[CsvAxisQuality] = []
    for axis in AXES:
        for path in getattr(args, f"{axis}_csv"):
            results.append(check_identification_csv(path, axis=axis, thresholds=thresholds))
    if not results:
        raise ValueError("no CSV inputs provided")
    failed = False
    for result in results:
        status = "OK" if result.ok else "FAIL"
        print(
            "{status}: {axis} {path} rows={rows} safe={safe} +={pos} -={neg} span={span:.4g}".format(
                status=status,
                axis=result.axis,
                path=result.path,
                rows=result.rows,
                safe=result.safe_rows,
                pos=result.positive_samples,
                neg=result.negative_samples,
                span=result.position_span,
            ),
            flush=True,
        )
        for warning in result.warnings:
            print("  WARN: {0}".format(warning), flush=True)
        for failure in result.failures:
            print("  FAIL: {0}".format(failure), flush=True)
        failed = failed or not result.ok
    return 1 if failed else 0


def check_identification_csv(
    path: str | Path,
    *,
    axis: str,
    thresholds: CsvQualityThresholds | None = None,
) -> CsvAxisQuality:
    axis = _validate_axis(axis)
    thresholds = thresholds or CsvQualityThresholds()
    path = Path(path)
    rows = 0
    safe_rows = 0
    safety_failed = 0
    positive = 0
    negative = 0
    positions: list[float] = []
    warnings: list[str] = []
    failures: list[str] = []
    requested_column = REQUESTED_COLUMNS[axis]
    applied_column = APPLIED_COLUMNS[axis]
    position_column = POSITION_COLUMNS[axis]

    with path.open() as file:
        reader = csv.DictReader(file)
        missing = [name for name in (requested_column, applied_column, position_column) if name not in (reader.fieldnames or [])]
        if missing:
            failures.append("missing columns: {0}".format(", ".join(missing)))
            return _result(axis, path, warnings, failures, rows, safe_rows, positive, negative, 0.0)
        for row in reader:
            if (row.get("signal_kind") or "").strip().lower() == "recenter":
                continue
            if (row.get("axis") or "").strip().lower() != axis:
                continue
            rows += 1
            safety_ok = _truthy(row.get("safety_ok", "1"))
            if safety_ok:
                safe_rows += 1
            else:
                safety_failed += 1
                continue
            command = _float_or_none(row.get(requested_column))
            if command is None or abs(command) <= 1e-6:
                command = _float_or_none(row.get(applied_column))
            if command is not None and command > 1e-6:
                positive += 1
            elif command is not None and command < -1e-6:
                negative += 1
            position = _float_or_none(row.get(position_column))
            if position is not None:
                positions.append(_yaw_to_degrees(position) if axis == "yaw" else position)

    position_span = _span(_unwrap_degrees(positions) if axis == "yaw" else positions)
    if rows < thresholds.min_rows:
        failures.append("rows {0} < {1}".format(rows, thresholds.min_rows))
    if safe_rows < thresholds.min_rows:
        failures.append("safe rows {0} < {1}".format(safe_rows, thresholds.min_rows))
    if positive < thresholds.min_signed_samples:
        failures.append("positive samples {0} < {1}".format(positive, thresholds.min_signed_samples))
    if negative < thresholds.min_signed_samples:
        failures.append("negative samples {0} < {1}".format(negative, thresholds.min_signed_samples))
    safety_fail_ratio = safety_failed / max(rows, 1)
    if safety_fail_ratio > thresholds.max_safety_fail_ratio:
        failures.append(
            "safety fail ratio {0:.3f} > {1:.3f}".format(
                safety_fail_ratio,
                thresholds.max_safety_fail_ratio,
            )
        )
    min_span = thresholds.min_yaw_span_deg if axis == "yaw" else thresholds.min_position_span
    if position_span < min_span:
        failures.append("position span {0:.4g} < {1:.4g}".format(position_span, min_span))
    if len(positions) < safe_rows:
        warnings.append("some safe rows are missing pose samples")
    return _result(axis, path, warnings, failures, rows, safe_rows, positive, negative, position_span)


def check_many(
    paths_by_axis: dict[str, list[str | Path]],
    *,
    thresholds: CsvQualityThresholds | None = None,
) -> list[CsvAxisQuality]:
    results: list[CsvAxisQuality] = []
    for axis, paths in paths_by_axis.items():
        for path in paths:
            results.append(check_identification_csv(path, axis=axis, thresholds=thresholds))
    return results


def _result(
    axis: str,
    path: Path,
    warnings: list[str],
    failures: list[str],
    rows: int,
    safe_rows: int,
    positive: int,
    negative: int,
    position_span: float,
) -> CsvAxisQuality:
    return CsvAxisQuality(
        axis=axis,
        path=path,
        ok=not failures,
        warnings=tuple(warnings),
        failures=tuple(failures),
        rows=rows,
        safe_rows=safe_rows,
        positive_samples=positive,
        negative_samples=negative,
        position_span=position_span,
    )


def _validate_axis(axis: str) -> str:
    axis = axis.strip().lower()
    if axis not in AXES:
        raise ValueError("unsupported axis {0!r}".format(axis))
    return axis


def _float_or_none(value) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _truthy(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _span(values: list[float]) -> float:
    if not values:
        return 0.0
    return max(values) - min(values)


def _yaw_to_degrees(value: float) -> float:
    return math.degrees(value) if abs(value) <= 6.5 else value


def _unwrap_degrees(values: list[float]) -> list[float]:
    if not values:
        return []
    out = [values[0]]
    offset = 0.0
    previous = values[0]
    for value in values[1:]:
        delta = value - previous
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        out.append(value + offset)
        previous = value
    return out


if __name__ == "__main__":
    sys.exit(main())
