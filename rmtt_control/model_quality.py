#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

from models.dji_velocity_model import VALID_MODEL_AXES, load_velocity_model


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "models" / "rmtt_velocity_model.json"


@dataclass(frozen=True)
class QualityThresholds:
    min_samples: int = 30
    min_r2: float = 0.20
    min_vaf: float = 0.20
    max_nrmse: float = 0.80
    fail_on_bootstrap: bool = False
    require_validation: bool = False


@dataclass(frozen=True)
class AxisQuality:
    axis: str
    ok: bool
    warnings: tuple[str, ...]
    failures: tuple[str, ...]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check fitted RMTT velocity model quality.")
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument("--min-r2", type=float, default=0.20)
    parser.add_argument("--min-vaf", type=float, default=0.20)
    parser.add_argument("--max-nrmse", type=float, default=0.80)
    parser.add_argument("--fail-on-bootstrap", action="store_true")
    parser.add_argument("--require-validation", action="store_true")
    args = parser.parse_args(argv)

    thresholds = QualityThresholds(
        min_samples=args.min_samples,
        min_r2=args.min_r2,
        min_vaf=args.min_vaf,
        max_nrmse=args.max_nrmse,
        fail_on_bootstrap=args.fail_on_bootstrap,
        require_validation=args.require_validation,
    )
    results = check_model_quality(Path(args.model), thresholds=thresholds)
    failed = False
    for result in results:
        status = "OK" if result.ok else "FAIL"
        print("{0}: {1}".format(status, result.axis), flush=True)
        for warning in result.warnings:
            print("  WARN: {0}".format(warning), flush=True)
        for failure in result.failures:
            print("  FAIL: {0}".format(failure), flush=True)
        failed = failed or not result.ok
    return 1 if failed else 0


def check_model_quality(
    path: str | Path = DEFAULT_MODEL,
    *,
    thresholds: QualityThresholds | None = None,
) -> list[AxisQuality]:
    thresholds = thresholds or QualityThresholds()
    model = load_velocity_model(path)
    model.require_axes(VALID_MODEL_AXES)
    return [
        _check_axis(axis, model.axis(axis), thresholds=thresholds)
        for axis in VALID_MODEL_AXES
    ]


def _check_axis(axis: str, model_axis, *, thresholds: QualityThresholds) -> AxisQuality:
    warnings: list[str] = []
    failures: list[str] = []
    fit = model_axis.fit or {}

    if fit.get("bootstrap") is True:
        message = "still uses bootstrap model"
        if thresholds.fail_on_bootstrap:
            failures.append(message)
        else:
            warnings.append(message)

    sample_count = _float_or_none(fit.get("sample_count"))
    if sample_count is None:
        warnings.append("fit.sample_count missing")
    elif sample_count < thresholds.min_samples:
        failures.append(
            "sample_count {0:g} < {1}".format(sample_count, thresholds.min_samples)
        )

    r2 = _float_or_none(fit.get("r2"))
    if r2 is None:
        warnings.append("fit.r2 missing")
    elif r2 < thresholds.min_r2:
        failures.append("r2 {0:.3f} < {1:.3f}".format(r2, thresholds.min_r2))

    vaf = _float_or_none(fit.get("vaf"))
    if vaf is None:
        warnings.append("fit.vaf missing")
    elif vaf < thresholds.min_vaf:
        failures.append("vaf {0:.3f} < {1:.3f}".format(vaf, thresholds.min_vaf))

    nrmse = _float_or_none(fit.get("nrmse"))
    if nrmse is None:
        warnings.append("fit.nrmse missing")
    elif nrmse > thresholds.max_nrmse:
        failures.append("nrmse {0:.3f} > {1:.3f}".format(nrmse, thresholds.max_nrmse))

    if model_axis.tau <= 0.0:
        failures.append("tau must be positive")
    if model_axis.vmax <= 0.0:
        failures.append("vmax must be positive")
    if model_axis.amax <= 0.0:
        failures.append("amax must be positive")
    if model_axis.Td < 0.0:
        failures.append("Td must be non-negative")

    _check_validation_metrics(
        model_axis.validation or {},
        thresholds=thresholds,
        warnings=warnings,
        failures=failures,
    )

    return AxisQuality(
        axis=axis,
        ok=not failures,
        warnings=tuple(warnings),
        failures=tuple(failures),
    )


def _float_or_none(value) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _check_validation_metrics(
    validation: dict,
    *,
    thresholds: QualityThresholds,
    warnings: list[str],
    failures: list[str],
) -> None:
    if not validation:
        message = "validation metrics missing"
        if thresholds.require_validation:
            failures.append(message)
        else:
            warnings.append(message)
        return
    if validation.get("independent") is not True:
        message = "validation is not marked independent"
        if thresholds.require_validation:
            failures.append(message)
        else:
            warnings.append(message)

    sample_count = _float_or_none(validation.get("sample_count"))
    if sample_count is None:
        warnings.append("validation.sample_count missing")
    elif sample_count < thresholds.min_samples:
        failures.append(
            "validation sample_count {0:g} < {1}".format(sample_count, thresholds.min_samples)
        )

    r2 = _float_or_none(validation.get("r2"))
    if r2 is None:
        warnings.append("validation.r2 missing")
    elif r2 < thresholds.min_r2:
        failures.append("validation r2 {0:.3f} < {1:.3f}".format(r2, thresholds.min_r2))

    vaf = _float_or_none(validation.get("vaf"))
    if vaf is None:
        warnings.append("validation.vaf missing")
    elif vaf < thresholds.min_vaf:
        failures.append("validation vaf {0:.3f} < {1:.3f}".format(vaf, thresholds.min_vaf))

    nrmse = _float_or_none(validation.get("nrmse"))
    if nrmse is None:
        warnings.append("validation.nrmse missing")
    elif nrmse > thresholds.max_nrmse:
        failures.append("validation nrmse {0:.3f} > {1:.3f}".format(nrmse, thresholds.max_nrmse))


if __name__ == "__main__":
    sys.exit(main())
