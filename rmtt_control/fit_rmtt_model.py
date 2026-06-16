#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import sys

from models.dji_velocity_model import load_velocity_model
from nmpc.identification.fit_rmtt import (
    RmttFitConfig,
    build_axis_model_document,
    build_full_model_document,
    fit_fopdt,
    load_rmtt_identification_csvs,
    write_comparison_csv,
    write_model_json,
)


AXES = ("pitch", "roll", "throttle", "yaw")
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "models" / "rmtt_velocity_model.json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Fit RMTT NMPC velocity model from identify_collect CSV logs.")
    for axis in AXES:
        parser.add_argument(
            f"--{axis}-csv",
            action="append",
            default=[],
            help=f"CSV file for {axis}; can be repeated",
        )
    parser.add_argument("--output", default=str(DEFAULT_MODEL))
    parser.add_argument("--comparison-dir", default="models/comparisons")
    parser.add_argument("--base-model", default=str(DEFAULT_MODEL))
    parser.add_argument("--require-all", action="store_true", help="fail unless all four axes have CSV input")
    parser.add_argument("--backup", action="store_true", help="backup output model before overwriting it")
    parser.add_argument("--td-upper-bound", type=float, default=0.50)
    parser.add_argument("--tau-lower-bound", type=float, default=0.05)
    parser.add_argument("--tau-upper-bound", type=float, default=1.80)
    args = parser.parse_args(argv)

    config = RmttFitConfig(
        td_upper_bound=args.td_upper_bound,
        tau_lower_bound=args.tau_lower_bound,
        tau_upper_bound=args.tau_upper_bound,
    )
    provided = {
        axis: [Path(value).expanduser() for value in getattr(args, f"{axis}_csv")]
        for axis in AXES
    }
    missing = [axis for axis, paths in provided.items() if not paths]
    if args.require_all and missing:
        raise ValueError("missing CSV for axes: {0}".format(", ".join(missing)))

    base_axes = {}
    base_path = Path(args.base_model).expanduser()
    if base_path.exists():
        base = load_velocity_model(base_path)
        for axis, model in base.axes.items():
            base_axes[axis] = {
                "K": model.K,
                "tau": model.tau,
                "Td": model.Td,
                "vmax": model.vmax,
                "amax": model.amax,
                "response": model.response,
                "unit": model.unit,
                "fit": dict(model.fit),
                "validation": dict(model.validation),
            }

    axes = dict(base_axes)
    comparison_dir = Path(args.comparison_dir)
    for axis, paths in provided.items():
        if not paths:
            print("skip {0}: no CSV, keeping base model axis if present".format(axis), flush=True)
            continue
        series = load_rmtt_identification_csvs(paths, axis=axis, config=config)
        fit = fit_fopdt(series, config=config)
        comparison_path = comparison_dir / f"{axis}_comparison.csv"
        write_comparison_csv(comparison_path, series, fit.predicted)
        axes[axis] = build_axis_model_document(
            axis=axis,
            series=series,
            fit=fit,
            source_paths=paths,
            note="Fitted from RMTT VRPN identify_collect logs.",
        )
        print(
            "{axis}: K={K:.4f} tau={tau:.3f} Td={Td:.3f} vmax={vmax:.3f} "
            "rmse={rmse:.4f} r2={r2:.3f} samples={samples}".format(
                axis=axis,
                K=fit.params.K,
                tau=fit.params.tau,
                Td=fit.params.Td,
                vmax=fit.params.vmax,
                rmse=fit.rmse,
                r2=fit.r2,
                samples=len(series.t),
            ),
            flush=True,
        )

    still_missing = [axis for axis in AXES if axis not in axes]
    if still_missing:
        raise ValueError("model is missing axes after merge: {0}".format(", ".join(still_missing)))

    output = Path(args.output).expanduser()
    if args.backup and output.exists():
        backup = output.with_suffix(output.suffix + ".bak")
        shutil.copyfile(output, backup)
        print("backup: {0}".format(backup), flush=True)
    document = build_full_model_document(
        {axis: axes[axis] for axis in AXES},
        note="RMTT model fitted from available axes; missing CSV axes were inherited from base model.",
    )
    write_model_json(output, document)
    print("model: {0}".format(output), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
