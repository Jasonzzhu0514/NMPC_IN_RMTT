#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

from rmtt_control import fit_rmtt_model
from rmtt_control import identify_collect
from rmtt_control import identification_quality
from rmtt_control import model_quality
from rmtt.adapter import RMTTClient
from rmtt_control.vrpn_pose_reader import DEFAULT_HOST, DEFAULT_PORT, DEFAULT_TRACKER
from rmtt_config import DEFAULT_RMTT_IP


AXES = ("pitch", "roll", "throttle", "yaw")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run bounded four-axis RMTT identification and fit model.")
    parser.add_argument("--ip", default=DEFAULT_RMTT_IP)
    parser.add_argument("--axes", default="pitch,roll,throttle,yaw")
    parser.add_argument("--pass-count", type=int, default=2, help="1 for single pass, 2 for independent train/validate passes")
    parser.add_argument("--signals", default="step")
    parser.add_argument("--amplitudes", default="10,20")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--model-output", default=str(fit_rmtt_model.DEFAULT_MODEL))
    parser.add_argument("--comparison-dir", default=None)
    parser.add_argument("--settle-sec", type=float, default=2.0)
    parser.add_argument("--pose-timeout-sec", type=float, default=0.5)
    parser.add_argument("--field-limit", type=float, default=1.5)
    parser.add_argument("--z-min", type=float, default=0.25)
    parser.add_argument("--z-max", type=float, default=2.0)
    parser.add_argument("--recenter", action="store_true", help="return to initial pose after marked XY excitations")
    parser.add_argument("--recenter-tolerance", type=float, default=0.10)
    parser.add_argument("--recenter-yaw-tolerance-deg", type=float, default=10.0)
    parser.add_argument("--recenter-timeout", type=float, default=8.0)
    parser.add_argument("--send", action="store_true", help="actually send rc commands")
    parser.add_argument("--confirm-risk", action="store_true", help="required with --send")
    parser.add_argument("--takeoff", action="store_true", help="take off before first axis; requires --send")
    parser.add_argument("--land", action="store_true", help="land after pipeline; requires --send")
    parser.add_argument(
        "--allow-open-airborne-handoff",
        action="store_true",
        help="internal workflow handoff: allow takeoff without landing in this process",
    )
    parser.add_argument("--fit", action="store_true", help="fit model after collection")
    parser.add_argument("--backup", action="store_true", help="backup model before overwriting")
    parser.add_argument("--skip-csv-quality", action="store_true", help="skip post-collection CSV health checks")
    parser.add_argument("--min-csv-rows", type=int, default=30)
    parser.add_argument("--min-signed-samples", type=int, default=10)
    parser.add_argument("--min-position-span", type=float, default=0.05)
    parser.add_argument("--min-yaw-span-deg", type=float, default=5.0)
    parser.add_argument("--max-safety-fail-ratio", type=float, default=0.0)
    parser.add_argument("--quality-gate", action="store_true", help="after --fit, fail unless fitted model passes quality checks")
    parser.add_argument("--quality-min-samples", type=int, default=30)
    parser.add_argument("--quality-min-r2", type=float, default=0.20)
    parser.add_argument("--quality-min-vaf", type=float, default=0.20)
    parser.add_argument("--quality-max-nrmse", type=float, default=0.80)
    parser.add_argument("--quality-fail-on-bootstrap", action="store_true")
    parser.add_argument("--quality-require-validation", action="store_true")
    parser.add_argument("--tracker", default=DEFAULT_TRACKER)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--method", choices=("auto", "native", "print"), default="auto")
    parser.add_argument("--vrpn-print-devices", default=None)
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--invert-yaw", action="store_true")
    args = parser.parse_args(argv)
    if args.send and not args.confirm_risk:
        print("Refusing to send stick commands without --confirm-risk.", flush=True)
        return 2
    if args.send and args.takeoff and not args.land and not args.allow_open_airborne_handoff:
        print(
            "Refusing --takeoff without --land. Use the workflow for identify -> xyzway handoff.",
            flush=True,
        )
        return 2
    if args.quality_gate and not args.fit:
        print("Refusing --quality-gate without --fit.", flush=True)
        return 2

    axes = _parse_axes(args.axes)
    if args.pass_count not in (1, 2):
        print("Refusing pass-count outside {1, 2}.", flush=True)
        return 2
    if args.quality_require_validation and args.pass_count < 2:
        print("Refusing --quality-require-validation with pass-count < 2.", flush=True)
        return 2
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_dir or f"identify_run_{stamp}")
    output_dir.mkdir(parents=True, exist_ok=True)
    comparison_dir = Path(args.comparison_dir or output_dir / "comparisons")

    outputs_by_pass: list[dict[str, Path]] = []
    airborne = False
    landed = False
    result_code = 1
    try:
        if args.send and args.takeoff:
            _takeoff(args.ip)
            airborne = True
        for pass_index in range(args.pass_count):
            pass_label = _pass_label(pass_index, args.pass_count)
            pass_dir = output_dir / pass_label if args.pass_count > 1 else output_dir
            pass_dir.mkdir(parents=True, exist_ok=True)
            pass_outputs: dict[str, Path] = {}
            print("[pass {}/{}] {}".format(pass_index + 1, args.pass_count, pass_label), flush=True)
            for index, axis in enumerate(axes):
                output = pass_dir / f"identify_{axis}_{stamp}.csv"
                print(
                    "[{}/{}:{}] collect {} -> {}".format(
                        index + 1,
                        len(axes),
                        pass_label,
                        axis,
                        output,
                    ),
                    flush=True,
                )
                rc = identify_collect.main(
                    [
                        "--ip",
                        args.ip,
                        "--axis",
                        axis,
                        "--signals",
                        args.signals,
                        "--amplitudes",
                        args.amplitudes,
                        "--rate",
                        str(args.rate),
                        "--output",
                        str(output),
                        "--pose-timeout-sec",
                        str(args.pose_timeout_sec),
                        "--field-limit",
                        str(args.field_limit),
                        "--z-min",
                        str(args.z_min),
                        "--z-max",
                        str(args.z_max),
                        "--recenter-tolerance",
                        str(args.recenter_tolerance),
                        "--recenter-yaw-tolerance-deg",
                        str(args.recenter_yaw_tolerance_deg),
                        "--recenter-timeout",
                        str(args.recenter_timeout),
                        "--tracker",
                        args.tracker,
                        "--host",
                        args.host,
                        "--port",
                        str(args.port),
                        "--method",
                        args.method,
                        "--z-offset",
                        str(args.z_offset),
                        *(
                            ["--vrpn-print-devices", args.vrpn_print_devices]
                            if args.vrpn_print_devices
                            else []
                        ),
                        *(["--invert-yaw"] if args.invert_yaw else []),
                        *(["--send"] if args.send else []),
                        *(["--confirm-risk"] if args.send else []),
                        *(["--recenter"] if args.recenter else []),
                    ]
                )
                if rc != 0:
                    result_code = rc
                    return rc
                pass_outputs[axis] = output
                if not args.skip_csv_quality:
                    quality_rc = _check_csv_quality(axis, output, args)
                    if quality_rc != 0:
                        result_code = quality_rc
                        return quality_rc
                if args.send and not (pass_index == args.pass_count - 1 and index == len(axes) - 1):
                    _settle(args.ip, args.settle_sec)
            outputs_by_pass.append(pass_outputs)

        result_code = 0
        if args.send and args.land:
            if _safe_action("land", _land, args.ip):
                landed = True
            else:
                result_code = 2
                return result_code

        if args.fit:
            fit_args: list[str] = [
                "--output",
                args.model_output,
                "--comparison-dir",
                str(comparison_dir),
            ]
            if args.quality_require_validation:
                fit_args.append("--require-validation")
            train_outputs = outputs_by_pass[0]
            for axis, output in train_outputs.items():
                fit_args.extend([f"--{axis}-csv", str(output)])
            if len(outputs_by_pass) > 1:
                for axis, output in outputs_by_pass[1].items():
                    fit_args.extend([f"--validate-{axis}-csv", str(output)])
            if args.backup:
                fit_args.append("--backup")
            print("[fit] {}".format(" ".join(fit_args)), flush=True)
            fit_rc = fit_rmtt_model.main(fit_args)
            if fit_rc != 0:
                result_code = fit_rc
            elif args.quality_gate:
                result_code = _check_model_quality(args)
            else:
                result_code = 0
        else:
            print("outputs:", flush=True)
            for pass_index, pass_outputs in enumerate(outputs_by_pass):
                print("  {0}:".format(_pass_label(pass_index, args.pass_count)), flush=True)
                for axis, output in pass_outputs.items():
                    print("    {0}: {1}".format(axis, output), flush=True)
        return result_code
    finally:
        if args.send:
            _safe_action("center sticks", _settle, args.ip, 0.2)
            handoff_ok = args.allow_open_airborne_handoff and result_code == 0
            if airborne and not landed and (args.land or not handoff_ok):
                _safe_action("emergency land", _land, args.ip)


def _parse_axes(value: str) -> tuple[str, ...]:
    axes = tuple(part.strip().lower() for part in value.split(",") if part.strip())
    invalid = [axis for axis in axes if axis not in AXES]
    if invalid:
        raise ValueError("invalid axis {0}; expected one of {1}".format(invalid[0], ",".join(AXES)))
    if not axes:
        raise ValueError("at least one axis is required")
    return axes


def _pass_label(pass_index: int, pass_count: int) -> str:
    if pass_count <= 1:
        return "single"
    return "train" if pass_index == 0 else "validate"


def _check_csv_quality(axis: str, output: Path, args: argparse.Namespace) -> int:
    thresholds = identification_quality.CsvQualityThresholds(
        min_rows=args.min_csv_rows,
        min_signed_samples=args.min_signed_samples,
        min_position_span=args.min_position_span,
        min_yaw_span_deg=args.min_yaw_span_deg,
        max_safety_fail_ratio=args.max_safety_fail_ratio,
    )
    result = identification_quality.check_identification_csv(
        output,
        axis=axis,
        thresholds=thresholds,
    )
    status = "OK" if result.ok else "FAIL"
    print(
        "[csv-quality] {status}: {axis} rows={rows} safe={safe} +={pos} -={neg} span={span:.4g}".format(
            status=status,
            axis=axis,
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
    return 0 if result.ok else 2


def _check_model_quality(args: argparse.Namespace) -> int:
    thresholds = model_quality.QualityThresholds(
        min_samples=args.quality_min_samples,
        min_r2=args.quality_min_r2,
        min_vaf=args.quality_min_vaf,
        max_nrmse=args.quality_max_nrmse,
        fail_on_bootstrap=args.quality_fail_on_bootstrap,
        require_validation=args.quality_require_validation,
    )
    results = model_quality.check_model_quality(args.model_output, thresholds=thresholds)
    failed = False
    for result in results:
        status = "OK" if result.ok else "FAIL"
        print("[model-quality] {0}: {1}".format(status, result.axis), flush=True)
        for warning in result.warnings:
            print("  WARN: {0}".format(warning), flush=True)
        for failure in result.failures:
            print("  FAIL: {0}".format(failure), flush=True)
        failed = failed or not result.ok
    return 1 if failed else 0


def _settle(ip: str, settle_sec: float) -> None:
    if settle_sec <= 0.0:
        return
    client = RMTTClient(ip)
    try:
        client.connect()
        client.center()
        time.sleep(settle_sec)
        client.center()
    finally:
        client.close()


def _takeoff(ip: str) -> None:
    client = RMTTClient(ip)
    try:
        client.connect()
        client.takeoff().wait_for_completed()
        time.sleep(1.0)
    finally:
        client.close()


def _land(ip: str) -> None:
    client = RMTTClient(ip)
    try:
        client.connect()
        client.center()
        client.land().wait_for_completed()
    finally:
        client.close()


def _safe_action(label: str, func, *args) -> bool:
    try:
        func(*args)
        return True
    except Exception as exc:
        print("WARN: {0} failed during cleanup: {1}".format(label, exc), flush=True)
        return False


if __name__ == "__main__":
    sys.exit(main())
