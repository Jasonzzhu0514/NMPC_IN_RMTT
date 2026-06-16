#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

from rmtt_control.model_quality import QualityThresholds, check_model_quality
from models.dji_velocity_model import VALID_MODEL_AXES, load_velocity_model
from rmtt.battery import DEFAULT_DRONE_CHECK_TIMEOUT_SEC, read_drone_battery_isolated
from rmtt_control.vrpn_pose_reader import DEFAULT_HOST, DEFAULT_PORT, DEFAULT_TRACKER, VrpnPoseReader
from rmtt_config import DEFAULT_RMTT_IP


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "models" / "rmtt_velocity_model.json"
DEFAULT_NATIVE_VRPN_HELPER = ROOT / "native" / "vrpn_pose_json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Preflight checks for RMTT NMPC scripts.")
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--ip", default=DEFAULT_RMTT_IP)
    parser.add_argument("--min-battery", type=int, default=30)
    parser.add_argument("--check-model-quality", action="store_true")
    parser.add_argument("--fail-on-bootstrap", action="store_true")
    parser.add_argument("--min-fit-samples", type=int, default=30)
    parser.add_argument("--min-r2", type=float, default=0.20)
    parser.add_argument("--min-vaf", type=float, default=0.20)
    parser.add_argument("--max-nrmse", type=float, default=0.80)
    parser.add_argument("--check-drone", action="store_true", help="connect to RMTT and read battery")
    parser.add_argument("--drone-check-timeout", type=float, default=DEFAULT_DRONE_CHECK_TIMEOUT_SEC)
    parser.add_argument("--check-vrpn-helper", action="store_true", help="verify native VRPN helper exists and is executable")
    parser.add_argument("--check-vrpn", action="store_true", help="connect to VRPN and wait for one pose")
    parser.add_argument("--tracker", default=DEFAULT_TRACKER)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--method", choices=("auto", "native", "print"), default="auto")
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--invert-yaw", action="store_true")
    parser.add_argument("--wait-timeout", type=float, default=5.0)
    args = parser.parse_args(argv)

    failures: list[str] = []
    warnings: list[str] = []

    _check_model(Path(args.model), failures=failures, warnings=warnings)
    if args.check_model_quality:
        _check_model_quality(args, failures=failures, warnings=warnings)
    if args.check_vrpn_helper:
        _check_vrpn_helper(failures=failures)
    if args.check_vrpn:
        _check_vrpn(args, failures=failures)
    if args.check_drone:
        _check_drone(args, failures=failures, warnings=warnings)

    for warning in warnings:
        print("WARN: {0}".format(warning), flush=True)
    for failure in failures:
        print("FAIL: {0}".format(failure), flush=True)
    if failures:
        return 1
    print("OK: preflight checks passed", flush=True)
    return 0


def _check_model(path: Path, *, failures: list[str], warnings: list[str]) -> None:
    try:
        model = load_velocity_model(path)
        model.require_axes(VALID_MODEL_AXES)
    except Exception as exc:
        failures.append("model load failed: {0}".format(exc))
        return

    print("model: {0}".format(path), flush=True)
    for axis in VALID_MODEL_AXES:
        axis_model = model.axis(axis)
        values = {
            "K": axis_model.K,
            "tau": axis_model.tau,
            "Td": axis_model.Td,
            "vmax": axis_model.vmax,
            "amax": axis_model.amax,
        }
        bad = [name for name, value in values.items() if not math.isfinite(value)]
        if bad:
            failures.append("{0} model has non-finite fields: {1}".format(axis, ", ".join(bad)))
        if axis_model.tau <= 0.0 or axis_model.vmax <= 0.0 or axis_model.amax <= 0.0:
            failures.append("{0} model has non-positive dynamic limits".format(axis))
        if axis_model.fit.get("bootstrap") is True:
            warnings.append("{0} axis still uses bootstrap model".format(axis))
        print(
            "  {axis}: K={K:.4g} tau={tau:.3g} Td={Td:.3g} vmax={vmax:.3g} amax={amax:.3g}".format(
                axis=axis,
                K=axis_model.K,
                tau=axis_model.tau,
                Td=axis_model.Td,
                vmax=axis_model.vmax,
                amax=axis_model.amax,
            ),
            flush=True,
        )


def _check_model_quality(args: argparse.Namespace, *, failures: list[str], warnings: list[str]) -> None:
    thresholds = QualityThresholds(
        min_samples=args.min_fit_samples,
        min_r2=args.min_r2,
        min_vaf=args.min_vaf,
        max_nrmse=args.max_nrmse,
        fail_on_bootstrap=args.fail_on_bootstrap,
    )
    try:
        results = check_model_quality(Path(args.model), thresholds=thresholds)
    except Exception as exc:
        failures.append("model quality check failed: {0}".format(exc))
        return
    for result in results:
        for warning in result.warnings:
            warnings.append("{0}: {1}".format(result.axis, warning))
        for failure in result.failures:
            failures.append("{0}: {1}".format(result.axis, failure))


def _check_vrpn(args: argparse.Namespace, *, failures: list[str]) -> None:
    reader = VrpnPoseReader(
        tracker=args.tracker,
        host=args.host,
        port=args.port,
        method=args.method,
        z_offset=args.z_offset,
        invert_yaw=args.invert_yaw,
    )
    try:
        reader.connect(wait_timeout=args.wait_timeout)
        sample = reader.latest()
        if sample is None:
            failures.append("VRPN connected but no sample was available")
            return
        age = time.time() - sample.timestamp
        print(
            "vrpn: x={0:.3f} y={1:.3f} z={2:.3f} yaw={3:.3f} age={4:.3f}s".format(
                sample.x,
                sample.y,
                sample.z,
                sample.yaw,
                age,
            ),
            flush=True,
        )
    except Exception as exc:
        failures.append("VRPN check failed: {0}".format(exc))
    finally:
        reader.close()


def _check_vrpn_helper(*, failures: list[str]) -> None:
    path = DEFAULT_NATIVE_VRPN_HELPER
    if not path.exists():
        failures.append(
            "native VRPN helper missing: {0}; run ./build_vrpn_helper.sh".format(path)
        )
        return
    if not path.is_file():
        failures.append("native VRPN helper is not a file: {0}".format(path))
        return
    if not path.stat().st_mode & 0o111:
        failures.append(
            "native VRPN helper is not executable: {0}; run chmod +x or rebuild".format(path)
        )
        return
    print("vrpn-helper: {0}".format(path), flush=True)


def _check_drone(args: argparse.Namespace, *, failures: list[str], warnings: list[str]) -> None:
    status, payload = read_drone_battery_isolated(args.ip, timeout=float(args.drone_check_timeout))
    if status == "timeout":
        failures.append("drone check timed out after {0:.1f}s".format(float(args.drone_check_timeout)))
        return
    if status == "error":
        failures.append("drone check failed: {0}".format(payload))
        return
    battery = payload
    print("drone: ip={0} battery={1}%".format(args.ip, battery), flush=True)
    if battery is None:
        warnings.append("battery read returned None")
    elif battery < args.min_battery:
        failures.append("battery {0}% is below minimum {1}%".format(battery, args.min_battery))


if __name__ == "__main__":
    sys.exit(main())
