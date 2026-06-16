#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
import sys

from rmtt_control import identification_quality
from rmtt_control import model_quality
from nmpc.identification.protocol import RMTT_MAX_IDENTIFICATION_AMPLITUDE
from runtime.xyz.mission import load_waypoints


AXES = ("pitch", "roll", "throttle", "yaw")
RMTT_STICK_LIMIT = 100
XYZWAY_RC_COLUMNS = ("roll", "pitch", "throttle", "yaw_cmd")


@dataclass(frozen=True)
class EvidenceResult:
    ok: bool
    failures: tuple[str, ...]
    warnings: tuple[str, ...]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check artifacts from an RMTT NMPC workflow run.")
    parser.add_argument("manifest", help="workflow_manifest.json path")
    parser.add_argument("--min-csv-rows", type=int, default=30)
    parser.add_argument("--min-signed-samples", type=int, default=10)
    parser.add_argument("--min-position-span", type=float, default=0.05)
    parser.add_argument("--min-yaw-span-deg", type=float, default=5.0)
    parser.add_argument("--max-safety-fail-ratio", type=float, default=0.0)
    parser.add_argument("--quality-min-samples", type=int, default=30)
    parser.add_argument("--quality-min-r2", type=float, default=0.20)
    parser.add_argument("--quality-min-vaf", type=float, default=0.20)
    parser.add_argument("--quality-max-nrmse", type=float, default=0.80)
    parser.add_argument("--min-xyzway-rows", type=int, default=1)
    parser.add_argument("--arrival-radius", type=float, default=0.10)
    parser.add_argument("--z-radius", type=float, default=0.08)
    parser.add_argument("--yaw-radius-deg", type=float, default=8.0)
    parser.add_argument("--require-send", action="store_true")
    parser.add_argument("--require-vrpn-check", action="store_true")
    parser.add_argument("--require-drone-check", action="store_true")
    parser.add_argument("--require-full-workflow", action="store_true")
    parser.add_argument("--require-managed-flight", action="store_true")
    parser.add_argument("--summary", action="store_true", help="print a concise artifact summary after checking")
    args = parser.parse_args(argv)

    result = check_workflow_evidence(
        args.manifest,
        csv_thresholds=identification_quality.CsvQualityThresholds(
            min_rows=args.min_csv_rows,
            min_signed_samples=args.min_signed_samples,
            min_position_span=args.min_position_span,
            min_yaw_span_deg=args.min_yaw_span_deg,
            max_safety_fail_ratio=args.max_safety_fail_ratio,
        ),
        model_thresholds=model_quality.QualityThresholds(
            min_samples=args.quality_min_samples,
            min_r2=args.quality_min_r2,
            min_vaf=args.quality_min_vaf,
            max_nrmse=args.quality_max_nrmse,
            fail_on_bootstrap=True,
        ),
        min_xyzway_rows=args.min_xyzway_rows,
        arrival_radius=args.arrival_radius,
        z_radius=args.z_radius,
        yaw_radius_deg=args.yaw_radius_deg,
        require_send=args.require_send,
        require_vrpn_check=args.require_vrpn_check,
        require_drone_check=args.require_drone_check,
        require_full_workflow=args.require_full_workflow,
        require_managed_flight=args.require_managed_flight,
    )
    for warning in result.warnings:
        print("WARN: {0}".format(warning), flush=True)
    for failure in result.failures:
        print("FAIL: {0}".format(failure), flush=True)
    if args.summary:
        print_workflow_summary(args.manifest)
    if not result.ok:
        return 1
    print("OK: workflow evidence passed", flush=True)
    return 0


def print_workflow_summary(manifest_path: str | Path) -> None:
    failures: list[str] = []
    path = Path(manifest_path)
    manifest = _load_manifest(path, failures=failures)
    if manifest is None:
        print("summary: unavailable ({0})".format("; ".join(failures)), flush=True)
        return
    manifest_dir = path.resolve().parent
    artifacts = manifest.get("artifacts") or {}
    stages = tuple(manifest.get("stages") or ())
    print("summary:", flush=True)
    print("  manifest: {0}".format(path), flush=True)
    print("  status: {0}".format(manifest.get("status")), flush=True)
    print("  stages: {0}".format(",".join(stages)), flush=True)
    print(
        "  run: send={0} takeoff={1} land={2}".format(
            bool(manifest.get("send")),
            bool(manifest.get("takeoff")),
            bool(manifest.get("land")),
        ),
        flush=True,
    )
    for item in manifest.get("results") or ():
        print(
            "  stage {0}: returncode={1} log={2}".format(
                item.get("stage"),
                item.get("returncode"),
                item.get("log"),
            ),
            flush=True,
        )
    _print_identification_summary(artifacts, manifest_dir=manifest_dir)
    model = artifacts.get("model")
    if model:
        print("  model: {0}".format(_resolve_manifest_path(model, manifest_dir)), flush=True)
    _print_xyzway_summary(artifacts, manifest_dir=manifest_dir)


def check_workflow_evidence(
    manifest_path: str | Path,
    *,
    csv_thresholds: identification_quality.CsvQualityThresholds | None = None,
    model_thresholds: model_quality.QualityThresholds | None = None,
    min_xyzway_rows: int = 1,
    arrival_radius: float = 0.10,
    z_radius: float = 0.08,
    yaw_radius_deg: float = 8.0,
    require_send: bool = False,
    require_vrpn_check: bool = False,
    require_drone_check: bool = False,
    require_full_workflow: bool = False,
    require_managed_flight: bool = False,
) -> EvidenceResult:
    csv_thresholds = csv_thresholds or identification_quality.CsvQualityThresholds()
    model_thresholds = model_thresholds or model_quality.QualityThresholds(fail_on_bootstrap=True)
    manifest_path = Path(manifest_path)
    failures: list[str] = []
    warnings: list[str] = []
    manifest = _load_manifest(manifest_path, failures=failures)
    if manifest is None:
        return EvidenceResult(False, tuple(failures), tuple(warnings))

    manifest_dir = manifest_path.resolve().parent
    artifacts = manifest.get("artifacts") or {}
    stages = tuple(manifest.get("stages") or ())
    if manifest.get("status") != "ok":
        failures.append("workflow status is {0!r}, expected 'ok'".format(manifest.get("status")))
    _check_required_run_mode(
        manifest,
        require_send=require_send,
        require_vrpn_check=require_vrpn_check,
        require_drone_check=require_drone_check,
        require_full_workflow=require_full_workflow,
        require_managed_flight=require_managed_flight,
        failures=failures,
    )
    _check_stage_results(manifest, failures=failures)
    _check_stage_logs(manifest, artifacts, manifest_dir=manifest_dir, failures=failures)
    _check_required_preflight_log_evidence(
        artifacts,
        manifest_dir=manifest_dir,
        require_vrpn_check=require_vrpn_check,
        require_drone_check=require_drone_check,
        failures=failures,
    )
    if "identify" in stages:
        _check_identification_artifacts(
            artifacts,
            csv_thresholds,
            manifest_dir=manifest_dir,
            failures=failures,
            warnings=warnings,
        )
    if "identify" in stages or "xyzway" in stages:
        _check_model_artifact(
            artifacts,
            model_thresholds,
            manifest_dir=manifest_dir,
            failures=failures,
            warnings=warnings,
        )
    if "identify" in stages:
        _check_model_sources(
            artifacts,
            manifest_dir=manifest_dir,
            failures=failures,
        )
    if "identify" in stages:
        _check_comparison_artifacts(
            artifacts,
            manifest_dir=manifest_dir,
            failures=failures,
        )
    if "xyzway" in stages:
        _check_xyzway_artifact(
            artifacts,
            manifest_dir=manifest_dir,
            min_rows=min_xyzway_rows,
            arrival_radius=arrival_radius,
            z_radius=z_radius,
            yaw_radius_deg=yaw_radius_deg,
            waypoint_path=manifest.get("waypoints"),
            failures=failures,
        )

    return EvidenceResult(not failures, tuple(failures), tuple(warnings))


def _load_manifest(path: Path, *, failures: list[str]) -> dict | None:
    try:
        with path.open() as file:
            return json.load(file)
    except Exception as exc:
        failures.append("manifest load failed: {0}".format(exc))
        return None


def _check_required_run_mode(
    manifest: dict,
    *,
    require_send: bool,
    require_vrpn_check: bool,
    require_drone_check: bool,
    require_full_workflow: bool,
    require_managed_flight: bool,
    failures: list[str],
) -> None:
    if require_full_workflow:
        stages = tuple(manifest.get("stages") or ())
        expected = ("preflight", "identify", "xyzway")
        if stages != expected:
            failures.append("workflow stages {0} do not match required full workflow {1}".format(stages, expected))
    if require_send and manifest.get("send") is not True:
        failures.append("workflow was not run with --send")
    if require_vrpn_check and not _stage_command_has_flag(manifest, "preflight", "--check-vrpn"):
        failures.append("preflight command did not include --check-vrpn")
    if require_drone_check and not _stage_command_has_flag(manifest, "preflight", "--check-drone"):
        failures.append("preflight command did not include --check-drone")
    if require_managed_flight:
        _check_required_managed_flight(manifest, failures=failures)


def _check_required_managed_flight(manifest: dict, *, failures: list[str]) -> None:
    if manifest.get("takeoff") is not True:
        failures.append("workflow was not run with --takeoff")
    if manifest.get("land") is not True:
        failures.append("workflow was not run with --land")
    if not _stage_command_has_flag(manifest, "identify", "--takeoff"):
        failures.append("identify command did not include --takeoff")
    if not _stage_command_has_flag(manifest, "identify", "--allow-open-airborne-handoff"):
        failures.append("identify command did not include --allow-open-airborne-handoff")
    if _stage_command_has_flag(manifest, "identify", "--land"):
        failures.append("identify command unexpectedly included --land during managed handoff")
    if _stage_command_has_flag(manifest, "xyzway", "--takeoff"):
        failures.append("xyzway command unexpectedly included --takeoff during managed handoff")
    if not _stage_command_has_flag(manifest, "xyzway", "--land"):
        failures.append("xyzway command did not include --land")


def _check_required_preflight_log_evidence(
    artifacts: dict,
    *,
    manifest_dir: Path,
    require_vrpn_check: bool,
    require_drone_check: bool,
    failures: list[str],
) -> None:
    if not (require_vrpn_check or require_drone_check):
        return
    path_value = artifacts.get("preflight_log")
    if not path_value:
        failures.append("missing preflight_log artifact")
        return
    path = _resolve_manifest_path(path_value, manifest_dir)
    if not path.exists() or not path.is_file():
        return
    text = path.read_text(errors="ignore")
    if require_vrpn_check and "vrpn: x=" not in text:
        failures.append("preflight log missing VRPN sample output")
    if require_drone_check and ("drone: ip=" not in text or " battery=" not in text):
        failures.append("preflight log missing drone battery output")


def _stage_command_has_flag(manifest: dict, stage: str, flag: str) -> bool:
    for item in manifest.get("commands") or ():
        if item.get("stage") == stage and flag in (item.get("argv") or ()):
            return True
    return False


def _check_stage_results(manifest: dict, *, failures: list[str]) -> None:
    stages = tuple(manifest.get("stages") or ())
    results = manifest.get("results") or []
    result_stages = tuple(item.get("stage") for item in results)
    if result_stages != stages:
        failures.append("stage results {0} do not match stages {1}".format(result_stages, stages))
    for item in results:
        if item.get("returncode") != 0:
            failures.append("stage {0} returned {1}".format(item.get("stage"), item.get("returncode")))


def _check_stage_logs(manifest: dict, artifacts: dict, *, manifest_dir: Path, failures: list[str]) -> None:
    for stage in manifest.get("stages") or ():
        paths = []
        artifact_log = artifacts.get("{0}_log".format(stage))
        if artifact_log:
            paths.append(_resolve_manifest_path(artifact_log, manifest_dir))
        for item in manifest.get("results") or ():
            if item.get("stage") == stage and item.get("log"):
                paths.append(_resolve_manifest_path(item["log"], manifest_dir))
        if not paths:
            failures.append("missing log path for stage {0}".format(stage))
            continue
        for path in paths:
            _check_nonempty_file(path, "stage log {0}".format(stage), failures=failures)


def _check_identification_artifacts(
    artifacts: dict,
    thresholds: identification_quality.CsvQualityThresholds,
    *,
    manifest_dir: Path,
    failures: list[str],
    warnings: list[str],
) -> None:
    directory = artifacts.get("identification_dir")
    if not directory:
        failures.append("missing identification_dir artifact")
        return
    base = _resolve_manifest_path(directory, manifest_dir)
    if not base.is_dir():
        failures.append("identification_dir is missing: {0}".format(base))
        return
    for axis in AXES:
        matches = sorted(base.glob("identify_{0}_*.csv".format(axis)))
        if not matches:
            failures.append("missing identification CSV for {0}".format(axis))
            continue
        result = identification_quality.check_identification_csv(matches[-1], axis=axis, thresholds=thresholds)
        for warning in result.warnings:
            warnings.append("{0}: {1}".format(axis, warning))
        for failure in result.failures:
            failures.append("{0}: {1}".format(axis, failure))
        _check_identification_stick_limit(
            matches[-1],
            axis=axis,
            limit=RMTT_MAX_IDENTIFICATION_AMPLITUDE,
            failures=failures,
        )


def _print_identification_summary(artifacts: dict, *, manifest_dir: Path) -> None:
    directory = artifacts.get("identification_dir")
    if not directory:
        return
    base = _resolve_manifest_path(directory, manifest_dir)
    print("  identification_dir: {0}".format(base), flush=True)
    if not base.is_dir():
        return
    for axis in AXES:
        matches = sorted(base.glob("identify_{0}_*.csv".format(axis)))
        if not matches:
            print("  identify {0}: missing".format(axis), flush=True)
            continue
        path = matches[-1]
        rows = _csv_data_row_count(path)
        print("  identify {0}: rows={1} csv={2}".format(axis, rows, path), flush=True)


def _check_identification_stick_limit(
    path: Path,
    *,
    axis: str,
    limit: int,
    failures: list[str],
) -> None:
    columns = (
        identification_quality.REQUESTED_COLUMNS[axis],
        identification_quality.APPLIED_COLUMNS[axis],
    )
    try:
        with path.open() as file:
            reader = csv.DictReader(file)
            for index, row in enumerate(reader, start=2):
                if (row.get("signal_kind") or "").strip().lower() == "recenter":
                    continue
                if (row.get("axis") or "").strip().lower() != axis:
                    continue
                for column in columns:
                    value = _float_or_none(row.get(column))
                    if value is None:
                        continue
                    if abs(value) > limit:
                        failures.append(
                            "{0}: {1} row {2} {3}={4:.4g} exceeds identification limit {5}".format(
                                axis,
                                path.name,
                                index,
                                column,
                                value,
                                limit,
                            )
                        )
                        return
    except Exception as exc:
        failures.append("{0}: stick limit check failed: {1}".format(axis, exc))


def _check_model_artifact(
    artifacts: dict,
    thresholds: model_quality.QualityThresholds,
    *,
    manifest_dir: Path,
    failures: list[str],
    warnings: list[str],
) -> None:
    model = artifacts.get("model")
    if not model:
        failures.append("missing model artifact")
        return
    path = _resolve_manifest_path(model, manifest_dir)
    _check_nonempty_file(path, "model", failures=failures)
    if not path.exists():
        return
    for result in model_quality.check_model_quality(path, thresholds=thresholds):
        for warning in result.warnings:
            warnings.append("{0}: {1}".format(result.axis, warning))
        for failure in result.failures:
            failures.append("{0}: {1}".format(result.axis, failure))


def _check_model_sources(
    artifacts: dict,
    *,
    manifest_dir: Path,
    failures: list[str],
) -> None:
    model = artifacts.get("model")
    identification_dir = artifacts.get("identification_dir")
    if not model or not identification_dir:
        return
    model_path = _resolve_manifest_path(model, manifest_dir)
    identification_path = _resolve_manifest_path(identification_dir, manifest_dir)
    if not model_path.exists() or not identification_path.is_dir():
        return
    try:
        with model_path.open() as file:
            document = json.load(file)
    except Exception as exc:
        failures.append("model source check failed: {0}".format(exc))
        return
    axes_doc = document.get("axes") or {}
    for axis in AXES:
        fit = (axes_doc.get(axis) or {}).get("fit") or {}
        source_csv = fit.get("source_csv")
        if not source_csv:
            failures.append("{0}: model fit.source_csv missing".format(axis))
            continue
        values = source_csv if isinstance(source_csv, list) else [source_csv]
        expected = {path.resolve() for path in identification_path.glob("identify_{0}_*.csv".format(axis))}
        sources = {
            _resolve_manifest_path(str(value), manifest_dir).resolve()
            for value in values
        }
        if not expected.intersection(sources):
            failures.append("{0}: model fit.source_csv does not reference this workflow identification CSV".format(axis))


def _check_comparison_artifacts(
    artifacts: dict,
    *,
    manifest_dir: Path,
    failures: list[str],
) -> None:
    directory = artifacts.get("comparison_dir")
    if not directory:
        failures.append("missing comparison_dir artifact")
        return
    base = _resolve_manifest_path(directory, manifest_dir)
    if not base.is_dir():
        failures.append("comparison_dir is missing: {0}".format(base))
        return
    required_header = ("t", "u", "actual", "predicted", "residual")
    for axis in AXES:
        path = base / "{0}_comparison.csv".format(axis)
        _check_nonempty_file(path, "{0} comparison CSV".format(axis), failures=failures)
        if not path.exists() or not path.is_file():
            continue
        with path.open() as file:
            reader = csv.reader(file)
            header = tuple(next(reader, ()))
            first_row = next(reader, None)
        if header != required_header:
            failures.append(
                "{0} comparison CSV header {1} != {2}".format(
                    axis,
                    header,
                    required_header,
                )
            )
        if first_row is None:
            failures.append("{0} comparison CSV has no data rows".format(axis))


def _check_xyzway_artifact(
    artifacts: dict,
    *,
    manifest_dir: Path,
    min_rows: int,
    arrival_radius: float,
    z_radius: float,
    yaw_radius_deg: float,
    waypoint_path: str | None,
    failures: list[str],
) -> None:
    path_value = artifacts.get("xyzway_log_csv")
    if not path_value:
        failures.append("missing xyzway_log_csv artifact")
        return
    path = _resolve_manifest_path(path_value, manifest_dir)
    _check_nonempty_file(path, "xyzway log CSV", failures=failures)
    if not path.exists():
        return
    with path.open() as file:
        reader = csv.DictReader(file)
        rows = list(reader)
    if len(rows) < min_rows:
        failures.append("xyzway rows {0} < {1}".format(len(rows), min_rows))
    if rows and "nmpc_json" not in rows[0]:
        failures.append("xyzway log missing nmpc_json column")
    if rows and "nmpc_json" in rows[0]:
        _check_xyzway_nmpc_json(rows, failures=failures)
    if rows:
        _check_xyzway_stick_limits(rows, failures=failures)
    if rows:
        _check_xyzway_terminal_row(
            rows[-1],
            manifest_dir=manifest_dir,
            arrival_radius=arrival_radius,
            z_radius=z_radius,
            yaw_radius_deg=yaw_radius_deg,
            waypoint_path=waypoint_path,
            failures=failures,
        )


def _print_xyzway_summary(artifacts: dict, *, manifest_dir: Path) -> None:
    path_value = artifacts.get("xyzway_log_csv")
    if not path_value:
        return
    path = _resolve_manifest_path(path_value, manifest_dir)
    print("  xyzway_log_csv: {0}".format(path), flush=True)
    if not path.exists() or not path.is_file():
        return
    try:
        with path.open() as file:
            rows = list(csv.DictReader(file))
    except Exception as exc:
        print("  xyzway: unreadable ({0})".format(exc), flush=True)
        return
    if not rows:
        print("  xyzway: rows=0", flush=True)
        return
    row = rows[-1]
    try:
        target_x = float(row["target_x"])
        target_y = float(row["target_y"])
        target_z = float(row["target_z"])
        target_yaw = float(row["target_yaw_deg"])
        x = float(row["x"])
        y = float(row["y"])
        z = float(row["z"])
        yaw = float(row["yaw"])
    except (KeyError, ValueError) as exc:
        print("  xyzway: rows={0} terminal=unavailable ({1})".format(len(rows), exc), flush=True)
        return
    yaw_deg = math.degrees(yaw) if abs(yaw) <= 6.5 else yaw
    xy_error = math.hypot(x - target_x, y - target_y)
    z_error = abs(z - target_z)
    yaw_error = abs(((target_yaw - yaw_deg + 180.0) % 360.0) - 180.0)
    print(
        "  xyzway: rows={rows} final=({x:.3f},{y:.3f},{z:.3f},{yaw:.1f}deg) "
        "target=({tx:.3f},{ty:.3f},{tz:.3f},{tyaw:.1f}deg) "
        "error_xy={xy:.3f} error_z={ze:.3f} error_yaw={ye:.1f}deg".format(
            rows=len(rows),
            x=x,
            y=y,
            z=z,
            yaw=yaw_deg,
            tx=target_x,
            ty=target_y,
            tz=target_z,
            tyaw=target_yaw,
            xy=xy_error,
            ze=z_error,
            ye=yaw_error,
        ),
        flush=True,
    )


def _check_xyzway_nmpc_json(rows: list[dict], *, failures: list[str]) -> None:
    for index, row in enumerate(rows, start=2):
        raw = row.get("nmpc_json")
        try:
            payload = json.loads(raw or "")
        except json.JSONDecodeError as exc:
            failures.append("xyzway row {0} nmpc_json is invalid JSON: {1}".format(index, exc))
            return
        if not isinstance(payload, dict):
            failures.append("xyzway row {0} nmpc_json is not an object".format(index))
            return
        if "nmpc_mission_reason" not in payload and "nmpc_flight_reason" not in payload:
            failures.append("xyzway row {0} nmpc_json missing NMPC reason".format(index))
            return


def _check_xyzway_stick_limits(rows: list[dict], *, failures: list[str]) -> None:
    for index, row in enumerate(rows, start=2):
        for column in XYZWAY_RC_COLUMNS:
            value = _float_or_none(row.get(column))
            if value is None:
                failures.append("xyzway row {0} has non-numeric {1}".format(index, column))
                return
            if abs(value) > RMTT_STICK_LIMIT:
                failures.append(
                    "xyzway row {0} {1}={2:.4g} exceeds RMTT stick limit {3}".format(
                        index,
                        column,
                        value,
                        RMTT_STICK_LIMIT,
                    )
                )
                return


def _check_xyzway_terminal_row(
    row: dict,
    *,
    manifest_dir: Path,
    arrival_radius: float,
    z_radius: float,
    yaw_radius_deg: float,
    waypoint_path: str | None,
    failures: list[str],
) -> None:
    required = ("target_x", "target_y", "target_z", "target_yaw_deg", "x", "y", "z", "yaw")
    missing = [name for name in required if row.get(name) in (None, "")]
    if missing:
        failures.append("xyzway terminal row missing columns: {0}".format(", ".join(missing)))
        return
    try:
        target_x = float(row["target_x"])
        target_y = float(row["target_y"])
        target_z = float(row["target_z"])
        target_yaw = float(row["target_yaw_deg"])
        x = float(row["x"])
        y = float(row["y"])
        z = float(row["z"])
        yaw = float(row["yaw"])
    except ValueError as exc:
        failures.append("xyzway terminal row has non-numeric pose/target: {0}".format(exc))
        return
    xy_error = math.hypot(x - target_x, y - target_y)
    z_error = abs(z - target_z)
    yaw_deg = math.degrees(yaw) if abs(yaw) <= 6.5 else yaw
    yaw_error = abs(((target_yaw - yaw_deg + 180.0) % 360.0) - 180.0)
    if xy_error > arrival_radius:
        failures.append("xyzway terminal xy error {0:.3f} > {1:.3f}".format(xy_error, arrival_radius))
    if z_error > z_radius:
        failures.append("xyzway terminal z error {0:.3f} > {1:.3f}".format(z_error, z_radius))
    if yaw_error > yaw_radius_deg:
        failures.append("xyzway terminal yaw error {0:.3f} > {1:.3f}".format(yaw_error, yaw_radius_deg))
    if waypoint_path:
        _check_manifest_final_waypoint(
            waypoint_path,
            manifest_dir=manifest_dir,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_yaw=target_yaw,
            x=x,
            y=y,
            z=z,
            yaw_deg=yaw_deg,
            arrival_radius=arrival_radius,
            z_radius=z_radius,
            yaw_radius_deg=yaw_radius_deg,
            failures=failures,
        )


def _check_manifest_final_waypoint(
    waypoint_path: str,
    *,
    manifest_dir: Path,
    target_x: float,
    target_y: float,
    target_z: float,
    target_yaw: float,
    x: float,
    y: float,
    z: float,
    yaw_deg: float,
    arrival_radius: float,
    z_radius: float,
    yaw_radius_deg: float,
    failures: list[str],
) -> None:
    try:
        waypoints = load_waypoints(_resolve_manifest_path(waypoint_path, manifest_dir))
    except Exception as exc:
        failures.append("waypoint file load failed: {0}".format(exc))
        return
    if not waypoints:
        failures.append("waypoint file is empty")
        return
    waypoint = waypoints[-1]
    target_xy_error = math.hypot(target_x - waypoint.x, target_y - waypoint.y)
    target_z_error = abs(target_z - waypoint.z)
    target_yaw_error = abs(((waypoint.yaw_deg - target_yaw + 180.0) % 360.0) - 180.0)
    pose_xy_error = math.hypot(x - waypoint.x, y - waypoint.y)
    pose_z_error = abs(z - waypoint.z)
    pose_yaw_error = abs(((waypoint.yaw_deg - yaw_deg + 180.0) % 360.0) - 180.0)
    if target_xy_error > 1e-6 or target_z_error > 1e-6 or target_yaw_error > 1e-6:
        failures.append("xyzway terminal target does not match final waypoint")
    if pose_xy_error > arrival_radius:
        failures.append("xyzway final waypoint xy error {0:.3f} > {1:.3f}".format(pose_xy_error, arrival_radius))
    if pose_z_error > z_radius:
        failures.append("xyzway final waypoint z error {0:.3f} > {1:.3f}".format(pose_z_error, z_radius))
    if pose_yaw_error > yaw_radius_deg:
        failures.append("xyzway final waypoint yaw error {0:.3f} > {1:.3f}".format(pose_yaw_error, yaw_radius_deg))


def _check_nonempty_file(path: Path, label: str, *, failures: list[str]) -> None:
    if not path.exists():
        failures.append("{0} missing: {1}".format(label, path))
    elif not path.is_file():
        failures.append("{0} is not a file: {1}".format(label, path))
    elif path.stat().st_size <= 0:
        failures.append("{0} is empty: {1}".format(label, path))


def _csv_data_row_count(path: Path) -> int | str:
    try:
        with path.open() as file:
            reader = csv.reader(file)
            next(reader, None)
            return sum(1 for _ in reader)
    except Exception as exc:
        return "unreadable:{0}".format(exc)


def _float_or_none(value) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _resolve_manifest_path(value: str | Path, manifest_dir: Path) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return manifest_dir / path


if __name__ == "__main__":
    sys.exit(main())
