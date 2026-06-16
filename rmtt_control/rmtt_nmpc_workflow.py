#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Orchestrate the RMTT NMPC flow from checks through identification to xyzway."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path
import subprocess
import sys
import time

from rmtt_control import fit_rmtt_model
from rmtt_control.vrpn_pose_reader import DEFAULT_HOST, DEFAULT_PORT, DEFAULT_TRACKER
from rmtt_control import workflow_evidence
from rmtt_config import DEFAULT_RMTT_IP


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = fit_rmtt_model.DEFAULT_MODEL
DEFAULT_WAYPOINTS = ROOT / "example_waypoints.json"
STAGES = ("preflight", "identify", "xyzway")
WORKFLOW_MANIFEST = "workflow_manifest.json"


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.send and not args.confirm_risk:
        print("Refusing to send stick commands without --confirm-risk.", flush=True)
        return 2
    if args.send and args.takeoff and not args.land:
        print("Refusing workflow --takeoff without --land.", flush=True)
        return 2

    work_dir = Path(args.work_dir or _default_work_dir()).expanduser()
    stages = _parse_stages(args.stages)

    commands = build_commands(args, work_dir=work_dir, stages=stages)
    print("workflow work_dir: {0}".format(work_dir), flush=True)
    if not args.print_only:
        work_dir.mkdir(parents=True, exist_ok=True)
        manifest = _initial_manifest(args, work_dir=work_dir, stages=stages, commands=commands)
        _write_manifest(work_dir, manifest)
    else:
        manifest = None
    for stage, command in commands:
        print("[{0}] {1}".format(stage, " ".join(command)), flush=True)
        if args.print_only:
            continue
        started_at = _utc_timestamp()
        start_monotonic = time.monotonic()
        log_path = work_dir / "{0}.log".format(stage)
        returncode = _run_stage(command, log_path=log_path)
        ended_at = _utc_timestamp()
        stage_result = {
            "stage": stage,
            "returncode": returncode,
            "started_at": started_at,
            "ended_at": ended_at,
            "duration_sec": round(time.monotonic() - start_monotonic, 3),
            "log": str(log_path),
        }
        assert manifest is not None
        manifest["results"].append(stage_result)
        if returncode != 0:
            print("[{0}] failed with code {1}".format(stage, returncode), flush=True)
            manifest["status"] = "failed"
            manifest["failed_stage"] = stage
            manifest["ended_at"] = ended_at
            _write_manifest(work_dir, manifest)
            return returncode
        _write_manifest(work_dir, manifest)
    if manifest is not None:
        manifest["status"] = "ok"
        manifest["ended_at"] = _utc_timestamp()
        _write_manifest(work_dir, manifest)
        if args.check_evidence:
            evidence_rc = _check_workflow_evidence(
                args,
                work_dir / WORKFLOW_MANIFEST,
                strict_hardware=bool(args.send),
            )
            manifest["evidence"] = {
                "status": "ok" if evidence_rc == 0 else "failed",
                "returncode": evidence_rc,
                "checked_at": _utc_timestamp(),
            }
            if evidence_rc != 0:
                manifest["status"] = "evidence_failed"
                manifest["ended_at"] = _utc_timestamp()
                _write_manifest(work_dir, manifest)
                return evidence_rc
            manifest["ended_at"] = _utc_timestamp()
            _write_manifest(work_dir, manifest)
    return 0


def build_commands(
    args: argparse.Namespace,
    *,
    work_dir: Path,
    stages: tuple[str, ...],
) -> list[tuple[str, list[str]]]:
    commands: list[tuple[str, list[str]]] = []
    model_path = Path(args.model).expanduser()
    comparison_dir = work_dir / "comparisons"
    log_csv = work_dir / "xyzway_run.csv"
    takeoff_stage = _takeoff_stage(args, stages)
    land_stage = _land_stage(args, stages)

    if "preflight" in stages:
        command = [
            sys.executable,
            "-m",
            "rmtt_control.preflight_check",
            "--model",
            str(model_path),
            "--ip",
            args.ip,
            "--min-battery",
            str(args.min_battery),
            "--tracker",
            args.tracker,
            "--host",
            args.host,
            "--method",
            args.vrpn_method,
            "--z-offset",
            str(args.z_offset),
            "--wait-timeout",
            str(args.wait_timeout),
        ]
        if args.port is not None:
            command.extend(["--port", str(args.port)])
        if args.invert_yaw:
            command.append("--invert-yaw")
        if args.check_vrpn or args.send:
            command.append("--check-vrpn")
        if _workflow_needs_vrpn_helper(args):
            command.append("--check-vrpn-helper")
        if args.check_drone or args.send:
            command.append("--check-drone")
        if _preflight_requires_real_model(args, stages):
            command.extend(
                [
                    "--check-model-quality",
                    "--fail-on-bootstrap",
                    "--min-fit-samples",
                    str(args.quality_min_samples),
                    "--min-r2",
                    str(args.quality_min_r2),
                    "--min-vaf",
                    str(args.quality_min_vaf),
                    "--max-nrmse",
                    str(args.quality_max_nrmse),
                ]
            )
        commands.append(("preflight", command))

    if "identify" in stages:
        command = [
            sys.executable,
            "-m",
            "rmtt_control.identify_pipeline",
            "--ip",
            args.ip,
            "--axes",
            args.axes,
            "--signals",
            args.signals,
            "--amplitudes",
            args.amplitudes,
            "--rate",
            str(args.identify_rate),
            "--output-dir",
            str(work_dir / "identification"),
            "--model-output",
            str(model_path),
            "--comparison-dir",
            str(comparison_dir),
            "--pose-timeout-sec",
            str(args.pose_timeout_sec),
            "--field-limit",
            str(args.field_limit),
            "--z-min",
            str(args.z_min),
            "--z-max",
            str(args.z_max),
            "--tracker",
            args.tracker,
            "--host",
            args.host,
            "--method",
            args.vrpn_method,
            "--z-offset",
            str(args.z_offset),
            "--fit",
            "--quality-gate",
            "--quality-fail-on-bootstrap",
            "--quality-min-samples",
            str(args.quality_min_samples),
            "--quality-min-r2",
            str(args.quality_min_r2),
            "--quality-min-vaf",
            str(args.quality_min_vaf),
            "--quality-max-nrmse",
            str(args.quality_max_nrmse),
        ]
        if args.port is not None:
            command.extend(["--port", str(args.port)])
        if args.invert_yaw:
            command.append("--invert-yaw")
        if args.recenter:
            command.append("--recenter")
        if args.backup_model:
            command.append("--backup")
        if args.send:
            command.append("--send")
            command.append("--confirm-risk")
        if takeoff_stage == "identify":
            command.append("--takeoff")
            if land_stage and land_stage != "identify":
                command.append("--allow-open-airborne-handoff")
        if land_stage == "identify":
            command.append("--land")
        commands.append(("identify", command))

    if "xyzway" in stages:
        command = [
            sys.executable,
            "-m",
            "rmtt_control.xyzway_nmpc",
            "--ip",
            args.ip,
            "--model",
            str(model_path),
            "--waypoints",
            str(Path(args.waypoints).expanduser()),
            "--source",
            args.xyz_source,
            "--controller",
            args.controller,
            "--rate",
            str(args.xyz_rate),
            "--log-csv",
            str(log_csv),
            "--pose-timeout-sec",
            str(args.pose_timeout_sec),
            "--field-limit",
            str(args.field_limit),
            "--z-min",
            str(args.z_min),
            "--z-max",
            str(args.z_max),
            "--tracker",
            args.tracker,
            "--host",
            args.host,
            "--method",
            args.vrpn_method,
            "--z-offset",
            str(args.z_offset),
            "--quality-min-samples",
            str(args.quality_min_samples),
            "--quality-min-r2",
            str(args.quality_min_r2),
            "--quality-min-vaf",
            str(args.quality_min_vaf),
            "--quality-max-nrmse",
            str(args.quality_max_nrmse),
        ]
        if args.port is not None:
            command.extend(["--port", str(args.port)])
        if args.invert_yaw:
            command.append("--invert-yaw")
        if args.reset_controller_per_waypoint:
            command.append("--reset-controller-per-waypoint")
        if _xyzway_requires_real_model(args, stages):
            command.append("--require-real-model")
        if args.allow_bootstrap_xyzway:
            command.append("--allow-bootstrap-model")
        if args.send:
            command.append("--send")
            command.append("--confirm-risk")
        if takeoff_stage == "xyzway":
            command.append("--takeoff")
        if land_stage == "xyzway":
            command.append("--land")
        commands.append(("xyzway", command))

    return commands


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the RMTT NMPC preflight -> identify -> xyzway workflow.")
    parser.add_argument("--stages", default="preflight,identify,xyzway", help="comma list: preflight,identify,xyzway")
    parser.add_argument("--work-dir", default=None)
    parser.add_argument("--print-only", action="store_true", help="print commands without running them")
    parser.add_argument("--check-evidence", action="store_true", help="validate manifest artifacts after successful run")
    parser.add_argument("--ip", default=DEFAULT_RMTT_IP)
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--waypoints", default=str(DEFAULT_WAYPOINTS))
    parser.add_argument("--send", action="store_true", help="forward --send to identify/xyzway")
    parser.add_argument("--confirm-risk", action="store_true", help="required together with --send")
    parser.add_argument("--takeoff", action="store_true")
    parser.add_argument("--land", action="store_true")
    parser.add_argument("--check-drone", action="store_true")
    parser.add_argument("--check-vrpn", action="store_true")
    parser.add_argument("--check-vrpn-helper", action="store_true")
    parser.add_argument("--min-battery", type=int, default=30)
    parser.add_argument("--require-real-model", action="store_true", help="preflight fails if model still uses bootstrap axes")
    parser.add_argument(
        "--allow-bootstrap-xyzway",
        action="store_true",
        help="allow xyzway without a prior identify stage and without strict model preflight",
    )
    parser.add_argument("--axes", default="pitch,roll,throttle,yaw")
    parser.add_argument("--signals", default="step")
    parser.add_argument("--amplitudes", default="10,20")
    parser.add_argument("--identify-rate", type=float, default=20.0)
    parser.add_argument("--recenter", action="store_true")
    parser.add_argument("--backup-model", action="store_true")
    parser.add_argument("--quality-min-samples", type=int, default=30)
    parser.add_argument("--quality-min-r2", type=float, default=0.20)
    parser.add_argument("--quality-min-vaf", type=float, default=0.20)
    parser.add_argument("--quality-max-nrmse", type=float, default=0.80)
    parser.add_argument("--xyz-source", choices=("vrpn", "static"), default="vrpn")
    parser.add_argument("--controller", choices=("mission", "flight"), default="mission")
    parser.add_argument("--xyz-rate", type=float, default=10.0)
    parser.add_argument("--reset-controller-per-waypoint", action="store_true")
    parser.add_argument("--pose-timeout-sec", type=float, default=0.5)
    parser.add_argument("--field-limit", type=float, default=1.5)
    parser.add_argument("--z-min", type=float, default=0.25)
    parser.add_argument("--z-max", type=float, default=2.0)
    parser.add_argument("--tracker", default=DEFAULT_TRACKER)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--vrpn-method", choices=("auto", "native", "print"), default="auto")
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--invert-yaw", action="store_true")
    parser.add_argument("--wait-timeout", type=float, default=5.0)
    return parser.parse_args(argv)


def _check_workflow_evidence(
    args: argparse.Namespace,
    manifest_path: Path,
    *,
    strict_hardware: bool = False,
) -> int:
    result = workflow_evidence.check_workflow_evidence(
        manifest_path,
        model_thresholds=workflow_evidence.model_quality.QualityThresholds(
            min_samples=args.quality_min_samples,
            min_r2=args.quality_min_r2,
            min_vaf=args.quality_min_vaf,
            max_nrmse=args.quality_max_nrmse,
            fail_on_bootstrap=True,
        ),
        require_send=strict_hardware,
        require_vrpn_check=strict_hardware,
        require_drone_check=strict_hardware,
        require_full_workflow=strict_hardware,
        require_managed_flight=strict_hardware,
    )
    for warning in result.warnings:
        print("[evidence] WARN: {0}".format(warning), flush=True)
    for failure in result.failures:
        print("[evidence] FAIL: {0}".format(failure), flush=True)
    workflow_evidence.print_workflow_summary(manifest_path)
    if not result.ok:
        print("[evidence] failed", flush=True)
        return 6
    print("[evidence] OK", flush=True)
    return 0


def _parse_stages(value: str) -> tuple[str, ...]:
    stages = tuple(part.strip().lower() for part in value.split(",") if part.strip())
    invalid = [stage for stage in stages if stage not in STAGES]
    if invalid:
        raise ValueError("invalid workflow stage {0!r}".format(invalid[0]))
    if not stages:
        raise ValueError("at least one workflow stage is required")
    return stages


def _preflight_requires_real_model(args: argparse.Namespace, stages: tuple[str, ...]) -> bool:
    if args.require_real_model:
        return True
    if "xyzway" not in stages:
        return False
    if "identify" in stages:
        return False
    return not args.allow_bootstrap_xyzway


def _workflow_needs_vrpn_helper(args: argparse.Namespace) -> bool:
    if args.vrpn_method == "print":
        return False
    return bool(args.check_vrpn_helper or args.check_vrpn or args.send)


def _xyzway_requires_real_model(args: argparse.Namespace, stages: tuple[str, ...]) -> bool:
    if "xyzway" not in stages:
        return False
    if args.allow_bootstrap_xyzway:
        return False
    if args.require_real_model:
        return True
    return "identify" not in stages


def _flight_stages(stages: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(stage for stage in stages if stage in {"identify", "xyzway"})


def _takeoff_stage(args: argparse.Namespace, stages: tuple[str, ...]) -> str | None:
    if not args.send or not args.takeoff:
        return None
    flight_stages = _flight_stages(stages)
    return flight_stages[0] if flight_stages else None


def _land_stage(args: argparse.Namespace, stages: tuple[str, ...]) -> str | None:
    if not args.send or not args.land:
        return None
    flight_stages = _flight_stages(stages)
    return flight_stages[-1] if flight_stages else None


def _default_work_dir() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return str(ROOT / "workflow_runs" / stamp)


def _initial_manifest(
    args: argparse.Namespace,
    *,
    work_dir: Path,
    stages: tuple[str, ...],
    commands: list[tuple[str, list[str]]],
) -> dict:
    model_path = Path(args.model).expanduser()
    return {
        "status": "running",
        "created_at": _utc_timestamp(),
        "ended_at": None,
        "work_dir": str(work_dir),
        "root": str(ROOT),
        "stages": list(stages),
        "send": bool(args.send),
        "takeoff": bool(args.takeoff),
        "land": bool(args.land),
        "ip": args.ip,
        "tracker": args.tracker,
        "host": args.host,
        "port": args.port,
        "vrpn_method": args.vrpn_method,
        "model": str(model_path),
        "waypoints": str(Path(args.waypoints).expanduser()),
        "artifacts": _workflow_artifacts(args, work_dir=work_dir, stages=stages, model_path=model_path),
        "commands": [
            {
                "stage": stage,
                "argv": command,
            }
            for stage, command in commands
        ],
        "results": [],
    }


def _workflow_artifacts(
    args: argparse.Namespace,
    *,
    work_dir: Path,
    stages: tuple[str, ...],
    model_path: Path,
) -> dict:
    artifacts = {
        "manifest": str(work_dir / WORKFLOW_MANIFEST),
        "model": str(model_path),
    }
    for stage in stages:
        artifacts["{0}_log".format(stage)] = str(work_dir / "{0}.log".format(stage))
    if "identify" in stages:
        artifacts["identification_dir"] = str(work_dir / "identification")
        artifacts["comparison_dir"] = str(work_dir / "comparisons")
    if "xyzway" in stages:
        artifacts["xyzway_log_csv"] = str(work_dir / "xyzway_run.csv")
    return artifacts


def _run_stage(command: list[str], *, log_path: Path) -> int:
    with log_path.open("w") as log:
        process = subprocess.Popen(
            command,
            cwd=str(ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        with process.stdout:
            for line in process.stdout:
                print(line, end="", flush=True)
                log.write(line)
                log.flush()
        return process.wait()


def _write_manifest(work_dir: Path, manifest: dict) -> None:
    path = work_dir / WORKFLOW_MANIFEST
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def _utc_timestamp() -> str:
    return datetime.utcnow().isoformat(timespec="seconds") + "Z"


if __name__ == "__main__":
    sys.exit(main())
