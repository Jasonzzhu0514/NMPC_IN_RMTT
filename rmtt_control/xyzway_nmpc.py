#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
import json
import sys
import time

from rmtt_control.nmpc_rmtt_bridge import NmpcMissionRmttBridge, NmpcRmttBridge
from rmtt_control.pose_source import StaticPoseSource
from rmtt.adapter import RMTTClient, StickCommand
from runtime.model_gate import (
    MODEL_QUALITY_RETURN_CODE,
    check_model_quality_gate,
    model_quality_gate_required,
)
from runtime.xyz.mission import (
    ArrivalThresholds,
    SafetyBounds,
    Waypoint,
    WaypointArrivalTracker,
    check_pose_safety,
    load_waypoints,
    validate_waypoints,
)
from rmtt_control.vrpn_pose_reader import DEFAULT_HOST, DEFAULT_PORT, DEFAULT_TRACKER, VrpnPoseReader
from rmtt_config import DEFAULT_RMTT_IP


XYZWAY_TIMEOUT_RETURN_CODE = 3
XYZWAY_MODEL_QUALITY_RETURN_CODE = MODEL_QUALITY_RETURN_CODE


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run an XYZ waypoint list using the local NMPC controller.")
    parser.add_argument("--ip", default=DEFAULT_RMTT_IP)
    parser.add_argument("--waypoints", required=True, help="JSON list or object containing waypoints")
    parser.add_argument("--source", choices=("vrpn", "static"), default="vrpn")
    parser.add_argument("--controller", choices=("mission", "flight"), default="mission")
    parser.add_argument(
        "--reset-controller-per-waypoint",
        action="store_true",
        help="reset NMPC state at each waypoint boundary; default keeps controller state continuous",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="velocity model JSON; defaults to models/rmtt_velocity_model.json",
    )
    parser.add_argument(
        "--require-real-model",
        action="store_true",
        help="reject bootstrap or low-quality fitted models before running",
    )
    parser.add_argument(
        "--allow-bootstrap-model",
        action="store_true",
        help="allow --send with a bootstrap model; intended only for controlled debugging",
    )
    parser.add_argument("--quality-min-samples", type=int, default=30)
    parser.add_argument("--quality-min-r2", type=float, default=0.20)
    parser.add_argument("--quality-min-vaf", type=float, default=0.20)
    parser.add_argument("--quality-max-nrmse", type=float, default=0.80)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--log-csv", default=None)
    parser.add_argument("--quiet", action="store_true", help="suppress per-step console status")
    parser.add_argument("--arrival-radius", type=float, default=0.10)
    parser.add_argument("--z-radius", type=float, default=0.08)
    parser.add_argument("--yaw-radius-deg", type=float, default=8.0)
    parser.add_argument("--max-waypoint-sec", type=float, default=25.0)
    parser.add_argument("--pose-timeout-sec", type=float, default=0.5)
    parser.add_argument("--field-limit", type=float, default=1.5)
    parser.add_argument("--z-min", type=float, default=0.25)
    parser.add_argument("--z-max", type=float, default=2.0)
    parser.add_argument("--send", action="store_true", help="actually send rc commands")
    parser.add_argument("--confirm-risk", action="store_true", help="required with --send")
    parser.add_argument("--takeoff", action="store_true", help="take off before path; requires --send")
    parser.add_argument("--land", action="store_true", help="land after path; requires --send")
    parser.add_argument("--tracker", default=DEFAULT_TRACKER)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--method", choices=("auto", "native", "print"), default="auto")
    parser.add_argument("--vrpn-print-devices", default=None)
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--invert-yaw", action="store_true")
    parser.add_argument("--static-x", type=float, default=0.0)
    parser.add_argument("--static-y", type=float, default=0.0)
    parser.add_argument("--static-z", type=float, default=0.8)
    parser.add_argument("--static-yaw", type=float, default=0.0)
    args = parser.parse_args(argv)
    if args.send and not args.confirm_risk:
        print("Refusing to send stick commands without --confirm-risk.", flush=True)
        return 2
    if args.send and args.takeoff and not args.land:
        print("Refusing --takeoff without --land for standalone xyzway.", flush=True)
        return 2

    if model_quality_gate_required(args):
        quality_rc = check_model_quality_gate(args, label="xyzway")
        if quality_rc != 0:
            return quality_rc

    waypoints = load_waypoints(args.waypoints)
    if not waypoints:
        raise ValueError("waypoint list is empty")
    safety_bounds = SafetyBounds(
        field_limit=args.field_limit,
        z_min=args.z_min,
        z_max=args.z_max,
        pose_timeout_sec=args.pose_timeout_sec,
    )
    validate_waypoints(waypoints, safety_bounds)
    arrival_thresholds = ArrivalThresholds(
        xy_radius=args.arrival_radius,
        z_radius=args.z_radius,
        yaw_radius_deg=args.yaw_radius_deg,
    )

    pose_source = _open_pose_source(args)
    client = RMTTClient(args.ip)
    if args.send:
        client.connect()

    bridge = (
        NmpcMissionRmttBridge(model_path=args.model)
        if args.controller == "mission"
        else NmpcRmttBridge(model_path=args.model)
    )
    period = 1.0 / max(args.rate, 0.1)
    log_file = open(args.log_csv, "w", newline="") if args.log_csv else None
    log_writer = _make_log_writer(log_file)
    airborne = False
    landed = False

    try:
        if args.send and args.takeoff:
            client.takeoff().wait_for_completed()
            airborne = True
            time.sleep(1.0)

        bridge.reset()
        for index, waypoint in enumerate(waypoints):
            if args.reset_controller_per_waypoint:
                bridge.reset()
            _print_status(
                args,
                "waypoint {0}/{1}: {2}".format(index + 1, len(waypoints), waypoint),
            )
            arrival = WaypointArrivalTracker(arrival_thresholds)
            deadline = time.time() + max(1.0, args.max_waypoint_sec)
            while time.time() < deadline:
                pose = pose_source.latest()
                safety = check_pose_safety(pose, safety_bounds)
                if not safety.ok:
                    _print_status(args, "safety stop: {0}; neutral".format(safety.reason))
                    if args.send:
                        client.send_stick(StickCommand())
                    return 2

                output = bridge.compute(
                    pose=pose,
                    target_x=waypoint.x,
                    target_y=waypoint.y,
                    target_z=waypoint.z,
                    target_yaw_deg=waypoint.yaw_deg,
                    phase="MOVE-XYZ",
                    **({"final_target": index == len(waypoints) - 1} if args.controller == "mission" else {}),
                )
                if args.send:
                    client.send_stick(output.command)

                now = time.time()
                result = output.result
                decision = getattr(output, "decision", None)
                _log_step(
                    log_writer,
                    waypoint_index=index + 1,
                    waypoint=waypoint,
                    pose=pose,
                    command=output.command,
                    reason=result.reason,
                    enabled=result.enabled,
                    mission_reason=None if decision is None else decision.reason,
                    mission_enabled=None if decision is None else decision.enabled,
                    authority_u=None if decision is None else decision.authority_u,
                    yaw_authority_u=None if decision is None else decision.yaw_authority_u,
                    nmpc_values=_nmpc_log_values(output),
                )
                if arrival.update(pose, waypoint, now):
                    _print_status(args, "arrived waypoint {0}".format(index + 1))
                    break
                _print_status(
                    args,
                    "wp={wp} reason={reason} mission={mission} pos=({x:.3f},{y:.3f},{z:.3f}) "
                    "rc=({roll},{pitch},{thr},{yaw})".format(
                        wp=index + 1,
                        reason=result.reason,
                        mission="-" if decision is None else decision.reason,
                        x=pose.x,
                        y=pose.y,
                        z=pose.z,
                        roll=output.command.roll,
                        pitch=output.command.pitch,
                        thr=output.command.throttle,
                        yaw=output.command.yaw,
                    ),
                )
                time.sleep(period)
            else:
                _print_status(args, "timeout waypoint {0}".format(index + 1))
                if args.send:
                    client.send_stick(StickCommand())
                return XYZWAY_TIMEOUT_RETURN_CODE

        if args.send:
            client.center()
        if args.send and args.land:
            client.land().wait_for_completed()
            landed = True
    except KeyboardInterrupt:
        if args.send:
            client.center()
        return 130
    finally:
        if args.send:
            _safe_center(client)
            if args.land and airborne and not landed:
                _safe_land(client)
            client.close()
        if log_file is not None:
            log_file.close()
        close = getattr(pose_source, "close", None)
        if close is not None:
            close()
    return 0


def _safe_center(client: RMTTClient) -> None:
    try:
        client.center()
    except Exception as exc:  # noqa: BLE001 - cleanup should not mask primary result.
        print("WARN: center sticks failed during cleanup: {0}".format(exc), flush=True)


def _safe_land(client: RMTTClient) -> None:
    try:
        client.land().wait_for_completed()
    except Exception as exc:  # noqa: BLE001 - cleanup should not mask primary result.
        print("WARN: emergency land failed during cleanup: {0}".format(exc), flush=True)


def _open_pose_source(args: argparse.Namespace):
    if args.source == "static":
        return StaticPoseSource(args.static_x, args.static_y, args.static_z, args.static_yaw)
    reader = VrpnPoseReader(
        tracker=args.tracker,
        host=args.host,
        port=args.port,
        vrpn_print_devices=args.vrpn_print_devices,
        method=args.method,
        z_offset=args.z_offset,
        invert_yaw=args.invert_yaw,
    )
    reader.connect(wait_timeout=5.0)
    return reader


def _make_log_writer(file):
    if file is None:
        return None
    fieldnames = [
        "time",
        "waypoint_index",
        "target_x",
        "target_y",
        "target_z",
        "target_yaw_deg",
        "x",
        "y",
        "z",
        "yaw",
        "roll",
        "pitch",
        "throttle",
        "yaw_cmd",
        "reason",
        "enabled",
        "mission_reason",
        "mission_enabled",
        "authority_u",
        "yaw_authority_u",
        "nmpc_json",
    ]
    writer = csv.DictWriter(file, fieldnames=fieldnames)
    writer.writeheader()
    return writer


def _log_step(
    writer,
    *,
    waypoint_index: int,
    waypoint: Waypoint,
    pose: PoseSample,
    command: StickCommand,
    reason: str,
    enabled: bool,
    mission_reason: str | None,
    mission_enabled: bool | None,
    authority_u: float | None,
    yaw_authority_u: float | None,
    nmpc_values: dict[str, object] | None = None,
) -> None:
    if writer is None:
        return
    writer.writerow(
        {
            "time": time.time(),
            "waypoint_index": waypoint_index,
            "target_x": waypoint.x,
            "target_y": waypoint.y,
            "target_z": waypoint.z,
            "target_yaw_deg": waypoint.yaw_deg,
            "x": pose.x,
            "y": pose.y,
            "z": pose.z,
            "yaw": pose.yaw,
            "roll": command.roll,
            "pitch": command.pitch,
            "throttle": command.throttle,
            "yaw_cmd": command.yaw,
            "reason": reason,
            "enabled": int(enabled),
            "mission_reason": mission_reason,
            "mission_enabled": None if mission_enabled is None else int(mission_enabled),
            "authority_u": authority_u,
            "yaw_authority_u": yaw_authority_u,
            "nmpc_json": _json_dumps(nmpc_values or {}),
        }
    )


def _nmpc_log_values(output) -> dict[str, object]:
    decision = getattr(output, "decision", None)
    if decision is not None and hasattr(decision, "as_log_values"):
        return dict(decision.as_log_values())
    result = getattr(output, "result", None)
    if result is not None and hasattr(result, "as_log_values"):
        return dict(result.as_log_values())
    return {}


def _json_dumps(values: dict[str, object]) -> str:
    return json.dumps(values, sort_keys=True, separators=(",", ":"))


def _print_status(args: argparse.Namespace, message: str) -> None:
    if not getattr(args, "quiet", False):
        print(message, flush=True)


if __name__ == "__main__":
    sys.exit(main())
