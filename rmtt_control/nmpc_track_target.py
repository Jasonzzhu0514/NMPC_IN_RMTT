#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import sys
import time

from rmtt_control.nmpc_rmtt_bridge import NmpcRmttBridge
from rmtt_control.pose_source import StaticPoseSource
from rmtt.adapter import RMTTClient, StickCommand
from runtime.model_gate import (
    MODEL_QUALITY_RETURN_CODE,
    check_model_quality_gate,
    model_quality_gate_required,
)
from rmtt_control.vrpn_pose_reader import DEFAULT_HOST, DEFAULT_PORT, DEFAULT_TRACKER, VrpnPoseReader
from rmtt_config import DEFAULT_RMTT_IP


TRACK_MODEL_QUALITY_RETURN_CODE = MODEL_QUALITY_RETURN_CODE


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Track one XYZ/yaw target with the local NMPC controller.")
    parser.add_argument("--ip", default=DEFAULT_RMTT_IP, help="RMTT IP address")
    parser.add_argument("--target-x", type=float, required=True)
    parser.add_argument("--target-y", type=float, required=True)
    parser.add_argument("--target-z", type=float, required=True)
    parser.add_argument("--target-yaw-deg", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--quiet", action="store_true", help="suppress per-step console status")
    parser.add_argument("--model", default=None, help="velocity model JSON; defaults to models/rmtt_velocity_model.json")
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
    parser.add_argument(
        "--quality-require-validation",
        action="store_true",
        help="require independent validation metrics in the fitted model",
    )
    parser.add_argument("--send", action="store_true", help="actually send rc commands to the drone")
    parser.add_argument("--confirm-risk", action="store_true", help="required with --send")
    parser.add_argument("--takeoff", action="store_true", help="take off before tracking; requires --send")
    parser.add_argument("--land", action="store_true", help="land after tracking; requires --send")
    parser.add_argument("--source", choices=("vrpn", "static"), default="vrpn")
    parser.add_argument("--tracker", default=DEFAULT_TRACKER)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--method", choices=("auto", "native", "print"), default="auto")
    parser.add_argument("--vrpn-print-devices", default=None)
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--invert-yaw", action="store_true")
    parser.add_argument("--static-x", type=float, default=0.0)
    parser.add_argument("--static-y", type=float, default=0.0)
    parser.add_argument("--static-z", type=float, default=0.0)
    parser.add_argument("--static-yaw", type=float, default=0.0)
    args = parser.parse_args(argv)
    if args.send and not args.confirm_risk:
        print("Refusing to send stick commands without --confirm-risk.", flush=True)
        return 2
    if args.send and args.takeoff and not args.land:
        print("Refusing --takeoff without --land for standalone target tracking.", flush=True)
        return 2
    if model_quality_gate_required(args):
        quality_rc = check_model_quality_gate(args, label="target tracking")
        if quality_rc != 0:
            return quality_rc

    pose_source = None
    if args.source == "vrpn":
        pose_source = VrpnPoseReader(
            tracker=args.tracker,
            host=args.host,
            port=args.port,
            vrpn_print_devices=args.vrpn_print_devices,
            method=args.method,
            z_offset=args.z_offset,
            invert_yaw=args.invert_yaw,
        )
        pose_source.connect(wait_timeout=5.0)
    else:
        pose_source = StaticPoseSource(args.static_x, args.static_y, args.static_z, args.static_yaw)

    client = RMTTClient(args.ip)
    if args.send:
        client.connect()

    bridge = NmpcRmttBridge(model_path=args.model)
    period = 1.0 / max(args.rate, 0.1)
    deadline = time.time() + max(0.0, args.duration)
    airborne = False
    landed = False

    try:
        if args.send and args.takeoff:
            client.takeoff().wait_for_completed()
            airborne = True
            time.sleep(1.0)

        while time.time() < deadline:
            pose = pose_source.latest()
            if pose is None:
                _print_status(args, "pose unavailable; sending neutral")
                if args.send:
                    client.send_stick(StickCommand())
                time.sleep(period)
                continue

            output = bridge.compute(
                pose=pose,
                target_x=args.target_x,
                target_y=args.target_y,
                target_z=args.target_z,
                target_yaw_deg=args.target_yaw_deg,
            )
            cmd = output.command
            result = output.result
            _print_status(
                args,
                "reason={reason} enabled={enabled} "
                "pos=({x:.3f},{y:.3f},{z:.3f}) "
                "u=({roll_u:.3f},{pitch_u:.3f},{thr_u:.3f},{yaw_u:.3f}) "
                "rc=({roll},{pitch},{thr},{yaw})".format(
                    reason=result.reason,
                    enabled=int(result.enabled),
                    x=pose.x,
                    y=pose.y,
                    z=pose.z,
                    roll_u=result.roll_u,
                    pitch_u=result.pitch_u,
                    thr_u=result.throttle_u,
                    yaw_u=result.yaw_u,
                    roll=cmd.roll,
                    pitch=cmd.pitch,
                    thr=cmd.throttle,
                    yaw=cmd.yaw,
                ),
            )
            if args.send:
                client.send_stick(cmd)
            time.sleep(period)

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


def _print_status(args: argparse.Namespace, message: str) -> None:
    if not getattr(args, "quiet", False):
        print(message, flush=True)


if __name__ == "__main__":
    sys.exit(main())
