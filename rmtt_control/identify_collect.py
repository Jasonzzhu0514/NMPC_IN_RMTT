#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import math
import sys
import time

from rmtt_control.nmpc_rmtt_bridge import NmpcMissionRmttBridge
from nmpc.identification.protocol import (
    RMTT_MAX_IDENTIFICATION_AMPLITUDE,
    build_stage_two_velocity_identification_steps,
)
from rmtt.adapter import RMTTClient, StickCommand
from rmtt_control.vrpn_pose_reader import DEFAULT_HOST, DEFAULT_PORT, DEFAULT_TRACKER, VrpnPoseReader
from rmtt_config import DEFAULT_RMTT_IP


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Collect RMTT stick-to-motion identification data.")
    parser.add_argument("--ip", default=DEFAULT_RMTT_IP)
    parser.add_argument("--axis", choices=("roll", "pitch", "throttle", "yaw"), required=True)
    parser.add_argument("--signals", default="step", help="comma list: step,multistep,large_jump,prbs,multisine,all")
    parser.add_argument("--amplitudes", default="10,20", help="comma-separated stick amplitudes; capped at 30")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--output", default=None)
    parser.add_argument("--send", action="store_true", help="actually send rc commands")
    parser.add_argument("--confirm-risk", action="store_true", help="required with --send")
    parser.add_argument("--pose-timeout-sec", type=float, default=0.5)
    parser.add_argument("--field-limit", type=float, default=1.5)
    parser.add_argument("--z-min", type=float, default=0.25)
    parser.add_argument("--z-max", type=float, default=2.0)
    parser.add_argument("--recenter", action="store_true", help="return to the initial VRPN pose after marked XY excitations")
    parser.add_argument("--recenter-tolerance", type=float, default=0.10)
    parser.add_argument("--recenter-yaw-tolerance-deg", type=float, default=10.0)
    parser.add_argument("--recenter-timeout", type=float, default=8.0)
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

    amplitudes = tuple(_parse_ints(args.amplitudes))
    signals = tuple(part.strip() for part in args.signals.split(",") if part.strip())
    steps = build_stage_two_velocity_identification_steps(
        axis=args.axis,
        signals=signals,
        amplitudes=amplitudes,
    )
    if not steps:
        raise RuntimeError("empty identification schedule")

    output = args.output or _default_output(args.axis)
    reader = VrpnPoseReader(
        tracker=args.tracker,
        host=args.host,
        port=args.port,
        vrpn_print_devices=args.vrpn_print_devices,
        method=args.method,
        z_offset=args.z_offset,
        invert_yaw=args.invert_yaw,
    )
    client = RMTTClient(args.ip)

    print(
        "axis={axis} steps={steps} output={output} max_amp={max_amp} send={send}".format(
            axis=args.axis,
            steps=len(steps),
            output=output,
            max_amp=RMTT_MAX_IDENTIFICATION_AMPLITUDE,
            send=int(args.send),
        ),
        flush=True,
    )

    reader.connect(wait_timeout=5.0)
    if args.send:
        client.connect()
    initial_sample = reader.latest()
    if initial_sample is None:
        reader.wait_for_sample(args.pose_timeout_sec)
        initial_sample = reader.latest()
    if initial_sample is None:
        raise RuntimeError("No initial VRPN pose available for identification.")
    recenter_bridge = NmpcMissionRmttBridge()

    period = 1.0 / max(args.rate, 1.0)
    fieldnames = [
        "wall_time",
        "elapsed",
        "step_elapsed",
        "step_start_wall_time",
        "axis",
        "step_index",
        "signal_kind",
        "step_name",
        "command_offset",
        "requires_recenter",
        "requested_roll",
        "requested_pitch",
        "requested_throttle",
        "requested_yaw",
        "roll",
        "pitch",
        "throttle",
        "yaw",
        "x",
        "y",
        "z",
        "yaw_pose",
        "pose_timestamp",
        "pose_age_sec",
        "safety_ok",
        "safety_reason",
        "recenter_target_x",
        "recenter_target_y",
        "recenter_target_z",
        "recenter_target_yaw",
    ]
    try:
        with open(output, "w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            start = time.time()
            for step in steps:
                command = _step_command(step.axis, step.offset)
                step_start = time.time()
                end_time = step_start + step.duration_sec
                print(
                    "step={idx} kind={kind} name={name} axis={axis} offset={offset} duration={duration:.2f}s".format(
                        idx=step.index,
                        kind=step.signal_kind,
                        name=step.name,
                        axis=step.axis,
                        offset=step.offset,
                        duration=step.duration_sec,
                    ),
                    flush=True,
                )
                while time.time() < end_time:
                    sample = reader.latest()
                    now = time.time()
                    safety_ok, safety_reason, pose_age = _safety_check(
                        sample,
                        now=now,
                        pose_timeout_sec=args.pose_timeout_sec,
                        field_limit=args.field_limit,
                        z_min=args.z_min,
                        z_max=args.z_max,
                    )
                    applied_command = command if safety_ok else StickCommand()
                    if args.send:
                        client.send_stick(applied_command)
                    writer.writerow(
                        {
                            "wall_time": now,
                            "elapsed": now - start,
                            "step_elapsed": now - step_start,
                            "step_start_wall_time": step_start,
                            "axis": step.axis,
                            "step_index": step.index,
                            "signal_kind": step.signal_kind,
                            "step_name": step.name,
                            "command_offset": step.offset,
                            "requires_recenter": int(step.requires_recenter),
                            "requested_roll": command.roll,
                            "requested_pitch": command.pitch,
                            "requested_throttle": command.throttle,
                            "requested_yaw": command.yaw,
                            "roll": applied_command.roll,
                            "pitch": applied_command.pitch,
                            "throttle": applied_command.throttle,
                            "yaw": applied_command.yaw,
                            "x": None if sample is None else sample.x,
                            "y": None if sample is None else sample.y,
                            "z": None if sample is None else sample.z,
                            "yaw_pose": None if sample is None else sample.yaw,
                            "pose_timestamp": None if sample is None else sample.timestamp,
                            "pose_age_sec": pose_age,
                            "safety_ok": int(safety_ok),
                            "safety_reason": safety_reason,
                            "recenter_target_x": initial_sample.x,
                            "recenter_target_y": initial_sample.y,
                            "recenter_target_z": initial_sample.z,
                            "recenter_target_yaw": initial_sample.yaw,
                        }
                    )
                    file.flush()
                    if not safety_ok:
                        print("safety stop: {0}".format(safety_reason), flush=True)
                        if args.send:
                            client.center()
                        return 2
                    time.sleep(period)
                if args.recenter and step.requires_recenter:
                    ok = _run_recenter(
                        writer,
                        reader=reader,
                        client=client,
                        bridge=recenter_bridge,
                        target=initial_sample,
                        start=start,
                        period=period,
                        send=args.send,
                        pose_timeout_sec=args.pose_timeout_sec,
                        field_limit=args.field_limit,
                        z_min=args.z_min,
                        z_max=args.z_max,
                        tolerance=args.recenter_tolerance,
                        yaw_tolerance_deg=args.recenter_yaw_tolerance_deg,
                        timeout=args.recenter_timeout,
                    )
                    if not ok:
                        if args.send:
                            client.center()
                        return 3
            if args.send:
                client.center()
    except KeyboardInterrupt:
        if args.send:
            client.center()
        return 130
    finally:
        if args.send:
            client.center()
            client.close()
        reader.close()
    print("done: {0}".format(output), flush=True)
    return 0


def _parse_ints(value: str) -> list[int]:
    values = []
    for part in value.split(","):
        part = part.strip()
        if part:
            values.append(int(part))
    return values


def _step_command(axis: str, offset: int) -> StickCommand:
    kwargs = {"roll": 0, "pitch": 0, "throttle": 0, "yaw": 0}
    kwargs[axis] = int(offset)
    return StickCommand(**kwargs)


def _default_output(axis: str) -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return "identify_{0}_{1}.csv".format(axis, stamp)


def _run_recenter(
    writer: csv.DictWriter,
    *,
    reader: VrpnPoseReader,
    client: RMTTClient,
    bridge: NmpcMissionRmttBridge,
    target,
    start: float,
    period: float,
    send: bool,
    pose_timeout_sec: float,
    field_limit: float,
    z_min: float,
    z_max: float,
    tolerance: float,
    yaw_tolerance_deg: float,
    timeout: float,
) -> bool:
    bridge.reset()
    deadline = time.time() + max(0.1, timeout)
    while time.time() < deadline:
        sample = reader.latest()
        now = time.time()
        safety_ok, safety_reason, pose_age = _safety_check(
            sample,
            now=now,
            pose_timeout_sec=pose_timeout_sec,
            field_limit=field_limit,
            z_min=z_min,
            z_max=z_max,
        )
        if not safety_ok:
            _write_recenter_row(
                writer,
                now=now,
                start=start,
                target=target,
                sample=sample,
                command=StickCommand(),
                pose_age=pose_age,
                safety_ok=False,
                safety_reason=safety_reason,
            )
            print("recenter safety stop: {0}".format(safety_reason), flush=True)
            return False
        if _recenter_reached(sample, target, tolerance=tolerance, yaw_tolerance_deg=yaw_tolerance_deg):
            if send:
                client.center()
            _write_recenter_row(
                writer,
                now=now,
                start=start,
                target=target,
                sample=sample,
                command=StickCommand(),
                pose_age=pose_age,
                safety_ok=True,
                safety_reason="recenter_done",
            )
            print("recenter done", flush=True)
            return True
        output = bridge.compute(
            pose=sample,
            target_x=target.x,
            target_y=target.y,
            target_z=target.z,
            target_yaw_deg=_yaw_degrees(target.yaw),
            phase="MOVE-XYZ",
        )
        if send:
            client.send_stick(output.command)
        _write_recenter_row(
            writer,
            now=now,
            start=start,
            target=target,
            sample=sample,
            command=output.command,
            pose_age=pose_age,
            safety_ok=True,
            safety_reason="recenter",
        )
        time.sleep(period)
    print("recenter timeout", flush=True)
    return False


def _write_recenter_row(
    writer: csv.DictWriter,
    *,
    now: float,
    start: float,
    target,
    sample,
    command: StickCommand,
    pose_age: float | None,
    safety_ok: bool,
    safety_reason: str,
) -> None:
    writer.writerow(
        {
            "wall_time": now,
            "elapsed": now - start,
            "step_elapsed": 0.0,
            "step_start_wall_time": now,
            "axis": "recenter",
            "step_index": -1,
            "signal_kind": "recenter",
            "step_name": "recenter",
            "command_offset": 0,
            "requires_recenter": 0,
            "requested_roll": command.roll,
            "requested_pitch": command.pitch,
            "requested_throttle": command.throttle,
            "requested_yaw": command.yaw,
            "roll": command.roll,
            "pitch": command.pitch,
            "throttle": command.throttle,
            "yaw": command.yaw,
            "x": None if sample is None else sample.x,
            "y": None if sample is None else sample.y,
            "z": None if sample is None else sample.z,
            "yaw_pose": None if sample is None else sample.yaw,
            "pose_timestamp": None if sample is None else sample.timestamp,
            "pose_age_sec": pose_age,
            "safety_ok": int(safety_ok),
            "safety_reason": safety_reason,
            "recenter_target_x": target.x,
            "recenter_target_y": target.y,
            "recenter_target_z": target.z,
            "recenter_target_yaw": target.yaw,
        }
    )


def _recenter_reached(sample, target, *, tolerance: float, yaw_tolerance_deg: float) -> bool:
    xy_error = math.hypot(sample.x - target.x, sample.y - target.y)
    z_error = abs(sample.z - target.z)
    yaw_error = abs((_yaw_degrees(target.yaw) - _yaw_degrees(sample.yaw) + 180.0) % 360.0 - 180.0)
    return xy_error <= tolerance and z_error <= tolerance and yaw_error <= yaw_tolerance_deg


def _yaw_degrees(yaw: float) -> float:
    return math.degrees(yaw) if abs(yaw) <= 6.5 else yaw


def _safety_check(
    sample,
    *,
    now: float,
    pose_timeout_sec: float,
    field_limit: float,
    z_min: float,
    z_max: float,
) -> tuple[bool, str, float | None]:
    if sample is None:
        return False, "missing_pose", None
    pose_age = now - sample.timestamp if sample.timestamp is not None else None
    if pose_age is not None and pose_timeout_sec > 0.0 and pose_age > pose_timeout_sec:
        return False, "stale_pose", pose_age
    limit = abs(float(field_limit))
    if abs(sample.x) > limit or abs(sample.y) > limit:
        return False, "xy_boundary", pose_age
    if sample.z < z_min:
        return False, "z_below_min", pose_age
    if sample.z > z_max:
        return False, "z_above_max", pose_age
    return True, "ok", pose_age


if __name__ == "__main__":
    sys.exit(main())
