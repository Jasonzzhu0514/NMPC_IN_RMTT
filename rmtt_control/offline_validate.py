#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import shutil
import sys
import tempfile

from rmtt_control import fit_rmtt_model
from rmtt_control.model_quality import QualityThresholds, check_model_quality
from rmtt_control.nmpc_rmtt_bridge import NmpcMissionRmttBridge
from rmtt_control.pose_source import PoseSample
from rmtt_control import preflight_check


AXES = ("pitch", "roll", "throttle", "yaw")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Offline validate identification-to-xyzway NMPC chain.")
    parser.add_argument("--work-dir", default=None)
    parser.add_argument("--keep", action="store_true")
    args = parser.parse_args(argv)

    temp_dir = Path(args.work_dir) if args.work_dir else Path(tempfile.mkdtemp(prefix="rmtt_offline_"))
    temp_dir.mkdir(parents=True, exist_ok=True)
    model_path = temp_dir / "rmtt_velocity_model.json"
    comparison_dir = temp_dir / "comparisons"
    try:
        csvs = {axis: _write_synthetic_csv(temp_dir, axis) for axis in AXES}
        fit_args: list[str] = [
            "--output",
            str(model_path),
            "--comparison-dir",
            str(comparison_dir),
            "--require-all",
            "--base-model",
            str(Path(__file__).resolve().parents[1] / "models" / "rmtt_velocity_model.json"),
        ]
        for axis, path in csvs.items():
            fit_args.extend([f"--{axis}-csv", str(path)])
        print("[fit]", " ".join(fit_args), flush=True)
        fit_rc = fit_rmtt_model.main(fit_args)
        if fit_rc != 0:
            return fit_rc

        quality = check_model_quality(
            model_path,
            thresholds=QualityThresholds(
                min_samples=20,
                min_r2=-1.0,
                min_vaf=-1.0,
                max_nrmse=2.0,
                fail_on_bootstrap=True,
            ),
        )
        if any(not item.ok for item in quality):
            for item in quality:
                print(item, flush=True)
            return 1
        print("[quality] ok", flush=True)

        preflight_rc = preflight_check.main(
            [
                "--model",
                str(model_path),
                "--check-model-quality",
                "--fail-on-bootstrap",
                "--min-fit-samples",
                "20",
                "--min-r2",
                "-1",
                "--min-vaf",
                "-1",
                "--max-nrmse",
                "2",
            ]
        )
        if preflight_rc != 0:
            return preflight_rc

        bridge = NmpcMissionRmttBridge(model_path=model_path)
        for t in (1.0, 1.12):
            output = bridge.compute(
                pose=PoseSample(x=0.0, y=0.0, z=0.8, yaw=0.0, timestamp=t),
                target_x=0.3,
                target_y=0.2,
                target_z=0.9,
                target_yaw_deg=20.0,
            )
            print(
                "[mission] t={0:.2f} reason={1} rc=({2},{3},{4},{5})".format(
                    t,
                    output.decision.reason,
                    output.command.roll,
                    output.command.pitch,
                    output.command.throttle,
                    output.command.yaw,
                ),
                flush=True,
            )
        print("[offline] ok: {0}".format(temp_dir), flush=True)
        return 0
    finally:
        if args.work_dir is None and not args.keep:
            shutil.rmtree(temp_dir, ignore_errors=True)


def _write_synthetic_csv(directory: Path, axis: str) -> Path:
    path = directory / f"synthetic_{axis}.csv"
    fields = [
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
    ]
    rows = _synthetic_rows(axis)
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return path


def _synthetic_rows(axis: str) -> list[dict[str, float | int | str]]:
    rows = []
    dt = 0.05
    t = 0.0
    x = 0.0
    y = 0.0
    z = 0.8
    yaw = 0.0
    vx = vy = vz = yaw_rate = 0.0
    schedule = [(0, 1.0), (20, 3.0), (0, 1.0), (-20, 3.0), (0, 1.0)]
    step_index = 0
    for command, duration in schedule:
        step_start = t
        count = int(duration / dt)
        for _ in range(count):
            u = command / 100.0
            if axis == "pitch":
                vx = _advance(vx, 1.0 * u, dt)
                x += vx * dt
            elif axis == "roll":
                vy = _advance(vy, 1.0 * u, dt)
                y += vy * dt
            elif axis == "throttle":
                vz = _advance(vz, 0.6 * u, dt)
                z += vz * dt
            elif axis == "yaw":
                yaw_rate = _advance(yaw_rate, 90.0 * u, dt)
                yaw += yaw_rate * dt
            rows.append(_row(axis, step_index, command, t, step_start, x, y, z, yaw))
            t += dt
        step_index += 1
    return rows


def _advance(value: float, target: float, dt: float) -> float:
    return value + ((target - value) / 0.35) * dt


def _row(
    axis: str,
    step_index: int,
    command: int,
    t: float,
    step_start: float,
    x: float,
    y: float,
    z: float,
    yaw: float,
) -> dict[str, float | int | str]:
    commands = {"roll": 0, "pitch": 0, "throttle": 0, "yaw": 0}
    commands[axis] = command
    return {
        "wall_time": 1000.0 + t,
        "elapsed": t,
        "step_elapsed": t - step_start,
        "step_start_wall_time": 1000.0 + step_start,
        "axis": axis,
        "step_index": step_index,
        "signal_kind": "multistep",
        "step_name": "center" if command == 0 else "step",
        "command_offset": command,
        "requires_recenter": 0,
        "requested_roll": commands["roll"],
        "requested_pitch": commands["pitch"],
        "requested_throttle": commands["throttle"],
        "requested_yaw": commands["yaw"],
        "roll": commands["roll"],
        "pitch": commands["pitch"],
        "throttle": commands["throttle"],
        "yaw": commands["yaw"],
        "x": x,
        "y": y,
        "z": z,
        "yaw_pose": yaw,
        "pose_timestamp": 1000.0 + t,
        "pose_age_sec": 0.0,
        "safety_ok": 1,
        "safety_reason": "ok",
    }


if __name__ == "__main__":
    sys.exit(main())
