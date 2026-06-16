#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import json
import math
import re
import os
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from rmtt_config import DEFAULT_VRPN_HOST, DEFAULT_VRPN_PORT, DEFAULT_VRPN_TRACKER

DEFAULT_TRACKER = DEFAULT_VRPN_TRACKER
DEFAULT_HOST = DEFAULT_VRPN_HOST
DEFAULT_PORT = DEFAULT_VRPN_PORT
ROOT = Path(__file__).resolve().parents[1]


@dataclass
class PoseSample:
    x: float
    y: float
    z: float
    yaw: float
    timestamp: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


_FLOAT = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
_POSE_RE = re.compile(
    r"\bpos\s*\(\s*"
    rf"(?P<x>{_FLOAT})\s*,\s*"
    rf"(?P<y>{_FLOAT})\s*,\s*"
    rf"(?P<z>{_FLOAT})\s*\)\s*;\s*"
    r"quat\s*\(\s*"
    rf"(?P<qx>{_FLOAT})\s*,\s*"
    rf"(?P<qy>{_FLOAT})\s*,\s*"
    rf"(?P<qz>{_FLOAT})\s*,\s*"
    rf"(?P<qw>{_FLOAT})\s*\)"
)


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _find_vrpn_print_devices() -> str:
    candidates = [
        shutil.which("vrpn_print_devices"),
        "/opt/ros/noetic/bin/vrpn_print_devices",
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return candidate
    raise FileNotFoundError(
        "Cannot find vrpn_print_devices. Install/source ROS VRPN, or pass "
        "--vrpn-print-devices /path/to/vrpn_print_devices."
    )


def _find_native_helper() -> str | None:
    candidates = [
        ROOT / "native" / "vrpn_pose_json",
        shutil.which("vrpn_pose_json"),
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(candidate)
    return None


class VrpnPoseReader:
    def __init__(
        self,
        tracker: str | None = DEFAULT_TRACKER,
        host: str | None = DEFAULT_HOST,
        port: int | None = DEFAULT_PORT,
        vrpn_print_devices: str | None = None,
        method: str = "auto",
        native_helper: str | None = None,
        z_offset: float = 0.0,
        invert_yaw: bool = False,
    ) -> None:
        self.tracker = tracker
        self.host = host
        self.port = port
        self._latest: PoseSample | None = None
        self._proc: subprocess.Popen[str] | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._last_line = ""
        self._vrpn_print_devices = vrpn_print_devices
        self._method = method
        self._native_helper = native_helper
        self._z_offset = float(z_offset)
        self._invert_yaw = bool(invert_yaw)

    @property
    def endpoint(self) -> str:
        if not self.tracker:
            raise ValueError("VRPN tracker is required. Pass --tracker or set RMTT_VRPN_TRACKER.")
        if not self.host:
            raise ValueError("VRPN host is required. Pass --host or set RMTT_VRPN_HOST.")
        if self.port is None:
            raise ValueError("VRPN port is required. Pass --port or set RMTT_VRPN_PORT.")
        return f"{self.tracker}@{self.host}:{self.port}"

    def connect(self, wait_timeout: float = 5.0) -> None:
        if self._proc is not None and self._proc.poll() is None:
            return

        _executable, command, parser = self._command_and_parser()
        env = os.environ.copy()
        ros_bin = "/opt/ros/noetic/bin"
        if Path(ros_bin).is_dir():
            env["PATH"] = ros_bin + os.pathsep + env.get("PATH", "")

        self._stop.clear()
        self._ready.clear()
        self._latest = None
        self._last_line = ""
        self._proc = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._parser = parser
        self._thread.start()

        if wait_timeout > 0.0 and not self.wait_for_sample(wait_timeout):
            proc = self._proc
            last_line = self._last_line
            self.close()
            if proc is not None and proc.poll() is not None:
                raise RuntimeError(
                    f"VRPN client exited before receiving pose from {self.endpoint}. "
                    f"Last output: {last_line or '<none>'}"
                )
            raise TimeoutError(
                f"No VRPN pose received from {self.endpoint} within "
                f"{wait_timeout:.1f}s. Last output: {last_line or '<none>'}"
            )

    def latest(self) -> Optional[PoseSample]:
        with self._lock:
            return self._latest

    def wait_for_sample(self, timeout: float) -> bool:
        return self._ready.wait(timeout)

    def close(self) -> None:
        self._stop.set()
        proc = self._proc
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)
        self._proc = None
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None

    def _read_loop(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return

        for raw_line in proc.stdout:
            if self._stop.is_set():
                break
            line = raw_line.strip()
            if not line:
                continue
            self._last_line = line
            sample = self._parser(line)
            if sample is None:
                continue
            sample = self._transform_sample(sample)
            with self._lock:
                self._latest = sample
            self._ready.set()

    def _transform_sample(self, sample: PoseSample) -> PoseSample:
        if self._z_offset == 0.0 and not self._invert_yaw:
            return sample
        return PoseSample(
            x=sample.x,
            y=sample.y,
            z=sample.z + self._z_offset,
            yaw=-sample.yaw if self._invert_yaw else sample.yaw,
            timestamp=sample.timestamp,
            qx=float(getattr(sample, "qx", 0.0)),
            qy=float(getattr(sample, "qy", 0.0)),
            qz=float(getattr(sample, "qz", 0.0)),
            qw=float(getattr(sample, "qw", 1.0)),
        )

    @staticmethod
    def _parse_pose_line(line: str) -> PoseSample | None:
        match = _POSE_RE.search(line)
        if match is None:
            return None

        x = float(match.group("x"))
        y = float(match.group("y"))
        z = float(match.group("z"))
        qx = float(match.group("qx"))
        qy = float(match.group("qy"))
        qz = float(match.group("qz"))
        qw = float(match.group("qw"))
        return PoseSample(
            x=x,
            y=y,
            z=z,
            yaw=_quat_to_yaw(qx, qy, qz, qw),
            timestamp=time.time(),
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

    @staticmethod
    def _parse_json_line(line: str) -> PoseSample | None:
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            return None
        try:
            z_offset = float(payload.get("z_offset", 0.0))
            invert_yaw = bool(payload.get("invert_yaw", False))
            return PoseSample(
                x=float(payload["x"]),
                y=float(payload["y"]),
                z=float(payload["z"]) - z_offset,
                yaw=-float(payload["yaw"]) if invert_yaw else float(payload["yaw"]),
                timestamp=float(payload.get("timestamp", time.time())),
                qx=float(payload.get("qx", 0.0)),
                qy=float(payload.get("qy", 0.0)),
                qz=float(payload.get("qz", 0.0)),
                qw=float(payload.get("qw", 1.0)),
            )
        except (KeyError, TypeError, ValueError):
            return None

    def _command_and_parser(self):
        method = self._method.strip().lower()
        if method not in {"auto", "native", "print"}:
            raise ValueError("method must be auto, native, or print")
        native = self._native_helper or _find_native_helper()
        if method in {"auto", "native"} and native:
            command = [native, "--endpoint", self.endpoint]
            if self._z_offset != 0.0:
                command.extend(["--z-offset", str(self._z_offset)])
            if self._invert_yaw:
                command.append("--invert-yaw")
            return native, command, self._parse_json_line
        if method == "native":
            raise FileNotFoundError(
                "Native VRPN helper not found. Run ./build_vrpn_helper.sh or pass native_helper."
            )
        executable = self._vrpn_print_devices or _find_vrpn_print_devices()
        return executable, [executable, self.endpoint], self._parse_pose_line


def _format_sample(sample: PoseSample) -> str:
    return (
        f"t={sample.timestamp:.3f} "
        f"x={sample.x:.4f} y={sample.y:.4f} z={sample.z:.4f} "
        f"yaw={sample.yaw:.4f}rad yaw_deg={math.degrees(sample.yaw):.2f} "
        f"quat=({sample.qx:.4f},{sample.qy:.4f},{sample.qz:.4f},{sample.qw:.4f})"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Read VRPN pose")
    parser.add_argument("--tracker", default=DEFAULT_TRACKER)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--rate", type=float, default=10.0, help="print rate in Hz")
    parser.add_argument("--once", action="store_true", help="print one sample and exit")
    parser.add_argument("--wait-timeout", type=float, default=5.0)
    parser.add_argument("--method", choices=("auto", "native", "print"), default="auto")
    parser.add_argument("--native-helper", default=None)
    parser.add_argument("--z-offset", type=float, default=0.0, help="meters added to VRPN z")
    parser.add_argument("--invert-yaw", action="store_true", help="invert yaw sign")
    parser.add_argument(
        "--vrpn-print-devices",
        default=None,
        help="path to vrpn_print_devices; defaults to PATH or /opt/ros/noetic/bin",
    )
    args = parser.parse_args()
    reader = VrpnPoseReader(
        args.tracker,
        args.host,
        args.port,
        vrpn_print_devices=args.vrpn_print_devices,
        method=args.method,
        native_helper=args.native_helper,
        z_offset=args.z_offset,
        invert_yaw=args.invert_yaw,
    )
    try:
        reader.connect(wait_timeout=args.wait_timeout)
        print(f"Connected to VRPN endpoint {reader.endpoint}")

        sample = reader.latest()
        if sample is not None:
            print(_format_sample(sample), flush=True)
        if args.once:
            return 0

        period = 1.0 / max(args.rate, 0.001)
        last_timestamp = sample.timestamp if sample is not None else None
        while True:
            sample = reader.latest()
            if sample is not None and sample.timestamp != last_timestamp:
                print(_format_sample(sample), flush=True)
                last_timestamp = sample.timestamp
            time.sleep(period)
    except KeyboardInterrupt:
        return 0
    finally:
        reader.close()


if __name__ == "__main__":
    sys.exit(main())
