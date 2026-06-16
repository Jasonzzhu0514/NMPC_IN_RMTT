#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import compileall
import importlib
from pathlib import Path
import re
import sys
import unittest

from rmtt_control import offline_validate
from rmtt_control import port_audit
from rmtt_control import preflight_check


ROOT = Path(__file__).resolve().parents[1]
IMPORTS = (
    "models.dji_velocity_model",
    "simulation.dji_velocity_plant",
    "runtime.model_gate",
    "nmpc.flight_controller",
    "nmpc.mission_controller",
    "nmpc.identification.protocol",
    "nmpc.identification.fit_rmtt",
    "rmtt_control.nmpc_rmtt_bridge",
    "rmtt_control.pose_source",
    "rmtt_control.vrpn_pose_reader",
    "rmtt.adapter",
    "rmtt.battery",
    "rmtt.scan_ip",
    "rmtt.takeoff_land",
    "rmtt.wifi",
    "scripts.rmtt_adapter",
    "scripts.scan_ip",
    "scripts.wifi_test",
    "scripts.battery_monitor",
    "scripts.takeoff_land",
    "rmtt_control.identify_collect",
    "rmtt_control.identification_quality",
    "rmtt_control.identify_pipeline",
    "rmtt_control.fit_rmtt_model",
    "rmtt_control.model_quality",
    "rmtt_control.offline_validate",
    "rmtt_control.port_audit",
    "rmtt_control.preflight_check",
    "rmtt_control.workflow_evidence",
    "rmtt_control.nmpc_track_target",
    "rmtt_control.rmtt_nmpc_workflow",
    "rmtt_control.xyzway_nmpc",
)
SCAN_ROOTS = (".",)
SCAN_SUFFIXES = {".py", ".cpp", ".h", ".hpp"}
SCAN_EXCLUDE_DIRS = {
    ".git",
    "__pycache__",
}
FORBIDDEN_PATTERNS = (
    "mqtt",
    "rospy",
    "roslib",
    "rclpy",
    "rospkg",
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Offline validation for the RMTT NMPC workspace.")
    parser.add_argument("--skip-offline-chain", action="store_true")
    args = parser.parse_args(argv)

    checks = [
        ("compile", _check_compile),
        ("imports", _check_imports),
        ("unit-tests", _check_unit_tests),
        ("port-audit", _check_port_audit),
        ("preflight", _check_preflight),
        ("forbidden-deps", _check_forbidden_dependencies),
    ]
    if not args.skip_offline_chain:
        checks.append(("offline-chain", _check_offline_chain))

    failed = False
    for name, func in checks:
        print("[check] {0}".format(name), flush=True)
        try:
            ok = func()
        except Exception as exc:
            print("[fail] {0}: {1}".format(name, exc), flush=True)
            ok = False
        failed = failed or not ok
    if failed:
        print("[workspace] validation failed", flush=True)
        return 1
    print("[workspace] validation ok", flush=True)
    return 0


def _check_compile() -> bool:
    paths = [
        str(ROOT / "nmpc"),
        str(ROOT / "models"),
        str(ROOT / "simulation"),
        str(ROOT / "rmtt"),
        str(ROOT / "rmtt_control"),
        str(ROOT / "scripts"),
    ]
    scripts = [str(path) for path in ROOT.glob("*.py")]
    ok = True
    for path in paths + scripts:
        ok = compileall.compile_file(path, quiet=1) if path.endswith(".py") else compileall.compile_dir(path, quiet=1)
        if not ok:
            return False
    return True


def _check_imports() -> bool:
    for name in IMPORTS:
        importlib.import_module(name)
        print("  ok {0}".format(name), flush=True)
    return True


def _check_unit_tests() -> bool:
    tests_dir = ROOT / "tests"
    if not tests_dir.exists():
        print("  no tests directory", flush=True)
        return False
    suite = unittest.defaultTestLoader.discover(str(tests_dir))
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(suite)
    return result.wasSuccessful()


def _check_preflight() -> bool:
    return preflight_check.main(["--check-vrpn-helper"]) == 0


def _check_port_audit() -> bool:
    return port_audit.main([]) == 0


def _check_offline_chain() -> bool:
    return offline_validate.main([]) == 0


def _check_forbidden_dependencies() -> bool:
    pattern = re.compile("|".join(re.escape(item) for item in FORBIDDEN_PATTERNS), re.IGNORECASE)
    matches: list[str] = []
    for root in SCAN_ROOTS:
        base = ROOT / root
        if not base.exists():
            continue
        for path in base.rglob("*"):
            if _skip_scan_path(path) or not path.is_file() or path.suffix not in SCAN_SUFFIXES:
                continue
            if path.name in {"validate_workspace.py", "port_audit.py"}:
                continue
            text = path.read_text(errors="ignore")
            for lineno, line in enumerate(text.splitlines(), start=1):
                if pattern.search(line):
                    matches.append("{0}:{1}: {2}".format(path.relative_to(ROOT), lineno, line.strip()))
    for match in matches:
        print("  forbidden {0}".format(match), flush=True)
    return not matches


def _skip_scan_path(path: Path) -> bool:
    try:
        relative = path.relative_to(ROOT)
    except ValueError:
        return True
    return any(part in SCAN_EXCLUDE_DIRS for part in relative.parts)


if __name__ == "__main__":
    sys.exit(main())
