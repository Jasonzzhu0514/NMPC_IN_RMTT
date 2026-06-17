#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import importlib
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_TEMPLATE = ROOT / "config" / "rmtt.example.json"
LOCAL_CONFIG = ROOT / "config" / "rmtt.local.json"
REQUIREMENTS = ROOT / "requirements.txt"
VRPN_HELPER = ROOT / "native" / "vrpn_pose_json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Prepare this RMTT workspace after clone. This does not connect to the drone."
    )
    parser.add_argument("--skip-pip", action="store_true", help="do not install Python dependencies")
    parser.add_argument("--no-user", action="store_true", help="do not pass --user to pip install")
    parser.add_argument("--build-vrpn-helper", action="store_true", help="compile native VRPN helper")
    parser.add_argument("--skip-validate", action="store_true", help="skip offline workspace validation")
    parser.add_argument(
        "--overwrite-config",
        action="store_true",
        help="overwrite config/rmtt.local.json from the example template",
    )
    args = parser.parse_args(argv)

    print("[1/5] checking Python", flush=True)
    _check_python()

    print("[2/5] preparing local config", flush=True)
    _prepare_config(overwrite=args.overwrite_config)

    if args.skip_pip:
        print("[3/5] skipping Python dependency install", flush=True)
    else:
        print("[3/5] installing Python dependencies", flush=True)
        _install_requirements(use_user=not args.no_user)

    print("[4/5] checking imports", flush=True)
    _check_imports()

    if args.build_vrpn_helper:
        print("[5/5] building VRPN helper", flush=True)
        _build_vrpn_helper()
    else:
        print("[5/5] skipping VRPN helper build", flush=True)

    if args.skip_validate:
        print("[validate] skipping offline workspace validation", flush=True)
    else:
        print("[validate] running offline workspace validation", flush=True)
        _run([sys.executable, "-m", "rmtt_control.validate_workspace", "--skip-offline-chain"])

    _print_next_steps()
    return 0


def _check_python() -> None:
    if sys.version_info < (3, 8):
        raise SystemExit("Python 3.8 or newer is required.")
    print("python: {0}".format(sys.executable), flush=True)


def _prepare_config(*, overwrite: bool) -> None:
    if not CONFIG_TEMPLATE.exists():
        raise SystemExit("missing config template: {0}".format(CONFIG_TEMPLATE))
    if LOCAL_CONFIG.exists() and not overwrite:
        print("config exists: {0}".format(_rel(LOCAL_CONFIG)), flush=True)
        return
    LOCAL_CONFIG.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(CONFIG_TEMPLATE, LOCAL_CONFIG)
    print("created: {0}".format(_rel(LOCAL_CONFIG)), flush=True)


def _install_requirements(*, use_user: bool) -> None:
    if not REQUIREMENTS.exists():
        raise SystemExit("missing requirements file: {0}".format(REQUIREMENTS))
    cmd = [sys.executable, "-m", "pip", "install", "-r", str(REQUIREMENTS)]
    if use_user:
        cmd.append("--user")
    _run(cmd)


def _check_imports() -> None:
    missing: list[str] = []
    for module in ("robomaster", "multi_robomaster", "netaddr", "netifaces"):
        try:
            importlib.import_module(module)
            print("ok: {0}".format(module), flush=True)
        except Exception as exc:
            missing.append("{0}: {1}".format(module, exc))
    if missing:
        raise SystemExit("missing imports after setup:\n{0}".format("\n".join(missing)))


def _build_vrpn_helper() -> None:
    script = ROOT / "build_vrpn_helper.sh"
    if not script.exists():
        raise SystemExit("missing VRPN build script: {0}".format(script))
    _run([str(script)])
    if not VRPN_HELPER.exists():
        raise SystemExit("VRPN helper build did not create: {0}".format(VRPN_HELPER))


def _run(cmd: list[str]) -> None:
    print("+ {0}".format(" ".join(cmd)), flush=True)
    subprocess.run(cmd, cwd=str(ROOT), check=True)


def _rel(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _print_next_steps() -> None:
    print("", flush=True)
    print("Next:", flush=True)
    print("  1. edit config/rmtt.local.json", flush=True)
    print("  2. connect this computer to the RMTT AP Wi-Fi", flush=True)
    print("  3. run: python3 scripts/wifi_test.py", flush=True)


if __name__ == "__main__":
    sys.exit(main())
