#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Audit that the local RMTT workspace contains the expected NMPC port surface."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import re
import sys

from nmpc.identification.protocol import RMTT_MAX_IDENTIFICATION_AMPLITUDE
from rmtt_control.nmpc_rmtt_bridge import RMTT_STICK_MAX, RMTT_STICK_MIN


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE_ROOT = Path(
    os.environ.get(
        "NMPC_SOURCE_ROOT",
        str(ROOT / "external" / "source_nmpc"),
    )
)
FORBIDDEN_PATTERNS = (
    "mqtt",
    "rospy",
    "roslib",
    "rclpy",
    "rospkg",
)
CORE_NMPC_FILES = (
    "flight/axis.py",
    "flight/math_utils.py",
    "flight/model.py",
    "flight/runtime.py",
    "flight/runtime_state.py",
    "flight/types.py",
    "flight_controller.py",
    "mission/axis/near_min.py",
    "mission/axis/z_near_min.py",
    "mission/math_utils.py",
    "mission/runtime.py",
    "mission/runtime_decision.py",
    "mission/segment_runtime.py",
    "mission/strategies.py",
    "mission/trim.py",
    "mission/types.py",
    "mission_controller.py",
    "position_3d_controller.py",
    "position_3d_types.py",
    "segment_guide.py",
    "identification/protocol.py",
    "identification/protocol_signals.py",
    "identification/protocol_types.py",
    "identification/single_axis/fit_math.py",
    "identification/single_axis/fit_models.py",
)
RMTT_ENTRYPOINTS = (
    "rmtt/__init__.py",
    "rmtt/adapter.py",
    "rmtt/battery.py",
    "rmtt/scan_ip.py",
    "rmtt/takeoff_land.py",
    "rmtt/wifi.py",
    "scripts/rmtt_adapter.py",
    "scripts/scan_ip.py",
    "scripts/wifi_test.py",
    "scripts/battery_monitor.py",
    "scripts/takeoff_land.py",
    "runtime/model_gate.py",
    "rmtt_control/pose_source.py",
    "rmtt_control/vrpn_pose_reader.py",
    "build_vrpn_helper.sh",
    "native/vrpn_pose_json.cpp",
    "rmtt_control/identify_collect.py",
    "rmtt_control/identify_pipeline.py",
    "rmtt_control/identification_quality.py",
    "rmtt_control/fit_rmtt_model.py",
    "rmtt_control/model_quality.py",
    "rmtt_control/workflow_evidence.py",
    "rmtt_control/preflight_check.py",
    "rmtt_control/nmpc_track_target.py",
    "rmtt_control/xyzway_nmpc.py",
    "rmtt_control/rmtt_nmpc_workflow.py",
)
SOURCE_REPLACEMENTS = (
    (
        "identification/single_axis/collect.py",
        ("rmtt_control/identify_collect.py", "rmtt_control/identify_pipeline.py"),
        "single-axis source collection is replaced by RMTT SDK + VRPN collection",
    ),
    (
        "identification/single_axis/fit.py",
        ("rmtt_control/fit_rmtt_model.py", "nmpc/identification/fit_rmtt.py"),
        "single-axis model fitting is replaced by RMTT CSV fitting",
    ),
    (
        "identification/single_axis/fit_io.py",
        ("rmtt_control/fit_rmtt_model.py", "nmpc/identification/fit_rmtt.py"),
        "source repository path IO is replaced by explicit RMTT model/CSV paths",
    ),
    (
        "identification/all_axis/pipeline.py",
        ("rmtt_control/identify_pipeline.py", "rmtt_control/rmtt_nmpc_workflow.py"),
        "all-axis source pipeline is replaced by bounded RMTT SDK workflow",
    ),
    (
        "identification/all_axis/build_model.py",
        ("rmtt_control/fit_rmtt_model.py", "rmtt_control/model_quality.py"),
        "all-axis build step is replaced by RMTT fit + quality gate",
    ),
    (
        "identification/all_axis/quality.py",
        ("rmtt_control/model_quality.py", "rmtt_control/identification_quality.py"),
        "source quality checks are replaced by CSV and fitted-model gates",
    ),
    (
        "identification/all_axis/pipeline_checks.py",
        ("rmtt_control/preflight_check.py", "rmtt_control/identification_quality.py"),
        "source readiness checks are replaced by VRPN/drone preflight and CSV gates",
    ),
)
HARDWARE_CONFIRMATION_ENTRYPOINTS = (
    "rmtt_control/identify_collect.py",
    "rmtt_control/identify_pipeline.py",
    "rmtt_control/nmpc_track_target.py",
    "rmtt_control/xyzway_nmpc.py",
    "scripts/takeoff_land.py",
    "rmtt_control/rmtt_nmpc_workflow.py",
)
HARDWARE_GUARD_IMPLEMENTATIONS = {
    "scripts/takeoff_land.py": ("scripts/takeoff_land.py", "rmtt/takeoff_land.py"),
}


@dataclass(frozen=True)
class AuditResult:
    ok: bool
    failures: tuple[str, ...]
    warnings: tuple[str, ...]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Audit RMTT NMPC port completeness and dependency boundaries.")
    parser.add_argument("--source-root", default=str(DEFAULT_SOURCE_ROOT))
    args = parser.parse_args(argv)
    result = audit_port(Path(args.source_root))
    for warning in result.warnings:
        print("WARN: {0}".format(warning), flush=True)
    for failure in result.failures:
        print("FAIL: {0}".format(failure), flush=True)
    if not result.ok:
        return 1
    print("OK: port audit passed", flush=True)
    return 0


def audit_port(source_root: Path = DEFAULT_SOURCE_ROOT) -> AuditResult:
    failures: list[str] = []
    warnings: list[str] = []
    compare_source = source_root.exists()
    if not compare_source:
        warnings.append("source NMPC root does not exist; source comparison skipped: {0}".format(source_root))

    if compare_source:
        for relative in CORE_NMPC_FILES:
            source = source_root / relative
            target = ROOT / "nmpc" / relative
            _check_file_pair(relative, source, target, failures=failures, warnings=warnings)
    else:
        for relative in CORE_NMPC_FILES:
            target = ROOT / "nmpc" / relative
            if not target.exists():
                failures.append("missing local NMPC core file: {0}".format(relative))
            elif target.stat().st_size <= 0:
                failures.append("empty local NMPC core file: {0}".format(relative))

    for relative in RMTT_ENTRYPOINTS:
        path = ROOT / relative
        if not path.exists():
            failures.append("missing RMTT entrypoint: {0}".format(relative))
        elif path.stat().st_size <= 0:
            failures.append("empty RMTT entrypoint: {0}".format(relative))

    if compare_source:
        _check_source_replacements(source_root, failures=failures)
    _check_executable_entrypoints(ROOT, failures=failures)
    _check_hardware_confirmation_guards(ROOT, failures=failures)
    _check_rmtt_limits(failures=failures)
    _check_stale_temp_files(ROOT, failures=failures)
    _check_forbidden_dependencies(ROOT, failures=failures)
    return AuditResult(not failures, tuple(failures), tuple(warnings))


def _check_file_pair(
    relative: str,
    source: Path,
    target: Path,
    *,
    failures: list[str],
    warnings: list[str],
) -> None:
    if not source.exists():
        failures.append("missing source core file: {0}".format(relative))
        return
    if not target.exists():
        failures.append("missing local NMPC core file: {0}".format(relative))
        return
    if source.stat().st_size <= 0:
        failures.append("empty source core file: {0}".format(relative))
    if target.stat().st_size <= 0:
        failures.append("empty local NMPC core file: {0}".format(relative))
    source_defs = _public_defs(source)
    target_defs = _public_defs(target)
    missing_defs = sorted(source_defs - target_defs)
    if missing_defs:
        warnings.append(
            "{0}: local port missing public definitions from source: {1}".format(
                relative,
                ", ".join(missing_defs[:8]),
            )
        )


def _public_defs(path: Path) -> set[str]:
    names: set[str] = set()
    pattern = re.compile(r"^(?:class|def)\s+([A-Za-z_][A-Za-z0-9_]*)\b")
    for line in path.read_text(errors="ignore").splitlines():
        match = pattern.match(line)
        if match and not match.group(1).startswith("_"):
            names.add(match.group(1))
    return names


def _check_forbidden_dependencies(root: Path, *, failures: list[str]) -> None:
    pattern = re.compile("|".join(re.escape(item) for item in FORBIDDEN_PATTERNS), re.IGNORECASE)
    for path in root.rglob("*"):
        if not path.is_file() or path.suffix not in {".py", ".cpp", ".h", ".hpp"}:
            continue
        if path.name == "port_audit.py" or path.name == "validate_workspace.py":
            continue
        text = path.read_text(errors="ignore")
        for lineno, line in enumerate(text.splitlines(), start=1):
            if pattern.search(line):
                failures.append(
                    "forbidden dependency {0}:{1}: {2}".format(
                        path.relative_to(root),
                        lineno,
                        line.strip(),
                    )
                )


def _check_source_replacements(source_root: Path, *, failures: list[str]) -> None:
    for source_relative, replacements, reason in SOURCE_REPLACEMENTS:
        source = source_root / source_relative
        if not source.exists():
            failures.append("missing source file for replacement audit: {0}".format(source_relative))
            continue
        for replacement in replacements:
            path = ROOT / replacement
            if not path.exists():
                failures.append(
                    "missing RMTT replacement for {0}: {1} ({2})".format(
                        source_relative,
                        replacement,
                        reason,
                    )
                )
            elif path.stat().st_size <= 0:
                failures.append(
                    "empty RMTT replacement for {0}: {1} ({2})".format(
                        source_relative,
                        replacement,
                        reason,
                    )
                )


def _check_hardware_confirmation_guards(root: Path, *, failures: list[str]) -> None:
    for relative in HARDWARE_CONFIRMATION_ENTRYPOINTS:
        paths = tuple(root / item for item in HARDWARE_GUARD_IMPLEMENTATIONS.get(relative, (relative,)))
        if not paths[0].exists():
            failures.append("missing hardware entrypoint for confirmation audit: {0}".format(relative))
            continue
        text = "\n".join(path.read_text(errors="ignore") for path in paths if path.exists())
        missing: list[str] = []
        if "--confirm-risk" not in text:
            missing.append("--confirm-risk argument")
        if "confirm_risk" not in text:
            missing.append("confirm_risk check")
        if "Refusing" not in text:
            missing.append("refusal path")
        if missing:
            failures.append(
                "hardware entrypoint lacks confirmation guard {0}: {1}".format(
                    relative,
                    ", ".join(missing),
                )
            )


def _check_executable_entrypoints(root: Path, *, failures: list[str]) -> None:
    for relative in RMTT_ENTRYPOINTS:
        if not relative.endswith(".sh"):
            continue
        path = root / relative
        if not path.exists():
            continue
        if not path.stat().st_mode & 0o111:
            failures.append("entrypoint is not executable: {0}".format(relative))


def _check_rmtt_limits(*, failures: list[str]) -> None:
    if RMTT_STICK_MIN != -100 or RMTT_STICK_MAX != 100:
        failures.append(
            "unexpected RMTT stick range: [{0}, {1}]".format(
                RMTT_STICK_MIN,
                RMTT_STICK_MAX,
            )
        )
    if RMTT_MAX_IDENTIFICATION_AMPLITUDE > 30:
        failures.append(
            "identification amplitude cap is too high: {0}".format(
                RMTT_MAX_IDENTIFICATION_AMPLITUDE,
            )
        )


def _check_stale_temp_files(root: Path, *, failures: list[str]) -> None:
    patterns = ("*.deleteme", "*.tmp", "*.bak")
    for pattern in patterns:
        for path in root.glob(pattern):
            if path.is_file():
                failures.append("stale temporary file in workspace root: {0}".format(path.name))


if __name__ == "__main__":
    sys.exit(main())
