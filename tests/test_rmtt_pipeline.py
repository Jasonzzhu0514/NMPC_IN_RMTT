from __future__ import annotations

import csv
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

from rmtt_control import identify_collect
from rmtt_control.identify_collect import _recenter_reached, _safety_check
from rmtt_control import identify_pipeline
from scripts import battery_monitor
from rmtt_control import nmpc_track_target
from rmtt_control import port_audit
from rmtt_control import preflight_check
import rmtt.battery as rmtt_battery
from scripts import takeoff_land
from rmtt_control import validate_workspace
from rmtt_control.identification_quality import CsvQualityThresholds, check_identification_csv
from rmtt_control.model_quality import QualityThresholds, check_model_quality
from nmpc.identification.protocol import (
    RMTT_MAX_IDENTIFICATION_AMPLITUDE,
    build_stage_two_velocity_identification_steps,
)
from nmpc.identification.fit_rmtt import RmttFitConfig, load_rmtt_identification_csvs
from rmtt_control.nmpc_rmtt_bridge import NmpcMissionRmttBridge
from rmtt_control.pose_source import PoseSample
from rmtt.adapter import StickCommand, normalized_to_rc
from runtime.model_gate import MODEL_QUALITY_RETURN_CODE, model_quality_gate_required
from runtime.xyz.mission import (
    ArrivalThresholds,
    SafetyBounds,
    Waypoint,
    WaypointArrivalTracker,
    check_pose_safety,
    load_waypoints,
    validate_waypoints,
)
from rmtt_control import rmtt_nmpc_workflow
from rmtt_control.vrpn_pose_reader import VrpnPoseReader
from rmtt_control import workflow_evidence
from rmtt_control import xyzway_nmpc


TEST_RMTT_IP = "198.51.100.43"


class SafetyCheckTest(unittest.TestCase):
    def test_missing_pose_fails(self) -> None:
        ok, reason, age = _safety_check(
            None,
            now=10.0,
            pose_timeout_sec=0.5,
            field_limit=1.5,
            z_min=0.25,
            z_max=2.0,
        )
        self.assertFalse(ok)
        self.assertEqual(reason, "missing_pose")
        self.assertIsNone(age)

    def test_boundaries(self) -> None:
        ok, reason, _ = _safety_check(
            PoseSample(2.0, 0.0, 0.8, 0.0, 10.0),
            now=10.0,
            pose_timeout_sec=0.5,
            field_limit=1.5,
            z_min=0.25,
            z_max=2.0,
        )
        self.assertFalse(ok)
        self.assertEqual(reason, "xy_boundary")

    def test_recenter_tolerance(self) -> None:
        target = PoseSample(0.0, 0.0, 0.8, 0.0, 1.0)
        near = PoseSample(0.04, 0.03, 0.82, 0.02, 1.0)
        far = PoseSample(0.2, 0.0, 0.8, 0.0, 1.0)
        self.assertTrue(_recenter_reached(near, target, tolerance=0.1, yaw_tolerance_deg=10.0))
        self.assertFalse(_recenter_reached(far, target, tolerance=0.1, yaw_tolerance_deg=10.0))


class FitFilteringTest(unittest.TestCase):
    def test_fit_loader_skips_recenter_and_safety_failed_rows(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "fit.csv"
            _write_filter_csv(path)
            series = load_rmtt_identification_csvs(
                [path],
                axis="pitch",
                config=RmttFitConfig(min_samples=5, transient_skip_sec=0.25),
            )
        self.assertEqual(len(series.t), 19)
        self.assertEqual(set(series.u), {0.2})
        self.assertEqual(series.segment_starts, [0])


class IdentificationQualityTest(unittest.TestCase):
    def test_identification_csv_quality_accepts_balanced_safe_data(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "quality.csv"
            _write_quality_csv(path, axis="pitch", safety_failed=False, one_sided=False)
            result = check_identification_csv(
                path,
                axis="pitch",
                thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                ),
            )
        self.assertTrue(result.ok)
        self.assertGreaterEqual(result.positive_samples, 5)
        self.assertGreaterEqual(result.negative_samples, 5)

    def test_identification_csv_quality_rejects_bad_data(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            one_sided_path = Path(tmp) / "one_sided.csv"
            failed_path = Path(tmp) / "failed.csv"
            _write_quality_csv(one_sided_path, axis="pitch", safety_failed=False, one_sided=True)
            _write_quality_csv(failed_path, axis="pitch", safety_failed=True, one_sided=False)
            thresholds = CsvQualityThresholds(
                min_rows=20,
                min_signed_samples=5,
                min_position_span=0.05,
            )
            one_sided = check_identification_csv(one_sided_path, axis="pitch", thresholds=thresholds)
            failed = check_identification_csv(failed_path, axis="pitch", thresholds=thresholds)
        self.assertFalse(one_sided.ok)
        self.assertTrue(any("negative samples" in item for item in one_sided.failures))
        self.assertFalse(failed.ok)
        self.assertTrue(any("safety fail ratio" in item for item in failed.failures))


class RmttStickLimitTest(unittest.TestCase):
    def test_identification_protocol_caps_all_signal_amplitudes(self) -> None:
        steps = build_stage_two_velocity_identification_steps(
            axis="pitch",
            signals=("all",),
            amplitudes=(10, 200),
            large_jump_amplitude=200,
            prbs_amplitude=200,
            multisine_amplitude=200,
        )
        self.assertTrue(steps)
        self.assertTrue(
            all(abs(step.offset) <= RMTT_MAX_IDENTIFICATION_AMPLITUDE for step in steps)
        )
        self.assertEqual(RMTT_MAX_IDENTIFICATION_AMPLITUDE, 30)

    def test_stick_commands_clamp_to_rmtt_range(self) -> None:
        command = StickCommand(roll=-130, pitch=120, throttle=101, yaw=-101).clamped()
        self.assertEqual(command, StickCommand(roll=-100, pitch=100, throttle=100, yaw=-100))
        self.assertEqual(normalized_to_rc(2.0), 100)
        self.assertEqual(normalized_to_rc(-2.0), -100)


class IdentifyPipelineCleanupTest(unittest.TestCase):
    def test_cleanup_failure_does_not_override_collection_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch.object(identify_pipeline.identify_collect, "main", return_value=7), \
                mock.patch.object(identify_pipeline, "_settle", side_effect=RuntimeError("center failed")):
                rc = identify_pipeline.main(
                    [
                        "--axes",
                        "pitch",
                        "--output-dir",
                        tmp,
                        "--send",
                        "--confirm-risk",
                    ]
                )
        self.assertEqual(rc, 7)

    def test_land_failure_turns_success_into_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "identify_pitch.csv"
            _write_quality_csv(output, axis="pitch", safety_failed=False, one_sided=False)

            def fake_collect(argv):
                out_index = argv.index("--output") + 1
                Path(argv[out_index]).write_text(output.read_text())
                return 0

            with mock.patch.object(identify_pipeline.identify_collect, "main", side_effect=fake_collect), \
                mock.patch.object(identify_pipeline, "_settle", return_value=None), \
                mock.patch.object(identify_pipeline, "_land", side_effect=RuntimeError("land failed")):
                rc = identify_pipeline.main(
                    [
                        "--axes",
                        "pitch",
                        "--output-dir",
                        tmp,
                        "--send",
                        "--confirm-risk",
                        "--land",
                        "--min-csv-rows",
                        "20",
                    ]
                )
        self.assertEqual(rc, 2)

    def test_failed_handoff_lands_after_takeoff(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch.object(identify_pipeline.identify_collect, "main", return_value=7), \
                mock.patch.object(identify_pipeline, "_takeoff", return_value=None), \
                mock.patch.object(identify_pipeline, "_settle", return_value=None), \
                mock.patch.object(identify_pipeline, "_land", return_value=None) as land:
                rc = identify_pipeline.main(
                    [
                        "--axes",
                        "pitch",
                        "--output-dir",
                        tmp,
                        "--send",
                        "--confirm-risk",
                        "--takeoff",
                        "--allow-open-airborne-handoff",
                    ]
                )
        self.assertEqual(rc, 7)
        land.assert_called_once()

    def test_successful_handoff_does_not_land(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "identify_pitch.csv"
            _write_quality_csv(output, axis="pitch", safety_failed=False, one_sided=False)

            def fake_collect(argv):
                out_index = argv.index("--output") + 1
                Path(argv[out_index]).write_text(output.read_text())
                return 0

            with mock.patch.object(identify_pipeline.identify_collect, "main", side_effect=fake_collect), \
                mock.patch.object(identify_pipeline, "_takeoff", return_value=None), \
                mock.patch.object(identify_pipeline, "_settle", return_value=None), \
                mock.patch.object(identify_pipeline, "_land", return_value=None) as land:
                rc = identify_pipeline.main(
                    [
                        "--axes",
                        "pitch",
                        "--output-dir",
                        tmp,
                        "--send",
                        "--confirm-risk",
                        "--takeoff",
                        "--allow-open-airborne-handoff",
                        "--min-csv-rows",
                        "20",
                    ]
                )
        self.assertEqual(rc, 0)
        land.assert_not_called()


class ModelQualityTest(unittest.TestCase):
    def test_quality_accepts_non_bootstrap_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            model_path = Path(tmp) / "model.json"
            _write_quality_model(model_path, bootstrap=False)
            results = check_model_quality(
                model_path,
                thresholds=QualityThresholds(
                    min_samples=30,
                    min_r2=0.2,
                    min_vaf=0.2,
                    max_nrmse=0.8,
                    fail_on_bootstrap=True,
                ),
            )
        self.assertTrue(all(item.ok for item in results))

    def test_quality_rejects_bootstrap_when_strict(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            model_path = Path(tmp) / "model.json"
            _write_quality_model(model_path, bootstrap=True)
            results = check_model_quality(
                model_path,
                thresholds=QualityThresholds(fail_on_bootstrap=True),
            )
        self.assertTrue(any(not item.ok for item in results))

    def test_runtime_model_gate_requirement_flags(self) -> None:
        args = mock.Mock(send=True, require_real_model=False, allow_bootstrap_model=False)
        self.assertTrue(model_quality_gate_required(args))
        args.allow_bootstrap_model = True
        self.assertFalse(model_quality_gate_required(args))
        self.assertEqual(MODEL_QUALITY_RETURN_CODE, nmpc_track_target.TRACK_MODEL_QUALITY_RETURN_CODE)
        self.assertEqual(MODEL_QUALITY_RETURN_CODE, xyzway_nmpc.XYZWAY_MODEL_QUALITY_RETURN_CODE)


class HardwareSendConfirmationTest(unittest.TestCase):
    def test_identify_collect_refuses_send_without_confirmation(self) -> None:
        rc = identify_collect.main(["--axis", "pitch", "--send"])
        self.assertEqual(rc, 2)

    def test_identify_pipeline_refuses_send_without_confirmation(self) -> None:
        rc = identify_pipeline.main(["--axes", "pitch", "--send"])
        self.assertEqual(rc, 2)

    def test_identify_pipeline_refuses_takeoff_without_land(self) -> None:
        rc = identify_pipeline.main(
            [
                "--axes",
                "pitch",
                "--send",
                "--confirm-risk",
                "--takeoff",
            ]
        )
        self.assertEqual(rc, 2)

    def test_identify_pipeline_allows_handoff_takeoff_without_land(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "identify_pitch.csv"
            _write_quality_csv(output, axis="pitch", safety_failed=False, one_sided=False)

            def fake_collect(argv):
                out_index = argv.index("--output") + 1
                Path(argv[out_index]).write_text(output.read_text())
                return 0

            with mock.patch.object(identify_pipeline.identify_collect, "main", side_effect=fake_collect), \
                mock.patch.object(identify_pipeline, "_takeoff", return_value=None), \
                mock.patch.object(identify_pipeline, "_settle", return_value=None):
                rc = identify_pipeline.main(
                    [
                        "--axes",
                        "pitch",
                        "--output-dir",
                        tmp,
                        "--send",
                        "--confirm-risk",
                        "--takeoff",
                        "--allow-open-airborne-handoff",
                        "--min-csv-rows",
                        "20",
                    ]
                )
        self.assertEqual(rc, 0)

    def test_xyzway_refuses_send_without_confirmation(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            waypoints = Path(tmp) / "waypoints.json"
            waypoints.write_text(json.dumps([{"x": 0.0, "y": 0.0, "z": 0.8, "yaw_deg": 0.0}]))
            rc = xyzway_nmpc.main(
                [
                    "--source",
                    "static",
                    "--waypoints",
                    str(waypoints),
                    "--send",
                    "--allow-bootstrap-model",
                    "--quiet",
                ]
            )
        self.assertEqual(rc, 2)

    def test_xyzway_refuses_takeoff_without_land(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            waypoints = Path(tmp) / "waypoints.json"
            waypoints.write_text(json.dumps([{"x": 0.0, "y": 0.0, "z": 0.8, "yaw_deg": 0.0}]))
            rc = xyzway_nmpc.main(
                [
                    "--source",
                    "static",
                    "--waypoints",
                    str(waypoints),
                    "--send",
                    "--confirm-risk",
                    "--allow-bootstrap-model",
                    "--takeoff",
                    "--quiet",
                ]
            )
        self.assertEqual(rc, 2)

    def test_nmpc_track_target_refuses_send_without_confirmation(self) -> None:
        rc = nmpc_track_target.main(
            [
                "--source",
                "static",
                "--target-x",
                "0",
                "--target-y",
                "0",
                "--target-z",
                "0.8",
                "--send",
            ]
        )
        self.assertEqual(rc, 2)

    def test_nmpc_track_target_refuses_takeoff_without_land(self) -> None:
        rc = nmpc_track_target.main(
            [
                "--source",
                "static",
                "--target-x",
                "0",
                "--target-y",
                "0",
                "--target-z",
                "0.8",
                "--send",
                "--confirm-risk",
                "--allow-bootstrap-model",
                "--takeoff",
            ]
        )
        self.assertEqual(rc, 2)

    def test_nmpc_track_target_send_rejects_bootstrap_model(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            model_path = Path(tmp) / "bootstrap_model.json"
            _write_quality_model(model_path, bootstrap=True)
            rc = nmpc_track_target.main(
                [
                    "--source",
                    "static",
                    "--target-x",
                    "0",
                    "--target-y",
                    "0",
                    "--target-z",
                    "0.8",
                    "--model",
                    str(model_path),
                    "--send",
                    "--confirm-risk",
                ]
            )
        self.assertEqual(rc, nmpc_track_target.TRACK_MODEL_QUALITY_RETURN_CODE)

    def test_nmpc_track_target_allows_bootstrap_model_when_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            model_path = Path(tmp) / "bootstrap_model.json"
            _write_quality_model(model_path, bootstrap=True)
            with mock.patch.object(nmpc_track_target.RMTTClient, "connect", return_value=None), \
                mock.patch.object(nmpc_track_target.RMTTClient, "send_stick", return_value=None), \
                mock.patch.object(nmpc_track_target.RMTTClient, "center", return_value=None), \
                mock.patch.object(nmpc_track_target.RMTTClient, "close", return_value=None):
                rc = nmpc_track_target.main(
                    [
                        "--source",
                        "static",
                        "--target-x",
                        "0",
                        "--target-y",
                        "0",
                        "--target-z",
                        "0.8",
                        "--duration",
                        "0.01",
                        "--rate",
                        "100",
                        "--quiet",
                        "--model",
                        str(model_path),
                        "--send",
                        "--confirm-risk",
                        "--allow-bootstrap-model",
                    ]
                )
        self.assertEqual(rc, 0)

    def test_nmpc_track_target_emergency_lands_after_takeoff_on_interrupt(self) -> None:
        class FakeAction:
            def wait_for_completed(self):
                return None

        class FakeClient:
            instances = []

            def __init__(self, ip):
                self.events = []
                FakeClient.instances.append(self)

            def connect(self):
                self.events.append("connect")

            def takeoff(self):
                self.events.append("takeoff")
                return FakeAction()

            def land(self):
                self.events.append("land")
                return FakeAction()

            def center(self):
                self.events.append("center")

            def close(self):
                self.events.append("close")

        with mock.patch.object(nmpc_track_target, "RMTTClient", FakeClient), \
            mock.patch.object(nmpc_track_target.time, "sleep", side_effect=KeyboardInterrupt):
            rc = nmpc_track_target.main(
                [
                    "--source",
                    "static",
                    "--target-x",
                    "0",
                    "--target-y",
                    "0",
                    "--target-z",
                    "0.8",
                    "--send",
                    "--confirm-risk",
                    "--allow-bootstrap-model",
                    "--takeoff",
                    "--land",
                    "--quiet",
                ]
            )
        self.assertEqual(rc, 130)
        self.assertIn("land", FakeClient.instances[0].events)

    def test_nmpc_track_target_forwards_vrpn_method(self) -> None:
        created = {}

        class FakeReader:
            def __init__(self, *args, **kwargs):
                created.update(kwargs)

            def connect(self, wait_timeout):
                return None

            def latest(self):
                return PoseSample(0.0, 0.0, 0.8, 0.0)

            def close(self):
                return None

        with tempfile.TemporaryDirectory() as tmp:
            model_path = Path(tmp) / "model.json"
            _write_quality_model(model_path, bootstrap=False)
            with mock.patch.object(nmpc_track_target, "VrpnPoseReader", FakeReader):
                rc = nmpc_track_target.main(
                    [
                        "--source",
                        "vrpn",
                        "--target-x",
                        "0",
                        "--target-y",
                        "0",
                        "--target-z",
                        "0.8",
                        "--duration",
                        "0",
                        "--method",
                        "print",
                        "--model",
                        str(model_path),
                        "--require-real-model",
                    ]
                )
        self.assertEqual(rc, 0)
        self.assertEqual(created["method"], "print")

    def test_takeoff_land_refuses_without_confirmation(self) -> None:
        rc = takeoff_land.main([])
        self.assertEqual(rc, 2)

    def test_battery_monitor_single_read_uses_isolated_reader(self) -> None:
        with mock.patch.object(
            battery_monitor,
            "read_drone_battery_isolated",
            return_value=("ok", 77),
        ) as reader:
            rc = battery_monitor.main(["--ip", TEST_RMTT_IP])
        self.assertEqual(rc, 0)
        reader.assert_called_once()

    def test_battery_monitor_timeout_returns_failure(self) -> None:
        with mock.patch.object(
            battery_monitor,
            "read_drone_battery_isolated",
            return_value=("timeout", None),
        ):
            rc = battery_monitor.main(["--ip", TEST_RMTT_IP, "--timeout", "0.1"])
        self.assertEqual(rc, 1)


class PortAuditTest(unittest.TestCase):
    def test_port_audit_passes_current_workspace(self) -> None:
        result = port_audit.audit_port()
        self.assertTrue(result.ok, result.failures)

    def test_file_pair_reports_missing_target(self) -> None:
        failures: list[str] = []
        warnings: list[str] = []
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "source.py"
            source.write_text("def sample():\n    return 1\n")
            port_audit._check_file_pair(
                "missing.py",
                source,
                Path(tmp) / "missing.py",
                failures=failures,
                warnings=warnings,
            )
        self.assertTrue(any("missing local NMPC core file" in item for item in failures))

    def test_source_replacement_audit_reports_missing_replacement(self) -> None:
        failures: list[str] = []
        with tempfile.TemporaryDirectory() as tmp:
            source_root = Path(tmp) / "source"
            source_file = source_root / "identification" / "single_axis" / "collect.py"
            source_file.parent.mkdir(parents=True)
            source_file.write_text("def main():\n    return 0\n")
            with mock.patch.object(
                port_audit,
                "SOURCE_REPLACEMENTS",
                (("identification/single_axis/collect.py", ("missing_rmtt_replacement.py",), "test"),),
            ):
                port_audit._check_source_replacements(source_root, failures=failures)
        self.assertTrue(any("missing RMTT replacement" in item for item in failures))

    def test_hardware_confirmation_audit_reports_missing_guard(self) -> None:
        failures: list[str] = []
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "unsafe.py").write_text("def main():\n    send_stick()\n")
            with mock.patch.object(port_audit, "HARDWARE_CONFIRMATION_ENTRYPOINTS", ("unsafe.py",)):
                port_audit._check_hardware_confirmation_guards(root, failures=failures)
        self.assertTrue(any("lacks confirmation guard" in item for item in failures))

    def test_executable_entrypoint_audit_reports_non_executable_script(self) -> None:
        failures: list[str] = []
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            script = root / "build.sh"
            script.write_text("#!/usr/bin/env bash\n")
            script.chmod(0o644)
            with mock.patch.object(port_audit, "RMTT_ENTRYPOINTS", ("build.sh",)):
                port_audit._check_executable_entrypoints(root, failures=failures)
        self.assertTrue(any("not executable" in item for item in failures))

    def test_port_audit_reports_stale_temp_files(self) -> None:
        failures: list[str] = []
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "leftover.deleteme").write_text("")
            port_audit._check_stale_temp_files(root, failures=failures)
        self.assertTrue(any("stale temporary file" in item for item in failures))

    def test_user_entrypoints_are_imported_and_audited(self) -> None:
        modules = {
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
            "rmtt_control.nmpc_track_target",
            "rmtt_control.xyzway_nmpc",
        }
        entrypoints = {module.replace(".", "/") + ".py" for module in modules}
        self.assertTrue(modules.issubset(set(validate_workspace.IMPORTS)))
        self.assertTrue(entrypoints.issubset(set(port_audit.RMTT_ENTRYPOINTS)))
        self.assertIn("build_vrpn_helper.sh", port_audit.RMTT_ENTRYPOINTS)


class MissionBridgeTest(unittest.TestCase):
    def test_mission_bridge_outputs_sticks_after_warmup(self) -> None:
        bridge = NmpcMissionRmttBridge()
        first = bridge.compute(
            pose=PoseSample(0.0, 0.0, 0.8, 0.0, 1.0),
            target_x=0.3,
            target_y=0.0,
            target_z=0.8,
            target_yaw_deg=0.0,
        )
        second = bridge.compute(
            pose=PoseSample(0.0, 0.0, 0.8, 0.0, 1.12),
            target_x=0.3,
            target_y=0.0,
            target_z=0.8,
            target_yaw_deg=0.0,
        )
        self.assertEqual(first.decision.reason, "warming_up_fallback")
        self.assertEqual(second.decision.reason, "ok")
        self.assertGreater(second.command.pitch, 0)


class XyzRuntimeMissionTest(unittest.TestCase):
    def test_waypoint_loader_accepts_object_and_validates_bounds(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "waypoints.json"
            path.write_text(
                json.dumps(
                    {
                        "waypoints": [
                            {"x": 0.1, "y": 0.2, "z": 0.8, "yaw_deg": 30, "hold_sec": 0.4},
                            [0.0, 0.0, 0.9, 0.0, 0.2],
                        ]
                    }
                )
            )
            waypoints = load_waypoints(path)
        self.assertEqual(len(waypoints), 2)
        self.assertEqual(waypoints[0], Waypoint(0.1, 0.2, 0.8, 30.0, 0.4))
        validate_waypoints(waypoints, SafetyBounds(field_limit=1.5, z_min=0.25, z_max=2.0))

    def test_pose_safety_rejects_stale_and_boundary_pose(self) -> None:
        stale = check_pose_safety(
            PoseSample(0.0, 0.0, 0.8, 0.0, timestamp=1.0),
            SafetyBounds(pose_timeout_sec=0.5),
            now=2.0,
        )
        outside = check_pose_safety(
            PoseSample(2.0, 0.0, 0.8, 0.0, timestamp=2.0),
            SafetyBounds(field_limit=1.5),
            now=2.0,
        )
        self.assertFalse(stale.ok)
        self.assertEqual(stale.reason, "stale_pose")
        self.assertFalse(outside.ok)
        self.assertEqual(outside.reason, "xy_boundary")

    def test_arrival_tracker_requires_hold_time(self) -> None:
        tracker = WaypointArrivalTracker(ArrivalThresholds(xy_radius=0.1, z_radius=0.1, yaw_radius_deg=5.0))
        waypoint = Waypoint(0.0, 0.0, 0.8, 0.0, hold_sec=0.5)
        pose = PoseSample(0.02, 0.0, 0.81, 0.0, timestamp=10.0)
        self.assertFalse(tracker.update(pose, waypoint, now=10.0))
        self.assertFalse(tracker.update(pose, waypoint, now=10.4))
        self.assertTrue(tracker.update(pose, waypoint, now=10.5))

    def test_xyzway_log_contains_nmpc_json(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "xyz.csv"
            with path.open("w", newline="") as file:
                writer = xyzway_nmpc._make_log_writer(file)
                xyzway_nmpc._log_step(
                    writer,
                    waypoint_index=1,
                    waypoint=Waypoint(0.1, 0.0, 0.8, 0.0, 0.1),
                    pose=PoseSample(0.0, 0.0, 0.8, 0.0, timestamp=1.0),
                    command=StickCommand(roll=1, pitch=2, throttle=3, yaw=4),
                    reason="ok",
                    enabled=True,
                    mission_reason="ok",
                    mission_enabled=True,
                    authority_u=0.08,
                    yaw_authority_u=0.5,
                    nmpc_values={"nmpc_mission_reason": "ok", "nmpc_target_x": 0.1},
                )
            with path.open() as file:
                rows = list(csv.DictReader(file))
        self.assertEqual(rows[0]["mission_reason"], "ok")
        payload = json.loads(rows[0]["nmpc_json"])
        self.assertEqual(payload["nmpc_mission_reason"], "ok")
        self.assertEqual(payload["nmpc_target_x"], 0.1)

    def test_xyzway_logs_arrival_frame(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            waypoints = root / "waypoints.json"
            log_csv = root / "xyzway.csv"
            waypoints.write_text(json.dumps([{"x": 0.0, "y": 0.0, "z": 0.8, "yaw_deg": 0.0}]))
            rc = xyzway_nmpc.main(
                [
                    "--source",
                    "static",
                    "--static-x",
                    "0.0",
                    "--static-y",
                    "0.0",
                    "--static-z",
                    "0.8",
                    "--static-yaw",
                    "0.0",
                    "--waypoints",
                    str(waypoints),
                    "--log-csv",
                    str(log_csv),
                    "--quiet",
                ]
            )
            with log_csv.open() as file:
                rows = list(csv.DictReader(file))
        self.assertEqual(rc, 0)
        self.assertGreaterEqual(len(rows), 1)
        self.assertEqual(float(rows[-1]["x"]), 0.0)
        self.assertEqual(float(rows[-1]["target_z"]), 0.8)

    def test_xyzway_timeout_return_code_is_nonzero(self) -> None:
        self.assertGreater(xyzway_nmpc.XYZWAY_TIMEOUT_RETURN_CODE, 0)

    def test_xyzway_send_rejects_bootstrap_model(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            waypoints = Path(tmp) / "waypoints.json"
            waypoints.write_text(json.dumps([{"x": 0.0, "y": 0.0, "z": 0.8, "yaw_deg": 0.0}]))
            model_path = Path(tmp) / "bootstrap_model.json"
            _write_quality_model(model_path, bootstrap=True)
            rc = xyzway_nmpc.main(
                [
                    "--source",
                    "static",
                    "--waypoints",
                    str(waypoints),
                    "--model",
                    str(model_path),
                    "--send",
                    "--confirm-risk",
                    "--quiet",
                ]
            )
        self.assertEqual(rc, xyzway_nmpc.XYZWAY_MODEL_QUALITY_RETURN_CODE)

    def test_xyzway_allows_bootstrap_model_when_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            waypoints = Path(tmp) / "waypoints.json"
            waypoints.write_text(json.dumps([{"x": 0.0, "y": 0.0, "z": 0.8, "yaw_deg": 0.0}]))
            model_path = Path(tmp) / "bootstrap_model.json"
            _write_quality_model(model_path, bootstrap=True)
            with mock.patch.object(xyzway_nmpc.RMTTClient, "connect", return_value=None), \
                mock.patch.object(xyzway_nmpc.RMTTClient, "send_stick", return_value=None), \
                mock.patch.object(xyzway_nmpc.RMTTClient, "center", return_value=None), \
                mock.patch.object(xyzway_nmpc.RMTTClient, "close", return_value=None):
                rc = xyzway_nmpc.main(
                    [
                        "--source",
                        "static",
                        "--waypoints",
                        str(waypoints),
                        "--model",
                        str(model_path),
                        "--send",
                        "--confirm-risk",
                        "--allow-bootstrap-model",
                        "--quiet",
                    ]
                )
        self.assertEqual(rc, 0)

    def test_xyzway_emergency_lands_after_takeoff_on_safety_stop(self) -> None:
        class FakeAction:
            def wait_for_completed(self):
                return None

        class FakeClient:
            instances = []

            def __init__(self, ip):
                self.events = []
                FakeClient.instances.append(self)

            def connect(self):
                self.events.append("connect")

            def takeoff(self):
                self.events.append("takeoff")
                return FakeAction()

            def land(self):
                self.events.append("land")
                return FakeAction()

            def send_stick(self, command):
                self.events.append("send_stick")

            def center(self):
                self.events.append("center")

            def close(self):
                self.events.append("close")

        with tempfile.TemporaryDirectory() as tmp:
            waypoints = Path(tmp) / "waypoints.json"
            waypoints.write_text(json.dumps([{"x": 0.0, "y": 0.0, "z": 0.8, "yaw_deg": 0.0}]))
            with mock.patch.object(xyzway_nmpc, "RMTTClient", FakeClient):
                rc = xyzway_nmpc.main(
                    [
                        "--source",
                        "static",
                        "--static-z",
                        "0.0",
                        "--waypoints",
                        str(waypoints),
                        "--send",
                        "--confirm-risk",
                        "--allow-bootstrap-model",
                        "--takeoff",
                        "--land",
                        "--quiet",
                    ]
                )
        self.assertEqual(rc, 2)
        self.assertIn("land", FakeClient.instances[0].events)

    def test_xyzway_static_timeout_fails_without_send(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            waypoints = Path(tmp) / "waypoints.json"
            waypoints.write_text(json.dumps([{"x": 0.5, "y": 0.0, "z": 0.8, "yaw_deg": 0.0}]))
            rc = xyzway_nmpc.main(
                [
                    "--source",
                    "static",
                    "--waypoints",
                    str(waypoints),
                    "--max-waypoint-sec",
                    "0.01",
                    "--rate",
                    "200",
                    "--quiet",
                ],
            )
        self.assertEqual(rc, xyzway_nmpc.XYZWAY_TIMEOUT_RETURN_CODE)


class VrpnPoseReaderTest(unittest.TestCase):
    def test_pose_transform_applies_z_offset_and_yaw_inversion(self) -> None:
        reader = VrpnPoseReader(z_offset=0.2, invert_yaw=True)
        sample = PoseSample(1.0, 2.0, 0.8, 0.3, 10.0)
        transformed = reader._transform_sample(sample)
        self.assertAlmostEqual(transformed.z, 1.0)
        self.assertAlmostEqual(transformed.yaw, -0.3)

    def test_native_json_parser_normalizes_helper_transforms(self) -> None:
        line = json.dumps(
            {
                "x": 1.0,
                "y": 2.0,
                "z": 1.0,
                "yaw": -0.3,
                "timestamp": 10.0,
                "z_offset": 0.2,
                "invert_yaw": True,
            }
        )
        sample = VrpnPoseReader._parse_json_line(line)
        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertAlmostEqual(sample.z, 0.8)
        self.assertAlmostEqual(sample.yaw, 0.3)


class PreflightVrpnHelperTest(unittest.TestCase):
    def test_vrpn_helper_check_accepts_executable_file(self) -> None:
        failures: list[str] = []
        with tempfile.TemporaryDirectory() as tmp:
            helper = Path(tmp) / "vrpn_pose_json"
            helper.write_text("#!/usr/bin/env bash\n")
            helper.chmod(0o755)
            with mock.patch.object(preflight_check, "DEFAULT_NATIVE_VRPN_HELPER", helper):
                preflight_check._check_vrpn_helper(failures=failures)
        self.assertEqual(failures, [])

    def test_vrpn_helper_check_rejects_missing_file(self) -> None:
        failures: list[str] = []
        with tempfile.TemporaryDirectory() as tmp:
            helper = Path(tmp) / "missing"
            with mock.patch.object(preflight_check, "DEFAULT_NATIVE_VRPN_HELPER", helper):
                preflight_check._check_vrpn_helper(failures=failures)
        self.assertTrue(any("missing" in item for item in failures))

    def test_workspace_preflight_checks_vrpn_helper(self) -> None:
        with mock.patch.object(validate_workspace.preflight_check, "main", return_value=0) as mocked:
            ok = validate_workspace._check_preflight()
        self.assertTrue(ok)
        mocked.assert_called_once_with(["--check-vrpn-helper"])

    def test_drone_check_accepts_worker_battery_result(self) -> None:
        failures: list[str] = []
        warnings: list[str] = []
        args = mock.Mock(ip=TEST_RMTT_IP, min_battery=30, drone_check_timeout=1.0)

        def fake_worker(ip, queue_out):
            queue_out.put(("ok", 88))

        with mock.patch.object(rmtt_battery, "_drone_check_worker", side_effect=fake_worker):
            preflight_check._check_drone(args, failures=failures, warnings=warnings)

        self.assertEqual(failures, [])
        self.assertEqual(warnings, [])

    def test_drone_check_times_out_worker(self) -> None:
        failures: list[str] = []
        warnings: list[str] = []
        args = mock.Mock(ip=TEST_RMTT_IP, min_battery=30, drone_check_timeout=0.1)

        def slow_worker(ip, queue_out):
            import time as _time

            _time.sleep(5.0)

        with mock.patch.object(rmtt_battery, "_drone_check_worker", side_effect=slow_worker):
            preflight_check._check_drone(args, failures=failures, warnings=warnings)

        self.assertTrue(any("timed out" in item for item in failures))
        self.assertEqual(warnings, [])

    def test_udp_battery_reader_accepts_text_response(self) -> None:
        class FakeSocket:
            sent = []
            replies = [b"ok", b"88"]

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

            def settimeout(self, timeout):
                self.timeout = timeout

            def bind(self, address):
                self.bound = address

            def sendto(self, payload, address):
                self.sent.append((payload, address))

            def recvfrom(self, size):
                return self.replies.pop(0), (TEST_RMTT_IP, 8889)

        fake = FakeSocket()
        with mock.patch.object(rmtt_battery.socket, "socket", return_value=fake):
            status, payload = rmtt_battery.read_tello_battery_udp(TEST_RMTT_IP, timeout=0.2)

        self.assertEqual((status, payload), ("ok", 88))
        self.assertEqual(fake.sent[0][0], b"command")
        self.assertEqual(fake.sent[1][0], b"battery?")

    def test_isolated_battery_reader_falls_back_after_udp_timeout(self) -> None:
        with mock.patch.object(
            rmtt_battery,
            "read_tello_battery_udp",
            return_value=("timeout", None),
        ), mock.patch.object(
            rmtt_battery,
            "_drone_check_worker",
            side_effect=lambda ip, queue_out: queue_out.put(("ok", 66)),
        ):
            status, payload = rmtt_battery.read_drone_battery_isolated(TEST_RMTT_IP, timeout=1.0)

        self.assertEqual((status, payload), ("ok", 66))


class WorkflowTest(unittest.TestCase):
    def test_workflow_default_commands_do_not_send(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(
            [
                "--stages",
                "preflight,identify,xyzway",
                "--print-only",
                "--work-dir",
                "/tmp/rmtt_workflow_test",
                "--waypoints",
                "example_waypoints.json",
            ]
        )
        commands = rmtt_nmpc_workflow.build_commands(
            args,
            work_dir=Path("/tmp/rmtt_workflow_test"),
            stages=("preflight", "identify", "xyzway"),
        )
        flat = [item for _, command in commands for item in command]
        self.assertNotIn("--send", flat)
        self.assertNotIn("--check-drone", flat)
        self.assertNotIn("--reset-controller-per-waypoint", flat)

    def test_workflow_can_forward_controller_reset_option(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(
            [
                "--stages",
                "xyzway",
                "--print-only",
                "--reset-controller-per-waypoint",
            ]
        )
        commands = rmtt_nmpc_workflow.build_commands(
            args,
            work_dir=Path("/tmp/rmtt_workflow_test"),
            stages=("xyzway",),
        )
        self.assertIn("--reset-controller-per-waypoint", commands[0][1])

    def test_workflow_forwards_vrpn_method_to_flight_stages(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(
            [
                "--stages",
                "preflight,identify,xyzway",
                "--print-only",
                "--vrpn-method",
                "print",
            ]
        )
        commands = dict(
            rmtt_nmpc_workflow.build_commands(
                args,
                work_dir=Path("/tmp/rmtt_workflow_test"),
                stages=("preflight", "identify", "xyzway"),
            )
        )
        self.assertEqual(commands["preflight"][commands["preflight"].index("--method") + 1], "print")
        self.assertEqual(commands["identify"][commands["identify"].index("--method") + 1], "print")
        self.assertEqual(commands["xyzway"][commands["xyzway"].index("--method") + 1], "print")

    def test_workflow_forwards_vrpn_helper_check_when_vrpn_checked(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(
            [
                "--stages",
                "preflight",
                "--print-only",
                "--check-vrpn",
                "--vrpn-method",
                "native",
            ]
        )
        commands = dict(
            rmtt_nmpc_workflow.build_commands(
                args,
                work_dir=Path("/tmp/rmtt_workflow_test"),
                stages=("preflight",),
            )
        )
        self.assertIn("--check-vrpn-helper", commands["preflight"])

    def test_workflow_skips_vrpn_helper_check_for_print_method(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(
            [
                "--stages",
                "preflight",
                "--print-only",
                "--check-vrpn",
                "--vrpn-method",
                "print",
            ]
        )
        commands = dict(
            rmtt_nmpc_workflow.build_commands(
                args,
                work_dir=Path("/tmp/rmtt_workflow_test"),
                stages=("preflight",),
            )
        )
        self.assertNotIn("--check-vrpn-helper", commands["preflight"])

    def test_workflow_requires_real_model_when_xyzway_skips_identify(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(
            [
                "--stages",
                "preflight,xyzway",
                "--print-only",
            ]
        )
        commands = dict(
            rmtt_nmpc_workflow.build_commands(
                args,
                work_dir=Path("/tmp/rmtt_workflow_test"),
                stages=("preflight", "xyzway"),
            )
        )
        self.assertIn("--fail-on-bootstrap", commands["preflight"])
        self.assertIn("--require-real-model", commands["xyzway"])

    def test_workflow_allows_bootstrap_xyzway_when_explicit(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(
            [
                "--stages",
                "preflight,xyzway",
                "--print-only",
                "--allow-bootstrap-xyzway",
            ]
        )
        commands = dict(
            rmtt_nmpc_workflow.build_commands(
                args,
                work_dir=Path("/tmp/rmtt_workflow_test"),
                stages=("preflight", "xyzway"),
            )
        )
        self.assertNotIn("--fail-on-bootstrap", commands["preflight"])
        self.assertIn("--allow-bootstrap-model", commands["xyzway"])

    def test_workflow_with_identify_does_not_require_old_model_for_xyzway(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(
            [
                "--stages",
                "preflight,identify,xyzway",
                "--print-only",
            ]
        )
        commands = dict(
            rmtt_nmpc_workflow.build_commands(
                args,
                work_dir=Path("/tmp/rmtt_workflow_test"),
                stages=("preflight", "identify", "xyzway"),
            )
        )
        self.assertNotIn("--require-real-model", commands["xyzway"])

    def test_workflow_assigns_takeoff_and_land_to_outer_flight_stages(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(
            [
                "--stages",
                "preflight,identify,xyzway",
                "--send",
                "--confirm-risk",
                "--takeoff",
                "--land",
            ]
        )
        commands = dict(
            rmtt_nmpc_workflow.build_commands(
                args,
                work_dir=Path("/tmp/rmtt_workflow_test"),
                stages=("preflight", "identify", "xyzway"),
            )
        )
        self.assertIn("--takeoff", commands["identify"])
        self.assertIn("--confirm-risk", commands["identify"])
        self.assertIn("--allow-open-airborne-handoff", commands["identify"])
        self.assertNotIn("--land", commands["identify"])
        self.assertNotIn("--takeoff", commands["xyzway"])
        self.assertIn("--confirm-risk", commands["xyzway"])
        self.assertIn("--land", commands["xyzway"])

    def test_workflow_identify_only_can_takeoff_and_land(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(
            [
                "--stages",
                "identify",
                "--send",
                "--confirm-risk",
                "--takeoff",
                "--land",
            ]
        )
        commands = dict(
            rmtt_nmpc_workflow.build_commands(
                args,
                work_dir=Path("/tmp/rmtt_workflow_test"),
                stages=("identify",),
            )
        )
        self.assertIn("--takeoff", commands["identify"])
        self.assertIn("--confirm-risk", commands["identify"])
        self.assertNotIn("--allow-open-airborne-handoff", commands["identify"])
        self.assertIn("--land", commands["identify"])

    def test_workflow_refuses_send_without_confirmation(self) -> None:
        rc = rmtt_nmpc_workflow.main(["--send", "--print-only", "--stages", "preflight"])
        self.assertEqual(rc, 2)

    def test_workflow_refuses_takeoff_without_land(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            work_dir = Path(tmp) / "bad_takeoff"
            rc = rmtt_nmpc_workflow.main(
                [
                    "--send",
                    "--confirm-risk",
                    "--takeoff",
                    "--stages",
                    "preflight,identify,xyzway",
                    "--work-dir",
                    str(work_dir),
                ]
            )
        self.assertEqual(rc, 2)
        self.assertFalse(work_dir.exists())

    def test_workflow_print_only_does_not_create_work_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            work_dir = Path(tmp) / "dry_run"
            rc = rmtt_nmpc_workflow.main(
                [
                    "--print-only",
                    "--stages",
                    "preflight",
                    "--work-dir",
                    str(work_dir),
                ]
            )
            self.assertEqual(rc, 0)
            self.assertFalse(work_dir.exists())

    def test_workflow_writes_manifest_on_success(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            work_dir = Path(tmp) / "run"
            with mock.patch.object(rmtt_nmpc_workflow, "_run_stage", return_value=0):
                rc = rmtt_nmpc_workflow.main(
                    [
                        "--stages",
                        "preflight,xyzway",
                        "--allow-bootstrap-xyzway",
                        "--work-dir",
                        str(work_dir),
                        "--waypoints",
                        "example_waypoints.json",
                    ]
                )
            manifest = json.loads((work_dir / rmtt_nmpc_workflow.WORKFLOW_MANIFEST).read_text())
        self.assertEqual(rc, 0)
        self.assertEqual(manifest["status"], "ok")
        self.assertEqual(manifest["stages"], ["preflight", "xyzway"])
        self.assertEqual([item["stage"] for item in manifest["results"]], ["preflight", "xyzway"])
        self.assertIn("xyzway_log_csv", manifest["artifacts"])
        self.assertIn("preflight_log", manifest["artifacts"])
        self.assertIn("xyzway_log", manifest["artifacts"])
        self.assertTrue(all("log" in item for item in manifest["results"]))
        self.assertIn("commands", manifest)

    def test_workflow_check_evidence_runs_after_success(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            work_dir = Path(tmp) / "run"
            with mock.patch.object(rmtt_nmpc_workflow, "_run_stage", return_value=0), \
                mock.patch.object(rmtt_nmpc_workflow, "_check_workflow_evidence", return_value=0) as evidence:
                rc = rmtt_nmpc_workflow.main(
                    [
                        "--stages",
                        "preflight,xyzway",
                        "--allow-bootstrap-xyzway",
                        "--check-evidence",
                        "--work-dir",
                        str(work_dir),
                    ]
                )
            manifest = json.loads((work_dir / rmtt_nmpc_workflow.WORKFLOW_MANIFEST).read_text())
        self.assertEqual(rc, 0)
        evidence.assert_called_once()
        self.assertFalse(evidence.call_args.kwargs["strict_hardware"])
        self.assertEqual(manifest["status"], "ok")
        self.assertEqual(manifest["evidence"]["status"], "ok")
        self.assertEqual(manifest["evidence"]["returncode"], 0)

    def test_workflow_check_evidence_requires_hardware_evidence_when_sending(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            work_dir = Path(tmp) / "run"
            with mock.patch.object(rmtt_nmpc_workflow, "_run_stage", return_value=0), \
                mock.patch.object(rmtt_nmpc_workflow, "_check_workflow_evidence", return_value=0) as evidence:
                rc = rmtt_nmpc_workflow.main(
                    [
                        "--stages",
                        "preflight",
                        "--send",
                        "--confirm-risk",
                        "--check-evidence",
                        "--work-dir",
                        str(work_dir),
                    ]
                )
        self.assertEqual(rc, 0)
        self.assertTrue(evidence.call_args.kwargs["strict_hardware"])

    def test_workflow_strict_evidence_requires_full_workflow(self) -> None:
        args = rmtt_nmpc_workflow._parse_args(["--check-evidence"])
        fake_result = workflow_evidence.EvidenceResult(ok=True, failures=(), warnings=())
        with tempfile.TemporaryDirectory() as tmp:
            manifest = Path(tmp) / rmtt_nmpc_workflow.WORKFLOW_MANIFEST
            manifest.write_text("{}")
            with mock.patch.object(
                workflow_evidence,
                "check_workflow_evidence",
                return_value=fake_result,
            ) as checker, mock.patch.object(
                workflow_evidence,
                "print_workflow_summary",
                return_value=None,
            ) as summary:
                rc = rmtt_nmpc_workflow._check_workflow_evidence(
                    args,
                    manifest,
                    strict_hardware=True,
                )
        self.assertEqual(rc, 0)
        self.assertTrue(checker.call_args.kwargs["require_full_workflow"])
        self.assertTrue(checker.call_args.kwargs["require_managed_flight"])
        summary.assert_called_once_with(manifest)

    def test_workflow_check_evidence_failure_fails_workflow(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            work_dir = Path(tmp) / "run"
            with mock.patch.object(rmtt_nmpc_workflow, "_run_stage", return_value=0), \
                mock.patch.object(rmtt_nmpc_workflow, "_check_workflow_evidence", return_value=6):
                rc = rmtt_nmpc_workflow.main(
                    [
                        "--stages",
                        "preflight,xyzway",
                        "--allow-bootstrap-xyzway",
                        "--check-evidence",
                        "--work-dir",
                        str(work_dir),
                    ]
                )
            manifest = json.loads((work_dir / rmtt_nmpc_workflow.WORKFLOW_MANIFEST).read_text())
        self.assertEqual(rc, 6)
        self.assertEqual(manifest["status"], "evidence_failed")
        self.assertEqual(manifest["evidence"]["status"], "failed")
        self.assertEqual(manifest["evidence"]["returncode"], 6)

    def test_workflow_manifest_records_failed_stage(self) -> None:
        def fake_run_stage(command, *, log_path):
            return 5 if "rmtt_control.xyzway_nmpc" in command else 0

        with tempfile.TemporaryDirectory() as tmp:
            work_dir = Path(tmp) / "run"
            with mock.patch.object(rmtt_nmpc_workflow, "_run_stage", side_effect=fake_run_stage):
                rc = rmtt_nmpc_workflow.main(
                    [
                        "--stages",
                        "preflight,xyzway",
                        "--allow-bootstrap-xyzway",
                        "--work-dir",
                        str(work_dir),
                    ]
                )
            manifest = json.loads((work_dir / rmtt_nmpc_workflow.WORKFLOW_MANIFEST).read_text())
        self.assertEqual(rc, 5)
        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(manifest["failed_stage"], "xyzway")
        self.assertEqual([item["returncode"] for item in manifest["results"]], [0, 5])

    def test_workflow_run_stage_writes_log(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "stage.log"
            rc = rmtt_nmpc_workflow._run_stage(
                [
                    sys.executable,
                    "-c",
                    "print('stage-output')",
                ],
                log_path=log_path,
            )
            log_text = log_path.read_text()
        self.assertEqual(rc, 0)
        self.assertIn("stage-output", log_text)


class WorkflowEvidenceTest(unittest.TestCase):
    def test_workflow_evidence_accepts_complete_run_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
                model_thresholds=QualityThresholds(
                    min_samples=30,
                    min_r2=0.2,
                    min_vaf=0.2,
                    max_nrmse=0.8,
                    fail_on_bootstrap=True,
                ),
            )
        self.assertTrue(result.ok, result.failures)

    def test_workflow_evidence_rejects_failed_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="failed")
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(min_rows=20, min_signed_samples=5),
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("workflow status" in item for item in result.failures))

    def test_workflow_evidence_rejects_xyzway_terminal_error(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            _write_xyzway_log_csv(root / "xyzway_run.csv", x=0.5, y=0.5)
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("terminal xy error" in item for item in result.failures))

    def test_workflow_evidence_rejects_xyzway_stick_over_limit(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            _write_xyzway_log_csv(root / "xyzway_run.csv", roll=101)
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("exceeds RMTT stick limit" in item for item in result.failures))

    def test_workflow_evidence_rejects_invalid_xyzway_nmpc_json(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            _write_xyzway_log_csv(root / "xyzway_run.csv", nmpc_json="{bad")
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("invalid JSON" in item for item in result.failures))

    def test_workflow_evidence_rejects_empty_xyzway_nmpc_json(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            _write_xyzway_log_csv(root / "xyzway_run.csv", nmpc_json="{}")
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("missing NMPC reason" in item for item in result.failures))

    def test_workflow_evidence_rejects_nonfinal_waypoint_log(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            _write_xyzway_log_csv(root / "xyzway_run.csv", target_x=0.0, x=0.0)
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("target does not match final waypoint" in item for item in result.failures))

    def test_workflow_evidence_rejects_identification_over_limit(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            _write_axis_quality_csv(
                root / "identification" / "identify_pitch_fixture.csv",
                axis="pitch",
                command=RMTT_MAX_IDENTIFICATION_AMPLITUDE + 1,
            )
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("exceeds identification limit" in item for item in result.failures))

    def test_workflow_evidence_rejects_missing_fit_comparison(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            (root / "comparisons" / "pitch_comparison.csv").unlink()
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("pitch comparison CSV" in item for item in result.failures))

    def test_workflow_evidence_rejects_model_source_outside_workflow_identification(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            _write_quality_model(root / "rmtt_velocity_model.json", bootstrap=False, source_dir=root / "other")
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("fit.source_csv" in item for item in result.failures))

    def test_workflow_evidence_resolves_paths_relative_to_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok", relative_paths=True)
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
            )
        self.assertTrue(result.ok, result.failures)

    def test_workflow_evidence_can_require_hardware_run_mode(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(
                root,
                status="ok",
                send=True,
                managed_flight=True,
                preflight_flags=("--check-vrpn", "--check-drone"),
                preflight_log_extra="vrpn: x=0.100 y=0.000 z=0.800 yaw=0.000 age=0.010s\ndrone: ip={0} battery=88%\n".format(TEST_RMTT_IP),
            )
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
                require_send=True,
                require_vrpn_check=True,
                require_drone_check=True,
                require_full_workflow=True,
                require_managed_flight=True,
            )
        self.assertTrue(result.ok, result.failures)

    def test_workflow_evidence_can_require_managed_flight(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok", managed_flight=True)
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
                require_managed_flight=True,
            )
        self.assertTrue(result.ok, result.failures)

    def test_workflow_evidence_rejects_missing_managed_flight(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
                require_managed_flight=True,
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("--takeoff" in item for item in result.failures))
        self.assertTrue(any("--allow-open-airborne-handoff" in item for item in result.failures))
        self.assertTrue(any("xyzway command did not include --land" in item for item in result.failures))

    def test_workflow_evidence_can_require_full_workflow(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
                require_full_workflow=True,
            )
        self.assertTrue(result.ok, result.failures)

    def test_workflow_evidence_rejects_incomplete_required_workflow(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            data = json.loads(Path(manifest).read_text())
            data["stages"] = ["preflight", "xyzway"]
            data["results"] = [item for item in data["results"] if item["stage"] != "identify"]
            Path(manifest).write_text(json.dumps(data))
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
                require_full_workflow=True,
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("required full workflow" in item for item in result.failures))

    def test_workflow_evidence_rejects_missing_hardware_run_mode(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
                require_send=True,
                require_vrpn_check=True,
                require_drone_check=True,
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("--send" in item for item in result.failures))
        self.assertTrue(any("--check-vrpn" in item for item in result.failures))
        self.assertTrue(any("--check-drone" in item for item in result.failures))

    def test_workflow_evidence_rejects_missing_hardware_log_output(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(
                root,
                status="ok",
                send=True,
                preflight_flags=("--check-vrpn", "--check-drone"),
            )
            result = workflow_evidence.check_workflow_evidence(
                manifest,
                csv_thresholds=CsvQualityThresholds(
                    min_rows=20,
                    min_signed_samples=5,
                    min_position_span=0.05,
                    min_yaw_span_deg=5.0,
                ),
                require_send=True,
                require_vrpn_check=True,
                require_drone_check=True,
            )
        self.assertFalse(result.ok)
        self.assertTrue(any("VRPN sample" in item for item in result.failures))
        self.assertTrue(any("drone battery" in item for item in result.failures))

    def test_workflow_evidence_summary_prints_key_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = _write_workflow_evidence_fixture(root, status="ok")
            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                workflow_evidence.print_workflow_summary(manifest)
            text = stream.getvalue()
        self.assertIn("summary:", text)
        self.assertIn("status: ok", text)
        self.assertIn("identify pitch: rows=24", text)
        self.assertIn("xyzway: rows=1", text)
        self.assertIn("error_xy=0.000", text)


def _write_filter_csv(path: Path) -> None:
    fields = [
        "elapsed",
        "axis",
        "signal_kind",
        "step_name",
        "step_index",
        "command_offset",
        "requested_pitch",
        "pitch",
        "x",
        "y",
        "z",
        "yaw_pose",
        "safety_ok",
        "step_elapsed",
    ]
    rows = []
    for index in range(20):
        rows.append(
            {
                "elapsed": index * 0.1,
                "axis": "pitch",
                "signal_kind": "multistep",
                "step_name": "step",
                "step_index": 1,
                "command_offset": 20,
                "requested_pitch": 20,
                "pitch": 20,
                "x": 0.01 * index,
                "y": 0,
                "z": 0.8,
                "yaw_pose": 0,
                "safety_ok": 1,
                "step_elapsed": 0.3 + index * 0.1,
            }
        )
    rows.append(
        {
            "elapsed": 3.0,
            "axis": "pitch",
            "signal_kind": "recenter",
            "step_name": "recenter",
            "step_index": -1,
            "command_offset": 0,
            "requested_pitch": 0,
            "pitch": 0,
            "x": 0,
            "y": 0,
            "z": 0.8,
            "yaw_pose": 0,
            "safety_ok": 1,
            "step_elapsed": 0,
        }
    )
    rows.append({**rows[-2], "elapsed": 4.0, "safety_ok": 0})
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _write_quality_csv(path: Path, *, axis: str, safety_failed: bool, one_sided: bool) -> None:
    fields = [
        "elapsed",
        "axis",
        "signal_kind",
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
        "safety_ok",
    ]
    rows = []
    for index in range(24):
        sign = 1 if one_sided or index < 12 else -1
        command = 20 * sign
        x = 0.02 * index if sign > 0 else 0.24 - 0.02 * (index - 12)
        row = {
            "elapsed": index * 0.1,
            "axis": axis,
            "signal_kind": "step",
            "requested_roll": 0,
            "requested_pitch": command if axis == "pitch" else 0,
            "requested_throttle": command if axis == "throttle" else 0,
            "requested_yaw": command if axis == "yaw" else 0,
            "roll": 0,
            "pitch": command if axis == "pitch" else 0,
            "throttle": command if axis == "throttle" else 0,
            "yaw": command if axis == "yaw" else 0,
            "x": x,
            "y": 0.0,
            "z": 0.8,
            "yaw_pose": 0.0,
            "safety_ok": 0 if safety_failed and index == 5 else 1,
        }
        rows.append(row)
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _write_quality_model(path: Path, *, bootstrap: bool, source_dir: Path | None = None) -> None:
    axes = {}
    for axis in ("pitch", "roll", "throttle", "yaw"):
        fit = (
            {"bootstrap": True}
            if bootstrap
            else {"sample_count": 80, "r2": 0.8, "vaf": 0.75, "nrmse": 0.2}
        )
        if source_dir is not None and not bootstrap:
            fit["source_csv"] = [str(source_dir / f"identify_{axis}_fixture.csv")]
        axes[axis] = {
            "K": 1.0 if axis != "yaw" else 100.0,
            "tau": 0.3,
            "Td": 0.05,
            "vmax": 1.0 if axis != "yaw" else 80.0,
            "amax": 2.0 if axis != "yaw" else 200.0,
            "response": "fopdt",
            "unit": "m_s",
            "fit": fit,
            "validation": {},
        }
    path.write_text(json.dumps({"metadata": {}, "axes": axes}))


def _write_workflow_evidence_fixture(
    root: Path,
    *,
    status: str,
    relative_paths: bool = False,
    send: bool = False,
    managed_flight: bool = False,
    preflight_flags: tuple[str, ...] = (),
    preflight_log_extra: str = "",
) -> Path:
    identification_dir = root / "identification"
    identification_dir.mkdir()
    for axis in ("pitch", "roll", "throttle", "yaw"):
        _write_axis_quality_csv(identification_dir / f"identify_{axis}_fixture.csv", axis=axis)
    comparison_dir = root / "comparisons"
    comparison_dir.mkdir()
    for axis in ("pitch", "roll", "throttle", "yaw"):
        _write_comparison_csv(comparison_dir / f"{axis}_comparison.csv")
    model_path = root / "rmtt_velocity_model.json"
    _write_quality_model(model_path, bootstrap=False, source_dir=identification_dir)
    waypoints_path = root / "waypoints.json"
    waypoints_path.write_text(json.dumps([{"x": 0.1, "y": 0.0, "z": 0.8, "yaw_deg": 0.0}]))
    xyzway_csv = root / "xyzway_run.csv"
    _write_xyzway_log_csv(xyzway_csv)
    def store(path: Path) -> str:
        return path.name if relative_paths else str(path)

    artifacts = {
        "manifest": store(root / rmtt_nmpc_workflow.WORKFLOW_MANIFEST),
        "model": store(model_path),
        "preflight_log": store(root / "preflight.log"),
        "identify_log": store(root / "identify.log"),
        "xyzway_log": store(root / "xyzway.log"),
        "identification_dir": "identification" if relative_paths else str(identification_dir),
        "comparison_dir": "comparisons" if relative_paths else str(comparison_dir),
        "xyzway_log_csv": store(xyzway_csv),
    }
    (root / "preflight.log").write_text("preflight.log\n{0}".format(preflight_log_extra))
    for name in ("identify.log", "xyzway.log"):
        (root / name).write_text("{0}\n".format(name))
    identify_argv = ["python3", "identify_pipeline.py"]
    xyzway_argv = ["python3", "xyzway_nmpc.py"]
    if managed_flight:
        identify_argv.extend(["--takeoff", "--allow-open-airborne-handoff"])
        xyzway_argv.append("--land")
    manifest = {
        "status": status,
        "stages": ["preflight", "identify", "xyzway"],
        "send": send,
        "takeoff": managed_flight,
        "land": managed_flight,
        "waypoints": store(waypoints_path),
        "artifacts": artifacts,
        "commands": [
            {"stage": "preflight", "argv": ["python3", "preflight_check.py", *preflight_flags]},
            {"stage": "identify", "argv": identify_argv},
            {"stage": "xyzway", "argv": xyzway_argv},
        ],
        "results": [
            {"stage": "preflight", "returncode": 0, "log": artifacts["preflight_log"]},
            {"stage": "identify", "returncode": 0, "log": artifacts["identify_log"]},
            {"stage": "xyzway", "returncode": 0 if status == "ok" else 3, "log": artifacts["xyzway_log"]},
        ],
    }
    path = root / rmtt_nmpc_workflow.WORKFLOW_MANIFEST
    path.write_text(json.dumps(manifest))
    return path


def _write_axis_quality_csv(path: Path, *, axis: str, command: int = 20) -> None:
    fields = [
        "elapsed",
        "axis",
        "signal_kind",
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
        "safety_ok",
    ]
    rows = []
    for index in range(24):
        sign = 1 if index < 12 else -1
        signed_command = command * sign
        progress = 0.02 * index if sign > 0 else 0.24 - 0.02 * (index - 12)
        row = {
            "elapsed": index * 0.1,
            "axis": axis,
            "signal_kind": "step",
            "requested_roll": signed_command if axis == "roll" else 0,
            "requested_pitch": signed_command if axis == "pitch" else 0,
            "requested_throttle": signed_command if axis == "throttle" else 0,
            "requested_yaw": signed_command if axis == "yaw" else 0,
            "roll": signed_command if axis == "roll" else 0,
            "pitch": signed_command if axis == "pitch" else 0,
            "throttle": signed_command if axis == "throttle" else 0,
            "yaw": signed_command if axis == "yaw" else 0,
            "x": progress if axis == "pitch" else 0.0,
            "y": progress if axis == "roll" else 0.0,
            "z": 0.8 + progress if axis == "throttle" else 0.8,
            "yaw_pose": progress if axis == "yaw" else 0.0,
            "safety_ok": 1,
        }
        rows.append(row)
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _write_comparison_csv(path: Path) -> None:
    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["t", "u", "actual", "predicted", "residual"])
        writer.writerow([0.0, 0.2, 0.1, 0.09, 0.01])


def _write_xyzway_log_csv(
    path: Path,
    *,
    target_x: float = 0.1,
    target_y: float = 0.0,
    target_z: float = 0.8,
    target_yaw_deg: float = 0.0,
    x: float = 0.1,
    y: float = 0.0,
    z: float = 0.8,
    yaw: float = 0.0,
    roll: int = 0,
    pitch: int = 0,
    throttle: int = 0,
    yaw_cmd: int = 0,
    nmpc_json: str | None = None,
) -> None:
    fields = [
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
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerow(
            {
                "time": 1.0,
                "waypoint_index": 1,
                "target_x": target_x,
                "target_y": target_y,
                "target_z": target_z,
                "target_yaw_deg": target_yaw_deg,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "roll": roll,
                "pitch": pitch,
                "throttle": throttle,
                "yaw_cmd": yaw_cmd,
                "reason": "ok",
                "enabled": 1,
                "mission_reason": "ok",
                "mission_enabled": 1,
                "authority_u": 0.1,
                "yaw_authority_u": 0.1,
                "nmpc_json": nmpc_json or json.dumps({"nmpc_mission_reason": "ok"}),
            }
        )


if __name__ == "__main__":
    unittest.main()
