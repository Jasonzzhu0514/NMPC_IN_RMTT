#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from dataclasses import dataclass
from math import degrees
from pathlib import Path
import time

from nmpc.flight_controller import NmpcFlightController
from nmpc.flight.types import NmpcFlightControlResult
from nmpc.mission_controller import NmpcMissionController
from nmpc.mission.types import NmpcMissionControlDecision
from rmtt_control.pose_source import PoseSample
from rmtt.adapter import StickCommand


RMTT_STICK_MIN = -100
RMTT_STICK_NEUTRAL = 0
RMTT_STICK_MAX = 100


@dataclass(frozen=True)
class NmpcRmttOutput:
    command: StickCommand
    result: NmpcFlightControlResult


class NmpcRmttBridge:
    def __init__(
        self,
        controller: NmpcFlightController | None = None,
        *,
        model_path: str | Path | None = None,
    ) -> None:
        self.controller = controller or NmpcFlightController(
            neutral=RMTT_STICK_NEUTRAL,
            stick_min=RMTT_STICK_MIN,
            stick_max=RMTT_STICK_MAX,
            **({"model_path": model_path} if model_path is not None else {}),
        )

    def reset(self) -> None:
        self.controller.reset()

    def compute(
        self,
        *,
        pose: PoseSample,
        target_x: float,
        target_y: float,
        target_z: float,
        target_yaw_deg: float,
        phase: str = "MOVE-XYZ",
    ) -> NmpcRmttOutput:
        timestamp = pose.timestamp if pose.timestamp is not None else time.time()
        yaw_deg = _pose_yaw_degrees(pose.yaw)
        result = self.controller.compute(
            timestamp=timestamp,
            current_x=pose.x,
            current_y=pose.y,
            current_z=pose.z,
            current_yaw=yaw_deg,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_yaw=target_yaw_deg,
            phase=phase,
        )
        return NmpcRmttOutput(
            command=StickCommand(
                roll=result.roll_absolute,
                pitch=result.pitch_absolute,
                throttle=result.throttle_absolute,
                yaw=result.yaw_absolute,
            ),
            result=result,
        )


@dataclass(frozen=True)
class NmpcMissionRmttOutput:
    command: StickCommand
    decision: NmpcMissionControlDecision

    @property
    def result(self) -> NmpcFlightControlResult:
        return self.decision.nmpc_flight


class NmpcMissionRmttBridge:
    def __init__(
        self,
        controller: NmpcMissionController | None = None,
        *,
        model_path: str | Path | None = None,
    ) -> None:
        nmpc_flight = None
        if controller is None and model_path is not None:
            nmpc_flight = NmpcFlightController(
                neutral=RMTT_STICK_NEUTRAL,
                stick_min=RMTT_STICK_MIN,
                stick_max=RMTT_STICK_MAX,
                model_path=model_path,
            )
        self.controller = controller or NmpcMissionController(
            neutral=RMTT_STICK_NEUTRAL,
            stick_min=RMTT_STICK_MIN,
            stick_max=RMTT_STICK_MAX,
            nmpc_flight=nmpc_flight,
        )

    def reset(self) -> None:
        self.controller.reset()

    def compute(
        self,
        *,
        pose: PoseSample,
        target_x: float,
        target_y: float,
        target_z: float,
        target_yaw_deg: float,
        phase: str = "MOVE-XYZ",
        final_target: bool = False,
    ) -> NmpcMissionRmttOutput:
        timestamp = pose.timestamp if pose.timestamp is not None else time.time()
        yaw_deg = _pose_yaw_degrees(pose.yaw)
        decision = self.controller.compute(
            timestamp=timestamp,
            current_x=pose.x,
            current_y=pose.y,
            current_z=pose.z,
            current_yaw=yaw_deg,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_yaw=target_yaw_deg,
            phase=phase,
            pid_roll=RMTT_STICK_NEUTRAL,
            pid_pitch=RMTT_STICK_NEUTRAL,
            pid_throttle=RMTT_STICK_NEUTRAL,
            pid_yaw=RMTT_STICK_NEUTRAL,
            final_target=final_target,
        )
        return NmpcMissionRmttOutput(
            command=StickCommand(
                roll=decision.roll,
                pitch=decision.pitch,
                throttle=decision.throttle,
                yaw=decision.yaw,
            ),
            decision=decision,
        )


def _pose_yaw_degrees(yaw: float) -> float:
    # VrpnPoseReader publishes radians; StaticPoseSource can be used with either
    # small radian values or explicit degree values. Treat large values as deg.
    yaw_f = float(yaw)
    if abs(yaw_f) <= 6.5:
        return degrees(yaw_f)
    return yaw_f
