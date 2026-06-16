#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees
from typing import Iterable

from robomaster import config, robot
from rmtt_config import DEFAULT_RMTT_IP


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class StickCommand:
    roll: int = 0
    pitch: int = 0
    throttle: int = 0
    yaw: int = 0

    def clamped(self, limit: int = 100) -> "StickCommand":
        return StickCommand(
            roll=clamp_stick(self.roll, limit),
            pitch=clamp_stick(self.pitch, limit),
            throttle=clamp_stick(self.throttle, limit),
            yaw=clamp_stick(self.yaw, limit),
        )


class RMTTClient:
    def __init__(self, ip: str | None = None) -> None:
        self.ip = ip
        self.drone: robot.Drone | None = None

    def connect(self) -> robot.Drone:
        if not self.ip:
            raise ValueError("RMTT IP is required. Pass --ip or set RMTT_IP.")
        config.ROBOT_IP_STR = self.ip
        self.drone = robot.Drone()
        self.drone.initialize(conn_type="sta")
        return self.drone

    def close(self) -> None:
        if self.drone is not None:
            self.drone.close()
            self.drone = None

    def send_stick(self, command: StickCommand) -> None:
        if self.drone is None:
            raise RuntimeError("RMTTClient is not connected")
        command = command.clamped()
        self.drone.flight.rc(
            a=command.roll,
            b=command.pitch,
            c=command.throttle,
            d=command.yaw,
        )

    def center(self) -> None:
        self.send_stick(StickCommand())

    def takeoff(self):
        if self.drone is None:
            raise RuntimeError("RMTTClient is not connected")
        return self.drone.flight.takeoff()

    def land(self):
        if self.drone is None:
            raise RuntimeError("RMTTClient is not connected")
        return self.drone.flight.land()

    def battery_percent(self) -> int | None:
        if self.drone is None:
            raise RuntimeError("RMTTClient is not connected")
        return self.drone.battery.get_battery()


def clamp_stick(value: float, limit: int = 100) -> int:
    return int(max(-limit, min(limit, round(value))))


def normalized_to_rc(value: float, *, limit: int = 100) -> int:
    value = max(-1.0, min(1.0, float(value)))
    return clamp_stick(value * limit, limit)


def quaternion_to_yaw_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return degrees(atan2(siny_cosp, cosy_cosp))


def pose_from_xyzyaw(x: float, y: float, z: float, yaw: float) -> Pose:
    return Pose(x=x, y=y, z=z, yaw=yaw)


def center_commands(commands: Iterable[StickCommand] | None = None) -> None:
    _ = commands
