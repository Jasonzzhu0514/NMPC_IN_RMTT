#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class PoseSample:
    x: float
    y: float
    z: float
    yaw: float
    timestamp: float | None = None


class PoseSource:
    def latest(self) -> Optional[PoseSample]:
        raise NotImplementedError


class StaticPoseSource(PoseSource):
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw: float = 0.0) -> None:
        self._sample = PoseSample(x=x, y=y, z=z, yaw=yaw)

    def latest(self) -> Optional[PoseSample]:
        return self._sample
