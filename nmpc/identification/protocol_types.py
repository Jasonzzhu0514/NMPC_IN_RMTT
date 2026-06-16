from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ExcitationStep:
    axis: str
    offset: int
    duration_sec: float
    name: str
    index: int
    signal_kind: str = "multistep"
    amplitude: int = 0
    requires_recenter: bool = False

    @property
    def is_center(self) -> bool:
        return self.offset == 0

