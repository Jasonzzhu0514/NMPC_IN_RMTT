from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SampleSeries:
    axis: str
    response_name: str
    source: str
    t: list[float]
    u: list[float]
    y: list[float]
    metadata: dict[str, float | str | bool] | None = None
    segment_starts: list[int] | None = None


@dataclass(frozen=True)
class ModelParams:
    K: float
    tau: float
    Td: float
    vmax: float
    amax: float


@dataclass(frozen=True)
class FitResult:
    params: ModelParams
    predicted: list[float]
    rmse: float
    nrmse: float
    r2: float
    vaf: float
