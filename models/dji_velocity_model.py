"""Identified DJI stick-to-velocity model definitions."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


VALID_MODEL_AXES = ("pitch", "roll", "throttle", "yaw")
STICK_COLUMNS = {
    "pitch": "pitch_u",
    "roll": "roll_u",
    "throttle": "throttle_u",
    "yaw": "yaw_u",
}
AXIS_POSITION_NAMES = {
    "pitch": "x",
    "roll": "y",
    "throttle": "z",
    "yaw": "yaw",
}
AXIS_VELOCITY_NAMES = {
    "pitch": "vx",
    "roll": "vy",
    "throttle": "vz",
    "yaw": "yaw_rate",
}
DEFAULT_MODEL_PATH = Path(__file__).with_name("rmtt_velocity_model.json")


@dataclass(frozen=True)
class AxisVelocityModel:
    axis: str
    K: float
    tau: float
    Td: float
    vmax: float
    amax: float
    response: str
    unit: str
    fit: dict[str, Any]
    validation: dict[str, Any]

    @property
    def position_name(self) -> str:
        return AXIS_POSITION_NAMES[self.axis]

    @property
    def velocity_name(self) -> str:
        return AXIS_VELOCITY_NAMES[self.axis]


@dataclass(frozen=True)
class DJIVelocityModel:
    axes: dict[str, AxisVelocityModel]
    metadata: dict[str, Any]
    source_path: Path | None = None

    def axis(self, name: str) -> AxisVelocityModel:
        key = name.strip().lower()
        try:
            return self.axes[key]
        except KeyError as exc:
            available = ", ".join(sorted(self.axes))
            raise KeyError(f"Axis {key!r} is not in model; available axes: {available}") from exc

    def require_axes(self, axes: tuple[str, ...] = VALID_MODEL_AXES) -> None:
        missing = [axis for axis in axes if axis not in self.axes]
        if missing:
            raise ValueError(f"Model is missing required axes: {', '.join(missing)}")


def load_velocity_model(path: str | Path = DEFAULT_MODEL_PATH) -> DJIVelocityModel:
    model_path = Path(path).expanduser()
    if not model_path.is_absolute():
        model_path = Path(__file__).resolve().parent / model_path
    with model_path.open() as file:
        document = json.load(file)
    model = velocity_model_from_document(document, source_path=model_path)
    return model


def velocity_model_from_document(
    document: dict[str, Any],
    *,
    source_path: Path | None = None,
) -> DJIVelocityModel:
    axes_document = document.get("axes")
    if not isinstance(axes_document, dict) or not axes_document:
        raise ValueError("Velocity model document must contain a non-empty 'axes' object.")

    axes: dict[str, AxisVelocityModel] = {}
    for axis, values in axes_document.items():
        key = axis.strip().lower()
        if key not in VALID_MODEL_AXES:
            raise ValueError(f"Unsupported model axis {axis!r}.")
        axes[key] = AxisVelocityModel(
            axis=key,
            K=_required_float(values, "K"),
            tau=_positive_float(values, "tau"),
            Td=max(0.0, _required_float(values, "Td")),
            vmax=abs(_required_float(values, "vmax")),
            amax=abs(_required_float(values, "amax")),
            response=str(values.get("response", "")),
            unit=str(values.get("unit", "")),
            fit=dict(values.get("fit", {})),
            validation=dict(values.get("validation", {})),
        )
    return DJIVelocityModel(
        axes=axes,
        metadata=dict(document.get("metadata", {})),
        source_path=source_path,
    )


def _required_float(values: dict[str, Any], key: str) -> float:
    try:
        value = float(values[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Model axis field {key!r} must be numeric.") from exc
    if value != value:
        raise ValueError(f"Model axis field {key!r} must be finite.")
    return value


def _positive_float(values: dict[str, Any], key: str) -> float:
    value = _required_float(values, key)
    if value <= 0.0:
        raise ValueError(f"Model axis field {key!r} must be positive.")
    return value
