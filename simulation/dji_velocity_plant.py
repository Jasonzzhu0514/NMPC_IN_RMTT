"""Discrete plant for identified DJI stick-to-velocity dynamics."""

from __future__ import annotations

from dataclasses import dataclass, field

from models.dji_velocity_model import (
    AXIS_POSITION_NAMES,
    AXIS_VELOCITY_NAMES,
    DJIVelocityModel,
    VALID_MODEL_AXES,
    AxisVelocityModel,
)


@dataclass(frozen=True)
class PlantState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0

    def position(self, axis: str) -> float:
        return float(getattr(self, AXIS_POSITION_NAMES[axis]))

    def velocity(self, axis: str) -> float:
        return float(getattr(self, AXIS_VELOCITY_NAMES[axis]))


@dataclass
class AxisPlantState:
    position: float = 0.0
    velocity: float = 0.0
    history_t: list[float] = field(default_factory=list)
    history_u: list[float] = field(default_factory=list)


class DJIVelocityPlant:
    """Step an identified decoupled DJI velocity model.

    The integrator mirrors the stage-3 FOPDT model:
    delayed input -> saturated target velocity -> first-order velocity response
    with acceleration limiting -> position integration.
    """

    def __init__(
        self,
        model: DJIVelocityModel,
        *,
        initial_state: PlantState | None = None,
        axes: tuple[str, ...] = VALID_MODEL_AXES,
        initial_command: dict[str, float] | None = None,
    ) -> None:
        self.model = model
        self.axes = tuple(axis for axis in axes if axis in model.axes)
        if not self.axes:
            raise ValueError("DJIVelocityPlant requires at least one model axis.")
        state = initial_state or PlantState()
        self.t = 0.0
        self._axis_states = {
            axis: AxisPlantState(
                position=state.position(axis),
                velocity=state.velocity(axis),
                history_t=[0.0] if initial_command is not None else [],
                history_u=[float(initial_command.get(axis, 0.0))] if initial_command is not None else [],
            )
            for axis in self.axes
        }

    @property
    def state(self) -> PlantState:
        values = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "yaw_rate": 0.0,
        }
        for axis, axis_state in self._axis_states.items():
            values[AXIS_POSITION_NAMES[axis]] = axis_state.position
            values[AXIS_VELOCITY_NAMES[axis]] = axis_state.velocity
        return PlantState(**values)

    def step(self, dt: float, command: dict[str, float]) -> PlantState:
        if dt < 0.0:
            raise ValueError("dt must be non-negative.")
        if dt == 0.0:
            return self.state

        for axis in self.axes:
            axis_model = self.model.axis(axis)
            axis_state = self._axis_states[axis]
            u = float(command.get(axis, 0.0))
            axis_state.history_t.append(self.t)
            axis_state.history_u.append(u)
            delayed_u = _interpolate(axis_state.history_t, axis_state.history_u, self.t - axis_model.Td)
            _advance_axis(axis_state, axis_model, delayed_u=delayed_u, dt=dt)
        self.t += dt
        return self.state


def _advance_axis(
    state: AxisPlantState,
    model: AxisVelocityModel,
    *,
    delayed_u: float,
    dt: float,
) -> None:
    target = _clamp(model.K * delayed_u, -model.vmax, model.vmax)
    accel = (target - state.velocity) / model.tau
    accel = _clamp(accel, -model.amax, model.amax)
    state.velocity += accel * dt
    state.position += state.velocity * dt


def _interpolate(t: list[float], values: list[float], sample_time: float) -> float:
    if not t:
        return 0.0
    if sample_time <= t[0]:
        return values[0]
    if sample_time >= t[-1]:
        return values[-1]
    low = 0
    high = len(t) - 1
    while high - low > 1:
        mid = (low + high) // 2
        if t[mid] <= sample_time:
            low = mid
        else:
            high = mid
    ratio = (sample_time - t[low]) / max(t[high] - t[low], 1e-9)
    return values[low] + ratio * (values[high] - values[low])


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
