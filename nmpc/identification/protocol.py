"""Excitation protocol for DJI stick-to-velocity identification."""

from __future__ import annotations

from nmpc.identification.protocol_signals import (
    append_large_jump as _append_large_jump,
    append_multisine as _append_multisine,
    append_multistep as _append_multistep,
    append_prbs as _append_prbs,
)
from nmpc.identification.protocol_types import ExcitationStep


VALID_AXES = {"roll", "pitch", "throttle", "yaw"}
VALID_SIGNALS = {"step", "multistep", "large_jump", "prbs", "multisine", "all"}
STAGE_TWO_SIGNALS = ("multistep", "large_jump", "prbs", "multisine")
RMTT_MAX_IDENTIFICATION_AMPLITUDE = 30


def build_velocity_identification_steps(
    *,
    axes: tuple[str, ...],
    amplitudes: tuple[int, ...],
    step_sec: float,
    center_sec: float,
) -> list[ExcitationStep]:
    validated_axes = _validate_axes(axes)
    validated_amplitudes = _validate_amplitudes(amplitudes)
    steps: list[ExcitationStep] = []
    for axis in validated_axes:
        _append_multistep(
            steps,
            axis=axis,
            amplitudes=validated_amplitudes,
            step_sec=step_sec,
            center_sec=center_sec,
        )
    return steps


def build_stage_two_velocity_identification_steps(
    *,
    axis: str,
    signals: tuple[str, ...] = ("step",),
    amplitudes: tuple[int, ...] = (10, 20),
    step_sec: float = 2.0,
    center_sec: float = 1.0,
    large_jump_amplitude: int = 25,
    large_jump_sec: float = 1.5,
    prbs_amplitude: int = 20,
    prbs_duration_sec: float = 20.0,
    prbs_switch_sec: float = 0.5,
    prbs_seed: int = 20260529,
    multisine_amplitude: int = 20,
    multisine_duration_sec: float = 20.0,
    multisine_segment_sec: float = 0.1,
    multisine_frequencies_hz: tuple[float, ...] = (0.05, 0.2, 0.7, 1.2),
) -> list[ExcitationStep]:
    """Build the stage-2 single-axis experiment schedule.

    The returned schedule may contain many segments, but every segment moves
    only ``axis``; callers must keep the other sticks centered.
    """

    validated_axis = _validate_single_axis(axis)
    validated_signals = _validate_signals(signals)
    validated_amplitudes = _validate_amplitudes(amplitudes)
    _validate_duration("step_sec", step_sec)
    _validate_duration("center_sec", center_sec)
    steps: list[ExcitationStep] = []

    for signal in validated_signals:
        if signal == "multistep":
            _append_multistep(
                steps,
                axis=validated_axis,
                amplitudes=validated_amplitudes,
                step_sec=step_sec,
                center_sec=center_sec,
            )
        elif signal == "large_jump":
            _append_large_jump(
                steps,
                axis=validated_axis,
                amplitude=_single_amplitude("large_jump_amplitude", large_jump_amplitude),
                jump_sec=_validate_duration("large_jump_sec", large_jump_sec),
                center_sec=center_sec,
            )
        elif signal == "prbs":
            _append_prbs(
                steps,
                axis=validated_axis,
                amplitude=_single_amplitude("prbs_amplitude", prbs_amplitude),
                duration_sec=_validate_duration("prbs_duration_sec", prbs_duration_sec),
                switch_sec=_validate_duration("prbs_switch_sec", prbs_switch_sec),
                center_sec=center_sec,
                seed=prbs_seed,
            )
        elif signal == "multisine":
            _append_multisine(
                steps,
                axis=validated_axis,
                amplitude=_single_amplitude("multisine_amplitude", multisine_amplitude),
                duration_sec=_validate_duration(
                    "multisine_duration_sec",
                    multisine_duration_sec,
                ),
                segment_sec=_validate_duration("multisine_segment_sec", multisine_segment_sec),
                center_sec=center_sec,
                frequencies_hz=_validate_frequencies(multisine_frequencies_hz),
            )
    return _with_small_field_recenter_marks(steps) if validated_axis in {"pitch", "roll"} else steps


def _validate_axes(axes: tuple[str, ...]) -> tuple[str, ...]:
    values = tuple(axis.strip().lower() for axis in axes if axis.strip())
    if not values:
        raise ValueError("At least one identification axis is required.")
    invalid = [axis for axis in values if axis not in VALID_AXES]
    if invalid:
        valid = ", ".join(sorted(VALID_AXES))
        raise ValueError(
            f"Unsupported identification axis {invalid[0]!r}; expected one of: {valid}"
        )
    return values


def _validate_single_axis(axis: str) -> str:
    values = _validate_axes((axis,))
    return values[0]


def _validate_amplitudes(amplitudes: tuple[int, ...]) -> tuple[int, ...]:
    values = tuple(
        min(abs(int(value)), RMTT_MAX_IDENTIFICATION_AMPLITUDE)
        for value in amplitudes
        if int(value) != 0
    )
    if not values:
        raise ValueError("At least one non-zero identification amplitude is required.")
    return values


def _single_amplitude(name: str, value: int) -> int:
    try:
        amplitude = abs(int(value))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a non-zero integer.") from exc
    if amplitude == 0:
        raise ValueError(f"{name} must be non-zero.")
    return min(amplitude, RMTT_MAX_IDENTIFICATION_AMPLITUDE)


def _validate_signals(signals: tuple[str, ...]) -> tuple[str, ...]:
    values = tuple(signal.strip().lower() for signal in signals if signal.strip())
    if not values:
        raise ValueError("At least one identification signal is required.")
    invalid = [signal for signal in values if signal not in VALID_SIGNALS]
    if invalid:
        valid = ", ".join(sorted(VALID_SIGNALS))
        raise ValueError(
            f"Unsupported identification signal {invalid[0]!r}; expected one of: {valid}"
        )
    if "all" in values:
        return STAGE_TWO_SIGNALS
    return tuple("multistep" if signal == "step" else signal for signal in values)


def _validate_duration(name: str, value: float) -> float:
    try:
        duration = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be positive.") from exc
    if duration <= 0:
        raise ValueError(f"{name} must be positive.")
    return duration


def _validate_frequencies(frequencies_hz: tuple[float, ...]) -> tuple[float, ...]:
    values = []
    for value in frequencies_hz:
        try:
            frequency = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("multisine frequencies must be positive numbers.") from exc
        if frequency <= 0:
            raise ValueError("multisine frequencies must be positive numbers.")
        values.append(frequency)
    if not values:
        raise ValueError("At least one multisine frequency is required.")
    return tuple(values)


def _with_small_field_recenter_marks(steps: list[ExcitationStep]) -> list[ExcitationStep]:
    return [
        ExcitationStep(
            axis=step.axis,
            offset=step.offset,
            duration_sec=step.duration_sec,
            name=step.name,
            index=step.index,
            signal_kind=step.signal_kind,
            amplitude=step.amplitude,
            requires_recenter=not step.is_center,
        )
        for step in steps
    ]
