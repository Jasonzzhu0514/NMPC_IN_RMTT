from __future__ import annotations

from math import ceil, pi, sin
import random

from nmpc.identification.protocol_types import ExcitationStep


def append_multistep(
    steps: list[ExcitationStep],
    *,
    axis: str,
    amplitudes: tuple[int, ...],
    step_sec: float,
    center_sec: float,
) -> None:
    for amplitude in amplitudes:
        for offset in (amplitude, 0, -amplitude, 0):
            append_step(
                steps,
                axis=axis,
                offset=offset,
                duration_sec=center_sec if offset == 0 else step_sec,
                name="center" if offset == 0 else "step",
                signal_kind="multistep",
                amplitude=amplitude,
            )


def append_large_jump(
    steps: list[ExcitationStep],
    *,
    axis: str,
    amplitude: int,
    jump_sec: float,
    center_sec: float,
) -> None:
    for offset, duration, name in (
        (0, center_sec, "center"),
        (amplitude, jump_sec, "large_jump"),
        (0, center_sec, "center"),
        (-amplitude, jump_sec, "large_jump"),
        (0, center_sec, "center"),
    ):
        append_step(
            steps,
            axis=axis,
            offset=offset,
            duration_sec=duration,
            name=name,
            signal_kind="large_jump",
            amplitude=amplitude,
        )


def append_prbs(
    steps: list[ExcitationStep],
    *,
    axis: str,
    amplitude: int,
    duration_sec: float,
    switch_sec: float,
    center_sec: float,
    seed: int,
) -> None:
    append_step(
        steps,
        axis=axis,
        offset=0,
        duration_sec=center_sec,
        name="center",
        signal_kind="prbs",
        amplitude=amplitude,
    )
    rng = random.Random(seed)
    remaining = duration_sec
    for _ in range(max(1, ceil(duration_sec / switch_sec))):
        if remaining <= 0:
            break
        segment_sec = min(switch_sec, remaining)
        sign = 1 if rng.getrandbits(1) else -1
        append_step(
            steps,
            axis=axis,
            offset=sign * amplitude,
            duration_sec=segment_sec,
            name="prbs",
            signal_kind="prbs",
            amplitude=amplitude,
        )
        remaining -= segment_sec
    append_step(
        steps,
        axis=axis,
        offset=0,
        duration_sec=center_sec,
        name="center",
        signal_kind="prbs",
        amplitude=amplitude,
    )


def append_multisine(
    steps: list[ExcitationStep],
    *,
    axis: str,
    amplitude: int,
    duration_sec: float,
    segment_sec: float,
    center_sec: float,
    frequencies_hz: tuple[float, ...],
) -> None:
    append_step(
        steps,
        axis=axis,
        offset=0,
        duration_sec=center_sec,
        name="center",
        signal_kind="multisine",
        amplitude=amplitude,
    )
    remaining = duration_sec
    elapsed = 0.0
    for _ in range(max(1, ceil(duration_sec / segment_sec))):
        if remaining <= 0:
            break
        actual_segment_sec = min(segment_sec, remaining)
        sample_time = elapsed + actual_segment_sec * 0.5
        normalized = sum(sin(2.0 * pi * freq * sample_time) for freq in frequencies_hz) / len(
            frequencies_hz
        )
        append_step(
            steps,
            axis=axis,
            offset=int(round(amplitude * normalized)),
            duration_sec=actual_segment_sec,
            name="multisine",
            signal_kind="multisine",
            amplitude=amplitude,
        )
        elapsed += actual_segment_sec
        remaining -= actual_segment_sec
    append_step(
        steps,
        axis=axis,
        offset=0,
        duration_sec=center_sec,
        name="center",
        signal_kind="multisine",
        amplitude=amplitude,
    )


def append_step(
    steps: list[ExcitationStep],
    *,
    axis: str,
    offset: int,
    duration_sec: float,
    name: str,
    signal_kind: str,
    amplitude: int,
) -> None:
    steps.append(
        ExcitationStep(
            axis=axis,
            offset=offset,
            duration_sec=duration_sec,
            name=name,
            index=len(steps),
            signal_kind=signal_kind,
            amplitude=amplitude,
        )
    )
