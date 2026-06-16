from __future__ import annotations

import json
from dataclasses import asdict

from nmpc.flight.math_utils import clamp
from nmpc.flight.types import NmpcFlightControllerConfig
from nmpc.position_3d_types import Position3DProfile
from models.dji_velocity_model import AxisVelocityModel, DJIVelocityModel


def profile_json(config: Position3DProfile) -> str:
    return json.dumps(asdict(config), sort_keys=True, separators=(",", ":"))


def conservative_xy_model(
    model: DJIVelocityModel,
    config: NmpcFlightControllerConfig,
) -> DJIVelocityModel:
    if not config.xy_conservative_model_enabled:
        return model
    tau_scale = max(1.0, float(config.xy_tau_scale))
    delay_add_sec = max(0.0, float(config.xy_delay_add_sec))
    amax_scale = clamp(float(config.xy_amax_scale), 0.05, 1.0)
    axes = dict(model.axes)
    for axis in ("pitch", "roll"):
        if axis not in axes:
            continue
        source = axes[axis]
        axes[axis] = AxisVelocityModel(
            axis=source.axis,
            K=source.K,
            tau=max(1e-6, source.tau * tau_scale),
            Td=max(0.0, source.Td + delay_add_sec),
            vmax=source.vmax,
            amax=max(1e-6, source.amax * amax_scale),
            response=source.response,
            unit=source.unit,
            fit=dict(source.fit),
            validation=dict(source.validation),
        )
    metadata = dict(model.metadata)
    metadata["nmpc_xy_conservative_model"] = {
        "enabled": True,
        "tau_scale": tau_scale,
        "delay_add_sec": delay_add_sec,
        "amax_scale": amax_scale,
    }
    return DJIVelocityModel(axes=axes, metadata=metadata, source_path=model.source_path)
