#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse

from rmtt_control.model_quality import QualityThresholds, check_model_quality
from models.dji_velocity_model import DEFAULT_MODEL_PATH


MODEL_QUALITY_RETURN_CODE = 4


def model_quality_gate_required(args: argparse.Namespace) -> bool:
    if args.allow_bootstrap_model:
        return False
    return bool(args.send or args.require_real_model)


def check_model_quality_gate(args: argparse.Namespace, *, label: str) -> int:
    thresholds = QualityThresholds(
        min_samples=args.quality_min_samples,
        min_r2=args.quality_min_r2,
        min_vaf=args.quality_min_vaf,
        max_nrmse=args.quality_max_nrmse,
        fail_on_bootstrap=True,
    )
    try:
        results = check_model_quality(args.model or DEFAULT_MODEL_PATH, thresholds=thresholds)
    except Exception as exc:
        print("model quality check failed: {0}".format(exc), flush=True)
        return MODEL_QUALITY_RETURN_CODE
    failed = False
    for result in results:
        if result.ok:
            continue
        failed = True
        print("model quality FAIL: {0}".format(result.axis), flush=True)
        for failure in result.failures:
            print("  FAIL: {0}".format(failure), flush=True)
    if failed:
        print(
            "Refusing {0} with bootstrap/low-quality model. "
            "Run identification and fit a real model first, or use "
            "--allow-bootstrap-model only for controlled no-flight debugging.".format(label),
            flush=True,
        )
        return MODEL_QUALITY_RETURN_CODE
    return 0
