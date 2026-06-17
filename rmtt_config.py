#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = ROOT / "config" / "rmtt.local.json"


def load_config(path: str | Path | None = None) -> dict:
    config_path = Path(path) if path else Path(os.environ.get("RMTT_CONFIG", DEFAULT_CONFIG_PATH))
    if not config_path.exists():
        return {}
    try:
        with config_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError as exc:
        raise RuntimeError("invalid RMTT config JSON: {0}: {1}".format(config_path, exc)) from exc
    if not isinstance(data, dict):
        raise RuntimeError("invalid RMTT config JSON: root must be an object: {0}".format(config_path))
    return data


CONFIG = load_config()


def _config_value(section: str, key: str, default=None):
    section_data = CONFIG.get(section, {})
    if not isinstance(section_data, dict):
        return default
    value = section_data.get(key, default)
    if value is None:
        return default
    return value


def config_str(env_name: str, section: str, key: str, default: str = "") -> str:
    env_value = os.environ.get(env_name)
    if env_value is not None:
        return env_value
    value = _config_value(section, key, default)
    if value is None:
        return default
    return str(value)


def config_int(env_name: str, section: str, key: str) -> int | None:
    value = os.environ.get(env_name)
    if value is None:
        value = _config_value(section, key)
    if value is None or value == "":
        return None
    return int(value)


DEFAULT_RMTT_IP = config_str("RMTT_IP", "drone", "ip")
DEFAULT_RMTT_AP_LOCAL_IP = config_str("RMTT_AP_LOCAL_IP", "drone", "ap_local_ip")
DEFAULT_RMTT_WIFI_SSID = config_str("RMTT_WIFI_SSID", "wifi", "ssid")
DEFAULT_RMTT_WIFI_PASSWORD = config_str("RMTT_WIFI_PASSWORD", "wifi", "password")
DEFAULT_VRPN_TRACKER = config_str("RMTT_VRPN_TRACKER", "vrpn", "tracker")
DEFAULT_VRPN_HOST = config_str("RMTT_VRPN_HOST", "vrpn", "host")
DEFAULT_VRPN_PORT = config_int("RMTT_VRPN_PORT", "vrpn", "port")
DEFAULT_NMPC_SOURCE_ROOT = config_str("NMPC_SOURCE_ROOT", "audit", "nmpc_source_root")
