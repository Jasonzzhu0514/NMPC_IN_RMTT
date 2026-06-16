#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import os


def env_str(name: str, default: str = "") -> str:
    return os.environ.get(name, default)


def env_int(name: str) -> int | None:
    value = os.environ.get(name)
    if value is None or value == "":
        return None
    return int(value)


DEFAULT_RMTT_IP = env_str("RMTT_IP")
DEFAULT_RMTT_AP_LOCAL_IP = env_str("RMTT_AP_LOCAL_IP")
DEFAULT_RMTT_WIFI_SSID = env_str("RMTT_WIFI_SSID")
DEFAULT_RMTT_WIFI_PASSWORD = env_str("RMTT_WIFI_PASSWORD")
DEFAULT_VRPN_TRACKER = env_str("RMTT_VRPN_TRACKER")
DEFAULT_VRPN_HOST = env_str("RMTT_VRPN_HOST")
DEFAULT_VRPN_PORT = env_int("RMTT_VRPN_PORT")
