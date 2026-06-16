#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import sys

import robomaster
from robomaster import robot
from rmtt_config import DEFAULT_RMTT_AP_LOCAL_IP, DEFAULT_RMTT_WIFI_PASSWORD, DEFAULT_RMTT_WIFI_SSID


DEFAULT_LOCAL_IP = DEFAULT_RMTT_AP_LOCAL_IP
DEFAULT_SSID = DEFAULT_RMTT_WIFI_SSID
DEFAULT_PASSWORD = DEFAULT_RMTT_WIFI_PASSWORD


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Configure RMTT STA Wi-Fi with RoboMaster SDK")
    parser.add_argument("--local-ip", default=DEFAULT_LOCAL_IP, help="computer IP on the current RMTT AP network")
    parser.add_argument("--ssid", default=DEFAULT_SSID)
    parser.add_argument("--password", default=DEFAULT_PASSWORD)
    args = parser.parse_args(argv)
    if not args.local_ip or not args.ssid or not args.password:
        parser.error("--local-ip, --ssid, and --password are required, or set RMTT_AP_LOCAL_IP/RMTT_WIFI_SSID/RMTT_WIFI_PASSWORD.")

    robomaster.config.LOCAL_IP_STR = args.local_ip
    drone = robot.Drone()
    drone.initialize()

    try:
        version = drone.get_sdk_version()
        print("Drone SDK Version: {0}".format(version), flush=True)
        drone.config_sta(ssid=args.ssid, password=args.password)
        print("STA config sent: ssid={0}".format(args.ssid), flush=True)
    finally:
        drone.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
