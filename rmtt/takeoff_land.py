#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import sys
import time

from robomaster import config, robot
from rmtt_config import DEFAULT_RMTT_IP


DEFAULT_DRONE_IP = DEFAULT_RMTT_IP


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="RMTT takeoff and land demo")
    parser.add_argument("--ip", default=DEFAULT_DRONE_IP, help="RMTT IP address")
    parser.add_argument("--hover", type=float, default=5.0, help="hover seconds before landing")
    parser.add_argument("--confirm-risk", action="store_true", help="required to take off")
    args = parser.parse_args(argv)
    if not args.confirm_risk:
        print("Refusing to take off without --confirm-risk.")
        return 2

    config.ROBOT_IP_STR = args.ip

    drone = robot.Drone()
    print("Connecting to RMTT at {0} ...".format(args.ip))
    drone.initialize(conn_type="sta")

    try:
        battery = drone.battery.get_battery()
        print("Battery: {0}%".format(battery))

        print("Takeoff ...")
        drone.flight.takeoff().wait_for_completed()

        print("Hover {0:.1f}s ...".format(args.hover))
        time.sleep(args.hover)

        print("Land ...")
        drone.flight.land().wait_for_completed()
        print("Done")
    finally:
        drone.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
