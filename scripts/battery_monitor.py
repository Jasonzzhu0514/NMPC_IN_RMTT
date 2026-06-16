#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

try:
    from scripts._bootstrap import add_repo_root
except ImportError:
    from _bootstrap import add_repo_root

add_repo_root()

from rmtt.battery import (
    DEFAULT_DRONE_CHECK_TIMEOUT_SEC,
    DEFAULT_DRONE_IP,
    _drone_check_worker,
    read_drone_battery_isolated,
    read_tello_battery_udp,
)


def main(argv=None) -> int:
    from rmtt import battery as battery_impl

    original = battery_impl.read_drone_battery_isolated
    battery_impl.read_drone_battery_isolated = read_drone_battery_isolated
    try:
        return battery_impl.main(argv)
    finally:
        battery_impl.read_drone_battery_isolated = original


if __name__ == "__main__":
    sys.exit(main())
