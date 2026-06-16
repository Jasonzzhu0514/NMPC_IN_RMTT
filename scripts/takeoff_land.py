#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

try:
    from scripts._bootstrap import add_repo_root
except ImportError:
    from _bootstrap import add_repo_root

add_repo_root()

from rmtt.takeoff_land import DEFAULT_DRONE_IP


def main(argv=None) -> int:
    from rmtt.takeoff_land import main as takeoff_land_main

    return takeoff_land_main(argv)


if __name__ == "__main__":
    sys.exit(main())
