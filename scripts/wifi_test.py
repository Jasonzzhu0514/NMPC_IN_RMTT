#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

try:
    from scripts._bootstrap import add_repo_root
except ImportError:
    from _bootstrap import add_repo_root

add_repo_root()

from rmtt.wifi import DEFAULT_LOCAL_IP, DEFAULT_PASSWORD, DEFAULT_SSID


def main(argv=None) -> int:
    from rmtt.wifi import main as wifi_main

    return wifi_main(argv)


if __name__ == "__main__":
    sys.exit(main())
