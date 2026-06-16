#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

try:
    from scripts._bootstrap import add_repo_root
except ImportError:
    from _bootstrap import add_repo_root

add_repo_root()

from rmtt.scan_ip import safe_get_subnets, scan_ip


def main(argv=None) -> int:
    from rmtt.scan_ip import main as scan_main

    return scan_main(argv)


if __name__ == "__main__":
    sys.exit(main())
