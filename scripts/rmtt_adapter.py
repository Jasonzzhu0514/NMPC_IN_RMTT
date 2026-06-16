#!/usr/bin/env python3
# -*- coding: utf-8 -*-

try:
    from scripts._bootstrap import add_repo_root
except ImportError:
    from _bootstrap import add_repo_root

add_repo_root()

from rmtt.adapter import *  # noqa: F401,F403
