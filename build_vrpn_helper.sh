#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$ROOT_DIR/native/vrpn_pose_json.cpp"
OUT="$ROOT_DIR/native/vrpn_pose_json"

CXX="${CXX:-g++}"
VRPN_PREFIX="${VRPN_PREFIX:-/opt/ros/noetic}"

"$CXX" -std=c++17 -O2 \
  -I"$VRPN_PREFIX/include" \
  "$SRC" \
  -L"$VRPN_PREFIX/lib" \
  -lvrpn -lquat -lpthread \
  -o "$OUT"

echo "$OUT"
