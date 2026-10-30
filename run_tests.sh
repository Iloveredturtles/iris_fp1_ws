#!/usr/bin/env bash
# Compile and run the logic tests. No ROS and no simulator needed.
#
#   ./run_tests.sh
#
# This checks the three header-only pieces: the UDP wire format, the Ackermann
# geometry, and the lane-centre decision. Everything else needs the sim.
set -euo pipefail

cd "$(dirname "$0")"

OUT=/tmp/iris_fp_tests

g++ -std=c++17 -O1 -Wall -Wextra \
  -Isrc/communication/include \
  -Isrc/vision/include \
  -Isrc/master/include \
  tests/test_logic.cpp \
  -o "$OUT" \
  $(pkg-config --cflags --libs opencv4)

"$OUT"
