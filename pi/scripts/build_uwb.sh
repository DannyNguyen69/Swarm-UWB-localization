#!/bin/bash
# Builds the C++ uwb_bridge on the Pi (needs the MAVSDK that dark-landing already uses). Output: cpp/build/uwb_bridge
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cmake -S "$ROOT/cpp" -B "$ROOT/cpp/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/cpp/build" -j"$(nproc)"
echo "xong: $ROOT/cpp/build/uwb_bridge"
