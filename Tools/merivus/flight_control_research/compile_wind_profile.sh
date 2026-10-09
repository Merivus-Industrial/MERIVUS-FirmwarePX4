#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
build="$repo/build/px4_sitl_default/build_gazebo-classic"
g++ -std=c++17 -shared -fPIC \
    "$repo/Tools/merivus/flight_control_research/wind_profile_plugin.cpp" \
    -I"$build" -I/usr/include/gazebo-11/gazebo/msgs \
    -o "$build/libafcr_wind_profile.so" \
    $(pkg-config --cflags --libs gazebo) \
    -L"$build" -lphysics_msgs -Wl,-rpath,"$build"
