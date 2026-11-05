#!/bin/bash
# PX4 SITL + Gazebo with the x500 that has a downward range finder (airframe 4016).
# Uses the existing build in ~/sim/PX4-Autopilot without rebuilding it, and its own working
# directory (sim/rootfs) so the parameters of your normal SITL runs are not touched.
#
#   sim/run_sitl.sh            # with the Gazebo window
#   HEADLESS=1 sim/run_sitl.sh # no window
set -e
PX4_DIR=${PX4_DIR:-$HOME/sim/PX4-Autopilot}
BUILD=$PX4_DIR/build/px4_sitl_default
HERE=$(cd "$(dirname "$0")" && pwd)

mkdir -p "$HERE/rootfs"
ln -sfn "$BUILD/etc" "$HERE/rootfs/etc"
cd "$HERE/rootfs"
export PX4_SYS_AUTOSTART=4016 PX4_SIM_MODEL=gz_x500_lidar_down GZ_IP=127.0.0.1
exec "$BUILD/bin/px4" "$@"
