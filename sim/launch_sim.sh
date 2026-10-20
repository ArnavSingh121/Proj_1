#!/usr/bin/env bash
# Terminal 1: PX4 SITL + Gazebo Harmonic, x500 with depth camera, warehouse world.
# Usage: ./launch_sim.sh [path/to/PX4-Autopilot]   (default: ~/PX4-Autopilot)
#        WORLD=walls ./launch_sim.sh                 (any world in worlds/; default: default_warehouse)
set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
WORLD="${WORLD:-default_warehouse}"
cd "$PX4_DIR"
# World file is worlds/default_warehouse.sdf; "warehouse" alone hangs at "Waiting for Gazebo world".
PX4_GZ_WORLD="$WORLD" make px4_sitl gz_x500_depth
