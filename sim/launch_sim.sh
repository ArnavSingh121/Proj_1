#!/usr/bin/env bash
# Terminal 1: PX4 SITL + Gazebo Harmonic, x500 with depth camera, warehouse world.
# Usage: ./launch_sim.sh [path/to/PX4-Autopilot]   (default: ~/PX4-Autopilot)
set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
cd "$PX4_DIR"
# World file is worlds/default_warehouse.sdf; "warehouse" alone hangs at "Waiting for Gazebo world".
PX4_GZ_WORLD=default_warehouse make px4_sitl gz_x500_depth
