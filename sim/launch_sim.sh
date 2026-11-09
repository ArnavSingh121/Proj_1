#!/usr/bin/env bash
# Terminal 1: PX4 SITL + Gazebo Harmonic, x500 with depth camera, warehouse world.
# Usage: ./launch_sim.sh [path/to/PX4-Autopilot]   (default: ~/PX4-Autopilot)
#        WORLD=walls ./launch_sim.sh                 (any world in worlds/; default: default_warehouse)
#        GPU=integrated ./launch_sim.sh              (render on the default GPU even if an NVIDIA one works)
#        NO_GCS=true ./launch_sim.sh                 (arm without QGroundControl, e.g. pxh> commander takeoff)
set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
WORLD="${WORLD:-default_warehouse}"
cd "$PX4_DIR"
# On a hybrid-graphics laptop (NVIDIA in on-demand mode) Gazebo renders on the integrated GPU and the
# camera/lidar sensors drag the sim down to a few percent of real time. Offload it to the NVIDIA GPU
# when its driver works; machines without one start exactly as before.
if [ "${GPU:-}" != integrated ] && nvidia-smi >/dev/null 2>&1; then
  echo "Rendering Gazebo on the NVIDIA GPU (GPU=integrated to turn off)"
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
# The gz x500 airframe sets NAV_DLL_ACT=2 (return on GCS link loss), which also blocks arming until a
# GCS connects. NO_GCS=true turns that off. PX4 saves parameters it sets at startup, so a normal start
# sets 2 again: otherwise one NO_GCS run would leave the failsafe off for every later run.
if [ "${NO_GCS:-false}" = true ]; then
  echo "NO_GCS=true: arming without a ground station; no failsafe if a GCS link is lost (sim only)"
  export PX4_PARAM_NAV_DLL_ACT=0
else
  export PX4_PARAM_NAV_DLL_ACT="${PX4_PARAM_NAV_DLL_ACT:-2}"
fi
# World file is worlds/default_warehouse.sdf; "warehouse" alone hangs at "Waiting for Gazebo world".
PX4_GZ_WORLD="$WORLD" make px4_sitl gz_x500_depth
