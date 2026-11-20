#!/usr/bin/env bash
# Stops what boot_env.sh started (QGroundControl too with --all).
# Usage: ./stop_env.sh [--all]
# Bracketed first letters keep a pattern from matching a shell whose command line contains it.
PATTERNS=('[r]tabmap.launch.py' '[m]avros.launch.py' 'ros_gz_bridge/[p]arameter_bridge' 'px4_sitl_default/bin/[p]x4$')
[ "${1:-}" = "--all" ] && PATTERNS+=('usr/bin/[Q]GroundControl')

for pat in "${PATTERNS[@]}"; do
  for p in $(pgrep -f "$pat"); do
    pg=$(ps -o pgid= -p "$p" | tr -d ' ')
    [ -n "$pg" ] && kill -INT -- "-$pg" 2>/dev/null
  done
done

# Give ROS 2 launch and Gazebo time to shut down, then clean up stragglers.
for _ in $(seq 1 15); do
  pgrep -f '[r]tabmap_|[r]gbd_odometry|[m]avros_node|[p]arameter_bridge|px4_sitl_default/bin/[p]x4$|[g]z sim -' >/dev/null || break
  sleep 1
done
pkill -TERM -f '[r]tabmap_viz|[r]tabmap_slam/rtabmap|[r]gbd_odometry|[m]avros_node|[p]arameter_bridge|px4_sitl_default/bin/[p]x4$|[g]z sim -' 2>/dev/null
echo "Stopped."
