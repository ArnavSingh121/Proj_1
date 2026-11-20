#!/usr/bin/env bash
# Save the map: ./save_map.sh <name>   (while ./map.sh <name> runs, or after it stopped)
# Writes Proj_1/maps/<name>_<date-time>/: <name>.db, map.pgm + map.yaml (2D), cloud.ply (3D), poses.txt
set -eo pipefail
[ -n "${1:-}" ] || { echo "usage: $0 <map name>"; exit 1; }
export ROS_DOMAIN_ID=1
# Loopback only: Wi-Fi hand-offs while walking can't break ROS 2 discovery (see a running map from
# another terminal with: ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=1 ros2 topic list).
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
EXTRA="$HOME/.local/ros-extra/setup.bash"  # RTAB-Map unpacked without sudo; skipped if absent
if [ -f "$EXTRA" ]; then source "$EXTRA"; fi
cd "$(dirname "$(dirname "$(readlink -f "$0")")")"
python3 tools/save_map_db.py --db "maps/$1.db"
