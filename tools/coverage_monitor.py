#!/usr/bin/env python3
# Live view of how much of the sim warehouse RTAB-Map has mapped, to know where to fly next and when the
# map is complete. On every map update it redraws, in the terminal, a top-down picture of the warehouse
# in 1 m blocks, shaded by how much of each block's floor the 2D map knows (blank: not seen yet), with
# the drone as '@@', and the share of the floor seen. Inside crates and shelves stays unknown, so 100%
# is out of reach.
# When visual odometry loses track, RTAB-Map starts a new map and the known area falls back; this is
# flagged, since make save-octomap exports only the newest map.
# Usage: python3 tools/coverage_monitor.py [--ros-args -p use_sim_time:=true]   (make coverage)
import os
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_tools  # noqa: E402

CLEAR = '\033[H\033[2J'
# A map that drops below this share of the largest known area seen is a new map, not noise.
RESTART_DROP = 0.7


class CoverageMonitor(Node):
    def __init__(self):
        super().__init__('coverage_monitor')
        self.declare_parameter('map_topic', '/rtabmap/map')
        self.declare_parameter('path_topic', '/rtabmap/mapPath')
        self.drone = None
        self.best = 0.0
        self.restarts = []
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value, self.on_map, 1)
        self.create_subscription(Path, self.get_parameter('path_topic').value, self.on_path, 1)
        print('Waiting for the map from RTAB-Map (make rtabmap)...')

    def on_path(self, msg):
        if msg.poses:
            p = msg.poses[-1].pose.position
            self.drone = (p.x, p.y)

    def on_map(self, msg):
        info = msg.info
        blocks, known, area = map_tools.block_coverage(
            msg.data, info.width, info.resolution, (info.origin.position.x, info.origin.position.y))
        if known < RESTART_DROP * self.best:
            self.restarts.append(time.strftime('%H:%M:%S'))
            self.best = known
        self.best = max(self.best, known)
        lines = [CLEAR + f'Warehouse floor seen: {100 * known / area:.0f}% ({known:.0f} of {area:.0f} m2)',
                 map_tools.draw_coverage(blocks, drone=self.drone),
                 'x -15 to +15 m left to right, y +7.7 (top) to -7.9 m (bottom), 1 m blocks.',
                 'Floor seen per block:  ░░ <50%   ▒▒ <90%   ██ 90%+   blank: not yet   @@ drone']
        if self.restarts:
            lines.append(f'\033[31mRTAB-Map started a new map at {", ".join(self.restarts)} (odometry lost track):'
                         ' make save-octomap exports only the newest one.\033[0m')
        print('\n'.join(lines), flush=True)


def main():
    rclpy.init()
    node = CoverageMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
