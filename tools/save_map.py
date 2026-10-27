#!/usr/bin/env python3
# Saves the current 2D occupancy map (/rtabmap/map) as <out>.pgm + <out>.yaml (ROS map_server format).
# The full 3D map stays in RTAB-Map's database (~/.ros/rtabmap.db); this is the grid other components use.
# Usage: python3 tools/save_map.py [--ros-args -p out:=~/maps/warehouse -p topic:=/rtabmap/map -p timeout:=10.0]
#        (make save-map OUT=~/maps/warehouse)
import os
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_tools  # noqa: E402


class SaveMap(Node):
    def __init__(self):
        super().__init__('save_map')
        self.declare_parameter('topic', '/rtabmap/map')
        self.declare_parameter('out', os.path.expanduser('~/maps/map'))
        self.declare_parameter('timeout', 10.0)
        self.grid = None
        # Default (volatile) QoS matches both latched and plain publishers; RTAB-Map republishes every update.
        self.create_subscription(OccupancyGrid, self.get_parameter('topic').value, self.on_map, 1)

    def on_map(self, msg):
        self.grid = msg


def main():
    rclpy.init()
    node = SaveMap()
    try:
        deadline = node.get_clock().now().nanoseconds + node.get_parameter('timeout').value * 1e9
        while rclpy.ok() and node.grid is None and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.grid is None:
            node.get_logger().error(f'No map on {node.get_parameter("topic").value}: is RTAB-Map running?')
            return 1
        g = node.grid
        q = g.info.origin.orientation
        origin = (g.info.origin.position.x, g.info.origin.position.y,
                  map_tools.yaw_from_quaternion(q.x, q.y, q.z, q.w))
        out = os.path.expanduser(node.get_parameter('out').value)
        path = map_tools.save(out, g.info.width, g.info.height, list(g.data), g.info.resolution, origin)
        known, occupied, free = map_tools.grid_stats(g.data)
        node.get_logger().info(f'Saved {g.info.width}x{g.info.height} map ({known} known cells: {occupied} '
                               f'occupied, {free} free) to {path}')
        return 0
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
