#!/usr/bin/env python3
# Reloads a map saved by save_map.py (or nav2's map_saver) and publishes it, latched, as a
# nav_msgs/OccupancyGrid, so other autonomy components can use it without RTAB-Map running.
# To continue mapping instead, restart RTAB-Map on its database: make rtabmap NEW_MAP=false.
# Usage: python3 tools/load_map.py --ros-args -p map:=~/maps/warehouse.yaml [-p topic:=/map -p frame_id:=map]
#        (make load-map MAP=~/maps/warehouse.yaml)
import math
import os
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_tools  # noqa: E402


class LoadMap(Node):
    def __init__(self):
        super().__init__('load_map')
        self.declare_parameter('map', '')
        self.declare_parameter('topic', '/map')
        self.declare_parameter('frame_id', 'map')
        path = os.path.expanduser(self.get_parameter('map').value)
        if not path:
            raise SystemExit('Set the map file: --ros-args -p map:=path/to/map.yaml')
        width, height, data, resolution, (x, y, yaw) = map_tools.load(path)

        msg = OccupancyGrid()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.map_load_time = msg.header.stamp
        msg.info.width, msg.info.height, msg.info.resolution = width, height, resolution
        msg.info.origin.position.x, msg.info.origin.position.y = x, y
        msg.info.origin.orientation.z, msg.info.origin.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.data = data
        # Latched, like map_server: late subscribers still get the map.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, self.get_parameter('topic').value, qos)
        self.pub.publish(msg)
        known, occupied, free = map_tools.grid_stats(data)
        self.get_logger().info(f'Publishing {path} ({width}x{height}, {resolution} m cells, {known} known) '
                               f'on {self.get_parameter("topic").value}')


def main():
    rclpy.init()
    node = LoadMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
