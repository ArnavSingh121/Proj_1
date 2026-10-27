#!/usr/bin/env python3
# Logs how the occupancy map grows during a run, to show it is built incrementally and to compare runs.
# Every map update from RTAB-Map adds a row to a CSV: time, grid size, known / occupied / free cells,
# mapped area, and the number of poses in the trajectory.
# Usage: python3 tools/map_monitor.py [--ros-args -p csv:=~/.ros/map_growth.csv -p use_sim_time:=true]
#        (make map-monitor)
import csv
import os
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_tools  # noqa: E402

FIELDS = ['time', 'width', 'height', 'known_cells', 'occupied_cells', 'free_cells', 'known_area_m2', 'poses']


class MapMonitor(Node):
    def __init__(self):
        super().__init__('map_monitor')
        self.declare_parameter('map_topic', '/rtabmap/map')
        self.declare_parameter('path_topic', '/rtabmap/mapPath')
        self.declare_parameter('csv', os.path.expanduser('~/.ros/map_growth.csv'))
        path = os.path.expanduser(self.get_parameter('csv').value)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.file = open(path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(FIELDS)
        self.poses = 0
        self.last_known = 0
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value, self.on_map, 1)
        self.create_subscription(Path, self.get_parameter('path_topic').value, self.on_path, 1)
        self.get_logger().info(f'Logging map growth to {path}')

    def on_path(self, msg):
        self.poses = len(msg.poses)

    def on_map(self, msg):
        known, occupied, free = map_tools.grid_stats(msg.data)
        area = known * msg.info.resolution ** 2
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.writer.writerow([f'{t:.3f}', msg.info.width, msg.info.height, known, occupied, free,
                              f'{area:.2f}', self.poses])
        self.file.flush()
        self.get_logger().info(f'map {msg.info.width}x{msg.info.height}: {known} known cells '
                               f'({known - self.last_known:+d}), {area:.1f} m2, {self.poses} poses')
        self.last_known = known

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MapMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
