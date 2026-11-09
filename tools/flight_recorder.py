#!/usr/bin/env python3
# Records every flight with nobody at the controls. Follows PX4's armed state from MAVROS:
#   arm     new folder <recordings_dir>/<date>_<time>/; a ros bag of `topics` starts, and the map restarts so
#           it begins at the takeoff point and holds nothing from before (such as being carried to the pad)
#   flying  the 2D map is written to the folder every `autosave` seconds, so a crash still leaves a map
#   disarm  the bag is closed, the full map is saved (tools/save_map_db.py) and <recordings_dir>/latest
#           points to the folder
# Each folder: bag/ (the recording), map.pgm + map.yaml (2D), cloud.ply (3D), poses.txt, the RTAB-Map
# database and flight.json (status, times, map and recording size). Replay one with tools/replay_flight.sh.
# sim/rtabmap.launch.py and jetson/zed_rtabmap.launch.py start this node with their topic lists.
# Usage: python3 tools/flight_recorder.py [--ros-args -p recordings_dir:=~/flight_recordings
#                                           -p database:=~/.ros/rtabmap.db -p "topics:=/a /b"]
import ctypes
import datetime
import os
import queue
import signal
import subprocess
import sys
import threading
import time

import rclpy
from mavros_msgs.msg import State
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_srvs.srv import Empty

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import flight_tools  # noqa: E402
import map_tools  # noqa: E402

PR_SET_PDEATHSIG = 1


def close_bag_if_parent_dies():
    """Runs in the recorder before exec: if this node is killed, the recorder gets SIGINT and closes the bag
    properly, instead of recording forever and filling the disk."""
    ctypes.CDLL('libc.so.6', use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGINT)


class FlightRecorder(Node):
    def __init__(self):
        super().__init__('flight_recorder')
        self.declare_parameter('recordings_dir', '~/flight_recordings')
        self.declare_parameter('database', '~/.ros/rtabmap.db')
        self.declare_parameter('topics', '')  # space-separated; empty records no bag, only the map
        self.declare_parameter('autosave', 30.0)  # s; the topic is free to read, so RTAB-Map doesn't pause
        p = lambda n: self.get_parameter(n).value
        self.root = os.path.expanduser(p('recordings_dir'))
        self.db = os.path.expanduser(p('database'))
        self.topics = p('topics').split()
        self.autosave_every = p('autosave')
        self.watcher = flight_tools.ArmWatcher()
        self.events = queue.Queue()
        self.grid = None
        self.folder = None
        self.started = None
        self.bag = None
        self.bag_checked = False
        self.reset_map = self.create_client(Empty, '/rtabmap/rtabmap/reset')
        self.reset_odom = self.create_client(Empty, '/rtabmap/reset_odom')  # absent with the ZED's odometry
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/rtabmap/map', self.on_map, latched)
        self.create_subscription(State, '/mavros/state', self.on_state, qos_profile_sensor_data)
        self.get_logger().info(f'Recording every flight to {self.root} ({len(self.topics)} topics)')

    def on_map(self, msg):
        self.grid = msg

    def on_state(self, msg):
        event = self.watcher.update(msg.connected, msg.armed)
        if event:
            self.events.put(event)

    def work(self):
        """Service calls, the recorder and file writes, off the thread that receives messages."""
        while True:
            try:
                event = self.events.get(timeout=self.autosave_every)
            except queue.Empty:
                event = None
            if event == 'takeoff':
                self.start(True)
            elif event == 'joined':
                self.start(False)
            elif event == 'landed':
                self.finish()
            elif self.folder:
                self.write_grid()
                self.check_bag()

    def call(self, client):
        if not client.wait_for_service(timeout_sec=2.0):
            return False
        future = client.call_async(Empty.Request())
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done()

    def start(self, reset):
        self.folder = flight_tools.new_flight_folder(self.root, datetime.datetime.now())
        self.started = flight_tools.timestamp()
        if reset:
            self.call(self.reset_odom)
            if not self.call(self.reset_map):
                self.get_logger().warn('RTAB-Map did not reset: this map may include the previous flight')
            self.grid = None
        self.start_bag()
        flight_tools.write_manifest(self.folder, status='flying', started=self.started)
        what = 'New flight' if reset else 'Joined a flight in progress (map kept)'
        self.get_logger().info(f'{what}: recording to {self.folder}')

    def start_bag(self):
        self.bag, self.bag_checked = None, False
        if not self.topics:
            return
        # stdin from /dev/null: the recorder's keyboard handler stops a process that shares a terminal.
        with open(os.path.join(self.folder, 'record.log'), 'w') as log:
            self.bag = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', os.path.join(self.folder, 'bag')] + self.topics,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True, preexec_fn=close_bag_if_parent_dies)

    def check_bag(self):
        """Says so loudly, once, if the recording isn't being written."""
        if self.bag is None or self.bag_checked:
            return
        self.bag_checked = True
        if self.bag.poll() is not None or not flight_tools.bag_files(os.path.join(self.folder, 'bag')):
            self.get_logger().error(f'NOT RECORDING: see {os.path.join(self.folder, "record.log")}')

    def stop_bag(self, timeout):
        if self.bag is None:
            return
        if self.bag.poll() is None:
            self.bag.send_signal(signal.SIGINT)
            try:
                self.bag.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.get_logger().error('The recorder did not close in time; stopping it')
                self.bag.kill()
                self.bag.wait()
        self.bag = None

    def write_grid(self):
        """Writes map.pgm + map.yaml; a power cut mid-write leaves the previous pair whole."""
        g = self.grid
        if g is None:
            return
        q = g.info.origin.orientation
        origin = (g.info.origin.position.x, g.info.origin.position.y,
                  map_tools.yaw_from_quaternion(q.x, q.y, q.z, q.w))
        tmp = os.path.join(self.folder, '.tmp', 'map')
        map_tools.save(tmp, g.info.width, g.info.height, list(g.data), g.info.resolution, origin)
        for ext in ('.pgm', '.yaml'):
            os.replace(tmp + ext, os.path.join(self.folder, 'map' + ext))
        os.rmdir(os.path.dirname(tmp))

    def finish(self):
        self.write_grid()  # first: the battery may be pulled right after landing
        self.stop_bag(timeout=60)
        save = subprocess.run([sys.executable, os.path.join(HERE, 'save_map_db.py'), '--db', self.db,
                               '--out', self.folder], capture_output=True, text=True, timeout=600)
        if save.returncode != 0:
            self.get_logger().error(f'Full map save failed: {save.stdout[-300:]} {save.stderr[-300:]}')
        poses = flight_tools.count_poses(os.path.join(self.folder, 'poses.txt'))
        known, occupied, free = map_tools.grid_stats(self.grid.data) if self.grid else (0, 0, 0)
        status = 'saved' if save.returncode == 0 else 'grid only'
        flight_tools.write_manifest(
            self.folder, status=status, started=self.started, ended=flight_tools.timestamp(), poses=poses,
            known_cells=known, occupied_cells=occupied, free_cells=free,
            recording=flight_tools.bag_summary(os.path.join(self.folder, 'bag')))
        if poses >= flight_tools.MIN_POSES:
            flight_tools.set_latest(self.root, self.folder)
            self.get_logger().info(f'Flight saved ({status}, {poses} poses): {self.folder}')
        else:
            self.get_logger().warn(f'Only {poses} poses, so `latest` is unchanged: {self.folder}')
        self.folder = None

    def interrupted(self):
        """Stopped during a flight: RTAB-Map is going down too, so keep what we have and close the bag."""
        if self.folder:
            self.write_grid()
            self.stop_bag(timeout=20)
            flight_tools.write_manifest(self.folder, status='interrupted', started=self.started,
                                        ended=flight_tools.timestamp(),
                                        recording=flight_tools.bag_summary(os.path.join(self.folder, 'bag')))


def main():
    rclpy.init()
    node = FlightRecorder()
    threading.Thread(target=node.work, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.interrupted()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
