#!/usr/bin/env python3
# WORK IN PROGRESS: written for the Jetson but not yet run on one. On a laptop it fails as it should
# (no ZED SDK depth, no IMU, no PX4).
# Checks the drone stack while `make jetson` runs (`make jetson-verify`): the depth must come from the
# ZED SDK (zed_wrapper), not zed_usb's CPU stereo, and every bridge must carry data:
# ZED -> ROS 2 (RGB, depth, IMU, odometry), RTAB-Map, the TF chain, and MAVROS <-> PX4.
# Exit status is non-zero if any check fails.
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import Buffer, TransformListener

ZED = '/zed/zed_node/'
OPTICAL = 'zed_left_camera_frame_optical'
LISTEN_S = 6.0

try:
    from mavros_msgs.msg import State
except ImportError:
    State = None

failed = []


def report(ok, what, detail=''):
    tag = {True: 'ok  ', False: 'FAIL', None: 'warn'}[ok]
    print(f'  {tag}  {what}' + (f': {detail}' if detail else ''))
    if ok is False:
        failed.append(what)


class Verify(Node):
    def __init__(self):
        super().__init__('drone_stack_verify')
        self.msgs = {}
        self.count = {}
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        for name, typ, topic in (('rgb', Image, ZED + 'rgb/color/rect/image'),
                                 ('rgb_info', CameraInfo, ZED + 'rgb/color/rect/camera_info'),
                                 ('depth', Image, ZED + 'depth/depth_registered'),
                                 ('imu', Imu, ZED + 'imu/data'),
                                 ('zed_odom', Odometry, ZED + 'odom'),
                                 ('rtabmap_odom', Odometry, '/rtabmap/odom')):
            self.create_subscription(typ, topic, lambda m, n=name: self.store(n, m), reliable)
        if State is not None:
            self.create_subscription(State, '/mavros/state', lambda m: self.store('mavros', m), 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

    def store(self, name, msg):
        self.msgs[name] = msg
        self.count[name] = self.count.get(name, 0) + 1

    def rate(self, name):
        return self.count.get(name, 0) / LISTEN_S


def main():
    rclpy.init()
    node = Verify()
    end = time.time() + LISTEN_S
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('Depth source')
    pubs = node.get_publishers_info_by_topic(ZED + 'depth/depth_registered')
    names = [f'{p.node_namespace.rstrip("/")}/{p.node_name}' for p in pubs]
    if names == ['/zed/zed_node']:
        report(True, 'depth comes from the ZED SDK (zed_wrapper /zed/zed_node)')
    elif any(n.endswith('zed_usb') for n in names):
        report(False, 'depth source', f'{", ".join(names)}: CPU stereo from zed_usb, not the ZED SDK')
    elif not names:
        report(False, 'depth source', 'nothing publishes ' + ZED + 'depth/depth_registered')
    else:
        report(False, 'depth source', f'more than one or unknown publisher: {", ".join(names)}')

    print('ZED -> ROS 2')
    for name, what in (('rgb', 'RGB image'), ('rgb_info', 'RGB camera info'), ('depth', 'depth image')):
        report(node.rate(name) > 2, what, f'{node.rate(name):.1f} Hz')
    rgb, depth = node.msgs.get('rgb'), node.msgs.get('depth')
    if rgb and depth:
        same = (rgb.width, rgb.height, rgb.header.frame_id) == (depth.width, depth.height, depth.header.frame_id)
        report(same and depth.header.frame_id == OPTICAL, 'depth registered to RGB',
               f'RGB {rgb.width}x{rgb.height} {rgb.encoding} [{rgb.header.frame_id}], '
               f'depth {depth.width}x{depth.height} {depth.encoding} [{depth.header.frame_id}]')
        d = np.frombuffer(depth.data, dtype=np.float32) if depth.encoding == '32FC1' else None
        if d is not None:
            fill = np.isfinite(d).mean() * 100
            # The SDK's neural depth fills almost the whole image; CPU stereo leaves holes.
            report(fill > 80 or None, 'depth fill', f'{fill:.0f}% of pixels have depth')
    report(node.rate('imu') > 50 or False, 'IMU', f'{node.rate("imu"):.0f} Hz (ZED SDK only)')
    zo = node.rate('zed_odom')
    report(zo > 2 or None, 'ZED visual-inertial odometry',
           f'{zo:.1f} Hz' + ('' if zo else ' (expected only with use_zed_odometry:=true)'))

    print('RTAB-Map')
    ro = node.rate('rtabmap_odom')
    report(ro > 1 or zo > 2 or False, 'odometry into RTAB-Map',
           'ZED odometry' if zo > 2 else f'RTAB-Map visual odometry {ro:.1f} Hz')
    for parent, child in (('zed_camera_link', OPTICAL), ('odom', 'zed_camera_link'), ('map', 'odom')):
        ok = node.tf.can_transform(parent, child, rclpy.time.Time())
        report(ok, f'TF {parent} -> {child}')

    print('MAVROS <-> PX4')
    if State is None:
        report(None, 'MAVROS', 'mavros_msgs not installed')
    elif 'mavros' not in node.msgs:
        report(False, 'MAVROS', 'no /mavros/state (is mavros running?)')
    else:
        s = node.msgs['mavros']
        report(s.connected, 'PX4 link', f'connected={s.connected} mode={s.mode} armed={s.armed}')

    node.destroy_node()
    rclpy.try_shutdown()
    print()
    print('All checks passed.' if not failed else f'{len(failed)} check(s) failed: {", ".join(failed)}')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
