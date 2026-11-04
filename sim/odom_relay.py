#!/usr/bin/env python3
# Simulated odometry input for RTAB-Map (sim/rtabmap.launch.py starts it).
# Takes Gazebo's odometry (/uav/odom, frames x500_depth_0/odom -> x500_depth_0/base_footprint), drops
# invalid samples (NaN, non-unit quaternion, time going backwards, teleport jumps) instead of passing them
# on, fills in the all-zero covariance, renames the frames to odom -> base_link, and publishes
# /uav/odom_checked. With publish_tf:=true it also broadcasts odom -> base_link, for odometry:=sim.
# Warns when odometry stops arriving and when it comes back; it never exits because of bad input.
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sensor_checks  # noqa: E402


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        self.declare_parameter('input', '/uav/odom')
        self.declare_parameter('output', '/uav/odom_checked')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('max_speed', 20.0)  # m/s; anything faster is a glitch
        self.declare_parameter('timeout', 1.0)  # s without odometry before warning
        p = lambda n: self.get_parameter(n).value
        self.odom_frame, self.base_frame = p('odom_frame'), p('base_frame')
        self.max_speed = p('max_speed')
        self.last = None
        self.dropped = 0
        self.staleness = sensor_checks.Staleness(p('timeout'))
        self.pub = self.create_publisher(Odometry, p('output'), 10)
        self.tf = TransformBroadcaster(self) if p('publish_tf') else None
        self.create_subscription(Odometry, p('input'), self.on_odom, 10)
        self.create_timer(0.5, self.check_stale)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_stale(self):
        msg = self.staleness.report(self.now())
        if msg:
            self.get_logger().warn(f'Odometry missing: {msg} on {self.get_parameter("input").value}')

    def on_odom(self, msg):
        try:
            self.relay(msg)
        except Exception as e:  # a malformed message must never take the node down
            self.get_logger().error(f'Dropped odometry message: {e}')

    def relay(self, msg):
        if self.staleness.seen(self.now()):
            self.get_logger().info('Odometry resumed')
        pose = msg.pose.pose
        position = (pose.position.x, pose.position.y, pose.position.z)
        orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        problem = sensor_checks.odometry_problem(position, orientation, stamp, self.last, self.max_speed)
        if problem:
            self.dropped += 1
            self.get_logger().warn(f'Dropped invalid odometry ({problem}); {self.dropped} dropped so far',
                                   throttle_duration_sec=2.0)
            return
        self.last = (position, stamp)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id, out.child_frame_id = self.odom_frame, self.base_frame
        out.pose.pose = pose
        out.pose.covariance = sensor_checks.with_covariance(msg.pose.covariance)
        out.twist.twist = msg.twist.twist
        out.twist.covariance = sensor_checks.with_covariance(msg.twist.covariance)
        self.pub.publish(out)

        if self.tf:
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = position
            t.transform.rotation = pose.orientation
            self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
