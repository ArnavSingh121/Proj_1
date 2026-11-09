#!/usr/bin/env python3
# WORK IN PROGRESS: tried only in the sim (2026-10-29), where it reached PX4 at about 7-8 Hz with
# ~130 ms latency. jetson/px4_vision.params, the PX4 settings it needs, does not exist yet. The sim test used
# EKF2_EV_CTRL 11, EKF2_GPS_CTRL 0, EKF2_HGT_REF 3 (vision), EKF2_EV_DELAY 130, EKF2_EV_NOISE_MD 1,
# EKF2_EV_POS_Z 0.24, EKF2_MAG_TYPE 5 (none); reset them afterwards, or the sim can't fly in Hold.
# RTAB-Map position -> PX4: publishes the drone's pose (TF parent_frame -> base_link) to MAVROS
# (/mavros/vision_pose/pose), which turns it into VISION_POSITION_ESTIMATE for PX4's EKF2.
# MAVROS converts ROS ENU/FLU to PX4 NED/FRD. PX4 must be set to use it (jetson/px4_vision.params).
#
# parent_frame 'odom' (default) is smooth but drifts slowly; 'map' adds RTAB-Map's loop-closure
# corrections, which arrive as jumps (EKF2 then resets its position to vision).
# Failsafe: if no new pose arrives for max_gap (tracking lost, odometry stopped, TF gone), nothing is
# sent, so PX4's EKF2 stops using vision after 0.4 s and its position-loss failsafe takes over.
# The 10 s report gives the pose latency (capture -> here): set EKF2_EV_DELAY to about that.
import statistics

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class VisionBridge(Node):
    def __init__(self):
        super().__init__('px4_vision_bridge')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')  # the drone's center (flight controller)
        self.declare_parameter('rate', 30.0)  # Hz polled; a pose is sent only when it is new
        self.declare_parameter('max_gap', 0.3)  # s without a new pose = lost (PX4 wants one every 0.2 s)
        self.declare_parameter('max_age', 1.0)  # s; older poses are never sent
        self.declare_parameter('topic', '/mavros/vision_pose/pose')
        p = lambda n: self.get_parameter(n).value
        self.parent, self.child = p('parent_frame'), p('child_frame')
        self.max_gap, self.max_age = p('max_gap'), p('max_age')
        self.pub = self.create_publisher(PoseStamped, p('topic'), 10)
        self.tf = Buffer(cache_time=Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf, self)
        self.last_stamp = None
        self.last_new = self.get_clock().now()
        self.sent, self.ages = 0, []
        self.sending = None  # last state reported in the log
        self.create_timer(1.0 / p('rate'), self.tick)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'{self.parent} -> {self.child} to {p("topic")}')

    def state(self, sending, why=''):
        if sending != self.sending:
            if sending:
                self.get_logger().info('Sending vision pose to PX4')
            else:
                self.get_logger().warn(f'NOT sending vision pose ({why}): PX4 will drop vision after 0.4 s')
            self.sending = sending

    def tick(self):
        try:
            t = self.tf.lookup_transform(self.parent, self.child, Time())
        except TransformException as e:
            self.state(False, f'no transform: {e}')
            return
        now, stamp = self.get_clock().now(), Time.from_msg(t.header.stamp)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            gap = (now - self.last_new).nanoseconds * 1e-9
            if gap > self.max_gap:
                self.state(False, f'no new pose for {gap:.1f} s')
            return
        age = (now - stamp).nanoseconds * 1e-9
        if age > self.max_age:
            self.state(False, f'pose is {age:.2f} s old')
            return
        self.last_stamp, self.last_new = stamp, now
        self.ages.append(age)
        msg = PoseStamped()
        msg.header.stamp = t.header.stamp  # the camera's capture time, so EKF2 can align it
        msg.header.frame_id = self.parent
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation = t.transform.rotation
        self.pub.publish(msg)
        self.sent += 1
        self.state(True)

    def report(self):
        if self.sent:
            self.get_logger().info(f'{self.sent / 10.0:.1f} Hz to PX4, '
                                   f'latency {statistics.median(self.ages) * 1000:.0f} ms (median)')
        self.sent, self.ages = 0, []


def main():
    rclpy.init()
    node = VisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
