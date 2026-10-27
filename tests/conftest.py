# Lets the tests import the scripts in sim/, tools/ and zed_usb/ directly. No ROS install is needed:
# test_zed_usb_publisher.py swaps in a small stand-in for rclpy (ros_stub.py).
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for d in ('sim', 'tools', 'zed_usb', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, d))
