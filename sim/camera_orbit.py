#!/usr/bin/env python3
# Turns the Gazebo window's camera around the drone it follows, from the gamepad that flies it.
# QGroundControl uses the two sticks; this only reads inputs QGroundControl leaves free:
#   D-pad left/right  orbit around the drone        D-pad up/down  camera higher / lower
#   RT / LT           zoom in / out (press harder: faster)    Y      back to the start view
#   X                 first-person view on/off: a window with the drone's own RGB camera, the images
#                     RTAB-Map gets (Gazebo's view camera cannot look forward from the drone).
#                     Esc or Q in that window also closes it.
# Both read the same /dev/input/js* device, which Linux allows. Moves the camera only: the drone,
# its sensors and RTAB-Map are not affected.
# Usage: python3 sim/camera_orbit.py [--device /dev/input/js0] [--model x500_depth_0] [--world W]
import argparse
import math
import os
import select
import struct
import sys
import time

# Linux joystick API (linux/joystick.h): struct js_event { u32 time; s16 value; u8 type; u8 number; }
EVENT = struct.Struct('IhBB')
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80
# Xbox-layout pads (xpad driver): axes 6/7 are the D-pad, 2/5 the triggers LT/RT (-32767 released,
# 32767 fully pressed), buttons 2/3 are X/Y. QGroundControl maps the triggers to gimbal pitch/yaw, which the
# x500 does not have, so they are free here.
HAT_X, HAT_Y = 6, 7
TRIGGER_LT, TRIGGER_RT = 2, 5
BUTTON_X, BUTTON_Y = 2, 3

ORBIT_DEG_PER_S = 60.0
ZOOM_PER_S = 1.0        # trigger fully pressed: distance changes by e (x2.7) per second
HEIGHT_M_PER_S = 1.0
# PX4 starts the follow camera at offset (-2, -2, 2) from the drone (px4-rc.gzsim).
START = (-135.0, math.hypot(2, 2), 2.0)


class Orbit:
    """Camera position around the drone: azimuth (deg), horizontal distance and height (m)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.azimuth, self.distance, self.height = START

    def step(self, hat_x, hat_y, zoom_in, zoom_out, dt):
        """Moves the camera for dt seconds of input. hat_x/hat_y are -1, 0 or 1 (left/up negative),
        zoom_in/zoom_out how far RT/LT are pressed (0 to 1)."""
        self.azimuth = (self.azimuth - hat_x * ORBIT_DEG_PER_S * dt + 180.0) % 360.0 - 180.0
        self.height = min(10.0, max(-1.0, self.height - hat_y * HEIGHT_M_PER_S * dt))
        zoom = (zoom_out - zoom_in) * ZOOM_PER_S * dt
        self.distance = min(15.0, max(1.0, self.distance * math.exp(zoom)))

    def offset(self):
        a = math.radians(self.azimuth)
        return (round(self.distance * math.cos(a), 3), round(self.distance * math.sin(a), 3),
                round(self.height, 3))


def sign(value):
    """D-pad axis value (-32767..32767) as -1, 0 or 1."""
    return (value > 16000) - (value < -16000)


def pressed(value):
    """Trigger axis value (-32767 released .. 32767 full) as 0..1, ignoring the first 10%."""
    amount = (value + 32767) / 65534
    return 0.0 if amount < 0.1 else min(1.0, (amount - 0.1) / 0.9)


class Pad:
    """State of the inputs we use, updated from raw js_event bytes."""

    def __init__(self):
        self.hat_x = self.hat_y = 0
        self.zoom_in = self.zoom_out = 0.0  # RT / LT, released until the pad reports otherwise
        self.buttons = set()

    def apply(self, data):
        """Applies one js_event. Returns the button number when a button was just pressed, else None."""
        _, value, kind, number = EVENT.unpack(data)
        initial = bool(kind & JS_EVENT_INIT)  # state reported on open, not a press
        kind &= ~JS_EVENT_INIT
        if kind == JS_EVENT_AXIS and number == HAT_X:
            self.hat_x = sign(value)
        elif kind == JS_EVENT_AXIS and number == HAT_Y:
            self.hat_y = sign(value)
        elif kind == JS_EVENT_AXIS and number == TRIGGER_RT:
            self.zoom_in = pressed(value)
        elif kind == JS_EVENT_AXIS and number == TRIGGER_LT:
            self.zoom_out = pressed(value)
        elif kind == JS_EVENT_BUTTON:
            new_press = number not in self.buttons and value and not initial
            (self.buttons.add if value else self.buttons.discard)(number)
            return number if new_press else None
        return None

    def moving(self):
        return bool(self.hat_x or self.hat_y or self.zoom_in or self.zoom_out)


def to_bgr(width, height, data):
    """Gazebo RGB_INT8 image bytes as the BGR array OpenCV shows."""
    import numpy
    return numpy.frombuffer(data, numpy.uint8).reshape(height, width, 3)[:, :, ::-1]


class FirstPerson:
    """Window with the drone's RGB camera. Subscribed only while open, so it costs nothing when off."""
    WINDOW = 'Drone camera (X or Esc: close)'

    def __init__(self, node, topic):
        from gz.msgs10.image_pb2 import Image
        self.node, self.topic, self.image_type = node, topic, Image
        self.frame = None
        self.on = False

    def _receive(self, msg):
        self.frame = (msg.width, msg.height, msg.data)

    def toggle(self):
        import cv2
        if self.on:
            self.node.unsubscribe(self.topic)
            cv2.destroyWindow(self.WINDOW)
            cv2.waitKey(1)
            self.on, self.frame = False, None
        else:
            self.node.subscribe(self.image_type, self.topic, self._receive)
            cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.WINDOW, 960, 720)
            self.on = True

    def show(self):
        """Draws the newest frame. Esc or Q in the window closes it, like X on the pad."""
        import cv2
        if not self.on:
            return
        frame, self.frame = self.frame, None
        if frame:
            cv2.imshow(self.WINDOW, to_bgr(*frame))
        # This OpenCV build does not report the window's own close button (the window comes back with
        # the next frame), so Esc or Q in the window closes it too.
        if (cv2.waitKey(1) & 0xFF) in (27, ord('q')):
            self.toggle()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/input/js0')
    parser.add_argument('--model', default='x500_depth_0')
    parser.add_argument('--world', default=os.environ.get('WORLD', 'default_warehouse'))
    args = parser.parse_args()

    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.vector3d_pb2 import Vector3d
    from gz.transport13 import Node

    node = Node()

    def send(offset):
        ok, _ = node.request('/gui/follow/offset', Vector3d(x=offset[0], y=offset[1], z=offset[2]),
                             Vector3d, Boolean, 1000)
        return ok

    orbit = Orbit()
    # Follow the drone (PX4 does this at startup; again here in case the view was moved since).
    if not node.request('/gui/follow', StringMsg(data=args.model), StringMsg, Boolean, 3000)[0]:
        sys.exit('No Gazebo window to move the camera of. Is the sim running (make sim)?')
    send(orbit.offset())
    first_person = FirstPerson(
        node, f'/world/{args.world}/model/{args.model}/link/camera_link/sensor/IMX214/image')
    print('D-pad: orbit and height, RT/LT: zoom in/out, Y: start view, X: first person. Ctrl+C to stop.')

    fd, pad, last = None, Pad(), time.monotonic()
    while True:
        if fd is None:  # (re)connect: the gamepad may be unplugged and plugged back in
            try:
                fd, pad = os.open(args.device, os.O_RDONLY | os.O_NONBLOCK), Pad()
                print(f'Reading {args.device}')
            except OSError:
                time.sleep(1.0)
                continue
        ready, _, _ = select.select([fd], [], [], 1 / 30)
        try:
            while ready:
                data = os.read(fd, EVENT.size)
                if len(data) < EVENT.size:
                    break
                button = pad.apply(data)
                if button == BUTTON_Y:
                    orbit.reset()
                    send(orbit.offset())
                elif button == BUTTON_X:
                    first_person.toggle()
        except BlockingIOError:
            pass
        except OSError:
            print(f'Lost {args.device}, waiting for it to come back')
            os.close(fd)
            fd = None
            continue
        now = time.monotonic()
        if pad.moving():
            orbit.step(pad.hat_x, pad.hat_y, pad.zoom_in, pad.zoom_out, now - last)
            send(orbit.offset())
        first_person.show()
        last = now


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
