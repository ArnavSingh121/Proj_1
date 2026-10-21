#!/usr/bin/env python3
# ZED camera over plain USB, for laptops without an NVIDIA GPU (the ZED SDK needs CUDA).
# Reads the side-by-side stereo video (UVC), rectifies it with the camera's factory calibration
# (downloaded once from Stereolabs by serial number), computes depth on the CPU with OpenCV SGBM,
# and publishes it under the same topics as the ZED ROS 2 wrapper, so RTAB-Map runs in the same
# RGB-D setup as on the drone. Depth is classic stereo matching, not the SDK's neural depth.
#
# Run as node 'zed_node' in namespace 'zed' (zed_usb_rtabmap.launch.py does this). If a NumPy 2 in
# ~/.local breaks the system OpenCV, run with PYTHONNOUSERSITE=1.
import configparser
import glob
import os
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

CALIB_URL = 'https://calib.stereolabs.com/?SN={}'
# Per-eye image width -> calibration section suffix in SN<serial>.conf
RES_NAMES = {672: 'VGA', 1280: 'HD', 1920: 'FHD', 2208: '2K'}
REQUEST = {'VGA': (1344, 376), 'HD720': (2560, 720), 'HD1080': (3840, 1080), 'HD2K': (4416, 1242)}


def find_zed():
    """Video device and serial number of the first ZED on USB."""
    device = next((f'/dev/{os.path.basename(p)}' for p in sorted(glob.glob('/sys/class/video4linux/video*'))
                   if open(f'{p}/name').read().startswith('ZED')), None)
    # The serial number is on the camera's HID (IMU) interface; the video interface reports a placeholder.
    serial = next((open(f'{d}/serial').read().strip() for d in glob.glob('/sys/bus/usb/devices/*')
                   if os.path.exists(f'{d}/idVendor') and open(f'{d}/idVendor').read().strip() == '2b03'
                   and 'HID' in open(f'{d}/product').read() and os.path.exists(f'{d}/serial')), None)
    return device, serial


def load_calibration(path, res, w, h):
    """OpenCV stereo calibration from a ZED .conf, as Stereolabs' zed-open-capture builds it."""
    c = configparser.ConfigParser()
    c.read(path)

    def cam(side):
        s = c[f'{side}_CAM_{res}']
        K = np.array([[float(s['fx']), 0, float(s['cx'])], [0, float(s['fy']), float(s['cy'])], [0, 0, 1]])
        if f'{side}_DISTO' in c:
            # ZED 2/2i: OpenCV rational model; the k1-k3 in the resolution sections are only its numerator.
            s = c[f'{side}_DISTO']
            D = np.array([float(s[k]) for k in ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6')])
        else:
            D = np.array([float(s[k]) for k in ('k1', 'k2', 'p1', 'p2', 'k3')])
        return K, D

    Kl, Dl = cam('LEFT')
    Kr, Dr = cam('RIGHT')
    st = c['STEREO']
    R = cv2.Rodrigues(np.array([float(st[f'RX_{res}']), float(st[f'CV_{res}']), float(st[f'RZ_{res}'])]))[0]
    T = np.array([float(st['Baseline']), float(st['TY']), float(st['TZ'])]) / 1000.0  # mm -> m
    return Kl, Dl, Kr, Dr, R, T


def flip_calibration(Kl, Dl, Kr, Dr, R, T, w, h):
    """Calibration for an upside-down camera: both images turned 180 degrees and left/right swapped."""
    F = np.diag([-1.0, -1.0, 1.0])

    def k(K):
        K = K.copy()
        K[0, 2], K[1, 2] = w - 1 - K[0, 2], h - 1 - K[1, 2]
        return K

    def d(D):
        D = D.copy()
        D[2:4] *= -1  # tangential p1, p2 change sign under a 180 degree turn; radial terms don't
        return D

    return k(Kr), d(Dr), k(Kl), d(Dl), F @ R.T @ F, -F @ R.T @ T


class ZedUsb(Node):
    def __init__(self):
        super().__init__('zed_node')
        self.declare_parameter('device', '')  # '' = first ZED found
        # CPU depth keeps up with VGA at 15 fps (~60 ms/frame); HD720 has 4x the pixels.
        self.declare_parameter('resolution', 'VGA')  # VGA, HD720, HD1080, HD2K (USB 2 only carries VGA)
        self.declare_parameter('fps', 15)
        self.declare_parameter('flip', False)  # true if the camera is mounted upside down
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('calib_dir', os.path.expanduser('~/.local/share/zed/settings'))
        p = lambda n: self.get_parameter(n).value
        self.min_depth, self.max_depth, self.flip = p('min_depth'), p('max_depth'), p('flip')

        device, serial = find_zed()
        if not device or not serial:
            self.get_logger().warn('Waiting for a ZED camera on USB...')
            while not (device and serial):
                time.sleep(1.0)
                device, serial = find_zed()
            time.sleep(2.0)  # let the camera finish starting up after it appears
        self.fixed_device = p('device')
        device = self.fixed_device or device
        rw, rh = REQUEST[p('resolution')]
        self.request = (rw, rh, p('fps'))
        self.cap = self.open(device)
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError(f'Cannot read from {device}')
        self.w, self.h = frame.shape[1] // 2, frame.shape[0]
        res = RES_NAMES[self.w]
        if (self.w * 2, self.h) != (rw, rh):
            self.get_logger().warn(f'Asked for {p("resolution")}, the camera gives {res} {self.w}x{self.h} '
                                   '(a USB 2 link only carries VGA: use a USB 3 port / flip the USB-C plug)')

        calib = os.path.join(p('calib_dir'), f'SN{serial}.conf')
        if not os.path.exists(calib):
            os.makedirs(p('calib_dir'), exist_ok=True)
            self.get_logger().info(f'Downloading factory calibration for SN {serial}')
            urllib.request.urlretrieve(CALIB_URL.format(serial), calib)
        cal = load_calibration(calib, res, self.w, self.h)
        if self.flip:
            cal = flip_calibration(*cal, self.w, self.h)
        Kl, Dl, Kr, Dr, R, T = cal
        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(Kl, Dl, Kr, Dr, (self.w, self.h), R, T,
                                                    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.map_l = cv2.initUndistortRectifyMap(Kl, Dl, R1, P1, (self.w, self.h), cv2.CV_16SC2)
        self.map_r = cv2.initUndistortRectifyMap(Kr, Dr, R2, P2, (self.w, self.h), cv2.CV_16SC2)
        self.P = P1
        self.fx_baseline = P1[0, 0] * abs(P2[0, 3] / P2[0, 0])  # depth = fx * baseline / disparity

        # Disparity search range covers everything from min_depth outwards.
        num_disp = int(np.ceil(self.fx_baseline / self.min_depth / 16.0)) * 16
        block = 5
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=num_disp, blockSize=block, P1=8 * block * block,
            P2=32 * block * block, disp12MaxDiff=1, uniquenessRatio=10, speckleWindowSize=100,
            speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

        self.frame_id = 'zed_left_camera_frame_optical'  # same frame name as the ZED wrapper
        self.pub_rgb = self.create_publisher(Image, '~/rgb/color/rect/image', 10)
        self.pub_rgb_info = self.create_publisher(CameraInfo, '~/rgb/color/rect/camera_info', 10)
        self.pub_depth = self.create_publisher(Image, '~/depth/depth_registered', 10)
        self.pub_depth_info = self.create_publisher(CameraInfo, '~/depth/camera_info', 10)
        self.get_logger().info(
            f'ZED SN {serial} on {device}: {res} {self.w}x{self.h}, baseline {self.fx_baseline / P1[0, 0]:.4f} m, '
            f'fx {P1[0, 0]:.1f}, flip {self.flip}, depth {self.min_depth}-{self.max_depth} m '
            f'({num_disp} disparities)')
        self.check_alignment(frame)

    def open(self, device):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        rw, rh, fps = self.request
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, rw)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, rh)
        cap.set(cv2.CAP_PROP_FPS, fps)
        return cap

    def reopen(self):
        """After a USB reset the ZED comes back under a new /dev/video number: find it and reopen."""
        self.cap.release()
        device = self.fixed_device or find_zed()[0]
        if device:
            self.cap = self.open(device)
            ok, frame = self.cap.read()
            if ok and frame.shape[1] // 2 == self.w and frame.shape[0] == self.h:
                self.get_logger().info(f'Camera back on {device}')
                return
            self.cap.release()
        time.sleep(1.0)

    def split(self, frame):
        left, right = frame[:, :self.w], frame[:, self.w:]
        if self.flip:
            left, right = cv2.rotate(right, cv2.ROTATE_180), cv2.rotate(left, cv2.ROTATE_180)
        return (cv2.remap(left, *self.map_l, cv2.INTER_LINEAR),
                cv2.remap(right, *self.map_r, cv2.INTER_LINEAR))

    def check_alignment(self, frame):
        """Warns if rectification looks wrong: matched features should share rows, with left x > right x."""
        left, right = self.split(frame)
        orb = cv2.ORB_create(1000)
        kl, dl = orb.detectAndCompute(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), None)
        kr, dr = orb.detectAndCompute(cv2.cvtColor(right, cv2.COLOR_BGR2GRAY), None)
        if dl is None or dr is None or len(dl) < 2 or len(dr) < 2:
            return
        pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(dl, dr, k=2)
        m = [a for a, b in (p for p in pairs if len(p) == 2) if a.distance < 0.75 * b.distance]  # ratio test
        if len(m) < 20:
            return
        dx = np.array([kl[x.queryIdx].pt[0] - kr[x.trainIdx].pt[0] for x in m])
        dy = np.array([kl[x.queryIdx].pt[1] - kr[x.trainIdx].pt[1] for x in m])
        good = np.abs(dy) < 3
        msg = (f'Rectification check: {len(m)} matches, median |dy| {np.median(np.abs(dy)):.2f} px, '
               f'{(dx[good] > 0).mean() * 100:.0f}% with left x > right x')
        if np.median(np.abs(dy)) > 1.5 or (dx[good] > 0).mean() < 0.8:
            self.get_logger().warn(msg + ' -> calibration or flip looks wrong')
        else:
            self.get_logger().info(msg)

    def info(self, stamp):
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = stamp, self.frame_id
        m.width, m.height, m.distortion_model = self.w, self.h, 'plumb_bob'
        m.d = [0.0] * 5  # images are already rectified
        m.k = self.P[:, :3].flatten().tolist()
        m.r = np.eye(3).flatten().tolist()
        m.p = self.P.flatten().tolist()
        return m

    def image(self, stamp, array, encoding):
        m = Image()
        m.header.stamp, m.header.frame_id = stamp, self.frame_id
        m.height, m.width = array.shape[:2]
        m.encoding = encoding
        m.step = array.strides[0]
        m.data = array.tobytes()
        return m

    def spin(self):
        failures = 0
        while rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                failures += 1
                if failures == 1:
                    self.get_logger().warn('Camera read failed (USB disconnect?), reconnecting')
                self.reopen()
                continue
            failures = 0
            stamp = self.get_clock().now().to_msg()
            left, right = self.split(frame)
            disp = self.sgbm.compute(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY),
                                     cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)).astype(np.float32) / 16.0
            with np.errstate(divide='ignore'):
                depth = np.where(disp > 0, self.fx_baseline / disp, np.nan).astype(np.float32)
            depth[(depth < self.min_depth) | (depth > self.max_depth)] = np.nan
            self.pub_rgb.publish(self.image(stamp, left, 'bgr8'))
            self.pub_rgb_info.publish(self.info(stamp))
            self.pub_depth.publish(self.image(stamp, depth, '32FC1'))
            self.pub_depth_info.publish(self.info(stamp))
            rclpy.spin_once(self, timeout_sec=0)


def main():
    rclpy.init()
    node = ZedUsb()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
