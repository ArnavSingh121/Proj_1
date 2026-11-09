"""Tier 2: unit tests for zed_usb/zed_usb_publisher.py, the only application logic in the repo.

No camera, network or ROS graph is needed: the sysfs tree, the video capture, the calibration
download and the ROS clock/publishers are replaced by fakes. rclpy and the message packages must be
importable, so source the ROS setup first (`make test` does); otherwise this file is skipped.

Calibration files are written by `write_conf` in the format Stereolabs serves (SN<serial>.conf).
"""
import importlib.util

import cv2
import numpy as np
import pytest

from conftest import REPO

rclpy = pytest.importorskip('rclpy', reason='rclpy not importable: source /opt/ros/humble/setup.bash')
from builtin_interfaces.msg import Time  # noqa: E402  (needs the ROS setup, like rclpy)

_spec = importlib.util.spec_from_file_location('zed_usb_publisher', REPO / 'zed_usb/zed_usb_publisher.py')
pub = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pub)

W, H = 672, 376      # per-eye VGA, what a ZED gives over USB 2
SERIAL = '38085162'


# ---- Fakes and fixture builders ----

def write_conf(path, *, distortion=True, rational=False, rotation=True):
    """A ZED factory calibration file with VGA and HD sections.

    distortion=False and rotation=False give an ideal camera (rectification is then almost the
    identity), which the node tests use so synthetic stereo images stay aligned after rectifying.
    rational=True adds the ZED 2/2i [LEFT_DISTO]/[RIGHT_DISTO] sections (8-coefficient model).
    """
    def cam(fx, cx, cy, k1):
        lines = [f'fx={fx}', f'fy={fx + 0.3}', f'cx={cx}', f'cy={cy}']
        coeffs = dict(k1=k1, k2=0.027, p1=0.0004, p2=-0.0003, k3=-0.011) if distortion else \
            dict(k1=0, k2=0, p1=0, p2=0, k3=0)
        return lines + [f'{k}={v}' for k, v in coeffs.items()]

    center = ((W - 1) / 2, (H - 1) / 2)
    sections = {
        'LEFT_CAM_VGA': cam(350.0, *(center if not distortion else (331.4, 187.2)), -0.17),
        'RIGHT_CAM_VGA': cam(351.0, *(center if not distortion else (338.9, 190.1)), -0.16),
        'LEFT_CAM_HD': cam(700.0, 640.2, 360.1, -0.18),
        'RIGHT_CAM_HD': cam(702.0, 641.3, 359.4, -0.19),
        'STEREO': ['Baseline=119.93', 'TY=0.21' if rotation else 'TY=0', 'TZ=-0.15' if rotation else 'TZ=0']
        + [f'{a}_{r}={v if rotation else 0}' for r in ('VGA', 'HD')
           for a, v in (('CV', 0.0021), ('RX', -0.0011), ('RZ', 0.0007))],
    }
    if rational:
        for side, k1 in (('LEFT', 1.71), ('RIGHT', 1.66)):
            sections[f'{side}_DISTO'] = [f'k1={k1}', 'k2=0.52', 'k3=0.02', 'k4=2.05', 'k5=1.04', 'k6=0.11',
                                         'p1=0.0002', 'p2=-0.0001']
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(f'[{name}]\n' + '\n'.join(body) + '\n\n' for name, body in sections.items()))
    return path


def texture(h, w, seed=0):
    """Random blocky gray texture: plenty of ORB corners and stereo-matchable detail."""
    small = np.random.default_rng(seed).integers(0, 256, (h // 4 + 1, w // 4 + 1), dtype=np.uint8)
    return cv2.cvtColor(cv2.resize(small, (w // 4 * 4 + 4, h // 4 * 4 + 4), interpolation=cv2.INTER_LINEAR)[:h, :w],
                        cv2.COLOR_GRAY2BGR)


def stereo_pair(shift=20, dy=0, swap=False):
    """Left/right views of a flat scene `shift` px of disparity away (left x > right x), optionally
    misaligned by `dy` rows or with left and right swapped."""
    scene = texture(H + 20, W + shift + 20)
    left = scene[10:10 + H, 0:W]                         # scene column c is at x = c on the left...
    right = scene[10 + dy:10 + dy + H, shift:shift + W]  # ...and at x = c - shift on the right
    return (right, left) if swap else (left, right)


def side_by_side(left, right):
    """The frame a ZED sends over UVC: both eyes next to each other."""
    return np.ascontiguousarray(np.hstack([left, right]))


def identity_maps(w=W, h=H):
    xs, ys = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    return xs, ys


class RecordingLogger:
    """Stands in for the node's rclpy logger: keeps (level, message) pairs."""

    def __init__(self):
        self.records = []

    def __getattr__(self, level):  # info, warn, warning, error, debug
        return lambda msg, *args, **kwargs: self.records.append((level, msg))

    def messages(self, *levels):
        return [msg for level, msg in self.records if level in levels]


class FakeCapture:
    """Stands in for cv2.VideoCapture: serves queued frames, None meaning a failed read."""

    def __init__(self, frames=()):
        self.frames = list(frames)
        self.props = {}
        self.released = False

    def set(self, prop, value):
        self.props[prop] = value
        return True

    def read(self):
        if not self.frames:
            return False, None
        frame = self.frames.pop(0)
        return frame is not None, frame

    def release(self):
        self.released = True


class FakePublisher:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture
def log(monkeypatch):
    logger = RecordingLogger()
    monkeypatch.setattr(pub.ZedUsb, 'get_logger', lambda self: logger)
    return logger


@pytest.fixture
def sleeps(monkeypatch):
    """time.sleep in the publisher, recorded instead of waited for (see Sleeps)."""
    calls = Sleeps()
    monkeypatch.setattr(pub.time, 'sleep', calls)
    return calls


class Retried(Exception):
    """Raised by the patched time.sleep to end a retry loop the publisher would otherwise run forever."""


class Sleeps(list):
    """Recorded time.sleep calls. The publisher never gives up on the camera or the calibration, so a
    test that wants to see it retry sets `limit`: the sleep after that many raises Retried."""
    limit = None

    def __call__(self, seconds):
        self.append(seconds)
        if self.limit is not None and len(self) > self.limit:
            raise Retried


@pytest.fixture
def make_node(monkeypatch, tmp_path, log, sleeps):
    """Builds a real ZedUsb node against fakes: make_node(frames, params={...}, calibration=True).

    HOME is moved to tmp_path, so the default calib_dir is tmp_path/.local/share/zed/settings.
    Returns (node, opened) where opened lists (device, api, FakeCapture) per VideoCapture call.
    """
    monkeypatch.setenv('HOME', str(tmp_path))
    calib_dir = tmp_path / '.local/share/zed/settings'
    nodes = []

    def make(frames, params=None, calibration=True, devices=(('/dev/video7', SERIAL),)):
        if calibration:
            write_conf(calib_dir / f'SN{SERIAL}.conf', distortion=False, rotation=False)
        found = list(devices)
        monkeypatch.setattr(pub, 'find_zed', lambda: found.pop(0) if len(found) > 1 else found[0])
        opened = []

        def video_capture(device, api):
            cap = FakeCapture(frames)
            opened.append((device, api, cap))
            return cap

        monkeypatch.setattr(pub.cv2, 'VideoCapture', video_capture)
        args = ['--ros-args'] + [a for k, v in (params or {}).items() for a in ('-p', f'{k}:={v}')]
        rclpy.init(args=args)
        node = pub.ZedUsb()
        nodes.append(node)
        return node, opened

    yield make
    for node in nodes:
        node.destroy_node()
    rclpy.try_shutdown()


def bare_node(**attrs):
    """A ZedUsb without running __init__ (no camera, no rclpy.init), for testing single methods."""
    node = pub.ZedUsb.__new__(pub.ZedUsb)
    node.w, node.h, node.flip = W, H, False
    node.failures = 0
    node.map_l = node.map_r = identity_maps()
    node.frame_id = 'zed_left_camera_frame_optical'
    node.P = np.array([[350.0, 0, 335.5, 0], [0, 350.0, 187.5, 0], [0, 0, 1, 0]])
    for k, v in attrs.items():
        setattr(node, k, v)
    return node


# ---- load_calibration ----

def test_load_calibration_reads_intrinsics_distortion_and_stereo_extrinsics(tmp_path):
    conf = write_conf(tmp_path / 'SN1.conf')
    Kl, Dl, Kr, Dr, R, T = pub.load_calibration(str(conf), 'VGA', W, H)
    np.testing.assert_allclose(Kl, [[350.0, 0, 331.4], [0, 350.3, 187.2], [0, 0, 1]])
    np.testing.assert_allclose(Kr, [[351.0, 0, 338.9], [0, 351.3, 190.1], [0, 0, 1]])
    np.testing.assert_allclose(Dl, [-0.17, 0.027, 0.0004, -0.0003, -0.011])  # OpenCV order k1 k2 p1 p2 k3
    assert Dr.shape == (5,)
    # Rotation vector is (RX, CV, RZ): CV is the y component in Stereolabs' files.
    np.testing.assert_allclose(R, cv2.Rodrigues(np.array([-0.0011, 0.0021, 0.0007]))[0])
    np.testing.assert_allclose(T, [0.11993, 0.00021, -0.00015])  # mm -> m


def test_load_calibration_picks_the_resolution_section(tmp_path):
    conf = write_conf(tmp_path / 'SN1.conf')
    Kl, *_ = pub.load_calibration(str(conf), 'HD', 1280, 720)
    assert (Kl[0, 0], Kl[0, 2]) == (700.0, 640.2)


def test_load_calibration_uses_the_rational_model_of_zed2_files(tmp_path):
    """ZED 2/2i files carry an 8-coefficient model in *_DISTO; the per-resolution k's are ignored."""
    conf = write_conf(tmp_path / 'SN1.conf', rational=True)
    _, Dl, _, Dr, _, _ = pub.load_calibration(str(conf), 'VGA', W, H)
    np.testing.assert_allclose(Dl, [1.71, 0.52, 0.0002, -0.0001, 0.02, 2.05, 1.04, 0.11])  # k1 k2 p1 p2 k3-k6
    assert Dr[0] == 1.66 and Dr.shape == (8,)


def test_load_calibration_fails_for_a_resolution_not_in_the_file(tmp_path):
    with pytest.raises(KeyError):
        pub.load_calibration(str(write_conf(tmp_path / 'SN1.conf')), '2K', 2208, 1242)


# ---- flip_calibration ----

def project(K, D, points, R=np.eye(3), T=np.zeros(3)):
    pixels, _ = cv2.projectPoints(points, cv2.Rodrigues(R)[0], T, K, D)
    return pixels.reshape(-1, 2)


@pytest.mark.parametrize('rational', [False, True], ids=['5-coeff', 'rational'])
def test_flipped_calibration_describes_the_upside_down_camera(tmp_path, rational):
    """Property test: mount the camera upside down and every scene point must land where the flipped
    calibration says.

    Upside down, the new left eye is the old right eye turned 180 degrees about its optical axis
    (x, y -> -x, -y), and every image is turned 180 degrees (u, v -> w-1-u, h-1-v).
    """
    cal = pub.load_calibration(str(write_conf(tmp_path / 'SN1.conf', rational=rational)), 'VGA', W, H)
    Kl, Dl, Kr, Dr, R, T = cal
    rng = np.random.default_rng(1)
    X = np.column_stack([rng.uniform(-1, 1, 50), rng.uniform(-0.6, 0.6, 50), rng.uniform(1.5, 5, 50)])
    left_px, right_px = project(Kl, Dl, X), project(Kr, Dr, X, R, T)  # OpenCV: X_right = R X_left + T

    fKl, fDl, fKr, fDr, fR, fT = pub.flip_calibration(*cal, W, H)
    F = np.diag([-1.0, -1.0, 1.0])
    X_new_left = (F @ (R @ X.T + T[:, None])).T  # the old right eye, turned over
    turn = np.array([W - 1, H - 1])
    np.testing.assert_allclose(project(fKl, fDl, X_new_left), turn - right_px, atol=1e-6)
    np.testing.assert_allclose(project(fKr, fDr, X_new_left, fR, fT), turn - left_px, atol=1e-6)


def test_flipping_twice_restores_the_calibration_and_never_mutates_input(tmp_path):
    cal = pub.load_calibration(str(write_conf(tmp_path / 'SN1.conf')), 'VGA', W, H)
    before = [a.copy() for a in cal]
    twice = pub.flip_calibration(*pub.flip_calibration(*cal, W, H), W, H)
    for original, a, b in zip(before, cal, twice):
        np.testing.assert_array_equal(a, original)
        np.testing.assert_allclose(b, original, atol=1e-12)


# ---- disparity_to_depth ----

def test_depth_is_focal_length_times_baseline_over_disparity():
    depth = pub.disparity_to_depth(np.array([[42.0, 21.0, 4.2]]), fx_baseline=42.0, min_depth=0.3, max_depth=10)
    assert depth.dtype == np.float32
    np.testing.assert_allclose(depth, [[1.0, 2.0, 10.0]])


def test_depth_is_nan_without_a_match_or_out_of_range():
    # SGBM marks unmatched pixels with negative disparity (-1/16 after scaling); 0 means infinitely far.
    disp = np.array([-0.0625, 0.0, 200.0, 4.0, 1.0])  # 0.21 m, 10.5 m and 42 m are out of 0.3-10 m
    depth = pub.disparity_to_depth(disp, fx_baseline=42.0, min_depth=0.3, max_depth=10)
    assert np.isnan(depth).all()


def test_depth_keeps_the_image_shape():
    disp = np.full((H, W), 21.0, dtype=np.float32)
    assert pub.disparity_to_depth(disp, 42.0, 0.3, 10).shape == (H, W)


# ---- find_zed (fake sysfs) ----

@pytest.fixture
def sysfs(tmp_path, monkeypatch):
    """Fake /sys tree under tmp_path: sysfs(video={'video0': 'name'}, usb={'1-2': {'idVendor': ...}})."""
    real_glob = pub.glob.glob
    monkeypatch.setattr(pub.glob, 'glob', lambda pattern: real_glob(str(tmp_path) + pattern))

    def build(video=None, usb=None):
        for node, name in (video or {}).items():
            d = tmp_path / 'sys/class/video4linux' / node
            d.mkdir(parents=True)
            (d / 'name').write_text(name + '\n')
        for dev, attrs in (usb or {}).items():
            d = tmp_path / 'sys/bus/usb/devices' / dev
            d.mkdir(parents=True)
            for k, v in attrs.items():
                (d / k).write_text(v + '\n')
    return build


WEBCAM = {'idVendor': '0bda', 'product': 'Integrated_Webcam_HD', 'serial': '200901010001'}
ZED_VIDEO_IF = {'idVendor': '2b03', 'product': 'ZED 2i', 'serial': 'OV0001'}  # placeholder serial
ZED_HID = {'idVendor': '2b03', 'product': 'ZED 2i HID Interface', 'serial': SERIAL}


def test_find_zed_returns_the_video_device_and_the_hid_serial(sysfs):
    sysfs(video={'video0': 'Integrated_Webcam_HD', 'video2': 'ZED 2i: ZED 2i', 'video3': 'ZED 2i: ZED 2i'},
          usb={'1-1': WEBCAM, '1-2': ZED_VIDEO_IF, '1-3': ZED_HID, '1-3:1.0': {}})  # 1-3:1.0: an interface dir
    assert pub.find_zed() == ('/dev/video2', SERIAL)


def test_find_zed_without_a_zed(sysfs):
    sysfs(video={'video0': 'Integrated_Webcam_HD'}, usb={'1-1': WEBCAM})
    assert pub.find_zed() == (None, None)


def test_find_zed_while_the_hid_interface_is_not_up_yet(sysfs):
    sysfs(video={'video2': 'ZED 2i: ZED 2i'}, usb={'1-2': ZED_VIDEO_IF})
    assert pub.find_zed() == ('/dev/video2', None)


def test_find_zed_ignores_hid_devices_of_other_vendors_and_hid_without_serial(sysfs):
    no_serial = {k: v for k, v in ZED_HID.items() if k != 'serial'}
    sysfs(video={'video2': 'ZED 2i: ZED 2i'},
          usb={'1-1': {'idVendor': '046d', 'product': 'USB Receiver HID', 'serial': 'X'}, '1-3': no_serial})
    assert pub.find_zed() == ('/dev/video2', None)


def test_find_zed_tolerates_a_stereolabs_device_without_product(sysfs):
    # Only the product-less device: with others present, glob order decides whether it is reached.
    sysfs(video={'video2': 'ZED 2i: ZED 2i'}, usb={'1-2': {'idVendor': '2b03'}})
    assert pub.find_zed() == ('/dev/video2', None)


def test_find_zed_picks_the_lower_numbered_node_past_video9(sysfs):
    """A ZED's capture node comes right before its metadata node (same name): video9, not video10."""
    sysfs(video={'video9': 'ZED 2i: ZED 2i', 'video10': 'ZED 2i: ZED 2i'}, usb={'1-3': ZED_HID})
    assert pub.find_zed() == ('/dev/video9', SERIAL)


# ---- Node set-up ----

def test_node_opens_the_camera_and_sets_up_stereo(make_node, log):
    left, right = stereo_pair()
    node, opened = make_node([side_by_side(left, right)])
    (device, api, cap), = opened
    assert (device, api) == ('/dev/video7', cv2.CAP_V4L2)
    assert (cap.props[cv2.CAP_PROP_FRAME_WIDTH], cap.props[cv2.CAP_PROP_FRAME_HEIGHT],
            cap.props[cv2.CAP_PROP_FPS]) == (1344, 376, 30)
    assert (node.w, node.h, node.flip) == (W, H, False)
    # depth = fx * baseline / disparity, baseline 119.93 mm from the calibration file.
    assert node.fx_baseline == pytest.approx(node.P[0, 0] * 0.11993, rel=1e-6)
    # SGBM searches every disparity from 0 up to the one of an object at min_depth (0.3 m).
    n = node.sgbm.getNumDisparities()
    assert n % 16 == 0 and node.fx_baseline / 0.3 <= n < node.fx_baseline / 0.3 + 16
    assert not log.messages('warn', 'warning')
    assert any('Rectification check' in m for m in log.messages('info'))


def test_min_depth_parameter_sets_the_disparity_range(make_node):
    node, _ = make_node([side_by_side(*stereo_pair())], params={'min_depth': 1.0})
    n = node.sgbm.getNumDisparities()
    assert node.fx_baseline / 1.0 <= n < node.fx_baseline / 1.0 + 16


def test_calibration_is_downloaded_once_by_serial(make_node, monkeypatch, tmp_path):
    downloads = []

    def urlretrieve(url, path):
        downloads.append((url, path))
        write_conf(tmp_path / 'served.conf', distortion=False, rotation=False).replace(path)

    monkeypatch.setattr(pub.urllib.request, 'urlretrieve', urlretrieve)
    make_node([side_by_side(*stereo_pair())], calibration=False)
    target = tmp_path / '.local/share/zed/settings' / f'SN{SERIAL}.conf'
    # Downloaded next to the target, then renamed into place once complete.
    assert downloads == [(f'https://calib.stereolabs.com/?SN={SERIAL}', f'{target}.part')]
    assert sorted(p.name for p in target.parent.iterdir()) == [target.name]


def test_existing_calibration_is_not_downloaded_again(make_node, monkeypatch):
    monkeypatch.setattr(pub.urllib.request, 'urlretrieve', lambda *a: pytest.fail('downloaded again'))
    make_node([side_by_side(*stereo_pair())])


def test_interrupted_download_leaves_no_calibration_file(make_node, monkeypatch, tmp_path, log, sleeps):
    """The node keeps retrying, and never leaves a partial SN<serial>.conf that a retry or a later
    start would trust."""
    attempts = []

    def urlretrieve(url, path):
        attempts.append(path)
        with open(path, 'w') as f:
            f.write('[LEFT_CAM_VGA]\nfx=35')
        raise OSError('connection reset')

    monkeypatch.setattr(pub.urllib.request, 'urlretrieve', urlretrieve)
    sleeps.limit = 2
    with pytest.raises(Retried):
        make_node([side_by_side(*stereo_pair())], calibration=False)
    assert len(attempts) == 3 and sleeps == [5.0, 5.0, 5.0]
    assert list((tmp_path / '.local/share/zed/settings').iterdir()) == []
    assert all('connection reset' in m for m in log.messages('error'))


@pytest.mark.parametrize('body', ['<html><body>Not found</body></html>', '[LEFT_CAM_VGA]\nfx=350\n'],
                         ids=['error-page', 'no-stereo-section'])
def test_download_that_is_not_a_calibration_file_is_rejected(make_node, monkeypatch, tmp_path, log, sleeps,
                                                              body):
    def urlretrieve(url, path):
        with open(path, 'w') as f:
            f.write(body)

    monkeypatch.setattr(pub.urllib.request, 'urlretrieve', urlretrieve)
    sleeps.limit = 1
    with pytest.raises(Retried):
        make_node([side_by_side(*stereo_pair())], calibration=False)
    assert list((tmp_path / '.local/share/zed/settings').iterdir()) == []
    assert any(f'SN {SERIAL} is not a ZED calibration file' in m for m in log.messages('error'))


def test_calibration_download_is_retried_until_it_succeeds(make_node, monkeypatch, tmp_path, sleeps):
    served = iter(['<html>Service unavailable</html>', None])

    def urlretrieve(url, path):
        body = next(served)
        if body is None:
            write_conf(tmp_path / 'served.conf', distortion=False, rotation=False).replace(path)
        else:
            with open(path, 'w') as f:
                f.write(body)

    monkeypatch.setattr(pub.urllib.request, 'urlretrieve', urlretrieve)
    node, _ = make_node([side_by_side(*stereo_pair())], calibration=False)
    assert sleeps == [5.0] and node.fx_baseline > 0
    assert [p.name for p in (tmp_path / '.local/share/zed/settings').iterdir()] == [f'SN{SERIAL}.conf']


def test_warns_when_the_camera_gives_a_lower_resolution_than_asked(make_node, log):
    _, opened = make_node([side_by_side(*stereo_pair())], params={'resolution': 'HD720'})
    assert opened[0][2].props[cv2.CAP_PROP_FRAME_WIDTH] == 2560
    assert any('USB 2' in m for m in log.messages('warn'))


def test_unreadable_camera_is_retried_at_start(make_node, log, sleeps):
    """A camera that gives no frame is closed and opened again, instead of stopping the node."""
    sleeps.limit = 3
    with pytest.raises(Retried):
        make_node([None])
    assert any('/dev/video7 is not delivering video yet (no frame)' in m for m in log.messages('warn'))


def test_waits_for_the_camera_to_appear(make_node, log, sleeps):
    make_node([side_by_side(*stereo_pair())],
              devices=[(None, None), ('/dev/video7', None), ('/dev/video7', SERIAL)])
    assert log.messages('warn') == ['Waiting for a ZED camera on USB...']
    assert sleeps == [1.0, 1.0, 2.0]  # polls every second, then lets the camera finish starting


def test_device_parameter_overrides_the_detected_device(make_node):
    _, opened = make_node([side_by_side(*stereo_pair())], params={'device': '/dev/video42'})
    assert opened[0][0] == '/dev/video42'


def test_flip_parameter_turns_and_swaps_the_eyes(make_node, log):
    """An upside-down camera sends each eye turned 180 degrees, in the other half of the frame.
    With flip:=true the node undoes that, and the rectification check is happy.

    (Without flip, an ideal camera like this one would pass the check too: turning both images and
    swapping them keeps disparity positive. On a real ZED the factory calibration, applied to the
    wrong eye, is what makes the check warn.)"""
    left, right = stereo_pair()
    upside_down = side_by_side(cv2.rotate(right, cv2.ROTATE_180), cv2.rotate(left, cv2.ROTATE_180))
    node, _ = make_node([upside_down], params={'flip': True})
    assert node.flip is True
    assert not log.messages('warn', 'warning'), log.records


# ---- split / check_alignment ----

def test_split_returns_the_two_eyes():
    frame = side_by_side(*stereo_pair())
    left, right = bare_node().split(frame)
    np.testing.assert_array_equal(left, frame[:, :W])
    np.testing.assert_array_equal(right, frame[:, W:])


def test_split_with_flip_turns_each_eye_and_swaps_them():
    frame = side_by_side(*stereo_pair())
    left, right = bare_node(flip=True).split(frame)
    np.testing.assert_array_equal(left, cv2.rotate(frame[:, W:], cv2.ROTATE_180))
    np.testing.assert_array_equal(right, cv2.rotate(frame[:, :W], cv2.ROTATE_180))


@pytest.mark.parametrize('pair, warns', [
    (dict(), False),           # rows match, left x > right x
    (dict(dy=5), True),        # rows off by 5 px: bad rectification
    (dict(swap=True), True),   # left x < right x: left and right eyes swapped
], ids=['aligned', 'rows-off', 'swapped'])
def test_rectification_check(log, pair, warns):
    bare_node().check_alignment(side_by_side(*stereo_pair(**pair)))
    warned = log.messages('warn')
    assert bool(warned) == warns, log.records
    assert len(log.records) == 1 and 'matches, median |dy|' in log.records[0][1]


def test_rectification_check_is_silent_without_features(log):
    bare_node().check_alignment(np.full((H, 2 * W, 3), 128, np.uint8))
    assert log.records == []


# ---- Messages ----

def test_camera_info_describes_the_rectified_image():
    node = bare_node()
    stamp = Time(sec=12, nanosec=34)
    m = node.info(stamp)
    assert (m.header.stamp, m.header.frame_id) == (stamp, 'zed_left_camera_frame_optical')
    assert (m.width, m.height, m.distortion_model) == (W, H, 'plumb_bob')
    assert list(m.d) == [0.0] * 5  # already rectified
    np.testing.assert_array_equal(np.reshape(m.k, (3, 3)), node.P[:, :3])
    np.testing.assert_array_equal(np.reshape(m.r, (3, 3)), np.eye(3))
    np.testing.assert_array_equal(np.reshape(m.p, (3, 4)), node.P)


@pytest.mark.parametrize('array, encoding, step', [
    (np.zeros((H, W, 3), np.uint8), 'bgr8', W * 3),
    (np.zeros((H, W), np.float32), '32FC1', W * 4),
])
def test_image_message_layout(array, encoding, step):
    m = bare_node().image(Time(sec=1), array, encoding)
    assert (m.height, m.width, m.encoding, m.step) == (H, W, encoding, step)
    assert len(m.data) == step * H and m.header.frame_id == 'zed_left_camera_frame_optical'


# ---- Main loop and reconnecting ----

@pytest.fixture
def looping_node(monkeypatch, log):
    """A bare node set up for spin(): spin_node(reads) runs one loop pass per queued read."""
    node = bare_node(min_depth=0.3, max_depth=10.0, fx_baseline=350.0 * 0.12,
                     sgbm=cv2.StereoSGBM_create(minDisparity=0, numDisparities=160, blockSize=5))
    node.pub_rgb, node.pub_rgb_info, node.pub_depth, node.pub_depth_info = (FakePublisher() for _ in range(4))
    node.reopens = 0

    def reopen():
        node.reopens += 1
    node.reopen = reopen
    monkeypatch.setattr(pub.ZedUsb, 'get_clock', lambda self: type('Clock', (), {
        'now': lambda _: type('Now', (), {'to_msg': lambda _: Time(sec=5, nanosec=6)})()})())
    monkeypatch.setattr(pub.rclpy, 'spin_once', lambda *a, **k: None)

    def spin_node(reads):
        node.cap = FakeCapture(reads)
        ok = iter([True] * len(reads) + [False])
        monkeypatch.setattr(pub.rclpy, 'ok', lambda: next(ok))
        node.spin()
        return node
    return spin_node


def test_spin_publishes_color_depth_and_both_camera_infos(looping_node):
    node = looping_node([side_by_side(*stereo_pair(shift=20))])
    rgb, rgb_info, depth, depth_info = (p.msgs for p in (node.pub_rgb, node.pub_rgb_info,
                                                         node.pub_depth, node.pub_depth_info))
    assert len(rgb) == len(rgb_info) == len(depth) == len(depth_info) == 1
    # One timestamp for all four: RTAB-Map syncs color and depth exactly (approx_sync false).
    assert {m.header.stamp.sec for m in rgb + rgb_info + depth + depth_info} == {5}
    assert (rgb[0].encoding, depth[0].encoding) == ('bgr8', '32FC1')
    d = np.frombuffer(bytes(depth[0].data), np.float32).reshape(H, W)
    valid = d[np.isfinite(d)]
    assert valid.size > 0.5 * d.size and ((valid >= 0.3) & (valid <= 10.0)).all()
    # The synthetic scene is 20 px of disparity away: depth = 42 / 20 = 2.1 m.
    assert np.median(valid) == pytest.approx(2.1, rel=0.05)


def test_spin_reconnects_after_failed_reads_and_warns_once_per_outage(looping_node, log):
    frame = side_by_side(*stereo_pair())
    node = looping_node([None, None, frame, None, frame])
    assert node.reopens == 3
    assert len(log.messages('warn')) == 2  # one warning per outage, not per failed read
    assert len(node.pub_rgb.msgs) == 2


def test_reopen_finds_the_camera_under_a_new_device(monkeypatch, log, sleeps):
    old = FakeCapture()
    node = bare_node(cap=old, fixed_device='')
    monkeypatch.setattr(pub, 'find_zed', lambda: ('/dev/video9', SERIAL))
    new = FakeCapture([side_by_side(*stereo_pair())])
    monkeypatch.setattr(node, 'open', lambda device: new)
    pub.ZedUsb.reopen(node)
    assert old.released and node.cap is new and not new.released
    assert log.messages('info') == ['Camera back on /dev/video9'] and sleeps == []


def test_reopen_rejects_a_camera_with_another_frame_size(monkeypatch, log, sleeps):
    node = bare_node(cap=FakeCapture(), fixed_device='')
    monkeypatch.setattr(pub, 'find_zed', lambda: ('/dev/video9', SERIAL))
    wrong = FakeCapture([np.zeros((720, 2560, 3), np.uint8)])
    monkeypatch.setattr(node, 'open', lambda device: wrong)
    pub.ZedUsb.reopen(node)
    assert wrong.released and sleeps == [1.0] and log.records == []


def test_reopen_waits_while_no_camera_is_present(monkeypatch, sleeps):
    node = bare_node(cap=FakeCapture(), fixed_device='')
    monkeypatch.setattr(pub, 'find_zed', lambda: (None, None))
    monkeypatch.setattr(node, 'open', lambda device: pytest.fail('opened without a device'))
    pub.ZedUsb.reopen(node)
    assert sleeps == [1.0]


def test_reopen_uses_the_fixed_device_without_searching(monkeypatch, log):
    node = bare_node(cap=FakeCapture(), fixed_device='/dev/video42')
    monkeypatch.setattr(pub, 'find_zed', lambda: pytest.fail('searched for the camera'))
    monkeypatch.setattr(node, 'open', lambda device: FakeCapture([side_by_side(*stereo_pair())]))
    pub.ZedUsb.reopen(node)
    assert log.messages('info') == ['Camera back on /dev/video42']
