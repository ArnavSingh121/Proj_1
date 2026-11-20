# Camera failures do not crash the ZED USB publisher (issue #12): a missing camera, failed or garbled
# reads, a USB disconnect mid-run, and an unreachable or corrupt calibration download are all waited out
# or skipped. Runs against a scripted fake camera and a stand-in for rclpy, so no hardware or ROS is needed.
import importlib
import sys
import types
import urllib.error

import numpy as np
import pytest

import ros_stub

W, H = 672, 376  # VGA, per eye
CALIBRATION = """
[LEFT_CAM_VGA]
fx = 350.0
fy = 350.0
cx = 336.0
cy = 188.0
k1 = 0.0
k2 = 0.0
p1 = 0.0
p2 = 0.0
k3 = 0.0

[RIGHT_CAM_VGA]
fx = 350.0
fy = 350.0
cx = 336.0
cy = 188.0
k1 = 0.0
k2 = 0.0
p1 = 0.0
p2 = 0.0
k3 = 0.0

[STEREO]
Baseline = 120.0
TY = 0.0
TZ = 0.0
RX_VGA = 0.0
CV_VGA = 0.0
RZ_VGA = 0.0
"""


def stereo_frame(width=W, height=H):
    rng = np.random.default_rng(0)
    return rng.integers(0, 255, (height, width * 2, 3), dtype=np.uint8)


class FakeCapture:
    """Plays back a script of read() results: a frame, None (read failed) or an exception to raise."""

    def __init__(self, script):
        self.script = script
        self.released = False

    def set(self, *_):
        return True

    def read(self):
        item = self.script.pop(0) if self.script else None
        if isinstance(item, Exception):
            raise item
        return (item is not None), item

    def release(self):
        self.released = True


class Camera:
    """Every VideoCapture() opened gets the next script; the last one repeats."""

    def __init__(self, *scripts):
        self.scripts = list(scripts)
        self.opened = []

    def __call__(self, device, _backend=None):
        script = self.scripts.pop(0) if len(self.scripts) > 1 else list(self.scripts[0])
        cap = FakeCapture(script)
        self.opened.append((device, cap))
        return cap


@pytest.fixture
def zed(monkeypatch, tmp_path):
    """The zed_usb_publisher module, imported against the ROS stand-in, with sleeps skipped."""
    for name, module in ros_stub.modules().items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop('zed_usb_publisher', None)
    mod = importlib.import_module('zed_usb_publisher')
    monkeypatch.setattr(mod.time, 'sleep', lambda s: None)
    monkeypatch.setattr(ros_stub.Node, 'parameter_overrides', {'calib_dir': str(tmp_path / 'calib')})
    yield mod
    sys.modules.pop('zed_usb_publisher', None)


def write_calibration(tmp_path, serial='12345'):
    d = tmp_path / 'calib'
    d.mkdir(exist_ok=True)
    (d / f'SN{serial}.conf').write_text(CALIBRATION)


def make_node(zed, monkeypatch, camera, found=None):
    """ZedUsb with find_zed() answering from `found` (a list, last answer repeats) and `camera` as the device."""
    found = list(found or [('/dev/video0', '12345')])
    monkeypatch.setattr(zed, 'find_zed', lambda: found.pop(0) if len(found) > 1 else found[0])
    monkeypatch.setattr(zed.cv2, 'VideoCapture', camera)
    return zed.ZedUsb()


# ---- startup ----

def test_starts_normally(zed, monkeypatch, tmp_path):
    write_calibration(tmp_path)
    node = make_node(zed, monkeypatch, Camera([stereo_frame()] * 3))
    assert (node.w, node.h) == (W, H)


def test_waits_for_a_camera_that_is_not_plugged_in(zed, monkeypatch, tmp_path):
    write_calibration(tmp_path)
    found = [(None, None), (None, None), ('/dev/video2', '12345')]
    node = make_node(zed, monkeypatch, Camera([stereo_frame()]), found)
    assert 'Waiting for a ZED' in node.get_logger().text('warn')


def test_retries_when_the_first_read_fails(zed, monkeypatch, tmp_path):
    # Used to raise RuntimeError('Cannot read from ...') and exit.
    write_calibration(tmp_path)
    camera = Camera([None], [zed.cv2.error('VIDIOC_STREAMON')], [stereo_frame()] * 3)
    node = make_node(zed, monkeypatch, camera)
    assert len(camera.opened) == 3
    assert camera.opened[0][1].released
    assert 'not delivering video' in node.get_logger().text('warn')


def test_retries_on_a_frame_of_unknown_resolution(zed, monkeypatch, tmp_path):
    # Used to raise KeyError in RES_NAMES[...].
    write_calibration(tmp_path)
    camera = Camera([stereo_frame(width=500)], [stereo_frame()] * 3)
    node = make_node(zed, monkeypatch, camera)
    assert node.w == W
    assert 'unexpected frame size' in node.get_logger().text('warn')


def test_unknown_resolution_parameter_falls_back_to_vga(zed, monkeypatch, tmp_path):
    write_calibration(tmp_path)
    monkeypatch.setattr(ros_stub.Node, 'parameter_overrides',
                        {'calib_dir': str(tmp_path / 'calib'), 'resolution': '4K'})
    node = make_node(zed, monkeypatch, Camera([stereo_frame()] * 3))
    assert 'Unknown resolution' in node.get_logger().text('warn')


def test_calibration_download_retries_while_offline(zed, monkeypatch, tmp_path):
    # Used to crash with URLError on a laptop without network.
    attempts = []

    def urlretrieve(url, path):
        attempts.append(url)
        if len(attempts) < 3:
            raise urllib.error.URLError('no network')
        with open(path, 'w') as f:
            f.write(CALIBRATION)

    monkeypatch.setattr(zed.urllib.request, 'urlretrieve', urlretrieve)
    node = make_node(zed, monkeypatch, Camera([stereo_frame()] * 3))
    assert len(attempts) == 3 and attempts[0].endswith('SN=12345')
    assert 'No usable calibration' in node.get_logger().text('error')
    assert (tmp_path / 'calib' / 'SN12345.conf').exists()
    assert not (tmp_path / 'calib' / 'SN12345.conf.part').exists()


def test_corrupt_calibration_file_is_downloaded_again(zed, monkeypatch, tmp_path):
    (tmp_path / 'calib').mkdir()
    (tmp_path / 'calib' / 'SN12345.conf').write_text('<html>Service unavailable</html>')

    def urlretrieve(url, path):
        with open(path, 'w') as f:
            f.write(CALIBRATION)

    monkeypatch.setattr(zed.urllib.request, 'urlretrieve', urlretrieve)
    node = make_node(zed, monkeypatch, Camera([stereo_frame()] * 3))
    assert node.fx_baseline > 0
    assert (tmp_path / 'calib' / 'SN12345.conf.bad').exists()


# ---- while running ----

def published(node):
    return {topic: len(pub.sent) for topic, pub in node.publishers_by_topic.items()}


def test_good_frame_publishes_rgb_depth_and_camera_info(zed, monkeypatch, tmp_path):
    write_calibration(tmp_path)
    node = make_node(zed, monkeypatch, Camera([stereo_frame()] * 3))
    assert node.step()
    assert set(published(node).values()) == {1}
    depth = node.publishers_by_topic['/zed/zed_node/depth/depth_registered'].sent[0]
    assert depth.encoding == '32FC1' and (depth.height, depth.width) == (H, W)


def test_usb_disconnect_mid_run_reconnects(zed, monkeypatch, tmp_path):
    write_calibration(tmp_path)
    # first device: startup frame, one good frame, then the cable is pulled
    camera = Camera([stereo_frame(), stereo_frame(), None], [], [stereo_frame()] * 5)
    node = make_node(zed, monkeypatch, camera)
    assert node.step()
    assert not node.step()  # read fails -> reopen; first reopen finds nothing
    assert not node.step()  # still nothing -> reopen; second reopen gets the camera back
    assert 'Camera back on' in node.get_logger().text('info')
    assert node.step()
    assert published(node)['/zed/zed_node/rgb/color/rect/image'] == 2
    assert node.get_logger().text('warn').count('Camera read failed') == 1  # reported once, not per frame


def test_unplugged_camera_that_does_not_come_back_never_raises(zed, monkeypatch, tmp_path):
    write_calibration(tmp_path)
    camera = Camera([stereo_frame()], [])
    node = make_node(zed, monkeypatch, camera, [('/dev/video0', '12345'), (None, None)])
    for _ in range(20):
        assert not node.step()
    assert published(node)['/zed/zed_node/rgb/color/rect/image'] == 0


def test_read_that_raises_is_treated_as_a_failed_read(zed, monkeypatch, tmp_path):
    write_calibration(tmp_path)
    camera = Camera([stereo_frame(), zed.cv2.error('select() timeout')], [stereo_frame()] * 3)
    node = make_node(zed, monkeypatch, camera)
    assert not node.step()
    assert node.step()


def test_garbled_frames_are_skipped(zed, monkeypatch, tmp_path):
    write_calibration(tmp_path)
    frames = [stereo_frame(), stereo_frame(width=300), np.zeros((H, W * 2), np.uint8), stereo_frame()]
    node = make_node(zed, monkeypatch, Camera(frames))
    assert not node.step()  # wrong size
    assert not node.step()  # grayscale
    assert node.step()
    assert 'unexpected size' in node.get_logger().text('warn')


def test_depth_has_no_values_outside_the_valid_range(zed, monkeypatch, tmp_path):
    write_calibration(tmp_path)
    node = make_node(zed, monkeypatch, Camera([stereo_frame()] * 3))
    left, right = node.split(stereo_frame())
    depth = node.depth(left, right)
    valid = depth[np.isfinite(depth)]
    assert depth.dtype == np.float32
    assert ((valid >= node.min_depth) & (valid <= node.max_depth)).all()


# ---- camera discovery ----

def test_find_zed_reads_sysfs(zed, tmp_path):
    video = tmp_path / 'class' / 'video4linux' / 'video3'
    video.mkdir(parents=True)
    (video / 'name').write_text('ZED 2i: ZED 2i\n')
    usb = tmp_path / 'bus' / 'usb' / 'devices'
    hid = usb / '1-2.1'
    hid.mkdir(parents=True)
    (hid / 'idVendor').write_text('2b03\n')
    (hid / 'product').write_text('ZED 2i HID INTERFACE\n')
    (hid / 'serial').write_text('38123456\n')
    gone = usb / '1-3'  # a device unplugged during the scan: its files are missing
    gone.mkdir()
    (gone / 'idVendor').write_text('2b03\n')
    assert zed.find_zed(str(tmp_path)) == ('/dev/video3', '38123456')


def test_find_zed_without_a_camera(zed, tmp_path):
    assert zed.find_zed(str(tmp_path)) == (None, None)


class GraphNode:
    """Just enough of a node for other_depth_sources: its name and who publishes the depth topic."""

    def __init__(self, publishers):
        self.publishers = publishers

    def get_name(self):
        return 'zed_usb'

    def get_namespace(self):
        return '/'

    def get_publishers_info_by_topic(self, topic):
        assert topic == '/zed/zed_node/depth/depth_registered'
        return [types.SimpleNamespace(node_namespace=ns, node_name=name) for ns, name in self.publishers]


@pytest.mark.parametrize('publishers, others', [
    ([('/', 'zed_usb')], []),
    ([('/', 'zed_usb'), ('/zed', 'zed_node')], ['/zed/zed_node']),
], ids=['alone', 'with the ZED wrapper'])
def test_a_second_depth_source_is_spotted(zed, publishers, others):
    """The publisher must not run next to the ZED wrapper: two depth sources would be mixed."""
    assert zed.other_depth_sources(GraphNode(publishers)) == others
