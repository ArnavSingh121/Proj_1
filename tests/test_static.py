"""Tier 1: static and cross-file consistency checks.

Most of this repo is configuration that only works if several files agree with each other: the
simulated camera (SDF) and RTAB-Map's static TF, the 0.3 m self-filter in three launch files, the
ZED topic names in the USB publisher and in the launch files, the ZED wrapper overrides. These tests
read the files and check that they still agree. Nothing is run except `gz sdf` and `shellcheck`
(each skipped if not installed).
"""
import math
import os
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from conftest import (PX4_MODELS, REPO, UNEVALUATED, assigned_values, declared_parameters, launch_arguments,
                      list_items, module_constants, run, static_tf)

MODELS = REPO / 'sim/models'
SIM_LAUNCH = REPO / 'sim/rtabmap.launch.py'
JETSON_LAUNCH = REPO / 'jetson/zed_rtabmap.launch.py'
USB_LAUNCH = REPO / 'zed_usb/zed_usb_rtabmap.launch.py'
PUBLISHER = REPO / 'zed_usb/zed_usb_publisher.py'
LAUNCH_FILES = [SIM_LAUNCH, JETSON_LAUNCH, USB_LAUNCH]
SHELL_SCRIPTS = sorted(REPO.glob('*/*.sh'))
ZED_PARAMS = REPO / 'jetson/zed_params.yaml'
README = (REPO / 'README.md').read_text()

# The near-range cut-off that keeps the drone's own props and legs out of the map (README).
SELF_FILTER_M = 0.3

# URIs that are not in this repo on purpose: they come from PX4's own models, and the custom
# models are copied on top of those (sim/install_models.sh).
EXTERNAL_URIS = {
    'x500',                                     # base airframe of x500_depth
    'model://OakD-Lite/meshes/OakDLite.dae',    # OakD-Lite mesh; only model.sdf is overridden here
}


def sdf(model):
    return ET.parse(MODELS / model / 'model.sdf').getroot()


def sensor(model, name):
    found = sdf(model).find(f".//sensor[@name='{name}']")
    assert found is not None, f'no sensor {name} in {model}'
    return found


def pose(text):
    return np.array([float(v) for v in text.split()])


def rpy_matrix(roll, pitch, yaw):
    """Rotation of tf2's static_transform_publisher for --roll/--pitch/--yaw (fixed axes: Rz Ry Rx)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def filter_values(flags):
    """{'Grid/RangeMin': 0.3, ...} from an RTAB-Map argument string like '--Grid/RangeMin 0.3 ...'."""
    return {k: float(v) for k, v in re.findall(r'--(\S+) ([0-9.]+)', flags)}


# ---- Gazebo models (sim/models) ----

@pytest.mark.parametrize('path', sorted(p for p in MODELS.glob('*/*') if p.is_file()),
                         ids=lambda p: str(p.relative_to(MODELS)))
def test_model_files_are_well_formed_xml(path):
    ET.parse(path)


@pytest.mark.parametrize('config', sorted(MODELS.glob('*/model.config')), ids=lambda p: p.parent.name)
def test_model_config_points_to_an_existing_sdf(config):
    target = ET.parse(config).getroot().find('sdf').text.strip()
    assert (config.parent / target).is_file()


@pytest.mark.skipif(not shutil.which('gz'), reason='gz (Gazebo) not installed')
@pytest.mark.parametrize('model', ['OakD-Lite', 'lidar_3d'])
def test_standalone_models_pass_gz_sdf_check(model):
    result = run(['gz', 'sdf', '-k', str(MODELS / model / 'model.sdf')])
    assert result.returncode == 0 and 'Valid.' in result.stdout, result.stdout + result.stderr


@pytest.mark.skipif(not shutil.which('gz'), reason='gz (Gazebo) not installed')
@pytest.mark.skipif(not PX4_MODELS.is_dir(), reason=f'no PX4 models in {PX4_MODELS}')
def test_x500_depth_passes_gz_sdf_check_once_installed_over_px4(tmp_path):
    """x500_depth includes PX4's x500 and our OakD-Lite/lidar_3d: check it as install_models.sh lays it out.

    Our OakD-Lite has no model.config of its own (it reuses PX4's), so the check runs on a copy of
    our models with PX4's model.config files filled in, with PX4's models as the fallback search path.
    """
    overlay = tmp_path / 'models'
    shutil.copytree(MODELS, overlay)
    for model in overlay.iterdir():
        stock = PX4_MODELS / model.name / 'model.config'
        if not (model / 'model.config').exists() and stock.exists():
            shutil.copy(stock, model / 'model.config')
    env = {'SDF_PATH': f'{overlay}:{PX4_MODELS}', 'PATH': '/usr/bin:/bin'}
    result = run(['gz', 'sdf', '-k', str(overlay / 'x500_depth/model.sdf')], env=env)
    assert result.returncode == 0 and 'Valid.' in result.stdout, result.stdout + result.stderr


def test_every_model_uri_is_in_the_repo_or_a_known_px4_asset():
    for path in MODELS.glob('*/model.sdf'):
        for uri in (e.text.strip() for e in ET.parse(path).getroot().iter('uri')):
            if uri in EXTERNAL_URIS:
                continue
            if not uri.startswith('model://'):  # relative to the model folder (depot: meshes/Depot.dae)
                assert (path.parent / uri).exists(), f'{path}: {uri} is not in its model folder'
                continue
            assert (MODELS / uri[len('model://'):]).exists(), f'{path}: {uri} is not in sim/models'


@pytest.mark.skipif(not PX4_MODELS.is_dir(), reason=f'no PX4 models in {PX4_MODELS}')
def test_external_uris_exist_in_px4():
    assert (PX4_MODELS / 'x500/model.sdf').is_file()
    assert (PX4_MODELS / 'OakD-Lite/meshes/OakDLite.dae').is_file()


def test_every_world_include_is_in_sim_models():
    """Gazebo hangs at "Waiting for Gazebo world" when an included model:// cannot be found."""
    for path in (REPO / 'sim/worlds').glob('*.sdf'):
        for uri in (e.text.strip() for e in ET.parse(path).getroot().iter('uri')):
            assert uri.startswith('model://'), f'{path}: unexpected uri {uri}'
            assert (MODELS / uri[len('model://'):] / 'model.sdf').is_file(), f'{path}: {uri} is not in sim/models'


def test_x500_depth_joints_attach_links_of_the_included_models():
    links = {link.get('name') for m in ('OakD-Lite', 'lidar_3d') for link in sdf(m).iter('link')}
    joints = {j.get('name'): (j.find('parent').text, j.find('child').text) for j in sdf('x500_depth').iter('joint')}
    assert joints == {'CameraJoint': ('base_link', 'camera_link'), 'LidarJoint': ('base_link', 'lidar_link')}
    # base_link comes from PX4's x500; the children must be links of our included models.
    assert {child for _, child in joints.values()} <= links


def test_x500_depth_publishes_3d_ground_truth_odometry():
    plugin = sdf('x500_depth').find(".//plugin[@name='gz::sim::systems::OdometryPublisher']")
    assert plugin is not None and plugin.find('dimensions').text == '3'


# ---- Simulated camera geometry ----

def test_rgb_and_depth_sensors_are_pixel_registered():
    """RTAB-Map gets RGB and depth without registration (sim/rtabmap.launch.py), so both sensors
    must share pose, FOV, resolution, rate and frame."""
    rgb, depth = sensor('OakD-Lite', 'IMX214'), sensor('OakD-Lite', 'StereoOV7251')
    for tag in ('pose', 'camera/horizontal_fov', 'camera/image/width', 'camera/image/height',
                'update_rate', 'gz_frame_id'):
        assert rgb.find(tag).text.strip() == depth.find(tag).text.strip(), tag
    assert rgb.find('gz_frame_id').text == 'camera_optical_frame'
    assert depth.get('type') == 'depth_camera' and depth.find('topic').text == 'depth_camera'


def test_simulated_intrinsics_match_the_readme():
    cam = sensor('OakD-Lite', 'StereoOV7251').find('camera')
    w, h = int(cam.find('image/width').text), int(cam.find('image/height').text)
    fx = (w / 2) / math.tan(float(cam.find('horizontal_fov').text) / 2)
    assert (w, h) == (640, 480)
    assert re.search(r'fx = fy = 432\.5, cx = 320, cy = 240', README)
    assert fx == pytest.approx(432.5, abs=0.05)


def test_sim_static_tf_position_is_the_camera_mount_in_the_sdf():
    """CAMERA_XYZ = OakD-Lite include pose in x500_depth + sensor pose in OakD-Lite."""
    include = next(i for i in sdf('x500_depth').iter('include') if i.find('uri').text == 'model://OakD-Lite')
    expected = pose(include.find('pose').text)[:3] + pose(sensor('OakD-Lite', 'IMX214').find('pose').text)[:3]
    tf = static_tf(SIM_LAUNCH, child='camera_optical_frame')  # the frame the bridged images carry
    assert [float(tf[k]) for k in ('--x', '--y', '--z')] == pytest.approx(expected.tolist(), abs=1e-9)
    assert tf['--frame-id'] == 'base_link'


def test_sim_lidar_tf_is_the_lidar_mount_in_the_sdf():
    """LIDAR_XYZ = lidar_3d include pose in x500_depth + gpu_lidar sensor pose in lidar_3d: the points are
    stamped 'lidar_link' (gz_frame_id) but measured from the sensor."""
    include = next(i for i in sdf('x500_depth').iter('include') if i.find('uri').text == 'model://lidar_3d')
    lidar = sensor('lidar_3d', 'gpu_lidar')
    expected = pose(include.find('pose').text)[:3] + pose(lidar.find('pose').text)[:3]
    tf = static_tf(SIM_LAUNCH, child=lidar.find('gz_frame_id').text)
    assert [float(tf[k]) for k in ('--x', '--y', '--z')] == pytest.approx(expected.tolist(), abs=1e-9)
    assert tf['--frame-id'] == 'base_link'
    assert all(float(tf.get(k, 0)) == 0 for k in ('--roll', '--pitch', '--yaw'))


def test_lidar_has_a_single_topic():
    """Two <topic> tags made the lidar's topic ambiguous; sim/bridge.sh relies on 'lidar_points'."""
    assert [t.text for t in sensor('lidar_3d', 'gpu_lidar').findall('topic')] == ['lidar_points']


@pytest.mark.parametrize('launch, parent, child', [
    (SIM_LAUNCH, 'base_link', 'camera_optical_frame'),
    (USB_LAUNCH, 'zed_camera_link', 'zed_left_camera_frame_optical'),
])
def test_static_tf_turns_body_axes_into_optical_axes(launch, parent, child):
    """Optical convention (REP 103): z forward, x right, y down, in a body frame x forward, y left, z up."""
    tf = static_tf(launch, child=child)
    assert tf['--frame-id'] == parent
    r = rpy_matrix(*(float(tf.get(k, 0)) for k in ('--roll', '--pitch', '--yaw')))
    np.testing.assert_allclose(r @ [0, 0, 1], [1, 0, 0], atol=1e-4)   # optical z = forward
    np.testing.assert_allclose(r @ [1, 0, 0], [0, -1, 0], atol=1e-4)  # optical x = right
    np.testing.assert_allclose(r @ [0, 1, 0], [0, 0, -1], atol=1e-4)  # optical y = down


def test_usb_static_tf_puts_the_left_lens_6cm_left_of_center():
    tf = static_tf(USB_LAUNCH)
    assert float(tf['--y']) == pytest.approx(0.06)
    assert float(tf.get('--x', 0)) == 0 and float(tf.get('--z', 0)) == 0


# ---- Values that must agree across the sim, Jetson and USB setups ----

def test_self_filter_is_identical_in_every_launch_file():
    filters = {p.name: (module_constants(p)['ODOM_SELF_FILTER'], module_constants(p)['SELF_FILTER'])
               for p in LAUNCH_FILES}
    assert len(set(filters.values())) == 1, filters
    odom, full = next(iter(filters.values()))
    assert filter_values(odom) == {'Vis/MinDepth': SELF_FILTER_M}
    assert filter_values(full) == {'Grid/RangeMin': SELF_FILTER_M, 'Vis/MinDepth': SELF_FILTER_M}


@pytest.mark.parametrize('launch', LAUNCH_FILES, ids=lambda p: p.name)
def test_rtabmap_receives_the_self_filter(launch):
    args = launch_arguments(launch, 'rgb_topic')
    consts = module_constants(launch)
    assert consts['ODOM_SELF_FILTER'] in args['odom_args']
    # rtabmap_args is a substitution list in the Jetson/USB files; the filter is its literal part.
    # The sim file builds it at launch time from its options, starting from `rtabmap_args = SELF_FILTER`.
    rtabmap_args = args['rtabmap_args']
    parts = rtabmap_args if isinstance(rtabmap_args, list) else [rtabmap_args]
    if rtabmap_args is UNEVALUATED:
        parts = assigned_values(launch, 'rtabmap_args')
    assert any(isinstance(p, str) and consts['SELF_FILTER'] in p for p in parts)


def test_zed_min_depth_matches_the_self_filter():
    params = yaml.safe_load(ZED_PARAMS.read_text())['/**']['ros__parameters']
    assert params['depth']['min_depth'] == SELF_FILTER_M
    assert declared_parameters(PUBLISHER)['min_depth'] == SELF_FILTER_M


def test_zed_topic_root_is_the_same_in_jetson_and_usb_launch_files():
    assert module_constants(JETSON_LAUNCH)['ZED'] == module_constants(USB_LAUNCH)['ZED'] == '/zed/zed_node/'


@pytest.mark.parametrize('launch', [JETSON_LAUNCH, USB_LAUNCH], ids=lambda p: p.name)
def test_rtabmap_inputs_are_topics_the_zed_publishes(launch):
    """The USB publisher stands in for the ZED wrapper: RTAB-Map must find its topics in both setups.

    The publisher publishes under TOPIC_ROOT, the wrapper's topic root, from its own node 'zed_usb'.
    """
    published = set(re.findall(r"create_publisher\(\w+, TOPIC_ROOT \+ '([^']+)'", PUBLISHER.read_text()))
    root = module_constants(launch)['ZED']
    assert module_constants(PUBLISHER)['TOPIC_ROOT'] == root
    args = launch_arguments(launch, 'rgb_topic')
    for key in ('rgb_topic', 'depth_topic', 'camera_info_topic'):
        assert args[key].startswith(root)
        assert args[key][len(root):] in published, key


def test_usb_publisher_keeps_its_own_node_name():
    """Named 'zed_usb', not the wrapper's 'zed_node', so jetson/verify.py can tell which one makes the depth."""
    cmd = next(items for items in list_items(USB_LAUNCH, 'cmd') if 'python3' in items)
    assert not any('__node:=' in str(item) or '__ns:=' in str(item) for item in cmd)
    assert "super().__init__('zed_usb')" in PUBLISHER.read_text()
    assert "additional_env={'PYTHONNOUSERSITE': '1'}" in USB_LAUNCH.read_text()


def test_publisher_frame_is_the_usb_static_tf_child_frame():
    frame = re.search(r"self\.frame_id = '([^']+)'", PUBLISHER.read_text()).group(1)
    assert frame == static_tf(USB_LAUNCH)['--child-frame-id']


def test_rtabmap_frame_and_sync_settings():
    sim, jetson, usb = (launch_arguments(p, 'rgb_topic') for p in LAUNCH_FILES)
    assert (sim['frame_id'], sim['approx_sync'], sim['use_sim_time']) == ('base_link', 'true', 'true')
    # The ZED (real or USB) stamps color and depth identically: exact sync, reliable QoS, wall time.
    for args in (jetson, usb):
        assert (args['frame_id'], args['approx_sync'], args['qos']) == ('zed_camera_link', 'false', '1')
    assert jetson['use_sim_time'] == 'false'
    assert usb['use_sim_time'] is UNEVALUATED  # launch argument: wall time unless replaying a bag
    assert sim['rgb_topic'] == '/uav/rgb/image' and sim['depth_topic'] == '/uav/depth/image'
    assert usb['visual_odometry'] == 'true'  # no IMU over plain USB
    assert jetson['visual_odometry'] is UNEVALUATED  # chosen at launch time by use_zed_odometry


# ---- jetson/zed_params.yaml ----

def zed_params():
    return yaml.safe_load(ZED_PARAMS.read_text())['/**']['ros__parameters']


def test_zed_publish_rate_does_not_exceed_grab_rate():
    general = zed_params()['general']
    assert 0 < general['pub_frame_rate'] <= general['grab_frame_rate']


def test_zed_publishes_only_what_rtabmap_uses():
    p = zed_params()
    on = [p['video']['publish_rgb'], p['sensors']['publish_imu'], p['depth']['publish_depth_map'],
          p['pos_tracking']['pos_tracking_enabled']]
    off = [p['video'][k] for k in ('publish_left_right', 'publish_raw', 'publish_gray', 'publish_stereo')]
    off += [p['depth'][k] for k in ('publish_point_cloud', 'publish_depth_confidence', 'publish_disparity')]
    off += [p['mapping']['mapping_enabled'], p['object_detection']['od_enabled'],
            p['body_tracking']['bt_enabled'], p['stream_server']['stream_enabled']]
    assert all(v is True for v in on) and all(v is False for v in off)
    assert p['depth']['depth_mode'] == 'NEURAL_LIGHT'  # SDK 5 has only neural depth modes


def _keys(tree, prefix=''):
    for k, v in tree.items():
        if isinstance(v, dict):
            yield from _keys(v, f'{prefix}{k}.')
        else:
            yield prefix + k


def _wrapper_config():
    ws = Path(os.environ.get('ZED_WS', Path.home() / 'zed_ws'))
    candidates = [ws / 'src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml',
                  Path('/opt/ros/humble/share/zed_wrapper/config/common_stereo.yaml')]
    return next((c for c in candidates if c.exists()), None)


@pytest.mark.skipif(_wrapper_config() is None, reason='zed_wrapper (common_stereo.yaml) not installed')
def test_zed_params_keys_exist_in_the_wrapper():
    """The wrapper silently ignores unknown keys, so a typo here would just not apply."""
    wrapper = yaml.safe_load(_wrapper_config().read_text())['/**']['ros__parameters']
    assert set(_keys(zed_params())) <= set(_keys(wrapper))


# ---- Publisher constants ----

def test_publisher_resolution_tables_agree():
    """Each requested side-by-side size has a matching per-eye width in RES_NAMES."""
    consts = module_constants(PUBLISHER)
    for name, (w, h) in consts['REQUEST'].items():
        assert w // 2 in consts['RES_NAMES'], name
    assert consts['REQUEST']['VGA'] == (1344, 376)
    assert declared_parameters(PUBLISHER)['resolution'] in consts['REQUEST']


# ---- Shell scripts ----

@pytest.mark.parametrize('script', SHELL_SCRIPTS, ids=lambda p: str(p.relative_to(REPO)))
def test_shell_script_syntax(script):
    result = run(['bash', '-n', str(script)])
    assert result.returncode == 0, result.stderr


@pytest.mark.skipif(not shutil.which('shellcheck'), reason='shellcheck not installed')
@pytest.mark.parametrize('script', SHELL_SCRIPTS, ids=lambda p: str(p.relative_to(REPO)))
def test_shellcheck(script):
    # SC1090/SC1091: sourcing ROS setup files and overlays chosen at run time is intended.
    result = run(['shellcheck', '-S', 'warning', '-e', 'SC1090,SC1091', str(script)])
    assert result.returncode == 0, result.stdout


def test_entry_points_are_executable_in_git():
    """The Makefile and README run these as ./path, which needs the executable bit committed."""
    # `git ls-files -s` lines: <mode> <object> <stage>\t<path>
    modes = {line.split('\t')[1]: line.split()[0] for line in run(['git', 'ls-files', '-s']).stdout.splitlines()}
    entry_points = [str(p.relative_to(REPO)) for p in SHELL_SCRIPTS] + ['zed_usb/zed_usb_publisher.py']
    assert {p: modes.get(p) for p in entry_points} == {p: '100755' for p in entry_points}
