"""Tier 3: behaviour of the shell scripts and the Makefile.

Every script runs for real, but the programs it would hand off to (ros2, make, curl, dpkg, sudo,
apt-get, ...) are stubs that only record their arguments (see `Stubs` in conftest.py). So these
tests check what each script *would* launch or install, without ROS nodes, PX4, sudo or network.
"""
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from conftest import REPO, needs_ros_setup, run

README = (REPO / 'README.md').read_text()
MAKEFILE = (REPO / 'Makefile').read_text()


def ros2_call(stubs):
    """The single `ros2 ...` call a launcher script made."""
    calls = stubs.calls('ros2')
    assert len(calls) == 1, calls
    return calls[0]


# ---- sim/install_models.sh ----

def px4_tree(root, stock=True):
    """A fake PX4-Autopilot checkout whose model folder holds PX4's stock files we override."""
    models = root / 'Tools/simulation/gz/models'
    models.mkdir(parents=True)
    if stock:
        for rel in ('x500_depth/model.sdf', 'OakD-Lite/model.sdf', 'OakD-Lite/model.config',
                    'OakD-Lite/meshes/OakDLite.dae'):
            (models / rel).parent.mkdir(parents=True, exist_ok=True)
            (models / rel).write_text(f'stock {rel}\n')
    return models


def repo_model_files():
    return sorted(str(p.relative_to(REPO / 'sim/models')) for p in (REPO / 'sim/models').rglob('*') if p.is_file())


def test_install_models_fails_without_a_px4_model_folder(tmp_path):
    result = run([str(REPO / 'sim/install_models.sh'), str(tmp_path / 'not-px4')])
    assert result.returncode == 1
    assert 'not found' in result.stderr and 'submodule' in result.stderr


def test_install_models_copies_every_model_file_and_backs_up_stock_ones(tmp_path):
    models = px4_tree(tmp_path / 'PX4')
    result = run([str(REPO / 'sim/install_models.sh'), str(tmp_path / 'PX4')])
    assert result.returncode == 0, result.stderr
    assert sorted(re.findall(r'^installed \./(.+)$', result.stdout, re.M)) == repo_model_files()
    for rel in repo_model_files():
        assert (models / rel).read_bytes() == (REPO / 'sim/models' / rel).read_bytes(), rel
    # Stock files we replaced are kept as *.orig; new files (lidar_3d) get no backup.
    assert (models / 'x500_depth/model.sdf.orig').read_text() == 'stock x500_depth/model.sdf\n'
    assert (models / 'OakD-Lite/model.sdf.orig').read_text() == 'stock OakD-Lite/model.sdf\n'
    assert not (models / 'lidar_3d/model.sdf.orig').exists()
    # Files we don't ship (PX4's OakD-Lite mesh and config) are left alone.
    assert (models / 'OakD-Lite/meshes/OakDLite.dae').read_text() == 'stock OakD-Lite/meshes/OakDLite.dae\n'
    assert (models / 'OakD-Lite/model.config').read_text() == 'stock OakD-Lite/model.config\n'


def test_install_models_copies_the_worlds(tmp_path):
    px4_tree(tmp_path / 'PX4')
    result = run([str(REPO / 'sim/install_models.sh'), str(tmp_path / 'PX4')])
    assert result.returncode == 0, result.stderr
    worlds = sorted(p.name for p in (REPO / 'sim/worlds').iterdir())
    assert 'default_warehouse.sdf' in worlds
    assert sorted(re.findall(r'^installed world \./(.+)$', result.stdout, re.M)) == worlds
    for name in worlds:
        installed = tmp_path / 'PX4/Tools/simulation/gz/worlds' / name
        assert installed.read_bytes() == (REPO / 'sim/worlds' / name).read_bytes(), name


def test_install_models_rerun_keeps_the_original_backup(tmp_path):
    models = px4_tree(tmp_path / 'PX4')
    for _ in range(2):
        assert run([str(REPO / 'sim/install_models.sh'), str(tmp_path / 'PX4')]).returncode == 0
    # A second run must not back up our own copy over PX4's original.
    assert (models / 'x500_depth/model.sdf.orig').read_text() == 'stock x500_depth/model.sdf\n'


def test_install_models_defaults_to_home_px4(tmp_path, stubs):
    models = px4_tree(tmp_path / 'PX4-Autopilot')
    result = run([str(REPO / 'sim/install_models.sh')], env=stubs.env(HOME=tmp_path))
    assert result.returncode == 0, result.stderr
    assert (models / 'lidar_3d/model.sdf').exists()


def test_install_models_handles_names_with_spaces(tmp_path):
    # A copy of the script next to its own models folder, which is where it reads models from.
    shutil.copy(REPO / 'sim/install_models.sh', tmp_path / 'install_models.sh')
    (tmp_path / 'models/my model').mkdir(parents=True)
    (tmp_path / 'models/my model/model.sdf').write_text('<sdf/>\n')
    models = px4_tree(tmp_path / 'PX4', stock=False)
    result = run([str(tmp_path / 'install_models.sh'), str(tmp_path / 'PX4')])
    assert result.returncode == 0, result.stderr
    assert (models / 'my model/model.sdf').read_text() == '<sdf/>\n'
    assert result.stdout == 'installed ./my model/model.sdf\n'


# ---- sim/launch_sim.sh ----

def test_launch_sim_builds_px4_sitl_with_the_x500_depth_in_the_warehouse(tmp_path, stubs):
    stubs.add('make')
    (tmp_path / 'PX4').mkdir()
    result = run([str(REPO / 'sim/launch_sim.sh'), str(tmp_path / 'PX4')], env=stubs.env())
    assert result.returncode == 0, result.stderr
    (call,) = stubs.calls('make')
    assert call['argv'] == ['px4_sitl', 'gz_x500_depth'] and call['cwd'] == str(tmp_path / 'PX4')
    # The world is the file name (worlds/default_warehouse.sdf); "warehouse" alone hangs.
    assert call['env']['PX4_GZ_WORLD'] == 'default_warehouse'


def test_launch_sim_world_and_default_px4_dir(tmp_path, stubs):
    stubs.add('make')
    (tmp_path / 'PX4-Autopilot').mkdir()
    result = run([str(REPO / 'sim/launch_sim.sh')], env=stubs.env(HOME=tmp_path, WORLD='walls'))
    assert result.returncode == 0, result.stderr
    (call,) = stubs.calls('make')
    assert call['cwd'] == str(tmp_path / 'PX4-Autopilot') and call['env']['PX4_GZ_WORLD'] == 'walls'


@pytest.mark.parametrize('nvidia_smi, gpu, offload', [
    ('exit 0', '', True),             # working NVIDIA driver: render on it
    ('exit 9', '', False),            # no NVIDIA GPU or driver: unchanged
    ('exit 0', 'integrated', False),  # opted out
])
def test_launch_sim_renders_on_the_nvidia_gpu_only_when_one_works(tmp_path, stubs, nvidia_smi, gpu, offload):
    stubs.add('make').add('nvidia-smi', script=nvidia_smi + '\n')
    (tmp_path / 'PX4').mkdir()
    env = {k: v for k, v in stubs.env(GPU=gpu).items() if not k.startswith(('__NV_', '__GLX_'))}
    result = run([str(REPO / 'sim/launch_sim.sh'), str(tmp_path / 'PX4')], env=env)
    assert result.returncode == 0, result.stderr
    (call,) = stubs.calls('make')
    expected = {'__NV_PRIME_RENDER_OFFLOAD': '1', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'} if offload else {}
    assert {k: v for k, v in call['env'].items() if k.startswith(('__NV_', '__GLX_'))} == expected


@pytest.mark.parametrize('extra, nav_dll_act', [
    ({'NO_GCS': 'true'}, '0'),                                    # arm without QGroundControl
    ({}, '2'),                                                    # undo an earlier NO_GCS run (PX4 saves params)
    ({'NO_GCS': 'false', 'PX4_PARAM_NAV_DLL_ACT': '1'}, '1'),     # a value the user set themselves is kept
])
def test_launch_sim_no_gcs_option(tmp_path, stubs, extra, nav_dll_act):
    stubs.add('make')
    (tmp_path / 'PX4').mkdir()
    env = stubs.env(**extra)
    for name in ('NO_GCS', 'PX4_PARAM_NAV_DLL_ACT'):
        if name not in extra:
            env.pop(name, None)
    result = run([str(REPO / 'sim/launch_sim.sh'), str(tmp_path / 'PX4')], env=env)
    assert result.returncode == 0, result.stderr
    (call,) = stubs.calls('make')
    assert call['env']['PX4_PARAM_NAV_DLL_ACT'] == nav_dll_act


def test_launch_sim_stops_if_px4_is_missing(tmp_path, stubs):
    stubs.add('make')
    result = run([str(REPO / 'sim/launch_sim.sh'), str(tmp_path / 'nope')], env=stubs.env())
    assert result.returncode != 0 and stubs.calls('make') == []


@pytest.mark.parametrize('own, expected', [(None, '60'), ('150', '150')])
def test_launch_sim_limits_manual_yaw_rate(tmp_path, stubs, own, expected):
    """Full yaw stick at PX4's 150 deg/s made the drone sink and RTAB-Map lose track; a value you set is kept."""
    stubs.add('make')
    (tmp_path / 'PX4').mkdir()
    env = stubs.env(**({'PX4_PARAM_MPC_MAN_Y_MAX': own} if own else {}))
    if not own:
        env.pop('PX4_PARAM_MPC_MAN_Y_MAX', None)
    assert run([str(REPO / 'sim/launch_sim.sh'), str(tmp_path / 'PX4')], env=env).returncode == 0
    assert stubs.calls('make')[0]['env']['PX4_PARAM_MPC_MAN_Y_MAX'] == expected


# ---- sim/reset_drone.sh ----

PX4_CLIENTS = ('px4-commander', 'px4-ekf2', 'px4-listener')


def reset_drone(tmp_path, stubs, rtabmap=True, gz_reply='data: true', **env):
    """Runs reset_drone.sh against a fake PX4 build whose client commands (px4-commander, ...) are stubs."""
    stubs.add('gz', 'ros2', *PX4_CLIENTS)
    bin_dir = tmp_path / 'PX4/build/px4_sitl_default/bin'
    bin_dir.mkdir(parents=True)
    for name in PX4_CLIENTS:
        (bin_dir / name).symlink_to(stubs.bin / name)
    services = '/rtabmap/rgbd_odometry/reset_odom\n' if rtabmap else '/other/service\n'
    return run([str(REPO / 'sim/reset_drone.sh'), str(tmp_path / 'PX4')], env=stubs.env(
        ROS_OVERLAY=stubs.overlay, STUB_GZ_STDOUT=gz_reply, STUB_ROS2_STDOUT=services,
        STUB_PX4_LISTENER_STDOUT='    pre_flight_checks_pass: True\n', **env))


@needs_ros_setup
def test_reset_drone_disarms_moves_it_upright_to_the_spawn_point_and_restarts_the_estimator(tmp_path, stubs):
    result = reset_drone(tmp_path, stubs)
    assert result.returncode == 0, result.stderr
    assert 'ready for takeoff' in result.stdout
    assert [c['argv'] for c in stubs.calls('px4-commander')] == [['disarm', '-f']]
    (gz,) = stubs.calls('gz')
    assert gz['argv'][:3] == ['service', '-s', '/world/default_warehouse/set_pose']
    request = gz['argv'][gz['argv'].index('--req') + 1]
    # PX4 spawns x500_depth_0 at the world origin; upright = identity orientation.
    assert 'name: "x500_depth_0"' in request and 'x: 0 y: 0' in request and 'orientation { w: 1 }' in request
    assert [c['argv'] for c in stubs.calls('px4-ekf2')] == [['stop'], ['start']]
    assert ['service', 'call', '/rtabmap/rgbd_odometry/reset_odom', 'std_srvs/srv/Empty'] in \
        [c['argv'] for c in stubs.calls('ros2')]


@needs_ros_setup
def test_reset_drone_world_and_no_rtabmap(tmp_path, stubs):
    result = reset_drone(tmp_path, stubs, rtabmap=False, WORLD='walls')
    assert result.returncode == 0, result.stderr
    assert stubs.calls('gz')[0]['argv'][2] == '/world/walls/set_pose'
    assert all(c['argv'][:2] != ['service', 'call'] for c in stubs.calls('ros2'))


@needs_ros_setup
def test_reset_drone_stops_if_the_drone_cannot_be_moved(tmp_path, stubs):
    """E.g. Gazebo's reset deleted the drone: then the estimator is left alone and the user is told why."""
    result = reset_drone(tmp_path, stubs, gz_reply='data: false')
    assert result.returncode == 1
    assert 'restart the sim' in result.stderr
    assert stubs.calls('px4-ekf2') == []


@needs_ros_setup
@pytest.mark.parametrize('own', [None, '10.0.0.5'])
def test_gazebo_tools_use_loopback_like_px4(tmp_path, stubs, monkeypatch, own):
    """PX4 starts Gazebo with GZ_IP=127.0.0.1. On the default interface the bridge and reset lost Gazebo
    whenever the Wi-Fi dropped (suspend). An address you set is kept."""
    expected = own or '127.0.0.1'
    extra = {'GZ_IP': own} if own else {}
    monkeypatch.delenv('GZ_IP', raising=False)
    assert reset_drone(tmp_path, stubs, **extra).returncode == 0
    assert stubs.calls('gz')[0]['env']['GZ_IP'] == expected
    stubs.add('ros2')
    (stubs.log / 'ros2.jsonl').unlink()
    env = stubs.env(ROS_OVERLAY=stubs.overlay, XDG_RUNTIME_DIR=tmp_path, **extra)
    assert run([str(REPO / 'sim/bridge.sh')], env=env).returncode == 0
    assert ros2_call(stubs)['env']['GZ_IP'] == expected


def test_reset_drone_needs_a_px4_build(tmp_path):
    result = run([str(REPO / 'sim/reset_drone.sh'), str(tmp_path / 'nope')])
    assert result.returncode == 1 and 'make px4' in result.stderr


# ---- sim/bridge.sh ----

def bridge(stubs, **env):
    """({gz_topic: (ros_type, direction, gz_type)}, {gz_topic: ros_topic}) from the config bridge.sh writes."""
    stubs.add('ros2')
    env.setdefault('XDG_RUNTIME_DIR', str(stubs.log))
    result = run([str(REPO / 'sim/bridge.sh')], env=stubs.env(ROS_OVERLAY=stubs.overlay, **env))
    assert result.returncode == 0, result.stderr
    argv = ros2_call(stubs)['argv']
    assert argv[:4] == ['run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args']
    assert argv[4] == '-p' and argv[5].startswith('config_file:=') and len(argv) == 6
    entries = yaml.safe_load(Path(argv[5].split(':=', 1)[1]).read_text())
    # Lazy: unused topics (the 10 MB /uav/points cloud) must not load the single-threaded bridge.
    assert all(e['lazy'] is True for e in entries)
    # '[' bridges Gazebo -> ROS only, as GZ_TO_ROS does.
    bridges = {e['gz_topic_name']: (e['ros_type_name'], '[' if e['direction'] == 'GZ_TO_ROS' else '?',
                                    e['gz_type_name']) for e in entries}
    remaps = {e['gz_topic_name']: e['ros_topic_name'] for e in entries}
    return bridges, remaps


def readme_inputs():
    """{topic: type} from the README's 'Inputs, from the bridge' table (the first table after it)."""
    table = README.split('Inputs, from the bridge:')[1].strip().split('\n\n')[0]
    return dict(re.findall(r'^\| `(/[^`]+)` \| `([^`]+)` \|', table, re.M))


@needs_ros_setup
def test_bridge_publishes_exactly_the_readme_inputs(stubs):
    bridges, remaps = bridge(stubs)
    assert len(bridges) == 9 and all(direction == '[' for _, direction, _ in bridges.values())
    assert set(remaps) == set(bridges), 'every bridged Gazebo topic gets a ROS name'
    ros_topics = {remaps[gz]: ros_type.replace('/msg/', '/') for gz, (ros_type, _, _) in bridges.items()}
    assert ros_topics == readme_inputs()


@needs_ros_setup
def test_bridge_message_types(stubs):
    expected = {
        '/uav/depth/image': ('sensor_msgs/msg/Image', 'gz.msgs.Image'),
        '/uav/points': ('sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked'),
        '/uav/depth/camera_info': ('sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
        '/uav/rgb/image': ('sensor_msgs/msg/Image', 'gz.msgs.Image'),
        '/uav/rgb/camera_info': ('sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
        '/uav/lidar/points': ('sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked'),
        '/uav/odom': ('nav_msgs/msg/Odometry', 'gz.msgs.OdometryWithCovariance'),
        '/uav/imu': ('sensor_msgs/msg/Imu', 'gz.msgs.IMU'),
        '/clock': ('rosgraph_msgs/msg/Clock', 'gz.msgs.Clock'),
    }
    bridges, remaps = bridge(stubs)
    assert {remaps[gz]: (b[0], b[2]) for gz, b in bridges.items()} == expected


@needs_ros_setup
@pytest.mark.parametrize('world', [None, 'walls'])
def test_bridge_gazebo_topics_match_the_world_and_the_models(stubs, world):
    bridges, remaps = bridge(stubs, **({'WORLD': world} if world else {}))
    world = world or 'default_warehouse'
    gz = {ros: gz for gz, ros in remaps.items()}
    model = 'x500_depth_0'  # PX4 spawns the first x500_depth as <model>_0
    oakd = ET.parse(REPO / 'sim/models/OakD-Lite/model.sdf').getroot()
    rgb_sensor = oakd.find(".//sensor[@type='camera']")
    camera_link = oakd.find('.//link').get('name')
    assert gz['/uav/rgb/image'] == f'/world/{world}/model/{model}/link/{camera_link}/sensor/{rgb_sensor.get("name")}/image'
    assert gz['/uav/rgb/camera_info'] == gz['/uav/rgb/image'].replace('/image', '/camera_info')
    # The depth sensor publishes on its <topic>; OdometryPublisher on /model/<name>/odometry_with_covariance.
    assert gz['/uav/depth/image'] == '/' + oakd.find(".//sensor[@type='depth_camera']/topic").text
    assert gz['/uav/points'] == gz['/uav/depth/image'] + '/points'
    assert gz['/uav/odom'] == f'/model/{model}/odometry_with_covariance'
    assert gz['/uav/imu'] == f'/world/{world}/model/{model}/link/base_link/sensor/imu_sensor/imu'
    assert gz['/clock'] == f'/world/{world}/clock'
    # gpu_lidar publishes a LaserScan on <topic> and the 3D cloud on <topic>/points.
    lidar = ET.parse(REPO / 'sim/models/lidar_3d/model.sdf').getroot().find(".//sensor[@type='gpu_lidar']")
    assert gz['/uav/lidar/points'] == f'/{lidar.find("topic").text}/points'


# ---- sim/rtabmap.sh, zed_usb/run.sh, jetson/run.sh ----

@needs_ros_setup
@pytest.mark.parametrize('script, launch_file, args', [
    ('sim/rtabmap.sh', 'sim/rtabmap.launch.py', ['odometry:=sim', 'lidar:=true', 'new_map:=false']),
    ('zed_usb/run.sh', 'zed_usb/zed_usb_rtabmap.launch.py', ['flip:=true', 'new_map:=false']),
])
def test_launcher_runs_its_launch_file_by_absolute_path(stubs, tmp_path, script, launch_file, args):
    """Run from another folder: the launch file is found next to the script, not in the cwd,
    and launch arguments are passed through."""
    stubs.add('ros2')
    result = run([str(REPO / script), *args], env=stubs.env(ROS_OVERLAY=stubs.overlay), cwd=tmp_path)
    assert result.returncode == 0, result.stderr
    assert ros2_call(stubs)['argv'] == ['launch', str(REPO / launch_file), *args]


@needs_ros_setup
@pytest.mark.parametrize('script', ['sim/bridge.sh', 'sim/rtabmap.sh'])
@pytest.mark.parametrize('own_profile', [None, '/my/profile.xml'])
def test_sim_scripts_use_the_shared_memory_dds_profile(stubs, tmp_path, script, own_profile):
    """Both ends of the camera images (bridge and RTAB-Map) need the profile, or the images go over UDP
    and reach RTAB-Map at a few Hz. A profile the user set is kept."""
    stubs.add('ros2')
    env = stubs.env(ROS_OVERLAY=stubs.overlay, XDG_RUNTIME_DIR=tmp_path)
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    if own_profile:
        env['FASTRTPS_DEFAULT_PROFILES_FILE'] = own_profile
    result = run([str(REPO / script)], env=env, cwd=tmp_path)
    assert result.returncode == 0, result.stderr
    expected = own_profile or str(REPO / 'sim/fastdds.xml')
    assert ros2_call(stubs)['env']['FASTRTPS_DEFAULT_PROFILES_FILE'] == expected


def test_fastdds_profile_is_the_default_and_uses_shared_memory_and_localhost_udp():
    ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    root = ET.parse(REPO / 'sim/fastdds.xml').getroot()
    transports = {t.findtext('d:transport_id', namespaces=ns): t for t in root.iterfind('.//d:transport_descriptor', ns)}
    participant = root.find('d:participant', ns)
    assert participant.get('is_default_profile') == 'true'
    used = [t.text for t in participant.iterfind('.//d:userTransports/d:transport_id', ns)]
    assert sorted(transports[t].findtext('d:type', namespaces=ns) for t in used) == ['SHM', 'UDPv4']
    shm = next(transports[t] for t in used if transports[t].findtext('d:type', namespaces=ns) == 'SHM')
    assert int(shm.findtext('d:segment_size', namespaces=ns)) >= 4 * 640 * 480 * 4  # several depth images
    udp = next(transports[t] for t in used if transports[t].findtext('d:type', namespaces=ns) == 'UDPv4')
    # Same reach as ROS_LOCALHOST_ONLY=1: nothing leaves the machine.
    assert [a.text for a in udp.iterfind('.//d:address', ns)] == ['127.0.0.1']


@needs_ros_setup
def test_make_zed_usb_passes_flip(stubs):
    stubs.add('ros2')
    env = stubs.env(ROS_OVERLAY=stubs.overlay)
    assert run(['make', '--no-print-directory', 'zed-usb', 'FLIP=true'], env=env).returncode == 0
    assert ros2_call(stubs)['argv'][2:] == ['flip:=true']


def test_jetson_run_requires_zed_model(stubs):
    stubs.add('ros2')
    env = stubs.env()
    env.pop('ZED_MODEL', None)
    result = run([str(REPO / 'jetson/run.sh')], env=env)
    assert result.returncode != 0 and 'set ZED_MODEL' in result.stderr
    assert stubs.calls('ros2') == []


@needs_ros_setup
def test_jetson_run_launches_with_the_camera_model_and_extra_args(stubs, tmp_path):
    ws = tmp_path / 'zed_ws'
    (ws / 'install').mkdir(parents=True)
    shutil.copy(stubs.overlay, ws / 'install/setup.bash')  # stands in for the built ZED wrapper
    stubs.add('ros2')
    result = run([str(REPO / 'jetson/run.sh'), 'fcu_url:=/dev/ttyACM0:57600'],
                 env=stubs.env(ZED_MODEL='zed2i', ZED_WS=ws), cwd=tmp_path)
    assert result.returncode == 0, result.stderr
    assert ros2_call(stubs)['argv'] == ['launch', str(REPO / 'jetson/zed_rtabmap.launch.py'),
                                        'camera_model:=zed2i', 'fcu_url:=/dev/ttyACM0:57600']


def test_jetson_run_fails_without_a_built_wrapper(stubs, tmp_path):
    stubs.add('ros2')
    result = run([str(REPO / 'jetson/run.sh')], env=stubs.env(ZED_MODEL='zed2i', ZED_WS=tmp_path / 'none'))
    assert result.returncode != 0 and stubs.calls('ros2') == []


# ---- jetson/install.sh ----

INSTALL = REPO / 'jetson/install.sh'
NOTHING_INSTALLED = 'exit 1\n'  # dpkg stub: no package is installed


def install_env(stubs, tmp_path, **env):
    """Stubs for everything install.sh could use to change the system or reach the network."""
    stubs.add('curl', 'sudo', 'apt-get', 'git', 'colcon', 'rosdep', 'usermod')
    stubs.add('dpkg', script=NOTHING_INSTALLED)
    return stubs.env(ZED_WS=tmp_path / 'zed_ws', **env)


def bash_function(script, name):
    """Source of one shell function from a script, to call it on its own."""
    match = re.search(rf'^{name}\(\) {{\n.*?^}}\n', script.read_text(), re.M | re.S)
    assert match, name
    return match.group(0)


@pytest.mark.parametrize('l4t, versions', [('36.4', '5.5 v5.5.0'), ('36.3', '5.2 v5.2.2'), ('36.2', None),
                                           ('35.4', None)])
def test_zed_versions_for_l4t(l4t, versions):
    result = run(['bash', '-c', bash_function(INSTALL, 'zed_versions_for') + f'zed_versions_for {l4t}'])
    assert (result.stdout.strip() or None, result.returncode == 0) == (versions, versions is not None)


@pytest.mark.parametrize('release, l4t', [
    ('# R36 (release), REVISION: 4.3, GCID: 38968081, BOARD: generic, EABI: aarch64, DATE: Wed Jan  8 01:49:37 UTC 2025', '36.4'),
    ('# R36 (release), REVISION: 3.0, GCID: 36191598, BOARD: generic, EABI: aarch64, DATE: Mon May  6 17:34:21 UTC 2024', '36.3'),
    ('# R35 (release), REVISION: 4.1, GCID: 33958178, BOARD: t186ref, EABI: aarch64, DATE: Tue Aug  1 19:57:35 UTC 2023', '35.4'),
])
def test_l4t_version_is_parsed_from_nv_tegra_release(tmp_path, release, l4t):
    sed = re.search(r"sed -nE '([^']+)' /etc/nv_tegra_release", INSTALL.read_text()).group(1)
    (tmp_path / 'nv_tegra_release').write_text(release + '\n')
    assert run(['sed', '-nE', sed, str(tmp_path / 'nv_tegra_release')]).stdout.strip() == l4t


@pytest.mark.parametrize('l4t, sdk, tag', [('36.4', '5.5', 'v5.5.0'), ('36.3', '5.2', 'v5.2.2')])
def test_install_check_previews_the_plan_without_changing_anything(stubs, tmp_path, l4t, sdk, tag):
    result = run([str(INSTALL), '--check'], env=install_env(stubs, tmp_path, JETSON_L4T=l4t))
    assert f'Jetson L4T {l4t} -> ZED SDK {sdk}, ZED ROS 2 wrapper {tag}' in result.stdout
    url = f'https://download.stereolabs.com/zedsdk/{sdk}/l4t{l4t}/jetsons'
    assert [c['argv'][-1] for c in stubs.calls('curl')] == [url]
    assert f'MISSING  ZED ROS 2 wrapper {tag} in {tmp_path / "zed_ws"}' in result.stdout
    assert 'MISSING  ros-humble-rtabmap-ros' in result.stdout
    assert result.returncode == 1 and 'Run ./jetson/install.sh to install what is missing.' in result.stdout
    for cmd in ('sudo', 'apt-get', 'git', 'colcon', 'rosdep', 'usermod'):
        assert stubs.calls(cmd) == [], f'--check ran {cmd}'


def test_install_check_reports_an_unreachable_installer(stubs, tmp_path):
    env = install_env(stubs, tmp_path, JETSON_L4T='36.4', STUB_CURL_RC=22)
    assert 'MISSING  ZED SDK installer not reachable' in run([str(INSTALL), '--check'], env=env).stdout


@pytest.mark.skipif(subprocess.run(['uname', '-m'], capture_output=True, text=True).stdout.strip() == 'aarch64',
                    reason='checks the report on a non-Jetson machine')
def test_install_check_flags_a_non_arm_machine(stubs, tmp_path):
    result = run([str(INSTALL), '--check'], env=install_env(stubs, tmp_path, JETSON_L4T='36.4'))
    assert re.search(r'MISSING  arm64 CPU \(this is \S+\)', result.stdout)


def test_install_rejects_an_unsupported_l4t(stubs, tmp_path):
    result = run([str(INSTALL), '--check'], env=install_env(stubs, tmp_path, JETSON_L4T='35.4'))
    assert result.returncode == 1 and 'L4T 35.4 is not supported' in result.stderr


def test_install_refuses_to_install_with_a_preview_l4t(stubs, tmp_path):
    result = run([str(INSTALL)], env=install_env(stubs, tmp_path, JETSON_L4T='36.4'))
    assert result.returncode == 1 and 'only for previewing with --check' in result.stderr
    assert stubs.calls('sudo') == [] and stubs.calls('apt-get') == []


@pytest.mark.skipif(os.path.exists('/etc/nv_tegra_release'), reason='this machine is a Jetson')
def test_install_refuses_to_run_off_a_jetson(stubs, tmp_path):
    env = install_env(stubs, tmp_path)
    env.pop('JETSON_L4T', None)
    result = run([str(INSTALL)], env=env)
    assert result.returncode == 1 and 'not a Jetson' in result.stderr
    assert stubs.calls('sudo') == []


# ---- Makefile ----

def make(*args, env=None):
    return run(['make', '--no-print-directory', *args], env=env)


def documented_targets():
    """Targets with a '## description': the ones `make help` shows."""
    return re.findall(r'^([a-zA-Z0-9_-]+):.*?## ', MAKEFILE, re.M)


def phony_targets():
    block = re.search(r'^\.PHONY:(.*?)(?<!\\)\n', MAKEFILE, re.M | re.S).group(1)
    return set(block.replace('\\\n', ' ').split())


def test_help_lists_every_documented_target_and_is_the_default():
    plain = re.sub(r'\x1b\[[0-9;]*m', '', make('help').stdout)
    listed = re.findall(r'^  (\S+)\s+\S', plain, re.M)
    assert listed == documented_targets()
    assert make().stdout == make('help').stdout


def test_every_target_is_phony():
    """None of the targets builds a file of its name, so all must be .PHONY (or a stray file named
    e.g. 'check' or 'test' would make them silently do nothing)."""
    targets = set(re.findall(r'^([a-zA-Z0-9_-]+):(?!=)', MAKEFILE, re.M))
    assert targets == phony_targets()


def test_deps_refuses_a_jetson_before_installing_anything():
    deps = re.search(r'^deps: (.*?) ##', MAKEFILE, re.M).group(1).split()
    assert deps[0] == 'not-jetson'


@pytest.mark.parametrize('machine, ok', [('aarch64', False), ('x86_64', True)])
def test_not_jetson_guard(stubs, machine, ok):
    stubs.add('uname')
    result = make('not-jetson', env=stubs.env(STUB_UNAME_STDOUT=machine + '\n'))
    assert (result.returncode == 0) == ok
    assert ('use \'make jetson-deps\' instead' in result.stdout) == (not ok)


@pytest.mark.parametrize('args, expected', [
    (['sim', 'PX4_DIR=/px4'], 'NO_GCS=false ./sim/launch_sim.sh /px4'),
    (['sim', 'PX4_DIR=/px4', 'NO_GCS=true'], 'NO_GCS=true ./sim/launch_sim.sh /px4'),
    (['models', 'PX4_DIR=/px4'], './sim/install_models.sh /px4'),
    (['bridge'], './sim/bridge.sh'),
    (['reset-drone', 'PX4_DIR=/px4'], './sim/reset_drone.sh /px4'),
    (['camera'], 'PYTHONNOUSERSITE=1 python3 sim/camera_orbit.py'),
    (['rtabmap'], './sim/rtabmap.sh odometry:=visual lidar:=false new_map:=true detection_rate:=0'),
    (['rtabmap', 'ODOMETRY=sim', 'LIDAR=true', 'NEW_MAP=false', 'RATE=1'],
     './sim/rtabmap.sh odometry:=sim lidar:=true new_map:=false detection_rate:=1'),
    (['save-map', 'OUT=/maps/w'],
     'source /opt/ros/humble/setup.bash && python3 tools/save_map.py --ros-args -p out:=/maps/w'),
    (['load-map', 'MAP=/maps/w.yaml'],
     'source /opt/ros/humble/setup.bash && python3 tools/load_map.py --ros-args -p map:=/maps/w.yaml'),
    (['load-map', 'OUT=/maps/w'],  # MAP defaults to the map save-map wrote
     'source /opt/ros/humble/setup.bash && python3 tools/load_map.py --ros-args -p map:=/maps/w.yaml'),
    (['map-monitor'],
     'source /opt/ros/humble/setup.bash && python3 tools/map_monitor.py --ros-args -p use_sim_time:=true'),
    (['zed-usb'], './zed_usb/run.sh flip:=false'),
    (['zed-usb', 'FLIP=true'], './zed_usb/run.sh flip:=true'),
    (['jetson', 'ZED_MODEL=zed2i'], 'ZED_MODEL=zed2i ./jetson/run.sh'),
    (['jetson-deps'], './jetson/install.sh'),
    (['jetson-check'], './jetson/install.sh --check'),
])
def test_run_targets_call_the_scripts(args, expected):
    result = make('-n', *args)
    assert result.returncode == 0 and result.stdout.strip() == expected


def test_px4_default_location_and_pinned_commit():
    out = make('-n', 'models').stdout.strip()
    assert re.fullmatch(r'\./sim/install_models\.sh /.+/PX4-Autopilot', out)
    assert re.search(r'^PX4_REF\s+\?= [0-9a-f]{40}$', MAKEFILE, re.M)
    # The README tells users to check out the same commit by hand.
    ref = re.search(r'^PX4_REF\s+\?= (\S+)', MAKEFILE, re.M).group(1)
    assert f'git checkout {ref}' in README


def dpkg_stub(stubs):
    """dpkg -s <pkg> succeeds iff <pkg> is in STUB_DPKG_INSTALLED ('*' = everything)."""
    stubs.add('dpkg', script='[ "$1" = -s ] && { [ "$STUB_DPKG_INSTALLED" = "*" ] || '
                             '[[ " $STUB_DPKG_INSTALLED " == *" $2 "* ]]; }\n')


def installed_px4(root, models=True):
    """A fake PX4 checkout with a SITL build and (optionally) all our model and world files installed."""
    binary = root / 'build/px4_sitl_default/bin/px4'
    binary.parent.mkdir(parents=True)
    binary.write_text('')
    binary.chmod(0o755)
    if models:
        shutil.copytree(REPO / 'sim/models', root / 'Tools/simulation/gz/models')
        shutil.copytree(REPO / 'sim/worlds', root / 'Tools/simulation/gz/worlds')
    return root


def check(stubs, tmp_path, installed, **variables):
    dpkg_stub(stubs)
    stubs.add('sudo')
    qgc = tmp_path / 'qgc.AppImage'
    qgc.write_text('')
    qgc.chmod(0o755)
    args = [f'{k}={v}' for k, v in {'QGC_APP': qgc, **variables}.items()]
    return make('check', *args, env=stubs.env(STUB_DPKG_INSTALLED=installed))


def test_check_reports_missing_packages_and_fails(stubs, tmp_path):
    result = check(stubs, tmp_path, installed='', PX4_DIR=tmp_path / 'nope')
    assert result.returncode != 0
    for line in ('MISSING  ros-humble-desktop', 'MISSING  ros-humble-ros-gzharmonic',
                 'MISSING  ros-humble-rtabmap-ros', f'MISSING  PX4 SITL build in {tmp_path / "nope"}',
                 'MISSING  custom models in PX4 (make models)', 'ok       QGroundControl'):
        assert line in result.stdout
    assert 'Some dependencies are missing' in result.stdout
    assert stubs.calls('sudo') == []


def test_check_accepts_an_installed_system(stubs, tmp_path):
    px4 = installed_px4(tmp_path / 'PX4')
    result = check(stubs, tmp_path, installed='*', PX4_DIR=px4)
    assert 'MISSING  ros' not in result.stdout
    assert 'ok       PX4 SITL build' in result.stdout and 'ok       custom models installed in PX4' in result.stdout
    if Path('/usr/share/GeographicLib/geoids').is_dir():  # the one item this test cannot fake
        assert result.returncode == 0 and 'Everything is installed.' in result.stdout


@pytest.mark.parametrize('model_file', ['x500_depth/model.sdf', 'OakD-Lite/model.sdf', 'lidar_3d/model.sdf'])
def test_check_spots_outdated_models_in_px4(stubs, tmp_path, model_file):
    """Every file we ship is compared, not just x500_depth (an old lidar_3d had a second <topic>)."""
    px4 = installed_px4(tmp_path / 'PX4')
    (px4 / 'Tools/simulation/gz/models' / model_file).write_text('stock\n')
    result = check(stubs, tmp_path, installed='*', PX4_DIR=px4)
    assert result.returncode != 0 and 'MISSING  custom models in PX4' in result.stdout


def qgc_app(tmp_path, stubs):
    """A QGroundControl "AppImage" that records being started."""
    stubs.add('QGroundControl.AppImage')
    return stubs.bin / 'QGroundControl.AppImage'


@pytest.mark.parametrize('installed, sudo_argv', [
    ('*', None),  # everything there: no sudo prompt at all
    ('gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libfuse2 libxcb-xinerama0 libxkbcommon-x11-0',
     ['apt-get', 'install', '-y', 'libxcb-cursor-dev']),  # only what is missing
])
def test_make_qgc_installs_only_missing_libraries_and_keeps_the_appimage(stubs, tmp_path, installed, sudo_argv):
    dpkg_stub(stubs)
    stubs.add('sudo', 'wget')
    app = qgc_app(tmp_path, stubs)
    result = make('qgc', f'QGC_APP={app}', env=stubs.env(STUB_DPKG_INSTALLED=installed))
    assert result.returncode == 0, result.stderr
    assert [c['argv'] for c in stubs.calls('sudo')] == ([sudo_argv] if sudo_argv else [])
    assert ('libraries already installed' in result.stdout) == (sudo_argv is None)
    assert stubs.calls('wget') == [] and 'QGroundControl already at' in result.stdout


def test_make_qgc_run_starts_the_appimage(stubs, tmp_path):
    app = qgc_app(tmp_path, stubs)
    assert make('qgc-run', f'QGC_APP={app}', env=stubs.env()).returncode == 0
    assert len(stubs.calls('QGroundControl.AppImage')) == 1


def test_make_qgc_run_without_qgc_says_how_to_install_it(tmp_path):
    result = make('qgc-run', f'QGC_APP={tmp_path / "none.AppImage"}')
    assert result.returncode != 0 and 'run make qgc' in result.stdout


def test_check_warns_about_the_fortress_bridge(stubs, tmp_path):
    result = check(stubs, tmp_path, installed='ros-humble-ros-gz-bridge', PX4_DIR=tmp_path)
    assert 'WARNING  ros-humble-ros-gz-bridge (Fortress) is installed' in result.stdout
