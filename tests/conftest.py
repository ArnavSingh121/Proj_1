"""Shared helpers for the Proj_1 test suite (see tests/README.md).

None of these tests need a running ROS graph, a camera, PX4, a GPU or the network:
- launch files and the publisher are read with `ast` instead of imported where possible, because
  importing them pulls in ROS packages;
- shell scripts run against stub commands (`ros2`, `make`, `curl`, `dpkg`, `sudo`, ...) that only
  record how they were called.
"""
import ast
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
# Lets the tests import the scripts in sim/, tools/ and zed_usb/ directly. No ROS install is needed for
# those: test_zed_usb_failures.py swaps in a small stand-in for rclpy (ros_stub.py).
for d in ('sim', 'tools', 'zed_usb', 'tests'):
    sys.path.insert(0, str(REPO / d))

ROS_SETUP = Path('/opt/ros/humble/setup.bash')
# PX4 checkout used by the optional checks against PX4's stock models (same default as the Makefile).
PX4_DIR = Path(os.environ.get('PX4_DIR', Path.home() / 'PX4-Autopilot'))
PX4_MODELS = PX4_DIR / 'Tools/simulation/gz/models'

# sim/, jetson/ and zed_usb/ scripts all `source /opt/ros/humble/setup.bash` unconditionally.
needs_ros_setup = pytest.mark.skipif(
    not ROS_SETUP.exists(), reason=f'the scripts source {ROS_SETUP}, which is not installed')


def run(cmd, env=None, cwd=REPO, timeout=60):
    """Runs a command and captures its output as text (never raises on a non-zero exit)."""
    return subprocess.run(cmd, cwd=cwd, env=env, capture_output=True, text=True, timeout=timeout)


# ---- Reading Python files without importing them ----

class Unevaluated:
    """Placeholder for a value that needs the launch runtime (LaunchConfiguration, os.path, ...)."""

    def __repr__(self):
        return '<unevaluated>'


UNEVALUATED = Unevaluated()


def _eval(node, names):
    return eval(compile(ast.Expression(node), '<ast>', 'eval'), {'__builtins__': {}}, dict(names))


def _try_eval(node, names):
    try:
        return _eval(node, names)
    except Exception:
        return UNEVALUATED


def module_constants(path):
    """Top-level `NAME = <expr>` assignments of a Python file, evaluated without importing the file.

    Only literals and expressions over earlier constants are evaluated (e.g. SELF_FILTER =
    '--Grid/RangeMin 0.3 ' + ODOM_SELF_FILTER); anything else (os.path calls, ...) is left out.
    """
    consts = {}
    for node in ast.parse(Path(path).read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            value = _try_eval(node.value, consts)
            if value is not UNEVALUATED:
                consts[node.targets[0].id] = value
    return consts


def literal_dicts(path):
    """Every dict literal in a file, entry by entry: values that need the launch runtime are UNEVALUATED.

    Used for the `launch_arguments={...}` passed to included launch files (rtabmap.launch.py, ...).
    """
    consts = module_constants(path)
    dicts = []
    for node in ast.walk(ast.parse(Path(path).read_text())):
        if isinstance(node, ast.Dict):
            d = {}
            for k, v in zip(node.keys, node.values):
                key = _try_eval(k, consts) if k is not None else UNEVALUATED
                if key is not UNEVALUATED and isinstance(v, ast.List):
                    d[key] = [_try_eval(item, consts) for item in v.elts]  # substitution lists
                elif key is not UNEVALUATED:
                    d[key] = _try_eval(v, consts)
            dicts.append(d)
    return dicts


def launch_arguments(path, marker):
    """The dict literal in a launch file that has the key `marker` (e.g. 'rgb_topic' for RTAB-Map's)."""
    found = [d for d in literal_dicts(path) if marker in d]
    assert len(found) == 1, f'expected one dict with {marker!r} in {path}, found {len(found)}'
    return found[0]


def list_items(path, keyword):
    """Every list literal passed as `keyword=[...]` to a call, item by item (UNEVALUATED where needed)."""
    consts = module_constants(path)
    lists = []
    for node in ast.walk(ast.parse(Path(path).read_text())):
        if isinstance(node, ast.keyword) and node.arg == keyword and isinstance(node.value, ast.List):
            lists.append([_try_eval(item, consts) for item in node.value.elts])
    return lists


def static_tf(path, child=None):
    """Arguments of a static_transform_publisher in a launch file, as {'--x': '0.1', ...}.

    `child` picks the transform by its --child-frame-id when the file has several (sim: camera and lidar).
    """
    found = [dict(zip(items[::2], items[1::2])) for items in list_items(path, 'arguments')
             if '--child-frame-id' in items]
    if child is not None:
        found = [tf for tf in found if tf['--child-frame-id'] == child]
    assert len(found) == 1, f'expected one static_transform_publisher{f" to {child}" if child else ""} in {path}'
    return found[0]


def assigned_values(path, name):
    """Values assigned to `name` anywhere in a file (also inside functions) that evaluate from constants.

    For launch arguments built in a function at launch time, e.g. `rtabmap_args = SELF_FILTER`.
    """
    consts = module_constants(path)
    values = []
    for node in ast.walk(ast.parse(Path(path).read_text())):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id == name):
            value = _try_eval(node.value, consts)
            if value is not UNEVALUATED:
                values.append(value)
    return values


def declared_parameters(path):
    """`declare_parameter(name, default)` calls in a node's source, as {name: default}."""
    params = {}
    for node in ast.walk(ast.parse(Path(path).read_text())):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'):
            params[_eval(node.args[0], {})] = _try_eval(node.args[1], {})
    return params


# ---- Stub commands for the shell-script tests ----

_PY_STUB = '''#!/usr/bin/env python3
# Test stub: records this call, then prints STUB_<NAME>_STDOUT and exits with STUB_<NAME>_RC.
import json, os, sys
name = os.path.basename(sys.argv[0])
key = name.upper().replace('-', '_')
with open({log!r}, 'a') as f:
    f.write(json.dumps({{'argv': sys.argv[1:], 'cwd': os.getcwd(), 'env': dict(os.environ)}}) + '\\n')
sys.stdout.write(os.environ.get('STUB_' + key + '_STDOUT', ''))
sys.exit(int(os.environ.get('STUB_' + key + '_RC', '0')))
'''


class Stubs:
    """Fake commands put first on PATH. Each call is recorded in <dir>/log/<name>.jsonl.

    `env()` is the environment to run a script with. Scripts that source the ROS setup get
    /opt/ros/humble/bin put in front of PATH again, so for those the stubs are put back in front by
    an overlay file, passed as ROS_OVERLAY (or as $ZED_WS/install/setup.bash for jetson/run.sh).
    """

    def __init__(self, root):
        self.bin = root / 'bin'
        self.log = root / 'log'
        self.bin.mkdir(parents=True)
        self.log.mkdir()
        self.overlay = root / 'overlay.bash'
        self.overlay.write_text(f'export PATH="{self.bin}:$PATH"\n')

    def add(self, *names, script=None):
        """Recording stub(s); `script` replaces the recorder with custom bash (e.g. dpkg's logic)."""
        for name in names:
            path = self.bin / name
            if script is None:
                path.write_text(_PY_STUB.format(log=str(self.log / f'{name}.jsonl')))
            else:
                path.write_text('#!/usr/bin/env bash\n' + script)
            path.chmod(0o755)
        return self

    def calls(self, name):
        """Every recorded call of a stub, oldest first: [{'argv': [...], 'cwd': ..., 'env': {...}}]."""
        path = self.log / f'{name}.jsonl'
        if not path.exists():
            return []
        return [json.loads(line) for line in path.read_text().splitlines()]

    def env(self, **extra):
        env = dict(os.environ)
        env['PATH'] = f'{self.bin}:{env["PATH"]}'
        env.update({k: str(v) for k, v in extra.items()})
        return env


@pytest.fixture
def stubs(tmp_path):
    return Stubs(tmp_path / 'stubs')
