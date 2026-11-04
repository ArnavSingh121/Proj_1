# Tests

```bash
make test                                # everything (sources ROS if it is installed)
make test PYTEST_ARGS='-k bridge -v'     # a subset, verbose
```

Running pytest directly works too. Source ROS first, or `test_zed_usb_publisher.py` (which needs
`rclpy`) and the launcher-script tests are skipped; the other files run without ROS:

```bash
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 python3 -m pytest tests
```

`PYTHONNOUSERSITE=1` keeps a NumPy 2 in `~/.local` from breaking the system OpenCV, for the same
reason `zed_usb_rtabmap.launch.py` sets it.

The suite needs no sudo, network, camera, GPU, running ROS graph or PX4 build. It takes a few seconds.
Everything it needs is installed by `make deps`: pytest, NumPy, OpenCV and PyYAML.

## What is tested

| File | Tier | What it covers |
|---|---|---|
| [`test_static.py`](test_static.py) | 1: static and consistency | The files that have to agree with each other. SDF models are valid (`gz sdf -k`, with `x500_depth` checked as installed over PX4's models). The RGB and depth sensors share pose, FOV and resolution. The intrinsics match the README. The static TFs match the SDF camera and lidar mounts and turn body axes into optical axes. The lidar has a single topic. The 0.3 m self-filter is the same in all three launch files, `zed_params.yaml` and the publisher. RTAB-Map's input topics and frame are ones the ZED publisher actually provides. `zed_params.yaml` only turns on what RTAB-Map uses. Scripts pass `bash -n` and `shellcheck` and are committed as executable. |
| [`test_zed_usb_publisher.py`](test_zed_usb_publisher.py) | 2: unit | `zed_usb/zed_usb_publisher.py`. Parsing calibration files (5-coefficient and ZED 2/2i rational models). Flipped calibration for an upside-down camera, as a projection property test. Disparity → depth conversion. Finding the camera in a fake `/sys`. Node set-up (capture settings, SGBM range, calibration download, waiting for the camera, `device`/`flip`/`resolution` parameters). The rectification check. The `CameraInfo`/`Image` messages. The main loop and reconnecting after a USB drop. Calibration downloads that fail or are not calibration files leave no file behind and are retried. |
| [`test_zed_usb_failures.py`](test_zed_usb_failures.py) | 2: unit | Camera failures never crash the publisher (issue #12). A scripted fake camera plays back a missing camera, failed, raising and garbled reads, frames of unknown size, a USB disconnect mid-run, and an offline or corrupt calibration download. Runs without ROS, against the stand-in in [`ros_stub.py`](ros_stub.py). |
| [`test_sensor_checks.py`](test_sensor_checks.py) | 2: unit | `sim/sensor_checks.py`, used by `sim/odom_relay.py`: NaN, non-unit quaternions, time going backwards and teleport jumps are rejected; zero covariance gets a usable variance; silence and recovery of the odometry stream are reported once. |
| [`test_map_tools.py`](test_map_tools.py) | 2: unit | `tools/map_tools.py`: map statistics, PGM orientation and round trip, `map_server` YAML, save and reload. |
| [`test_scripts.py`](test_scripts.py) | 3: scripts and Makefile | What each script would run, with stubbed `ros2`, `make`, `curl`, `dpkg`, `sudo`, …. `install_models.sh` copies files and keeps `.orig` backups. `launch_sim.sh` picks the world and PX4 folder. `bridge.sh` publishes exactly the README's input topics (including the lidar), from Gazebo topics that exist in our models. The launchers find their launch files and pass arguments through. `jetson/run.sh` requires `ZED_MODEL`. `jetson/install.sh` maps L4T versions to ZED SDK versions and its `--check` changes nothing. The Makefile's help, `.PHONY` list, aarch64 guard, run and map targets (`ODOMETRY`, `LIDAR`, `NEW_MAP`, `OUT`, `MAP`) and `make check` report (every model file compared) are checked. |

Not covered yet are tier 4 (evaluating the launch descriptions with the ROS launch API) and tier 5
(`launch_testing` of the publisher on a ROS graph), plus end-to-end runs of the simulation and the
drone.

## How the tests avoid the real system

- **Launch files are read, not imported.** Importing them pulls in ROS packages (`zed_wrapper` is
  only on the Jetson). The helpers in [`conftest.py`](conftest.py) (`module_constants`,
  `launch_arguments`, `static_tf`, …) read the constants and dict literals with Python's `ast`.
  Values that only exist at launch time, such as `LaunchConfiguration` or `PythonExpression`, show
  up as `UNEVALUATED`.
- **Stub commands.** The `stubs` fixture puts fake programs first on `PATH`. Each one records its
  arguments, working directory and environment, which tests read back with
  `stubs.calls('ros2')`. Scripts that source the ROS setup get `/opt/ros/humble/bin` back in front
  of `PATH`, so for them the stubs are re-added through `ROS_OVERLAY=stubs.overlay` (or
  `$ZED_WS/install/setup.bash` for `jetson/run.sh`).
- **A stand-in for ROS.** `test_zed_usb_failures.py` imports the publisher against
  [`ros_stub.py`](ros_stub.py) (minimal `rclpy`, `Node`, `Image`, `CameraInfo`), so it runs
  without a ROS install. `test_zed_usb_publisher.py` uses the real `rclpy` instead.
- **Publisher fakes.** `FakeCapture` replaces `cv2.VideoCapture` and serves synthetic side-by-side
  stereo frames, where `None` means a failed read. `write_conf` writes a Stereolabs-format
  calibration file. The `sysfs` fixture builds a fake `/sys` tree. `bare_node()` makes a `ZedUsb`
  without its constructor, so single methods can be tested without a camera. The publisher retries
  a missing camera or calibration forever. The `sleeps` fixture records `time.sleep` instead of
  waiting, and a test that sets `sleeps.limit` ends the retry loop with `Retried` after that many
  waits.
- **Optional checks skip themselves.** Checks that need `gz`, `shellcheck`, a PX4 checkout
  (`PX4_DIR`, default `~/PX4-Autopilot`) or the ZED wrapper's `common_stereo.yaml` are skipped
  when that tool or file isn't there. Run with `-rs` to see what was skipped.

## Known issues (expected failures)

None at the moment. When a test describes a bug that isn't fixed yet, mark it
`@pytest.mark.xfail(strict=True, reason='known issue: ...')`. It then shows up as `x` instead of
failing the run. Once the code is fixed the test passes, and `strict=True` fails the run so the
marker gets removed.

## Adding tests

- When a value has to match in several files, add the check to `test_static.py` next to similar ones.
- Publisher logic is easiest to test as a module-level function, like `disparity_to_depth`,
  `load_calibration` and `flip_calibration`. Keep new logic out of `ZedUsb.spin()` where possible.
- For a new script, run it from `test_scripts.py` with `stubs.add(...)` for every command that
  would change the system or reach the network, and check that `--check` or preview modes never
  call those commands.
