# Proj_1: UAV Mapping and Localization System

CS 4800 (Software Engineering), Cal Poly Pomona.

A drone estimates its position and builds a 3D map of its surroundings with RTAB-Map
(RGB-D SLAM). In simulation, PX4 + Gazebo Harmonic stream the drone's camera, IMU and odometry
into ROS 2. On the real drone, a Jetson runs the same mapping with a Stereolabs ZED camera.

| Folder | What |
|---|---|
| [`sim/`](sim) | Simulation: custom drone models, launch, bridge and RTAB-Map scripts |
| [`jetson/`](jetson) | Real drone: install script and launch files for the Jetson + ZED |
| [`zed_usb/`](zed_usb) | The real ZED on a laptop without an NVIDIA GPU (CPU depth, same topics as on the drone) |
| [`tools/`](tools) | Save, reload and monitor the occupancy map (works with all three setups) |
| [`tests/`](tests) | Test suite (`make test`), no simulator or camera needed |
| [`Makefile`](Makefile) | One-command install and run targets for both (`make help`) |

## Environment

- Ubuntu 22.04, ROS 2 Humble
- PX4-Autopilot built from source, Gazebo Harmonic 8.x
- RTAB-Map, OctoMap server, MAVROS, QGroundControl 5.0.8

> Use `ros-humble-ros-gzharmonic`, **not** the default `ros-humble-ros-gz-bridge`.
> The default one is built for Gazebo Fortress: the topics show up but no data flows.

## Install dependencies (simulation, on a laptop)

The [Makefile](Makefile) installs everything on a fresh Ubuntu 22.04 machine.
It skips steps that are already done, so it is safe to re-run.

```bash
make check   # see what is installed / missing (changes nothing)
make deps    # install everything (asks for sudo; PX4 build takes a while)
make help    # list individual targets (ros, gazebo, slam, px4, models, qgc, ...)
```

PX4 is cloned to `~/PX4-Autopilot` at the commit we validated (`PX4_REF` in the Makefile).
Use a different folder with `make deps PX4_DIR=/path/to/PX4-Autopilot`.

An existing `~/PX4-Autopilot` is left on whatever version it is on. `make check` prints that
version; to move it to the validated one:

```bash
cd ~/PX4-Autopilot && git fetch origin && git checkout 2c47a16f563b86d895b394a0f3dd8848b0ccdd78 \
  && git submodule update --init --recursive && make px4_sitl
```

QGroundControl is pinned to v5.0.8: newer releases need Ubuntu 24.04 and do not start on 22.04.

## One-time setup: install our drone models into PX4

Our drone needs custom Gazebo models, in [`sim/models/`](sim/models):

| Model | Change |
|---|---|
| `x500_depth` | Adds the OdometryPublisher plugin (ground-truth odometry, 3D) and a 3D lidar |
| `OakD-Lite` | 640x480 RGB + depth, frame id `camera_optical_frame`; RGB field of view matches the depth camera (1.274 rad) so the two images line up pixel for pixel |
| `lidar_3d` | 3D lidar model (not in stock PX4) |

`make deps` installs them. To re-copy them by hand (originals are backed up as `*.orig`):

```bash
make models   # same as ./sim/install_models.sh ~/PX4-Autopilot
```

## Running the simulation

Terminal 1 starts PX4 + Gazebo. The first start can take a minute or two.

```bash
make sim      # same as ./sim/launch_sim.sh ~/PX4-Autopilot
```

Terminal 2 starts the Gazebo → ROS 2 bridge. Leave it running.

```bash
make bridge   # same as ./sim/bridge.sh
```

Terminal 3 starts RTAB-Map (RGB-D SLAM with its own visual odometry) and its viewer.
Fly the drone (QGroundControl, or `commander takeoff` in the PX4 shell) to build the map.

```bash
make rtabmap  # same as ./sim/rtabmap.sh; map database: ~/.ros/rtabmap.db (reset each start)
```

Options (all can be combined):

| Option | What |
|---|---|
| `make rtabmap ODOMETRY=sim` | Use Gazebo's simulated odometry (`/uav/odom`) instead of RTAB-Map's visual odometry. It is ground truth, so use `visual` (default) when measuring localization accuracy |
| `make rtabmap LIDAR=true` | Also feed the simulated 3D lidar (`/uav/lidar/points`) to RTAB-Map; the occupancy grid then uses both lidar and depth |
| `make rtabmap NEW_MAP=false` | Reload the map in `~/.ros/rtabmap.db` and keep mapping, instead of starting over |

Whatever the mode, [`sim/odom_relay.py`](sim/odom_relay.py) checks the simulated odometry: samples with
NaN values, a non-unit quaternion, time going backwards or an impossible jump are dropped (with a
warning) instead of being passed on, and it warns when odometry stops arriving and when it resumes.
Visual odometry restarts right away when it loses track (`Odom/ResetCountdown 1`) instead of stalling.

### Saving, reloading and monitoring the map

```bash
make save-map OUT=~/maps/warehouse          # current 2D occupancy grid -> warehouse.pgm + warehouse.yaml
make load-map MAP=~/maps/warehouse.yaml     # publish it again on /map (latched) for other components
make rtabmap NEW_MAP=false                  # or continue the full RTAB-Map map from its database
make map-monitor                            # log map growth (known/occupied/free cells) to ~/.ros/map_growth.csv
```

The `.pgm` + `.yaml` pair is the standard ROS `map_server` format, so nav2's map server can load it too.
`map-monitor` adds a row per map update, which shows the map growing during a flight and lets runs
be compared. On the Jetson or with `zed-usb`, run the same scripts with `python3 tools/<script>.py`
(drop `use_sim_time`).

### Worlds

The default world is `default_warehouse`, supplied by the professor. **It is not in this repo**:
copy `default_warehouse.sdf` into `~/PX4-Autopilot/Tools/simulation/gz/worlds/` (and any models
it uses into `.../gz/models/`). Without it, `make sim` waits forever at "Waiting for Gazebo world".

Any other world works by passing the same `WORLD` to `make sim` and `make bridge`:

```bash
WORLD=walls make sim
WORLD=walls make bridge
```

## ROS 2 topics

Inputs, from the bridge:

| Topic | Type | What |
|---|---|---|
| `/uav/depth/image` | `sensor_msgs/Image` | Depth image, meters (32-bit float) |
| `/uav/depth/camera_info` | `sensor_msgs/CameraInfo` | Depth intrinsics (fx = fy = 432.5, cx = 320, cy = 240) |
| `/uav/points` | `sensor_msgs/PointCloud2` | 3D point cloud (frame `camera_optical_frame`) |
| `/uav/rgb/image` | `sensor_msgs/Image` | Color image |
| `/uav/rgb/camera_info` | `sensor_msgs/CameraInfo` | Color intrinsics (same as depth) |
| `/uav/lidar/points` | `sensor_msgs/PointCloud2` | 3D lidar, 16 beams, 0.9-100 m (frame `lidar_link`) |
| `/uav/odom` | `nav_msgs/Odometry` | **Ground truth** (`x500_depth_0/odom` → `x500_depth_0/base_footprint`) |
| `/uav/imu` | `sensor_msgs/Imu` | IMU |
| `/clock` | `rosgraph_msgs/Clock` | Sim time |

From [`sim/odom_relay.py`](sim/odom_relay.py), which `make rtabmap` starts:

| Topic | Type | What |
|---|---|---|
| `/uav/odom_checked` | `nav_msgs/Odometry` | `/uav/odom` with invalid samples dropped, frames `odom` → `base_link`, covariance filled in. Used by `ODOMETRY=sim` |

Outputs, from RTAB-Map:

| Topic | Type | What |
|---|---|---|
| `/rtabmap/odom` | `nav_msgs/Odometry` | Estimated pose (visual odometry; not published with `ODOMETRY=sim`) |
| `/rtabmap/localization_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Pose in the map |
| `/rtabmap/mapPath` | `nav_msgs/Path` | Trajectory |
| `/rtabmap/map` | `nav_msgs/OccupancyGrid` | 2D occupancy map, 5 cm cells |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | 3D point-cloud map |
| `/rtabmap/octomap_full` | `octomap_msgs/Octomap` | 3D occupancy map (OctoMap) |

Frames: `map` → `odom` → `base_link` → `camera_optical_frame`. Anything closer than 0.3 m to the
camera is ignored (`Grid/RangeMin`, `Vis/MinDepth`), so the drone's own parts are never mapped.

Quick check that data is flowing:

```bash
ros2 topic hz /uav/points
ros2 topic echo /uav/odom --once
ros2 topic hz /rtabmap/odom
```

## Using the real ZED on a laptop (no NVIDIA GPU)

The ZED SDK needs an NVIDIA GPU, but the camera itself is a plain USB (UVC) stereo camera.
[`zed_usb/`](zed_usb) reads it directly, rectifies it with the camera's factory calibration
(downloaded once from Stereolabs by serial number into `~/.local/share/zed/settings/`), computes
depth on the CPU (OpenCV SGBM), and publishes the **same topics as the ZED ROS 2 wrapper**, so
RTAB-Map runs in the same RGB-D setup as on the drone.

```bash
make zed-usb              # camera upright
make zed-usb FLIP=true    # camera upside down (images turned, left/right swapped)
```

- Depth is classic stereo matching, not the SDK's neural depth: expect holes on plain walls/ceilings.
- VGA (672x376) at 15 fps: what the CPU depth keeps up with. No IMU (that needs the SDK).
- If the camera keeps disconnecting every few seconds while idle, USB autosuspend is the cause
  (it is stable while streaming). Disable it for Stereolabs devices once:
  `echo 'ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2b03", ATTR{power/control}="on"' | sudo tee /etc/udev/rules.d/99-zed-no-autosuspend.rules && sudo udevadm control --reload`
- Only VGA over a USB 2 link: for USB 3, use a USB 3 port and try flipping the USB-C plug.
- Camera problems never stop the publisher: it waits for an unplugged camera, reconnects after a USB
  reset, skips garbled frames, and retries the calibration download until the network is back.
  The launch file also restarts it if it ever exits.

## Running on the drone (Jetson + ZED)

The drone runs the same RTAB-Map mapping on its Jetson, with a Stereolabs ZED camera instead of
the simulated one, and MAVROS to talk to PX4. Files are in [`jetson/`](jetson):

| File | What |
|---|---|
| `install.sh` | Installs ROS 2 Humble, RTAB-Map, MAVROS, the ZED SDK, and builds the ZED ROS 2 wrapper |
| `zed_rtabmap.launch.py` | ZED camera → RTAB-Map, plus MAVROS |
| `zed_params.yaml` | ZED settings: publishes only RGB, depth, IMU and odometry |
| `run.sh` | Starts `zed_rtabmap.launch.py` |

Needs **JetPack 6** (L4T 36.3 or 36.4, Ubuntu 22.04): ROS 2 Humble packages are only built for 22.04.
`install.sh` reads the L4T version and picks the matching ZED SDK and wrapper
(L4T 36.4 → SDK 5.5 + wrapper v5.5.0, L4T 36.3 → SDK 5.2 + wrapper v5.2.2).
The ZED SDK needs the Jetson's NVIDIA GPU, so it is installed and built on the Jetson itself.

On the Jetson:

```bash
git clone https://github.com/ArnavSingh121/Proj_1.git && cd Proj_1
make jetson-check                 # what is installed / missing (changes nothing)
make jetson-deps                  # install everything (asks for sudo; the ZED wrapper build takes a while)
make jetson ZED_MODEL=zed2i       # use your model: zed, zedm, zed2, zed2i, zedx, zedxm, ...
```

- The ZED SDK keeps its AI module: in SDK 5 every depth mode is neural (`NEURAL_LIGHT` is used).
  Object detection, body tracking, spatial mapping and streaming are off in `zed_params.yaml`.
- ZED X / ZED X Mini (GMSL) also need Stereolabs' ZED Link driver, installed separately.
- Odometry comes from the ZED (visual-inertial). To use RTAB-Map's own instead:
  `ZED_MODEL=zed2i ./jetson/run.sh use_zed_odometry:=false`.
- PX4 is reached on the Jetson UART (TELEM2) at `/dev/ttyTHS1:921600`. For USB:
  `ZED_MODEL=zed2i ./jetson/run.sh fcu_url:=/dev/ttyACM0:57600`.
- Anything closer than 0.3 m to the camera is ignored (ZED `depth.min_depth` and RTAB-Map
  `Grid/RangeMin` / `Vis/MinDepth`), so the drone's own props and legs don't end up in the map.
- To watch the map from a laptop, use the same `ROS_DOMAIN_ID` on both and run `rtabmap_viz` there,
  or start with `rtabmap_viz:=true` if the Jetson has a screen.

## Tests

```bash
make test
```

It needs no sudo, network, camera or running simulation, and takes a few seconds. It checks:

- that the models, launch files, ZED settings and scripts agree with each other, and what the
  scripts and Makefile targets would run;
- the ZED USB publisher: calibration, depth, messages, and that camera failures (missing camera,
  failed or garbled reads, a USB disconnect, an offline or corrupt calibration download) are
  waited out instead of crashing it;
- that invalid or missing simulated odometry is detected and dropped (`sim/sensor_checks.py`);
- that maps survive a save/reload round trip and their statistics track incremental growth (`tools/`).

See [`tests/README.md`](tests/README.md) for details.

## Troubleshooting

| Problem | Fix |
|---|---|
| `make sim` stays at "Waiting for Gazebo world" | The world file is missing (see [Worlds](#worlds)), or its `<world name>` differs from its file name |
| PX4 refuses to arm: "Found 0 compass" in a custom world | A world with its own `<plugin>` list skips PX4's defaults: add the `gz-sim-magnetometer-system` and `gz-sim-air-speed-system` plugins |
| ROS 2 topics show up but no data arrives | The Fortress bridge is installed: use `ros-humble-ros-gzharmonic`. Or build ros_gz (humble branch, `GZ_VERSION=harmonic`) and run `ROS_OVERLAY=<ws>/install/setup.bash make bridge` |
| New terminals see no ROS 2 topics after the Wi-Fi changed | Restart the simulation, bridge and RTAB-Map |
| QGroundControl: "GLIBC_2.38 not found" | That is QGC 5.1+; `make qgc` installs v5.0.8 |
