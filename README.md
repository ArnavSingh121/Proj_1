# Proj_1: UAV Mapping and Localization System

CS 4800 (Software Engineering), Cal Poly Pomona.

A simulated drone (PX4 + Gazebo Harmonic) streams odometry and depth data into ROS 2.
We estimate its pose and build a 3D map with RTAB-Map + OctoMap.

## Environment

- Ubuntu 22.04, ROS 2 Humble
- PX4-Autopilot built from source, Gazebo Harmonic 8.x
- RTAB-Map, OctoMap server, MAVROS, QGroundControl

> Use `ros-humble-ros-gzharmonic`, **not** the default `ros-humble-ros-gz-bridge`.
> The default one is built for Gazebo Fortress: the topics show up but no data flows.

## Install dependencies

The [Makefile](Makefile) installs everything on a fresh Ubuntu 22.04 machine.
It skips steps that are already done, so it is safe to re-run.

```bash
make check   # see what is installed / missing (changes nothing)
make deps    # install everything (asks for sudo; PX4 build takes a while)
make help    # list individual targets (ros, gazebo, slam, px4, models, qgc, ...)
```

PX4 is cloned to `~/PX4-Autopilot` at the commit we validated (`PX4_REF` in the Makefile).
Use a different folder with `make deps PX4_DIR=/path/to/PX4-Autopilot`.

## One-time setup: install our drone models into PX4

Our drone needs custom Gazebo models, in [`sim/models/`](sim/models):

| Model | Change |
|---|---|
| `x500_depth` | Adds the OdometryPublisher plugin (ground-truth odometry, 3D) and a 3D lidar |
| `OakD-Lite` | 640x480 RGB image, frame id `camera_optical_frame` |
| `lidar_3d` | 3D lidar model (not in stock PX4) |

`make deps` installs them. To re-copy them by hand (originals are backed up as `*.orig`):

```bash
make models   # same as ./sim/install_models.sh ~/PX4-Autopilot
```

## Running the simulation

Terminal 1 starts PX4 + Gazebo in the warehouse world. The first start can take a minute or two.

```bash
make sim      # same as ./sim/launch_sim.sh ~/PX4-Autopilot
```

Terminal 2 starts the Gazebo → ROS 2 bridge. Leave it running.

```bash
make bridge   # same as ./sim/bridge.sh
```

## ROS 2 topics

| Topic | Type | What |
|---|---|---|
| `/uav/depth/image` | `sensor_msgs/Image` | Depth image |
| `/uav/depth/camera_info` | `sensor_msgs/CameraInfo` | Depth intrinsics (fx = fy = 432.5, cx = 320, cy = 240) |
| `/uav/points` | `sensor_msgs/PointCloud2` | 3D point cloud (frame `camera_optical_frame`) |
| `/uav/rgb/image` | `sensor_msgs/Image` | Color image |
| `/uav/rgb/camera_info` | `sensor_msgs/CameraInfo` | Color intrinsics |
| `/uav/odom` | `nav_msgs/Odometry` | **Ground truth** (`x500_depth_0/odom` → `x500_depth_0/base_footprint`), only for accuracy evaluation |
| `/uav/imu` | `sensor_msgs/Imu` | IMU |
| `/clock` | `rosgraph_msgs/Clock` | Sim time |

Quick check that data is flowing:

```bash
ros2 topic hz /uav/points
ros2 topic echo /uav/odom --once
```
