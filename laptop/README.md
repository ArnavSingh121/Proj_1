# Laptop scripts

Run the whole stack on one laptop with one command each, instead of the four terminals in the main
README. Written for Anton's laptop, where RTAB-Map, MAVROS, the Harmonic `ros_gz_bridge` and
QGroundControl were installed without sudo (`~/.local/ros-extra`, `~/ros_gz_harmonic_ws`,
`~/.local/qgc`). Those overlays are skipped when they are not there, so the scripts also work with the
apt installs from `make deps`. Logs go to `logs/` in the repo (`PROJ1_LOGS` to change it).

| Script | What |
|---|---|
| `boot_env.sh [world] [--fly] [--headless]` | QGroundControl, PX4 + Gazebo, the bridge, RTAB-Map (with the flight recorder) and MAVROS, each in the background. `--fly` flies a demo flight: take off, turn 360 degrees, land; the recording is saved on disarm |
| `stop_env.sh [--all]` | Stops what `boot_env.sh` started (`--all`: QGroundControl too) |
| `replay_flight.sh [flight] [speed]` | Replays a recorded flight (`tools/replay_flight.sh`) |
| `map.sh <name> [flip] [localize] [nobag]` | Maps with the real ZED on this laptop into `maps/<name>.db` and records a bag; Ctrl-C stops |
| `save_map.sh <name>` | Saves that map to `maps/<name>_<time>/` (run it before stopping `map.sh`) |
| `replay_map.sh <name> [speed] [bag]` | Replays the newest `map.sh` bag through RTAB-Map |
| `DEMO_GUIDE.md` | Step-by-step demo |

To run them from another folder, symlink them, e.g. `ln -s ~/Desktop/Drone/Proj_1/laptop/*.sh ~/Desktop/Drone/`.
They find the repo through the symlink.
