# Proj_1 demo: every flight is recorded and mapped automatically

All commands run from `~/Desktop/Drone` (`cd ~/Desktop/Drone`), where the scripts are symlinks to
`Proj_1/laptop/`. Logs are in `Proj_1/logs/`.

## Before the demo
- **Plug the laptop in.** On battery the sim runs slower, and the laptop sleeps after 20 minutes idle.
- Close other heavy apps (the sim uses most of the CPU).
- Dry run once, the day before or an hour before: `./boot_env.sh --fly`, then `./stop_env.sh --all`.
  A cold first start takes up to about 5 minutes; after that, about 2.5 minutes from start to saved recording.

## 1. Tests (10 seconds)
```bash
cd Proj_1 && make test && cd ..        # expect: 215 passed, 1 skipped
```

## 2. A live flight, recorded with nobody touching anything
```bash
./boot_env.sh --fly
```
Windows open: Gazebo (the warehouse), the RTAB-Map viewer (the map) and QGroundControl.
The script boots everything, takes off, turns 360 degrees and lands. What to point out:
- **Arming** starts a recording and a fresh map at the takeoff point.
- During the turn, the map builds up in the RTAB-Map viewer.
- **Disarming** after landing saves everything. The script ends with
  `recording: /home/antonc13/flight_recordings/<date>_<time>`.

Flying by hand from QGroundControl works the same way: every arm to disarm is one recording.

## 3. Show what a flight leaves behind
```bash
ls ~/flight_recordings/latest/            # bag/  map.pgm  map.yaml  cloud.ply  poses.txt  rtabmap.db  flight.json
cat ~/flight_recordings/latest/flight.json
eog ~/flight_recordings/latest/map.pgm    # the 2D map (white = free, black = obstacle)
source ~/.local/ros-extra/setup.bash && rtabmap-databaseViewer ~/flight_recordings/latest/rtabmap.db   # full 3D map
```

## 4. Replay the flight (no simulator needed)
```bash
./stop_env.sh --all
./replay_flight.sh latest       # or: ./replay_flight.sh latest 2   (twice as fast)
```
The map rebuilds on screen from the recording, about 10 seconds after starting. Ctrl-C stops it.
The replay writes `replay.db` and never changes the recorded map.

## 5. How other groups use the map
```bash
make -C Proj_1 load-map MAP=$HOME/flight_recordings/latest/map.yaml   # serves it on /map
```
Any ROS 2 node can subscribe to `/map` (`nav_msgs/OccupancyGrid`, QoS transient local + reliable).

## 6. Optional: the real ZED camera (hallway)
```bash
./map.sh hallsdemo          # use a new name; halls.db mixes earlier walks
./save_map.sh hallsdemo     # in a second terminal, BEFORE Ctrl-C
# Ctrl-C in the first terminal, wait for "Bag closed"
./replay_map.sh hallsdemo
```

## If something goes wrong
- Anything hangs or fails: `./stop_env.sh --all`, then run the step again.
- The live flight fails: a finished recording is already there (`~/flight_recordings/2026-11-08_23-20-33`).
  Skip to steps 3 and 4.
- `map.sh` or `replay_flight.sh` says the sim is running: `./stop_env.sh --all` first.

## Honest limits, if asked
- The sim runs at about half real-time speed on this laptop.
- The real-drone path (Jetson + ZED + PX4) uses the same recorder with the ZED topics, but it hasn't
  been run on the drone hardware yet.
- Real-camera hallway maps mark too much as obstacle (cause not confirmed yet).
- The changes are on GitHub branch `flight-recordings`, not merged into `main` yet (needs a PR review).
