#!/usr/bin/env bash
# Replay a recorded flight on this laptop: ./replay_flight.sh [flight | latest] [speed]   e.g. ./replay_flight.sh latest 2
# Flights are in ~/flight_recordings (one folder per flight, recorded from arm to disarm). Stop the sim first
# (./stop_env.sh). RTAB-Map rebuilds the map on screen; Ctrl-C stops.
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=1
EXTRA="$HOME/.local/ros-extra/setup.bash"  # RTAB-Map unpacked without sudo; skipped if absent
if [ -f "$EXTRA" ]; then export ROS_OVERLAY="$EXTRA"; fi
exec "$(dirname "$(dirname "$(readlink -f "$0")")")/tools/replay_flight.sh" "$@"
