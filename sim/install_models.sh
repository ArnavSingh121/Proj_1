#!/usr/bin/env bash
# Copies our custom Gazebo models into a PX4-Autopilot checkout.
# Originals are backed up as *.orig the first time.
# Usage: ./install_models.sh [path/to/PX4-Autopilot]   (default: ~/PX4-Autopilot)
set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
SRC="$(cd "$(dirname "$0")" && pwd)/models"
DST="$PX4_DIR/Tools/simulation/gz/models"

if [ ! -d "$DST" ]; then
  echo "error: $DST not found (is PX4_DIR right? did you run 'git submodule update --init'?)" >&2
  exit 1
fi

for f in $(cd "$SRC" && find . -type f); do
  target="$DST/$f"
  mkdir -p "$(dirname "$target")"
  if [ -f "$target" ] && [ ! -f "$target.orig" ]; then
    cp "$target" "$target.orig"
  fi
  cp "$SRC/$f" "$target"
  echo "installed $f"
done
