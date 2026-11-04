#!/usr/bin/env bash
# Copies our custom Gazebo models (sim/models) and worlds (sim/worlds) into a PX4-Autopilot checkout.
# Originals are backed up as *.orig the first time.
# Usage: ./install_models.sh [path/to/PX4-Autopilot]   (default: ~/PX4-Autopilot)
set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
HERE="$(cd "$(dirname "$0")" && pwd)"
DST="$PX4_DIR/Tools/simulation/gz/models"

if [ ! -d "$DST" ]; then
  echo "error: $DST not found (is PX4_DIR right? did you run 'git submodule update --init'?)" >&2
  exit 1
fi

# install SRC DST LABEL: copy every file under SRC to the same path under DST.
install() {
  local src="$1" dst="$2" label="$3" f target
  # NUL-separated, so file names with spaces stay whole.
  while IFS= read -r -d '' f; do
    target="$dst/$f"
    mkdir -p "$(dirname "$target")"
    if [ -f "$target" ] && [ ! -f "$target.orig" ]; then
      cp "$target" "$target.orig"
    fi
    cp "$src/$f" "$target"
    echo "installed $label$f"
  done < <(cd "$src" && find . -type f -print0)
}

install "$HERE/models" "$DST" ""
if [ -d "$HERE/worlds" ]; then
  install "$HERE/worlds" "$PX4_DIR/Tools/simulation/gz/worlds" "world "
fi
