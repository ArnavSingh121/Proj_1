#!/usr/bin/env bash
# Run ON the drone's Jetson (JetPack 6, Ubuntu 22.04). Installs the drone side of Proj_1:
# ROS 2 Humble (base), RTAB-Map, MAVROS + GeographicLib data, the ZED SDK, and builds the ZED ROS 2
# wrapper. Every step is skipped if already done, so it is safe to re-run.
# Usage: ./jetson/install.sh           install (asks for sudo)
#        ./jetson/install.sh --check   report what is installed / missing, change nothing
#        JETSON_L4T=36.4 ./jetson/install.sh --check   preview the plan on a non-Jetson machine
set -eo pipefail

ZED_WS="${ZED_WS:-$HOME/zed_ws}"
ZED_REPO=https://github.com/stereolabs/zed-ros2-wrapper.git
ROS_SETUP=/opt/ros/humble/setup.bash
APT_PKGS="ros-humble-ros-base python3-colcon-common-extensions python3-rosdep \
  ros-humble-rtabmap-ros ros-humble-mavros ros-humble-mavros-extras zstd git curl build-essential"

# ZED SDK + matching ZED ROS 2 wrapper release, by Jetson L4T release (/etc/nv_tegra_release).
# L4T 36.4 = JetPack 6.1/6.2. L4T 36.3 = JetPack 6.0, for which the newest ZED SDK is 5.2.
zed_versions_for() {
  case "$1" in
    36.4) echo "5.5 v5.5.0" ;;
    36.3) echo "5.2 v5.2.2" ;;
    *) return 1 ;;
  esac
}

CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

ok() { printf '  ok       %s\n' "$1"; }
missing() { printf '  MISSING  %s\n' "$1"; MISSING=1; }
die() { echo "error: $*" >&2; exit 1; }

# ---- Platform ----
L4T="${JETSON_L4T:-}"
if [ -z "$L4T" ] && [ -f /etc/nv_tegra_release ]; then
  # e.g. "# R36 (release), REVISION: 4.3, ..." -> 36.4
  L4T=$(sed -nE 's/^# R([0-9]+) \(release\), REVISION: ([0-9]+)\..*/\1.\2/p' /etc/nv_tegra_release)
fi
if [ -z "$L4T" ]; then
  die "not a Jetson (no /etc/nv_tegra_release). Run this on the drone's Jetson, not the laptop."
fi
versions=$(zed_versions_for "$L4T") \
  || die "L4T $L4T is not supported. Needs JetPack 6 (L4T 36.3 or 36.4): ROS 2 Humble apt packages need Ubuntu 22.04."
read -r ZED_SDK ZED_TAG <<<"$versions"
ZED_URL="https://download.stereolabs.com/zedsdk/$ZED_SDK/l4t$L4T/jetsons"
echo "Jetson L4T $L4T -> ZED SDK $ZED_SDK, ZED ROS 2 wrapper $ZED_TAG"

installed_sdk() {
  local h=/usr/local/zed/include/sl/Camera.hpp
  [ -f "$h" ] || return 1
  printf '%s.%s\n' "$(sed -nE 's/^#define ZED_SDK_MAJOR_VERSION ([0-9]+).*/\1/p' "$h")" \
    "$(sed -nE 's/^#define ZED_SDK_MINOR_VERSION ([0-9]+).*/\1/p' "$h")"
}
wrapper_built() { [ -f "$ZED_WS/install/zed_wrapper/share/zed_wrapper/package.xml" ]; }

# ---- --check: report only ----
if [ "$CHECK" = 1 ]; then
  MISSING=0
  [ "$(uname -m)" = aarch64 ] && ok "arm64 (aarch64)" || missing "arm64 CPU (this is $(uname -m))"
  [ "$(lsb_release -rs 2>/dev/null)" = 22.04 ] && ok "Ubuntu 22.04" || missing "Ubuntu 22.04"
  [ -d /usr/local/cuda ] && ok "CUDA (/usr/local/cuda)" || missing "CUDA (JetPack: nvidia-jetpack)"
  for p in $APT_PKGS; do dpkg -s "$p" >/dev/null 2>&1 && ok "$p" || missing "$p"; done
  [ -d /usr/share/GeographicLib/geoids ] && ok "GeographicLib datasets" || missing "GeographicLib datasets"
  sdk=$(installed_sdk || true)
  [ "$sdk" = "$ZED_SDK" ] && ok "ZED SDK $sdk" || missing "ZED SDK $ZED_SDK (installed: ${sdk:-none})"
  wrapper_built && ok "ZED ROS 2 wrapper built in $ZED_WS" || missing "ZED ROS 2 wrapper $ZED_TAG in $ZED_WS"
  id -nG | grep -qw dialout && ok "user in dialout group (serial link to PX4)" || missing "user in dialout group"
  curl -sfIL -o /dev/null "$ZED_URL" && ok "ZED SDK installer reachable: $ZED_URL" \
    || missing "ZED SDK installer not reachable: $ZED_URL"
  echo
  [ "$MISSING" = 0 ] && echo "Everything is installed." || { echo "Run ./jetson/install.sh to install what is missing."; exit 1; }
  exit 0
fi

[ -n "${JETSON_L4T:-}" ] && die "JETSON_L4T is only for previewing with --check."
[ "$(uname -m)" = aarch64 ] || die "expected an arm64 Jetson, got $(uname -m)"
[ "$(lsb_release -rs)" = 22.04 ] || die "expected Ubuntu 22.04 (JetPack 6), got $(lsb_release -ds)"
sudo -v

# ---- ROS 2 Humble, RTAB-Map, MAVROS ----
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  echo "Adding the ROS 2 apt repository"
  sudo apt-get update && sudo apt-get install -y curl software-properties-common
  sudo add-apt-repository -y universe
  ver=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F tag_name | awk -F'"' '{print $4}')
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.$(lsb_release -cs)_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi
sudo apt-get update
# shellcheck disable=SC2086
sudo apt-get install -y $APT_PKGS
# The ZED wrapper needs the CUDA toolkit; JetPack images flashed without the SDK components lack it.
[ -d /usr/local/cuda ] || sudo apt-get install -y nvidia-jetpack
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update

if [ ! -d /usr/share/GeographicLib/geoids ]; then
  sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
fi

# ---- ZED SDK ----
# Keeps the AI module: in SDK 5 every depth mode is neural (NEURAL_LIGHT/NEURAL/NEURAL_PLUS), so
# skip_od_module would remove depth. Object detection and body tracking stay off in zed_params.yaml.
# Keeps the tools (ZED_Explorer, ZED_Diagnostic) to check the camera. Skips the Python API and sl_hub.
if [ "$(installed_sdk || true)" = "$ZED_SDK" ]; then
  echo "ZED SDK $ZED_SDK already installed"
else
  run="$HOME/Downloads/ZED_SDK_L4T${L4T}_v${ZED_SDK}.zstd.run"
  mkdir -p "$HOME/Downloads"
  [ -f "$run" ] || curl -fL -o "$run" "$ZED_URL"
  chmod +x "$run"
  "$run" -- silent skip_python skip_hub
fi

# ---- ZED ROS 2 wrapper (only zed_wrapper and what it depends on) ----
mkdir -p "$ZED_WS/src"
if [ ! -d "$ZED_WS/src/zed-ros2-wrapper" ]; then
  git clone --depth 1 --branch "$ZED_TAG" "$ZED_REPO" "$ZED_WS/src/zed-ros2-wrapper"
else
  have=$(git -C "$ZED_WS/src/zed-ros2-wrapper" describe --tags --always)
  [ "$have" = "$ZED_TAG" ] || echo "warning: $ZED_WS/src/zed-ros2-wrapper is at $have, expected $ZED_TAG (left as is)"
fi
if wrapper_built; then
  echo "ZED ROS 2 wrapper already built in $ZED_WS"
else
  source "$ROS_SETUP"
  (cd "$ZED_WS" && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble)
  # About 3 GB of RAM per compile job: an 8 GB Orin Nano builds 2 at a time instead of running out.
  jobs=$(( $(free -g | awk '/^Mem:/{print $2}') / 3 )); [ "$jobs" -ge 1 ] || jobs=1
  [ "$jobs" -le "$(nproc)" ] || jobs=$(nproc)
  (cd "$ZED_WS" && MAKEFLAGS="-j$jobs" colcon build --symlink-install --packages-up-to zed_wrapper \
    --parallel-workers 1 --cmake-args=-DCMAKE_BUILD_TYPE=Release)
fi

# ---- Serial link to PX4 (TELEM2 -> Jetson UART) ----
if ! id -nG "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
  echo "Added $USER to the dialout group: log out and back in before running."
fi

echo
echo "Done. Check with: ./jetson/install.sh --check"
echo "Run with:         make jetson ZED_MODEL=<zed2i|zedm|zedx|...>"
