# Proj_1: UAV Mapping and Localization System
# Installs every dependency on a fresh Ubuntu 22.04 (jammy) machine.
#
#   make help     list targets
#   make deps     install everything (asks for sudo)
#   make check    report what is installed / missing (no sudo, changes nothing)
#   make test     run the test suite (tests/README.md)
#   make jetson-deps / jetson-check / jetson   the same for the drone's Jetson (ZED + RTAB-Map + MAVROS)

SHELL := /bin/bash
.DEFAULT_GOAL := help

# ---- Settings (override on the command line, e.g. make px4 PX4_DIR=~/src/PX4) ----
PX4_DIR  ?= $(HOME)/PX4-Autopilot
PX4_REPO ?= https://github.com/PX4/PX4-Autopilot.git
# PX4 commit the team has validated (v1.18.0-beta1-603). Change only on purpose.
PX4_REF  ?= 2c47a16f563b86d895b394a0f3dd8848b0ccdd78
# QGC 5.0.x is the last series built for Ubuntu 22.04; "latest" (5.1+) needs glibc 2.38 (Ubuntu 24.04).
QGC_URL  ?= https://github.com/mavlink/qgroundcontrol/releases/download/v5.0.8/QGroundControl-x86_64.AppImage
QGC_APP  ?= $(HOME)/QGroundControl-v5.0.8-x86_64.AppImage
# ZED camera model on the drone, for `make jetson` (zed, zedm, zed2, zed2i, zedx, zedxm, ...)
ZED_MODEL ?=
# `make sim`: true lets PX4 arm without QGroundControl (pxh> commander takeoff)
NO_GCS ?= false
# `make zed-usb`: true if the ZED is upside down
FLIP ?= false
# `make rtabmap`: odometry source (visual | sim), simulated 3D lidar (true | false), fresh map or reload
ODOMETRY ?= visual
LIDAR    ?= false
NEW_MAP  ?= true
# `make rtabmap`: map update rate in Hz, 0 = every camera frame (1 eases the CPU on battery)
RATE     ?= 0
# `make save-map` / `make load-map`: occupancy map files (<OUT>.pgm + <OUT>.yaml)
OUT ?= $(HOME)/maps/map
MAP ?= $(OUT).yaml
# `make test`: extra pytest arguments, e.g. make test PYTEST_ARGS='-k bridge -v'
PYTEST_ARGS ?=

ROS_SETUP := /opt/ros/humble/setup.bash
GZ_KEYRING := /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
GZ_LIST    := /etc/apt/sources.list.d/gazebo-stable.list

# Base tools
BASE_PKGS := git curl wget lsb-release gnupg software-properties-common build-essential

# ROS 2 Humble + build/test tools
ROS_PKGS := \
	ros-humble-desktop \
	python3-colcon-common-extensions \
	python3-rosdep \
	python3-pytest \
	ros-humble-ament-cmake-pytest \
	ros-humble-tf2-tools

# Gazebo Harmonic + its ROS 2 bridge.
# ros-humble-ros-gzharmonic comes from the Gazebo (OSRF) repo. Do NOT use the default
# ros-humble-ros-gz-bridge: it is built for Gazebo Fortress and no data flows.
GZ_PKGS := gz-harmonic ros-humble-ros-gzharmonic

# SLAM + mapping + PX4 link
SLAM_PKGS := ros-humble-rtabmap-ros ros-humble-octomap-server ros-humble-mavros

# QGroundControl runtime libraries (from the QGC install guide)
QGC_PKGS := gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
	libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev

ALL_PKGS := $(BASE_PKGS) $(ROS_PKGS) $(GZ_PKGS) $(SLAM_PKGS) $(QGC_PKGS)

.PHONY: help deps not-jetson base ros-repo gz-repo ros gazebo slam mavros-geo px4 models qgc \
	check test sim reset-drone camera bridge rtabmap save-map load-map map-monitor zed-usb jetson-deps jetson-check jetson

help: ## Show this help
	@grep -E '^[a-zA-Z0-9_-]+:.*?## ' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}'

deps: not-jetson base ros gazebo slam mavros-geo px4 models qgc ## Install everything (sudo)
	@echo
	@echo "All dependencies installed. Run 'make check' to verify."
	@echo "Add 'source $(ROS_SETUP)' to your ~/.bashrc if it is not there yet."

# The simulation stack (Gazebo, PX4 SITL, QGC AppImage) is x86-only.
not-jetson:
	@if [ "$$(uname -m)" = aarch64 ]; then \
		echo "This is an arm64 machine (the Jetson?): use 'make jetson-deps' instead."; exit 1; fi

base: ## Base tools (git, curl, compilers)
	sudo apt-get update
	sudo apt-get install -y $(BASE_PKGS)

ros-repo: base ## Add the ROS 2 apt repository
	@if dpkg -s ros2-apt-source >/dev/null 2>&1; then \
		echo "ROS 2 apt source already installed"; \
	else \
		sudo add-apt-repository -y universe; \
		ver=$$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
			| grep -F tag_name | awk -F'"' '{print $$4}'); \
		curl -L -o /tmp/ros2-apt-source.deb \
			"https://github.com/ros-infrastructure/ros-apt-source/releases/download/$${ver}/ros2-apt-source_$${ver}.$$(lsb_release -cs)_all.deb"; \
		sudo dpkg -i /tmp/ros2-apt-source.deb; \
	fi

gz-repo: base ## Add the Gazebo (OSRF) apt repository
	@if [ -f $(GZ_LIST) ]; then \
		echo "Gazebo apt source already installed"; \
	else \
		sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o $(GZ_KEYRING); \
		echo "deb [arch=$$(dpkg --print-architecture) signed-by=$(GZ_KEYRING)] http://packages.osrfoundation.org/gazebo/ubuntu-stable $$(lsb_release -cs) main" \
			| sudo tee $(GZ_LIST) >/dev/null; \
	fi

ros: ros-repo ## ROS 2 Humble desktop + colcon, rosdep, pytest
	sudo apt-get update
	sudo apt-get install -y $(ROS_PKGS)
	@[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
	rosdep update

gazebo: ros-repo gz-repo ## Gazebo Harmonic + ros_gz bridge for Harmonic
	sudo apt-get update
	sudo apt-get install -y $(GZ_PKGS)

slam: ros-repo ## RTAB-Map, OctoMap server, MAVROS
	sudo apt-get update
	sudo apt-get install -y $(SLAM_PKGS)

mavros-geo: slam ## GeographicLib datasets MAVROS needs at runtime
	@if [ -d /usr/share/GeographicLib/geoids ]; then \
		echo "GeographicLib datasets already installed"; \
	else \
		sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh; \
	fi

px4: base ## Clone PX4 at PX4_REF, run its setup script, build SITL
	@if [ ! -d $(PX4_DIR)/.git ]; then \
		git clone $(PX4_REPO) $(PX4_DIR) && \
		git -C $(PX4_DIR) checkout $(PX4_REF) && \
		git -C $(PX4_DIR) submodule update --init --recursive; \
	else \
		echo "PX4 already at $(PX4_DIR) ($$(git -C $(PX4_DIR) describe --tags --always))"; \
	fi
	bash $(PX4_DIR)/Tools/setup/ubuntu.sh --no-nuttx
	$(MAKE) -C $(PX4_DIR) px4_sitl

models: ## Copy our custom Gazebo models and worlds (sim/models, sim/worlds) into PX4
	./sim/install_models.sh $(PX4_DIR)

qgc: ## QGroundControl AppImage + its libraries
	sudo apt-get install -y $(QGC_PKGS)
	@if [ -f $(QGC_APP) ]; then \
		echo "QGroundControl already at $(QGC_APP)"; \
	else \
		wget -O $(QGC_APP) $(QGC_URL); \
	fi
	chmod +x $(QGC_APP)

check: ## Report what is installed and what is missing (no sudo)
	@missing=0; \
	for p in $(ALL_PKGS); do \
		if dpkg -s $$p >/dev/null 2>&1; then printf "  ok       %s\n" $$p; \
		else printf "  MISSING  %s\n" $$p; missing=1; fi; \
	done; \
	if dpkg -s ros-humble-ros-gz-bridge >/dev/null 2>&1; then \
		echo "  WARNING  ros-humble-ros-gz-bridge (Fortress) is installed; use ros-humble-ros-gzharmonic"; fi; \
	if [ -d /usr/share/GeographicLib/geoids ]; then echo "  ok       GeographicLib datasets"; \
	else echo "  MISSING  GeographicLib datasets (make mavros-geo)"; missing=1; fi; \
	if [ -x $(PX4_DIR)/build/px4_sitl_default/bin/px4 ]; then \
		echo "  ok       PX4 SITL build ($$(git -C $(PX4_DIR) describe --tags --always))"; \
	else echo "  MISSING  PX4 SITL build in $(PX4_DIR) (make px4)"; missing=1; fi; \
	if (cd sim/models && find . -type f) | while read -r f; do \
			cmp -s sim/models/$$f $(PX4_DIR)/Tools/simulation/gz/models/$$f || exit 1; done; then \
		echo "  ok       custom models installed in PX4"; \
	else echo "  MISSING  custom models in PX4 (make models)"; missing=1; fi; \
	if (cd sim/worlds && find . -type f) | while read -r f; do \
			cmp -s sim/worlds/$$f $(PX4_DIR)/Tools/simulation/gz/worlds/$$f || exit 1; done; then \
		echo "  ok       custom worlds installed in PX4"; \
	else echo "  MISSING  custom worlds in PX4 (make models)"; missing=1; fi; \
	if [ -x $(QGC_APP) ]; then echo "  ok       QGroundControl"; \
	else echo "  MISSING  QGroundControl (make qgc)"; missing=1; fi; \
	echo; \
	if [ $$missing = 0 ]; then echo "Everything is installed."; \
	else echo "Some dependencies are missing. Run 'make deps' or the target shown."; exit 1; fi

# ROS is sourced for rclpy (publisher unit tests) and the launcher scripts; without it those tests skip.
# PYTHONNOUSERSITE=1: the system OpenCV breaks if a NumPy 2 in ~/.local is picked up.
test: ## Run the tests: config consistency, ZED USB publisher, scripts + Makefile (no sudo, no network)
	@if [ -f $(ROS_SETUP) ]; then source $(ROS_SETUP); fi; \
	PYTHONNOUSERSITE=1 python3 -m pytest tests $(PYTEST_ARGS)

sim: ## Run PX4 + Gazebo in the warehouse world (terminal 1), e.g. make sim NO_GCS=true
	NO_GCS=$(NO_GCS) ./sim/launch_sim.sh $(PX4_DIR)

reset-drone: ## Put the drone back upright at its start point without restarting the sim (e.g. after a crash)
	./sim/reset_drone.sh $(PX4_DIR)

camera: ## Turn Gazebo's follow camera around the drone with the gamepad (D-pad, LT/RT, Y)
	python3 sim/camera_orbit.py

bridge: ## Run the Gazebo -> ROS 2 bridge (terminal 2)
	./sim/bridge.sh

rtabmap: ## Run RTAB-Map on the bridged topics (terminal 3), e.g. make rtabmap ODOMETRY=sim LIDAR=true NEW_MAP=false RATE=1
	./sim/rtabmap.sh odometry:=$(ODOMETRY) lidar:=$(LIDAR) new_map:=$(NEW_MAP) detection_rate:=$(RATE)

save-map: ## Save the current 2D occupancy map (.pgm + .yaml, default ~/maps/map), e.g. make save-map OUT=~/maps/warehouse
	source $(ROS_SETUP) && python3 tools/save_map.py --ros-args -p out:=$(OUT)

load-map: ## Publish a saved occupancy map on /map, e.g. make load-map MAP=~/maps/warehouse.yaml
	source $(ROS_SETUP) && python3 tools/load_map.py --ros-args -p map:=$(MAP)

map-monitor: ## Log map growth to ~/.ros/map_growth.csv while RTAB-Map runs (sim time)
	source $(ROS_SETUP) && python3 tools/map_monitor.py --ros-args -p use_sim_time:=true

zed-usb: ## Run RTAB-Map on a ZED plugged into this laptop (no NVIDIA GPU), e.g. make zed-usb FLIP=true
	./zed_usb/run.sh flip:=$(FLIP)

# ---- On the drone's Jetson (JetPack 6) ----
jetson-deps: ## (Jetson) ROS 2, RTAB-Map, MAVROS, ZED SDK + build the ZED ROS 2 wrapper (sudo)
	./jetson/install.sh

jetson-check: ## (Jetson) Report what is installed / missing (no sudo)
	./jetson/install.sh --check

jetson: ## (Jetson) Run ZED + RTAB-Map + MAVROS, e.g. make jetson ZED_MODEL=zed2i
	ZED_MODEL=$(ZED_MODEL) ./jetson/run.sh
