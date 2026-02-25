# Makefile — install ROS 1 Noetic, Gazebo Classic, PX4, QGC on Ubuntu 20.04 (Focal)
# Strict bash; path & arch aware.

# USAGE:
#   make all
#   source ~/.bashrc
#   make gazebo_classic_fix_build

SHELL := /bin/bash
.ONESHELL:
SHELLFLAGS := -eu -o pipefail -c

ARCH      := $(shell dpkg --print-architecture)   # amd64 or arm64
CODENAME  := $(shell lsb_release -cs)             # focal expected
HOME_DIR  := $(shell echo $$HOME)
QGC_VERSION ?= v4.2.8

# Path to your Morphable-Drone repo for tag fix / external build clean
MORPH_REPO ?= $(HOME_DIR)/iRaL/Morphable-Drone

QGC_URL_AMD64 := https://github.com/mavlink/qgroundcontrol/releases/download/$(QGC_VERSION)/QGroundControl.AppImage
QGC_INSTALL_DIR := $(HOME_DIR)/Applications
QGC_BIN := $(QGC_INSTALL_DIR)/QGroundControl.AppImage
QGC_WRAPPER := /usr/local/bin/qgroundcontrol

ROS1_KEYRING := /usr/share/keyrings/ros-archive-keyring.gpg
OSRF_KEYRING := /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Install Gazebo ROS(1) bridge plugins (libgazebo_ros_*) (set to 0 to skip)
INSTALL_GAZEBO_ROS ?= 1

.PHONY: help all info base ros gazebo gazebo_classic gazebo_classic_fix_build px4 px4_build qgc clean_caches tags fix_tags_px4 fix_tags_morph

help:
	@echo "Targets:"
	@echo "  make all                    -> base + ros (Noetic) + gazebo_classic + clean external + px4 + tags + qgc"
	@echo "  make info                   -> print detected arch/codename"
	@echo "  make base                   -> base dev tools, multimedia, colcon"
	@echo "  make ros                    -> ROS 1 Noetic (Focal) + dev tools"
	@echo "  make gazebo_classic         -> Gazebo Classic 11 + env setup"
	@echo "  make gazebo                 -> alias for gazebo_classic"
	@echo "  make gazebo_classic_fix_build -> clean external sitl_gazebo build in MORPH_REPO"
	@echo "  make px4                    -> clone PX4 and install toolchains/deps"
	@echo "  make px4_build              -> build PX4 SITL (optional; heavy)"
	@echo "  make tags                   -> fix PX4 upstream tags + add local v* tag in MORPH_REPO"
	@echo "  make qgc                    -> QGroundControl (AppImage on amd64, Flatpak on arm64)"
	@echo "  make clean_caches           -> apt & pip cache cleanup"

all: info base ros gazebo_classic gazebo_classic_fix_build px4 tags qgc
	@echo "✅ Done. Open a new shell or 'source ~/.bashrc' to pick up environments."

info:
	@echo "ARCH     = $(ARCH)"
	@echo "CODENAME = $(CODENAME) (expected: focal)"

base:
	@echo "🔧 Installing base tools, multimedia, and colcon..."
	sudo -v
	sudo apt-get update
	sudo apt-get install -y --no-install-recommends \
	  locales tzdata ca-certificates curl wget gnupg lsb-release software-properties-common \
	  build-essential git cmake ninja-build ccache pkg-config \
	  python3 python3-pip python3-jinja2 python3-empy python3-numpy python3-yaml python3-dev \
	  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libfuse2 \
	  usbutils udev modemmanager x11-apps mesa-utils
	sudo locale-gen en_US.UTF-8; sudo update-locale LANG=en_US.UTF-8
	python3 -m pip install -U pip
	python3 -m pip install -U colcon-common-extensions
	ccache -M 512M || true

# ---- ROS 1 Noetic (Focal) ----
ros:
	@if [ "$(CODENAME)" != "focal" ]; then \
	  echo "⚠️  This target is for Ubuntu 20.04 (focal). Detected $(CODENAME)."; \
	fi
	@echo "🪪 Adding ROS 1 apt repository (keyring)…"
	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o $(ROS1_KEYRING)
	echo "deb [signed-by=$(ROS1_KEYRING)] http://packages.ros.org/ros/ubuntu $(CODENAME) main" \
	  | sudo tee /etc/apt/sources.list.d/ros1.list >/dev/null
	sudo apt-get update
	@echo "🤖 Installing ROS 1 Noetic Desktop Full + dev tools…"
	sudo apt-get install -y \
	  ros-noetic-desktop-full \
	  python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
	  python3-catkin-tools
	@sudo rosdep init 2>/dev/null || true
	@rosdep update || true
	@grep -qxF 'source /opt/ros/noetic/setup.bash' $(HOME_DIR)/.bashrc || \
	  echo 'source /opt/ros/noetic/setup.bash' >> $(HOME_DIR)/.bashrc

# ---- Gazebo Classic 11 ----
gazebo_classic:
	@echo "🏗️  Installing Gazebo Classic 11 for $(ARCH)/$(CODENAME)…"
	sudo apt-get update
	@echo "📦 Using OSRF stable repository for Gazebo Classic…"
	curl -sSL https://packages.osrfoundation.org/gazebo.gpg | sudo tee $(OSRF_KEYRING) >/dev/null
	sudo chmod 644 $(OSRF_KEYRING)
	echo "deb [arch=$$(dpkg --print-architecture) signed-by=$(OSRF_KEYRING)] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(CODENAME) main" \
	  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
	sudo apt-get update
	sudo apt-get install -y --no-install-recommends gazebo libgazebo11 libgazebo-dev
	@if [ "$(INSTALL_GAZEBO_ROS)" = "1" ]; then \
	  echo "🔌 Installing ROS(1) Gazebo plugins (libgazebo_ros_*)…"; \
	  sudo apt-get install -y --no-install-recommends ros-noetic-gazebo-ros-pkgs; \
	fi
	@grep -qxF 'source /usr/share/gazebo/setup.sh' $(HOME_DIR)/.bashrc || \
	  echo 'source /usr/share/gazebo/setup.sh' >> $(HOME_DIR)/.bashrc
	@if [ -d /usr/lib/aarch64-linux-gnu/cmake/gazebo ]; then \
	  grep -qxF 'export gazebo_DIR=/usr/lib/aarch64-linux-gnu/cmake/gazebo' $(HOME_DIR)/.bashrc || \
	    echo 'export gazebo_DIR=/usr/lib/aarch64-linux-gnu/cmake/gazebo' >> $(HOME_DIR)/.bashrc; \
	fi
	@echo "✅ Gazebo Classic installed. Open a new shell or 'source ~/.bashrc'."

# convenience alias
gazebo: gazebo_classic

# Clean external sitl_gazebo so CMake re-detects Classic paths
gazebo_classic_fix_build:
	@echo "🧼 Cleaning external sitl_gazebo build so CMake re-detects Classic…"
	@if [ -d $(MORPH_REPO) ]; then \
	  rm -rf $(MORPH_REPO)/build/px4_sitl_default/build_gazebo \
	         $(MORPH_REPO)/build/px4_sitl_default/external/Stamp/sitl_gazebo || true; \
	else \
	  echo "⚠️  MORPH_REPO='$(MORPH_REPO)' not found; skip clean."; \
	fi

px4:
	@echo "🛩️  Cloning PX4-Autopilot and installing toolchains…"
	sudo apt-get update
	if [ ! -d $(HOME_DIR)/PX4-Autopilot ]; then \
	  git clone https://github.com/PX4/PX4-Autopilot.git --recursive $(HOME_DIR)/PX4-Autopilot; \
	else \
	  git -C $(HOME_DIR)/PX4-Autopilot submodule update --init --recursive; \
	fi
	bash $(HOME_DIR)/PX4-Autopilot/Tools/setup/ubuntu.sh
	@echo "➡️  Optional: 'make px4_build' to compile SITL (heavy)."

px4_build:
	@echo "🛠️  Building PX4 SITL (this can take a while)…"
	make -C $(HOME_DIR)/PX4-Autopilot px4_sitl

# ---- Tag fixes (unchanged) ----
tags: fix_tags_px4 fix_tags_morph
	@echo "✅ Tag fixes complete."

fix_tags_px4:
	@echo "🔖 Ensuring PX4-Autopilot has upstream tags…"
	if [ -d $(HOME_DIR)/PX4-Autopilot/.git ]; then \
	  git -C $(HOME_DIR)/PX4-Autopilot remote add upstream https://github.com/PX4/PX4-Autopilot.git 2>/dev/null || true; \
	  git -C $(HOME_DIR)/PX4-Autopilot fetch upstream --tags || true; \
	  if [ -f $(HOME_DIR)/PX4-Autopilot/.git/shallow ]; then \
	    git -C $(HOME_DIR)/PX4-Autopilot fetch --unshallow --tags || true; \
	  fi; \
	  echo "   -> PX4 describe:"; git -C $(HOME_DIR)/PX4-Autopilot describe --tags --dirty --always || true; \
	else \
	  echo "   -> PX4-Autopilot not present; skipping."; \
	fi

fix_tags_morph:
	@echo "🔖 Ensuring local v* tag exists for $(MORPH_REPO) (to satisfy PX4 version script)…"
	if [ -d $(MORPH_REPO)/.git ]; then \
	  if ! git -C $(MORPH_REPO) describe --tags --abbrev=0 2>/dev/null | grep -q '^v'; then \
	    echo "   -> adding local tag v1.16.0 at HEAD (local-only)"; \
	    git -C $(MORPH_REPO) tag -f v1.16.0; \
	  else \
	    echo "   -> v* tag already present; OK."; \
	  fi; \
	  echo "   -> Morphable describe:"; git -C $(MORPH_REPO) describe --tags --dirty --always || true; \
	else \
	  echo "   -> $(MORPH_REPO) not found; skipping."; \
	fi

qgc:
	@if [ "$(ARCH)" = "amd64" ]; then \
	  echo "🖥️  Installing QGroundControl AppImage for amd64…"; \
	  sudo apt-get update && sudo apt-get install -y --no-install-recommends libfuse2 \
	    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl; \
	  mkdir -p $(QGC_INSTALL_DIR); \
	  wget -qO $(QGC_BIN) "$(QGC_URL_AMD64)"; \
	  chmod +x $(QGC_BIN); \
	  echo -e '#!/usr/bin/env bash\nexec "$(QGC_BIN)" "$$@"' | sudo tee $(QGC_WRAPPER) >/dev/null; \
	  sudo chmod +x $(QGC_WRAPPER); \
	  echo "✅ QGC installed. Run: qgroundcontrol"; \
	else \
	  echo "🖥️  Installing QGroundControl via Flatpak for $(ARCH)…"; \
	  sudo apt-get update && sudo apt-get install -y --no-install-recommends flatpak software-properties-common; \
	  if ! command -v flatpak >/dev/null 2>&1; then \
	    echo "Installing Flatpak…"; \
	    sudo apt-get install -y flatpak; \
	  fi; \
	  V=$$(flatpak --version 2>/dev/null | awk '{print $$2}'); \
	  if [ -z "$$V" ] || ! dpkg --compare-versions "$$V" ge "1.12.0"; then \
	    echo "Updating Flatpak from PPA (current=$$V)…"; \
	    sudo add-apt-repository -y ppa:flatpak/stable; \
	    sudo apt-get update; \
	    sudo apt-get install -y flatpak; \
	  fi; \
	  sudo flatpak remote-delete flathub 2>/dev/null || true; \
	  sudo flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo; \
	  sudo flatpak install -y flathub org.mavlink.qgroundcontrol; \
	  echo -e '#!/usr/bin/env bash\nexec flatpak run --device=all org.mavlink.qgroundcontrol "$$@"' | sudo tee $(QGC_WRAPPER) >/dev/null; \
	  sudo chmod +x $(QGC_WRAPPER); \
	  echo "✅ QGC installed. Run: qgroundcontrol"; \
	fi


clean_caches:
	@echo "🧹 Cleaning apt & pip caches…"
	sudo apt-get clean
	sudo rm -rf /var/lib/apt/lists/*
	python3 -m pip cache purge || true
