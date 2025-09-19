#!/bin/bash

WS=$1

source "$WS/src/robot_idl/common.sh" "$@"

# Subtract 2 from total cores
CORES=$(( $(nproc) - 2 ))

# Ensure at least 1 core is used
if [ "$CORES" -lt 1 ]; then
  CORES=1
fi

# List of required packages
DEPENDENCIES=(
    build-essential
    cmake
    git
    curl
    libeigen3-dev
    libyaml-cpp-dev
)

# Ensure script is run as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (e.g., sudo ./setup.sh)"
    exit 1
fi

# treat ros2 install special 
check_and_install "ros-humble-desktop" "install_ros"

# Loop through and install each
for pkg in "${DEPENDENCIES[@]}"; do
    check_and_install "$pkg"
done

# Ensure the directory exists
mkdir -p "$WS"/src

cd "$WS"
pwd
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_idl abv_controller 





