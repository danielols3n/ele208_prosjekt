#!/usr/bin/env bash
# Bygger ROS 2-pakken og starter robotmodellen i Gazebo.
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
if ! python3 -c 'import xacro' >/dev/null 2>&1; then
  echo 'Xacro mangler. Installer med: sudo apt install ros-jazzy-xacro' >&2
  exit 1
fi
if pgrep -af '^gz sim' ; then
  echo 'Gazebo kjører allerede. Avslutt den før denne varianten startes.' >&2
  exit 1
fi
colcon build --symlink-install --base-paths src
source install/setup.bash
exec ros2 launch lift_window_sim sim.launch.py
