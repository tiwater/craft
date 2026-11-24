#!/usr/bin/env bash
set -euo pipefail

echo "[CRAFT] Installing Gazebo Fortress (OGRE2 PBR + DART) & ROS 2 Humble bridge..."
sudo apt-get update
sudo apt-get install -y ignition-fortress ros-humble-ros-gz

echo "[CRAFT] Verifying installation..."
ign gazebo --versions
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E '^ros_gz'
echo "[CRAFT] Gazebo Fortress + ROS 2 Humble ready."
