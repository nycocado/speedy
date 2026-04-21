#!/bin/bash
# ==============================================================================
# SPEEDY IOT SYSTEM
# Script:  04_ros2_jazzy.sh
# Purpose: ROS 2 Jazzy Jalisco Base Installation
# ==============================================================================
set -e

echo "====================================================================="
echo "[SPEEDY-PI] STAGE 4: ROS 2 Framework Installation"
echo "====================================================================="

echo "[INFO] Configuring system repositories (universe)..."
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y

echo "[INFO] Adding official ROS 2 repository keys..."
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "[INFO] Installing ROS 2 Jazzy Base and development tools..."
sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools python3-rosdep -y

echo "[INFO] Initializing and updating rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo "---------------------------------------------------------------------"
echo "[SUCCESS] Stage 4 complete. ROS 2 Jazzy is installed."
echo "---------------------------------------------------------------------"
