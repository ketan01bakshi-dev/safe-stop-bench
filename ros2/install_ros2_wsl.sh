#!/usr/bin/env bash
# Install ROS 2 (ros-base + rclpy) in WSL Ubuntu for the bench's ROS 2 adapter (v2.8).
# Official method (docs.ros.org, "Ubuntu (deb packages)"): ros2-apt-source package, then apt.
#   wsl -d Ubuntu -- bash "/mnt/<drive>/<path to the repo>/ros2/install_ros2_wsl.sh"
set -euo pipefail
. /etc/os-release
DISTRO=${ROS_DISTRO_NAME:-lyrical}          # Ubuntu 26.04 "resolute" -> ROS 2 Lyrical Luth
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -qq
sudo apt-get install -y -qq software-properties-common curl >/dev/null
sudo add-apt-repository -y universe >/dev/null
VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -sL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${VERSION_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt-get update -qq
sudo apt-get install -y -qq "ros-${DISTRO}-ros-base" >/dev/null
echo "installed: $(ls /opt/ros)"
bash -c "source /opt/ros/${DISTRO}/setup.bash && python3 -c 'import rclpy, std_msgs; print(\"rclpy ok\")' && ros2 --help >/dev/null && echo ros2-cli-ok"
