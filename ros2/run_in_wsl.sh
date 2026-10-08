#!/usr/bin/env bash
# Run a bench command in WSL with ROS 2 sourced:  run_in_wsl.sh python3 run.py --dut ros2 --b2b-dut
set -eo pipefail
source /opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash
cd "$(dirname "$0")/.."
exec "$@"
