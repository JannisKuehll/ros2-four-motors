#!/usr/bin/env bash
# Starts the motor demo on Linux (e.g. Raspberry Pi with Ubuntu).
# Usage: ./run.sh                      start the demo
#        ./run.sh --check-connections  only check the four boards (no motor moves)
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/lyrical/setup.bash}"
if [ ! -f "$ROS_SETUP" ]; then
  echo "ERROR: ROS 2 not found at $ROS_SETUP (set ROS_SETUP to your setup.bash)." >&2
  exit 1
fi
source "$ROS_SETUP"

# All nodes run in this one process, so ROS traffic can stay on this computer.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

cd "$(dirname "$0")"
exec python3 -u motor_demo.py "$@"
