#!/bin/bash
set -e

cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh

if ! timeout 3 bash -c "</dev/tcp/192.168.137.248/11311" 2>/dev/null; then
  echo "ERROR: ROS master 192.168.137.248:11311 is unreachable."
  echo "Check hotspot connectivity and start roscore on car1 first."
  exit 1
fi

roslaunch cluster_bringup car3_slave.launch \
  lidar_port:="${1:-/dev/ttyUSB0}"
