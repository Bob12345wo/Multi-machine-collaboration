#!/bin/bash
set -e

cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh

roslaunch cluster_bringup car2_slave.launch \
  lidar_port:="${1:-/dev/ttyUSB0}"
