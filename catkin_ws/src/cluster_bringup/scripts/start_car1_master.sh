#!/bin/bash
set -e

cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh

roslaunch cluster_bringup car1_master.launch \
  lidar_port:="${1:-/dev/ttyUSB0}" \
  start_map_follower:=false
