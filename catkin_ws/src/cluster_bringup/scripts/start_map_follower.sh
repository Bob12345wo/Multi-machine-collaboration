#!/bin/bash
set -e

cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh

if rosnode list >/tmp/cluster_nodes.$$ 2>/dev/null; then
  if grep -qE '^/robot[23]/move_base$' /tmp/cluster_nodes.$$; then
    echo "ERROR: move_base follower is already running."
    echo "Do not run start_map_follower.sh and nav_follower.launch together."
    rm -f /tmp/cluster_nodes.$$
    exit 1
  fi
fi
rm -f /tmp/cluster_nodes.$$

roslaunch cluster_bringup start_map_follower.launch
