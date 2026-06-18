#!/bin/bash
set -e

cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh

if [ "${1:-}" != "--force-follower-nav" ]; then
  echo "ERROR: follower-side move_base navigation is deprecated for close formation."
  echo "Use this instead:"
  echo "  ~/agilex_ws/src/cluster_bringup/scripts/start_map_follower.sh"
  echo "For car1-only navigation after map followers are running:"
  echo "  ~/agilex_ws/src/cluster_bringup/scripts/start_car1_nav.sh"
  echo
  echo "If you still need the old experimental robot2/robot3 move_base test, run:"
  echo "  $0 --force-follower-nav [roslaunch args...]"
  exit 1
fi
shift

missing_pkg=0
for pkg in move_base global_planner dwa_local_planner costmap_2d; do
  if ! rospack find "$pkg" >/dev/null 2>&1; then
    echo "ERROR: missing ROS navigation package: $pkg"
    missing_pkg=1
  fi
done
if [ "$missing_pkg" -ne 0 ]; then
  echo "Install navigation packages first, for example:"
  echo "  sudo apt-get install ros-melodic-navigation ros-melodic-move-base ros-melodic-global-planner ros-melodic-dwa-local-planner"
  exit 1
fi

if rosnode list >/tmp/cluster_nodes.$$ 2>/dev/null; then
  if grep -qE '^/map_follower_robot[23]$' /tmp/cluster_nodes.$$; then
    echo "ERROR: map_follower is already running."
    echo "Stop start_map_follower.sh before starting navigation follower."
    rm -f /tmp/cluster_nodes.$$
    exit 1
  fi
fi
rm -f /tmp/cluster_nodes.$$

roslaunch cluster_bringup nav_follower.launch "$@"
