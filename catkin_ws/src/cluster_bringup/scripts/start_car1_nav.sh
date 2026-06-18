#!/bin/bash
set -e

cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh

if [ "$ROBOT_NS" != "robot1" ]; then
  echo "ERROR: start_car1_nav.sh must be run on car1."
  exit 1
fi

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
  if grep -qE '^/robot1/teleop_keyboard$' /tmp/cluster_nodes.$$; then
    echo "ERROR: /robot1/teleop_keyboard is running."
    echo "Stop keyboard teleop before starting car1 navigation so manual and navigation commands do not compete."
    rm -f /tmp/cluster_nodes.$$
    exit 1
  fi
  if grep -qE '^/robot1/move_base$' /tmp/cluster_nodes.$$; then
    echo "ERROR: /robot1/move_base is already running."
    rm -f /tmp/cluster_nodes.$$
    exit 1
  fi
  if ! grep -qE '^/map_follower_robot2$' /tmp/cluster_nodes.$$ ||
     ! grep -qE '^/map_follower_robot3$' /tmp/cluster_nodes.$$; then
    echo "WARN: map_follower_robot2/3 are not both running."
    echo "Followers will not keep formation until start_map_follower.sh is running."
  fi
fi
rm -f /tmp/cluster_nodes.$$

# roslaunch does not reliably remove parameters uploaded by an older launch.
# Stale pre-Hydro DWA keys otherwise override or confuse the current config.
rosparam delete /robot1/move_base >/dev/null 2>&1 || true

FORMATION="${1:-3}"
if rosservice list | grep -qx "/robot1/set_mode"; then
  rosservice call /robot1/set_mode "mode: 2
formation: ${FORMATION}
offset_x: 0.0
offset_y: 0.0
offset_yaw: 0.0" >/dev/null || true
fi

echo "Starting car1 move_base. Send goals to /robot1/move_base_simple/goal."
echo "Navigation velocity topic: /robot1/nav_vel"
echo "leader_controller remains the only /robot1/cmd_vel publisher."
roslaunch cluster_bringup car1_nav.launch
